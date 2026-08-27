import math

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from sros_sdk_py import SrpClient
from sros_sdk_py.srp import RequestFailedError
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

from .utils.ros_bridge import srp_to_ros_unit


class LidarReporter:
  _ANGLE_MIN = -math.pi
  _ANGLE_MAX = math.pi
  _ANGLE_INCREMENT = math.radians(1.0)
  _RANGE_MIN = 0.0
  _RANGE_MAX = 100.0
  _FRONT_LIDAR_UUID = 111
  _REAR_LIDAR_UUID = 112

  _node: Node
  _srp_client: SrpClient
  _front_publisher: Publisher
  _rear_publisher: Publisher
  _points_frame_id: str
  _front_frame_id: str
  _rear_frame_id: str
  _enabled: bool
  _laser_packet_count: int
  _tf_buffer: Buffer
  _tf_listener: TransformListener
  _service_group = MutuallyExclusiveCallbackGroup()

  def __init__(
    self,
    node: Node,
    srp_client: SrpClient,
    points_frame_id: str,
    front_frame_id: str | None = None,
    rear_frame_id: str | None = None,
    enabled: bool = False,
  ):
    self._node = node
    self._srp_client = srp_client
    self._points_frame_id = points_frame_id
    self._front_frame_id = front_frame_id or points_frame_id
    self._rear_frame_id = rear_frame_id or points_frame_id
    self._enabled = False
    self._laser_packet_count = 0
    self._tf_buffer = Buffer()
    self._tf_listener = TransformListener(self._tf_buffer, self._node)

    self._front_publisher = node.create_publisher(LaserScan, 'front/scan', 10)
    self._rear_publisher = node.create_publisher(LaserScan, 'rear/scan', 10)
    self._lidar_enabled_srv = node.create_service(
      SetBool,
      'lidar_enabled',
      self._handle_lidar_enabled,
      callback_group=self._service_group,
    )

    self._install_laser_callback()

    if enabled:
      try:
        self._set_lidar_upload_enabled(True)
        self._enabled = True
      except RequestFailedError as e:
        self._node.get_logger().error(
          f'Enable lidar reporting failed, error_code: {e.result_code}'
        )

  @property
  def enabled(self) -> bool:
    return self._enabled

  def _install_laser_callback(self):
    self._srp_client.set_laser_point_callback(self.publish_lidar_scans)

  def _set_lidar_upload_enabled(self, enabled: bool):
    return self._srp_client.set_laser_point_upload(enabled)

  def _handle_lidar_enabled(self, request: SetBool.Request, response: SetBool.Response):
    target_enable = request.data

    try:
      self._set_lidar_upload_enabled(target_enable)
    except RequestFailedError as e:
      self._node.get_logger().error(
        f'Lidar reporting srv called failed, error_code: {e.result_code}'
      )
      response.success = False
      response.message = f'{e.result_code}'
      return response
    except Exception as e:
      self._node.get_logger().error(f'Lidar reporting srv called failed: {e}')
      response.success = False
      response.message = str(e)
      return response

    self._enabled = target_enable
    response.success = True
    response.message = f'Lidar reporting successfully set to {target_enable}.'
    self._node.get_logger().info(
      f'Lidar reporting state transitioned to {target_enable} successfully.'
    )
    return response

  def publish_lidar_scans(self, laser_points):
    if laser_points is None:
      return

    self._laser_packet_count += 1
    if self._laser_packet_count == 1:
      loc_count = len(getattr(laser_points, 'loc_laser_points', []))
      self._node.get_logger().info(
        f'First lidar packet received (enabled={self._enabled}, loc={loc_count})'
      )

    if not self._enabled:
      return

    if not self._transforms_ready():
      return

    front_points, rear_points = self._collect_lidar_points(laser_points)
    front_points = self._transform_scan_points(front_points, self._front_frame_id)
    rear_points = self._transform_scan_points(rear_points, self._rear_frame_id)

    if not front_points and not rear_points:
      return

    if rclpy.ok():
      try:
        stamp = self._node.get_clock().now().to_msg()
        if front_points:
          self._front_publisher.publish(
            self._make_scan(stamp, self._front_frame_id, front_points)
          )
        if rear_points:
          self._rear_publisher.publish(
            self._make_scan(stamp, self._rear_frame_id, rear_points)
          )
      except Exception as e:
        self._node.get_logger().error(f'Publish lidar points failed: {e}')
    else:
      self._node.get_logger().debug(
        'Skipping lidar publish: ROS2 context is invalid (shutting down)'
      )

  def _collect_lidar_points(self, laser_points):
    front_points = []
    rear_points = []
    for sensor_points in getattr(laser_points, 'loc_laser_points', []):
      points = list(
        self._make_scan_points(
          getattr(sensor_points, 'xs', []),
          getattr(sensor_points, 'ys', []),
          getattr(sensor_points, 'reliabilitys', []),
        )
      )
      if not points:
        continue

      sensor_uuid = int(getattr(sensor_points, 'sensor_uuid', 0) or 0)
      if sensor_uuid == self._FRONT_LIDAR_UUID:
        front_points.extend(points)
      elif sensor_uuid == self._REAR_LIDAR_UUID:
        rear_points.extend(points)

    return front_points, rear_points

  def _make_scan_points(self, xs, ys, reliabilitys):
    for index, (x, y) in enumerate(zip(xs, ys)):
      intensity = float(reliabilitys[index]) if index < len(reliabilitys) else 0.0
      yield srp_to_ros_unit(x), srp_to_ros_unit(y), intensity

  def _transform_scan_points(self, scan_points, target_frame_id: str):
    if not scan_points:
      return []
    if target_frame_id == self._points_frame_id:
      return scan_points

    try:
      transform = self._tf_buffer.lookup_transform(
        target_frame_id,
        self._points_frame_id,
        Time(),
      )
    except Exception:
      return []

    t = transform.transform.translation
    q = transform.transform.rotation

    transformed_points = []
    for x, y, intensity in scan_points:
      x_t, y_t = self._transform_point_xy(x, y, t.x, t.y, t.z, q.x, q.y, q.z, q.w)
      transformed_points.append((x_t, y_t, intensity))
    return transformed_points

  def _transforms_ready(self) -> bool:
    front_ready = self._tf_buffer.can_transform(
      self._front_frame_id,
      self._points_frame_id,
      Time(),
    )
    rear_ready = self._tf_buffer.can_transform(
      self._rear_frame_id,
      self._points_frame_id,
      Time(),
    )
    return front_ready and rear_ready

  def _transform_point_xy(
    self,
    x: float,
    y: float,
    tx: float,
    ty: float,
    tz: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
  ):
    # Rotate point by quaternion then translate (source frame -> target frame).
    vx = x
    vy = y
    vz = 0.0

    ux = qx
    uy = qy
    uz = qz
    s = qw

    dot_uv = ux * vx + uy * vy + uz * vz
    dot_uu = ux * ux + uy * uy + uz * uz
    cross_x = uy * vz - uz * vy
    cross_y = uz * vx - ux * vz
    cross_z = ux * vy - uy * vx

    rx = 2.0 * dot_uv * ux + (s * s - dot_uu) * vx + 2.0 * s * cross_x
    ry = 2.0 * dot_uv * uy + (s * s - dot_uu) * vy + 2.0 * s * cross_y
    _rz = 2.0 * dot_uv * uz + (s * s - dot_uu) * vz + 2.0 * s * cross_z

    return rx + tx, ry + ty

  def _make_scan(self, stamp, frame_id: str, scan_points) -> LaserScan:
    scan = LaserScan()
    scan.header.stamp = stamp
    scan.header.frame_id = frame_id
    scan.angle_min = self._ANGLE_MIN
    scan.angle_max = self._ANGLE_MAX
    scan.angle_increment = self._ANGLE_INCREMENT
    scan.time_increment = 0.0
    scan.scan_time = 0.0
    scan.range_min = self._RANGE_MIN
    scan.range_max = self._RANGE_MAX

    beam_count = (
      int(round((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1
    )
    # Use finite out-of-range defaults to keep viewers like Foxglove from
    # flagging Infinity as invalid while preserving "no return" semantics.
    scan.ranges = [scan.range_max + 1.0] * beam_count
    scan.intensities = [0.0] * beam_count

    for x, y, intensity in scan_points:
      distance = math.hypot(x, y)
      if distance < scan.range_min or distance > scan.range_max:
        continue

      angle = math.atan2(y, x)
      index = int(round((angle - scan.angle_min) / scan.angle_increment))
      if 0 <= index < beam_count and distance < scan.ranges[index]:
        scan.ranges[index] = distance
        scan.intensities[index] = intensity

    return scan
