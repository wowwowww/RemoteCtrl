import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.publisher import Publisher
from sros_sdk_py import SrpClient
from sros_sdk_py.main_pb2 import (
  HardwareState as HardwareStateProtocol,
)
from sros_sdk_py.main_pb2 import (
  MovementTask as MovementTaskProtocol,
)
from sros_sdk_py.main_pb2 import (
  SystemState as SystemStateProtocol,
)
from tf2_ros import TransformBroadcaster, TransformStamped

from sr_amr_interfaces.msg import (
  BatteryState as BatteryStateMsg,
)
from sr_amr_interfaces.msg import (
  SystemState as SystemStateMsg,
)

from .estop_srv import EStopStateChecker
from .remote_control import RemoteControlStateChecker
from .utils.ros_bridge import srp_to_ros_unit, to_ros_pose


class MovementTaskStateCtrl:
  @staticmethod
  def is_moving(task: MovementTaskProtocol) -> bool:
    return task.state not in [
      MovementTaskProtocol.MT_NA,
      MovementTaskProtocol.MT_FINISHED,
    ]


class StatePublisher:
  _node: Node
  _srp_client: SrpClient
  _sys_state_publisher: Publisher
  _battery_state_publisher: Publisher
  _odom_publisher: Publisher
  _frame_id: str
  _parent_frame_id: str
  _odom_frame_id: str
  _odom_topic: str
  _remote_control_state_checker: RemoteControlStateChecker
  _tf_broadcaster: TransformBroadcaster
  _odom_x: float
  _odom_y: float
  _odom_yaw: float
  _last_odom_time_ns: int | None
  _odom_initialized: bool
  _odom_use_integration: bool

  def __init__(
    self,
    node: Node,
    client: SrpClient,
    frame_id: str,
    parent_frame_id: str,
    odom_frame_id: str,
    odom_topic: str,
    odom_use_integration: bool,
  ):
    self._node = node
    self._srp_client = client
    self._frame_id = frame_id
    self._parent_frame_id = parent_frame_id
    self._odom_frame_id = odom_frame_id
    self._odom_topic = odom_topic
    self._odom_use_integration = odom_use_integration
    self._remote_control_state_checker = RemoteControlStateChecker(client)
    self._tf_broadcaster = TransformBroadcaster(node)
    self._odom_x = 0.0
    self._odom_y = 0.0
    self._odom_yaw = 0.0
    self._last_odom_time_ns = None
    self._odom_initialized = False

    self._sys_state_publisher = self._node.create_publisher(
      SystemStateMsg, 'system_state', 10
    )
    self._battery_state_publisher = self._node.create_publisher(
      BatteryStateMsg, 'battery_state', 10
    )
    self._odom_publisher = self._node.create_publisher(Odometry, self._odom_topic, 10)

    self._srp_client.add_system_state_callback(self.publish_sys_state)
    self._srp_client.add_hardware_state_callback(self.publish_battery_state)

  def _srp_location_state_to_msg_location_state(
    self, location_state: SystemStateProtocol.LocationState
  ) -> int:
    State = SystemStateProtocol.LocationState

    match location_state:
      case State.LOCATION_STATE_ZERO:
        return SystemStateMsg.LOCATION_STATE_ZERO
      case State.LOCATION_STATE_NONE:
        return SystemStateMsg.LOCATION_STATE_NONE
      case State.LOCATION_STATE_INITIALING:
        return SystemStateMsg.LOCATION_STATE_INITIALING
      case State.LOCATION_STATE_RUNNING:
        return SystemStateMsg.LOCATION_STATE_RUNNING
      case State.LOCATION_STATE_RELOCATING:
        return SystemStateMsg.LOCATION_STATE_RELOCATING
      case State.LOCATION_STATE_ERROR:
        return SystemStateMsg.LOCATION_STATE_ERROR

    return SystemStateMsg.LOCATION_STATE_UNKNOWN

  def publish_sys_state(self, system_state: SystemStateProtocol):
    if system_state is None:
      return

    current_pose = to_ros_pose(system_state.location_pose)

    msg = SystemStateMsg()
    msg.header.stamp = self._node.get_clock().now().to_msg()
    msg.header.frame_id = self._parent_frame_id

    msg.current_pose = current_pose
    msg.current_pose.header = msg.header

    msg.remote_control_active = (
      self._remote_control_state_checker.get_remote_control_status()
    )
    msg.remote_control_oba_active = (
      self._remote_control_state_checker.get_remote_control_oba_status()
    )
    msg.estop_active = EStopStateChecker.is_emergency_stopped(system_state)

    msg.current_map_name = system_state.map_name
    msg.current_station_id = system_state.station_no

    msg.location_confidence_score = system_state.location_pose.confidence
    msg.location_state = self._srp_location_state_to_msg_location_state(
      system_state.location_state
    )

    mc_state = system_state.mc_state
    msg.linear_velocity_x = srp_to_ros_unit(mc_state.v_x)
    msg.linear_velocity_y = srp_to_ros_unit(mc_state.v_y)
    msg.angular_velocity = srp_to_ros_unit(mc_state.w)

    msg.executing_movement_task = MovementTaskStateCtrl.is_moving(
      system_state.movement_state
    )
    msg.movement_task_manual_paused = (
      system_state.sys_state == SystemStateProtocol.SYS_STATE_TASK_MANUAL_PAUSED
    )
    msg.movement_task_obstacle_paused = (
      system_state.sys_state == SystemStateProtocol.SYS_STATE_TASK_NAV_PAUSED
    )

    if rclpy.ok():
      try:
        self._sys_state_publisher.publish(msg)
        self._publish_tf(
          system_state.location_pose,
          msg.linear_velocity_x,
          msg.linear_velocity_y,
          msg.angular_velocity,
        )
      except Exception as e:
        self._node.get_logger().error(f'Publish failed: {e}')
    else:
      self._node.get_logger().debug(
        'Skipping publish: ROS2 context is invalid (shutting down)'
      )

  def _battery_state_to_msg_battery_state(
    self, battery_state: HardwareStateProtocol.BatteryState
  ) -> int:
    State = HardwareStateProtocol.BatteryState

    match battery_state:
      case State.BATTERY_NA:
        return BatteryStateMsg.STATE_NA
      case State.BATTERY_CHARGING:
        return BatteryStateMsg.STATE_CHARING
      case State.BATTERY_NO_CHARGING:
        return BatteryStateMsg.STATE_NO_CHARING

    return BatteryStateMsg.STATE_UNKNOWN

  def publish_battery_state(self, hardware_state: HardwareStateProtocol):
    if hardware_state is None:
      return

    msg = BatteryStateMsg()

    msg.remaining_percentage = hardware_state.battery_percentage
    msg.remaining_capacity = hardware_state.battery_remain_capacity
    msg.remaining_time = hardware_state.battery_remain_time

    msg.state = self._battery_state_to_msg_battery_state(hardware_state.battery_state)

    msg.voltage = hardware_state.battery_voltage
    msg.current = hardware_state.battery_current
    msg.temperature = hardware_state.battery_temperature

    msg.nominal_capacity = hardware_state.battery_nominal_capacity
    msg.use_cycles = hardware_state.battery_use_cycles

    msg.sn = hardware_state.battery_sn

    if rclpy.ok():
      try:
        self._battery_state_publisher.publish(msg)
      except Exception as e:
        self._node.get_logger().error(f'Publish failed: {e}')
    else:
      self._node.get_logger().debug(
        'Skipping publish: ROS2 context is invalid (shutting down)'
      )

  def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

  def _quaternion_from_yaw(self, yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

  def _compose_2d(
    self,
    x1: float,
    y1: float,
    yaw1: float,
    x2: float,
    y2: float,
    yaw2: float,
  ) -> tuple[float, float, float]:
    cos_yaw = math.cos(yaw1)
    sin_yaw = math.sin(yaw1)
    x = x1 + cos_yaw * x2 - sin_yaw * y2
    y = y1 + sin_yaw * x2 + cos_yaw * y2
    return (x, y, yaw1 + yaw2)

  def _inverse_2d(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    inv_x = -(cos_yaw * x + sin_yaw * y)
    inv_y = -(-sin_yaw * x + cos_yaw * y)
    return (inv_x, inv_y, -yaw)

  def _publish_tf(self, system_pose, vx: float, vy: float, w: float):
    if system_pose is None:
      return

    now = self._node.get_clock().now()
    now_ns = now.nanoseconds

    ros_pose = to_ros_pose(system_pose)
    map_position = ros_pose.pose.position
    map_orientation = ros_pose.pose.orientation
    map_yaw = self._yaw_from_quaternion(
      map_orientation.x,
      map_orientation.y,
      map_orientation.z,
      map_orientation.w,
    )

    if not self._odom_use_integration:
      self._odom_x = map_position.x
      self._odom_y = map_position.y
      self._odom_yaw = map_yaw
      self._odom_initialized = True
      self._last_odom_time_ns = now_ns
    elif not self._odom_initialized:
      self._odom_x = map_position.x
      self._odom_y = map_position.y
      self._odom_yaw = map_yaw
      self._odom_initialized = True
      self._last_odom_time_ns = now_ns
    elif self._last_odom_time_ns is not None and now_ns > self._last_odom_time_ns:
      dt = (now_ns - self._last_odom_time_ns) / 1e9

      cos_yaw = math.cos(self._odom_yaw)
      sin_yaw = math.sin(self._odom_yaw)
      self._odom_x += (vx * cos_yaw - vy * sin_yaw) * dt
      self._odom_y += (vx * sin_yaw + vy * cos_yaw) * dt
      self._odom_yaw += w * dt
      self._last_odom_time_ns = now_ns

    odom_to_base = TransformStamped()
    odom_to_base.header.stamp = now.to_msg()
    odom_to_base.header.frame_id = self._odom_frame_id
    odom_to_base.child_frame_id = self._frame_id
    odom_to_base.transform.translation.x = self._odom_x
    odom_to_base.transform.translation.y = self._odom_y
    odom_to_base.transform.translation.z = 0.0
    qx, qy, qz, qw = self._quaternion_from_yaw(self._odom_yaw)
    odom_to_base.transform.rotation.x = qx
    odom_to_base.transform.rotation.y = qy
    odom_to_base.transform.rotation.z = qz
    odom_to_base.transform.rotation.w = qw

    if self._odom_use_integration:
      odom_to_base_inv = self._inverse_2d(self._odom_x, self._odom_y, self._odom_yaw)
      map_to_odom = self._compose_2d(
        map_position.x,
        map_position.y,
        map_yaw,
        odom_to_base_inv[0],
        odom_to_base_inv[1],
        odom_to_base_inv[2],
      )
    else:
      map_to_odom = (0.0, 0.0, 0.0)

    map_to_odom_tf = TransformStamped()
    map_to_odom_tf.header.stamp = now.to_msg()
    map_to_odom_tf.header.frame_id = self._parent_frame_id
    map_to_odom_tf.child_frame_id = self._odom_frame_id
    map_to_odom_tf.transform.translation.x = map_to_odom[0]
    map_to_odom_tf.transform.translation.y = map_to_odom[1]
    map_to_odom_tf.transform.translation.z = 0.0
    qx, qy, qz, qw = self._quaternion_from_yaw(map_to_odom[2])
    map_to_odom_tf.transform.rotation.x = qx
    map_to_odom_tf.transform.rotation.y = qy
    map_to_odom_tf.transform.rotation.z = qz
    map_to_odom_tf.transform.rotation.w = qw

    self._tf_broadcaster.sendTransform([map_to_odom_tf, odom_to_base])

    odom_msg = Odometry()
    odom_msg.header.stamp = now.to_msg()
    odom_msg.header.frame_id = self._odom_frame_id
    odom_msg.child_frame_id = self._frame_id
    odom_msg.pose.pose.position.x = self._odom_x
    odom_msg.pose.pose.position.y = self._odom_y
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation.x = odom_to_base.transform.rotation.x
    odom_msg.pose.pose.orientation.y = odom_to_base.transform.rotation.y
    odom_msg.pose.pose.orientation.z = odom_to_base.transform.rotation.z
    odom_msg.pose.pose.orientation.w = odom_to_base.transform.rotation.w
    odom_msg.twist.twist.linear.x = vx
    odom_msg.twist.twist.linear.y = vy
    odom_msg.twist.twist.angular.z = w
    self._odom_publisher.publish(odom_msg)
