import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sros_sdk_py import SrpClient, SrpConnectionConfig

from .charge_action import ChargeActionServer
from .estop_srv import EStopSrv
from .lidar_reporter import LidarReporter
from .localization_action import LocalizationActionServer
from .move_follow_path_action import MoveFollowPathAction
from .move_pause_srv import MovePauseSrv
from .move_to_station_action import MoveToStationAction
from .remote_control import RemoteController
from .state_publisher import StatePublisher
from .utils.internal_state import InternalState


def _parameter_value_to_bool(value) -> bool:
  if isinstance(value, bool):
    return value
  if isinstance(value, str):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')
  return bool(value)


def _get_parameter_value(node: Node, name: str, default=None):
  try:
    parameter = node.get_parameter(name)
  except Exception:
    return default

  value = getattr(parameter, 'value', default)
  return default if value is None else value


class SrAmrControl(Node):
  _srp_client = SrpClient()
  _internal_state: InternalState = InternalState()
  _frame_id: str
  _parent_frame_id: str

  _move_to_station_action: MoveToStationAction
  _move_follow_path_action: MoveFollowPathAction
  _state_publisher: StatePublisher
  _remote_controller: RemoteController
  _lidar_reporter: LidarReporter
  _move_pause_srv: MovePauseSrv
  _estop_srv: EStopSrv
  _charge_action: ChargeActionServer
  _localization_action: LocalizationActionServer

  _sync_state_timer: Time

  def __init__(self):
    super().__init__('sr_amr_control')

    self.declare_parameter(
      'connect_ip', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
    )
    self.declare_parameter('connect_username', '')
    self.declare_parameter('connect_passwd', '')
    self.declare_parameter('frame_id', 'base_link')
    self.declare_parameter('parent_frame_id', 'map')
    self.declare_parameter('odom_frame_id', 'odom')
    self.declare_parameter('odom_topic', 'odom')
    self.declare_parameter('odom_use_integration', True)
    self.declare_parameter('lidar_points_frame_id', 'map')
    self.declare_parameter('lidar_front_frame_id', 'right_front_laser_link')
    self.declare_parameter('lidar_rear_frame_id', 'left_behind_laser_link')
    self.declare_parameter('lidar', False)

    connect_ip = _get_parameter_value(self, 'connect_ip')
    connect_username: str | None = _get_parameter_value(self, 'connect_username', '')
    connect_passwd: str | None = _get_parameter_value(self, 'connect_passwd', '')
    frame_id = _get_parameter_value(self, 'frame_id', 'base_link')
    parent_frame_id = _get_parameter_value(self, 'parent_frame_id', 'map')
    odom_frame_id: str | None = _get_parameter_value(self, 'odom_frame_id', 'odom')
    odom_topic: str | None = _get_parameter_value(self, 'odom_topic', 'odom')
    odom_use_integration = _parameter_value_to_bool(
      _get_parameter_value(self, 'odom_use_integration', True)
    )
    lidar_points_frame_id: str | None = _get_parameter_value(
      self, 'lidar_points_frame_id', 'map'
    )
    lidar_front_parameter = _get_parameter_value(self, 'lidar_front_frame_id', None)
    lidar_rear_parameter = _get_parameter_value(self, 'lidar_rear_frame_id', None)
    lidar_enabled = _parameter_value_to_bool(_get_parameter_value(self, 'lidar', False))

    if frame_id:
      self._frame_id = frame_id

    if parent_frame_id:
      self._parent_frame_id = parent_frame_id

    if not odom_frame_id:
      odom_frame_id = 'odom'

    if not odom_topic:
      odom_topic = 'odom'

    lidar_front_frame_id = self._frame_id
    lidar_rear_frame_id = self._frame_id
    if lidar_front_parameter:
      lidar_front_frame_id = lidar_front_parameter
    if lidar_rear_parameter:
      lidar_rear_frame_id = lidar_rear_parameter

    if not connect_ip:
      self.get_logger().error('Required parameter [connect_ip] is not set!')
      return

    connection_config = SrpConnectionConfig(ip=connect_ip)

    if connect_username:
      connection_config.username = connect_username
    if connect_passwd:
      connection_config.passwd = connect_passwd

    connected = self._srp_client.connect(connection_config)

    if not connected:
      self.get_logger().error(f'Cannot connect to {connect_ip}')
      return

    self.get_logger().info(f'Successfully connect to {connect_ip}')

    self._move_to_station_action = MoveToStationAction(
      self, self._srp_client, self._internal_state
    )
    self._move_follow_path_action = MoveFollowPathAction(
      self, self._srp_client, self._internal_state
    )
    self._move_pause_srv = MovePauseSrv(self, self._srp_client)
    self._state_publisher = StatePublisher(
      self,
      self._srp_client,
      self._frame_id,
      self._parent_frame_id,
      odom_frame_id,
      odom_topic,
      odom_use_integration,
    )
    self._lidar_reporter = LidarReporter(
      self,
      self._srp_client,
      lidar_points_frame_id,  # type: ignore
      front_frame_id=lidar_front_frame_id,
      rear_frame_id=lidar_rear_frame_id,
      enabled=lidar_enabled,
    )
    self._remote_controller = RemoteController(self, self._srp_client)
    self._estop_srv = EStopSrv(self, self._srp_client)
    self._charge_action = ChargeActionServer(self, self._srp_client)
    self._localization_action = LocalizationActionServer(self, self._srp_client)

    self._get_status_timer = self.create_timer(0.3, self._sync_state)

  def disconnect(self):
    is_ok = rclpy.ok()
    if is_ok:
      self.get_logger().info('Disconnecting...')
    else:
      print('Disconnecting... (ROS context ended)')
    self._srp_client.disconnect()
    if is_ok:
      self.get_logger().info('Disconnected succeed.')
    else:
      print('Disconnected succeed (ROS context ended).')

  def _sync_state(self):
    self._srp_client.fetch_system_state()
    self._srp_client.fetch_hardware_state()


def main(args=None):
  rclpy.init(args=args)
  control_node = SrAmrControl()
  executor = MultiThreadedExecutor()
  try:
    rclpy.spin(control_node, executor=executor)
  except KeyboardInterrupt:
    pass
  finally:
    control_node.disconnect()
    control_node.destroy_node()
    try:
      rclpy.shutdown()
    except Exception:
      pass


if __name__ == '__main__':
  main()
