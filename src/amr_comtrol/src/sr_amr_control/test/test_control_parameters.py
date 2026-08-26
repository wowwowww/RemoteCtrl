import importlib
import sys
import types
import unittest
from unittest import mock


class FakeParameter:
  def __init__(self, value):
    self.value = value


class FakeSrpClient:
  connect_result = False
  last_config = None

  def connect(self, config):
    FakeSrpClient.last_config = config
    return FakeSrpClient.connect_result

  def disconnect(self):
    pass


class FakeSrpConnectionConfig:
  def __init__(self, ip):
    self.ip = ip
    self.username = ''
    self.passwd = ''


class FakeLogger:
  def error(self, *args, **kwargs):
    pass

  def info(self, *args, **kwargs):
    pass


def _ros_stubs():
  rclpy = types.ModuleType('rclpy')
  rclpy.ok = lambda: True
  executors = types.ModuleType('rclpy.executors')
  executors.MultiThreadedExecutor = object
  node_module = types.ModuleType('rclpy.node')
  node_module.Node = type(
    'Node',
    (),
    {
      '__init__': lambda self, *args, **kwargs: None,
      'declare_parameter': lambda self, *args, **kwargs: None,
      'get_parameter': lambda self, name: FakeParameter(None),
      'get_logger': lambda self: FakeLogger(),
      'create_timer': lambda self, *args, **kwargs: object(),
    },
  )
  time_module = types.ModuleType('rclpy.time')
  time_module.Time = object

  rcl_interfaces = types.ModuleType('rcl_interfaces')
  rcl_interfaces_msg = types.ModuleType('rcl_interfaces.msg')
  rcl_interfaces_msg.ParameterDescriptor = type(
    'ParameterDescriptor', (), {'__init__': lambda self, *args, **kwargs: None}
  )
  rcl_interfaces_msg.ParameterType = type('ParameterType', (), {'PARAMETER_STRING': 4})

  sros_sdk_py = types.ModuleType('sros_sdk_py')
  sros_sdk_py.SrpClient = FakeSrpClient
  sros_sdk_py.SrpConnectionConfig = FakeSrpConnectionConfig

  dependency_classes = {
    'sr_amr_control.charge_action': 'ChargeActionServer',
    'sr_amr_control.estop_srv': 'EStopSrv',
    'sr_amr_control.lidar_reporter': 'LidarReporter',
    'sr_amr_control.localization_action': 'LocalizationActionServer',
    'sr_amr_control.move_follow_path_action': 'MoveFollowPathAction',
    'sr_amr_control.move_pause_srv': 'MovePauseSrv',
    'sr_amr_control.move_to_station_action': 'MoveToStationAction',
    'sr_amr_control.remote_control': 'RemoteController',
    'sr_amr_control.state_publisher': 'StatePublisher',
  }
  dependency_modules = {}
  for module_name, class_name in dependency_classes.items():
    dependency_module = types.ModuleType(module_name)
    setattr(dependency_module, class_name, mock.Mock(name=class_name))
    dependency_modules[module_name] = dependency_module

  internal_state_module = types.ModuleType('sr_amr_control.utils.internal_state')
  internal_state_module.InternalState = type('InternalState', (), {})

  return {
    'rclpy': rclpy,
    'rclpy.executors': executors,
    'rclpy.node': node_module,
    'rclpy.time': time_module,
    'rcl_interfaces': rcl_interfaces,
    'rcl_interfaces.msg': rcl_interfaces_msg,
    'sros_sdk_py': sros_sdk_py,
    'sr_amr_control.utils.internal_state': internal_state_module,
    **dependency_modules,
  }


class TestControlParameters(unittest.TestCase):
  def test_connection_config_includes_username_and_password(self):
    def get_parameter(name):
      values = {
        'connect_ip': '192.0.2.10',
        'connect_username': 'operator',
        'connect_passwd': 'secret',
        'frame_id': 'base_footprint',
        'parent_frame_id': 'odom',
        'lidar_points_frame_id': 'map',
        'lidar_front_frame_id': 'right_front_laser_link',
        'lidar_rear_frame_id': 'left_behind_laser_link',
        'lidar': False,
      }
      return FakeParameter(values[name])

    FakeSrpClient.last_config = None
    FakeSrpClient.connect_result = False

    package = importlib.import_module('sr_amr_control')
    original_main_module = sys.modules.pop('sr_amr_control.main', None)
    had_main_module = original_main_module is not None
    original_main = getattr(package, 'main', None)
    had_main = hasattr(package, 'main')
    with mock.patch.dict(sys.modules, _ros_stubs()):
      try:
        from sr_amr_control import main as control_main

        control_main.SrAmrControl._srp_client = FakeSrpClient()

        with (
          mock.patch.object(control_main.SrAmrControl, 'declare_parameter'),
          mock.patch.object(
            control_main.SrAmrControl, 'get_parameter', side_effect=get_parameter
          ),
          mock.patch.object(
            control_main.SrAmrControl, 'get_logger', return_value=FakeLogger()
          ),
        ):
          control_main.SrAmrControl()
      finally:
        if had_main:
          package.main = original_main
        elif hasattr(package, 'main'):
          delattr(package, 'main')

        if had_main_module:
          sys.modules['sr_amr_control.main'] = original_main_module

    self.assertEqual(FakeSrpClient.last_config.ip, '192.0.2.10')
    self.assertEqual(FakeSrpClient.last_config.username, 'operator')
    self.assertEqual(FakeSrpClient.last_config.passwd, 'secret')

  def test_lidar_parameter_defaults_false_and_is_passed_to_reporter(self):
    def get_parameter(name):
      values = {
        'connect_ip': '192.0.2.10',
        'connect_username': '',
        'connect_passwd': '',
        'frame_id': 'base_footprint',
        'parent_frame_id': 'odom',
        'lidar_points_frame_id': 'map',
        'lidar_front_frame_id': 'right_front_laser_link',
        'lidar_rear_frame_id': 'left_behind_laser_link',
        'lidar': True,
      }
      return FakeParameter(values[name])

    FakeSrpClient.connect_result = True

    package = importlib.import_module('sr_amr_control')
    original_main_module = sys.modules.pop('sr_amr_control.main', None)
    had_main_module = original_main_module is not None
    original_main = getattr(package, 'main', None)
    had_main = hasattr(package, 'main')
    with mock.patch.dict(sys.modules, _ros_stubs()):
      try:
        from sr_amr_control import main as control_main

        control_main.SrAmrControl._srp_client = FakeSrpClient()

        with (
          mock.patch.object(control_main.SrAmrControl, 'declare_parameter') as declare,
          mock.patch.object(
            control_main.SrAmrControl, 'get_parameter', side_effect=get_parameter
          ),
          mock.patch.object(
            control_main.SrAmrControl, 'get_logger', return_value=FakeLogger()
          ),
        ):
          control_main.SrAmrControl()
      finally:
        if had_main:
          package.main = original_main
        elif hasattr(package, 'main'):
          delattr(package, 'main')

        if had_main_module:
          sys.modules['sr_amr_control.main'] = original_main_module

    declare.assert_any_call('lidar', False)
    control_main.LidarReporter.assert_called_once()
    _, _, frame_id = control_main.LidarReporter.call_args.args
    self.assertEqual(frame_id, 'map')
    self.assertTrue(control_main.LidarReporter.call_args.kwargs['enabled'])
    FakeSrpClient.connect_result = False

  def test_lidar_string_false_is_not_enabled(self):
    def get_parameter(name):
      values = {
        'connect_ip': '192.0.2.10',
        'connect_username': '',
        'connect_passwd': '',
        'frame_id': 'base_footprint',
        'parent_frame_id': 'odom',
        'lidar_points_frame_id': 'map',
        'lidar_front_frame_id': 'right_front_laser_link',
        'lidar_rear_frame_id': 'left_behind_laser_link',
        'lidar': 'false',
      }
      return FakeParameter(values[name])

    FakeSrpClient.connect_result = True

    package = importlib.import_module('sr_amr_control')
    original_main_module = sys.modules.pop('sr_amr_control.main', None)
    had_main_module = original_main_module is not None
    original_main = getattr(package, 'main', None)
    had_main = hasattr(package, 'main')
    with mock.patch.dict(sys.modules, _ros_stubs()):
      try:
        from sr_amr_control import main as control_main

        control_main.SrAmrControl._srp_client = FakeSrpClient()

        with (
          mock.patch.object(control_main.SrAmrControl, 'declare_parameter'),
          mock.patch.object(
            control_main.SrAmrControl, 'get_parameter', side_effect=get_parameter
          ),
          mock.patch.object(
            control_main.SrAmrControl, 'get_logger', return_value=FakeLogger()
          ),
        ):
          control_main.SrAmrControl()
      finally:
        if had_main:
          package.main = original_main
        elif hasattr(package, 'main'):
          delattr(package, 'main')

        if had_main_module:
          sys.modules['sr_amr_control.main'] = original_main_module

    self.assertFalse(control_main.LidarReporter.call_args.kwargs['enabled'])
    FakeSrpClient.connect_result = False


if __name__ == '__main__':
  unittest.main()
