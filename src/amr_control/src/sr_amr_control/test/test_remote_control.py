import importlib
import sys
import types
import unittest
from unittest import mock


class Twist:
  def __init__(self):
    self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class Joy:
  def __init__(self, axes=None, buttons=None):
    self.axes = list(axes or [])
    self.buttons = list(buttons or [])


class SetBool:
  class Request:
    def __init__(self, data=False):
      self.data = data

  class Response:
    def __init__(self):
      self.success = False
      self.message = ''


class RequestFailedError(Exception):
  def __init__(self, result_code):
    super().__init__(result_code)
    self.result_code = result_code


def _install_ros_stubs():
  geometry_msgs = types.ModuleType('geometry_msgs')
  geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
  geometry_msgs_msg.Twist = Twist

  sensor_msgs = types.ModuleType('sensor_msgs')
  sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
  sensor_msgs_msg.Joy = Joy

  rclpy = types.ModuleType('rclpy')
  callback_groups = types.ModuleType('rclpy.callback_groups')
  callback_groups.MutuallyExclusiveCallbackGroup = type(
    'MutuallyExclusiveCallbackGroup',
    (),
    {'__init__': lambda self, *args, **kwargs: None},
  )
  node_module = types.ModuleType('rclpy.node')
  node_module.Node = object

  sros_sdk_py = types.ModuleType('sros_sdk_py')
  sros_sdk_py.SrpClient = object
  sros_sdk_py.SrpConnectionConfig = object
  main_pb2 = types.ModuleType('sros_sdk_py.main_pb2')
  main_pb2.SystemState = type(
    'SystemState',
    (),
    {
      'OperationState': type('OperationState', (), {'OPERATION_MANUAL': 'manual'}),
      'EmergencyState': type(
        'EmergencyState',
        (),
        {
          'STATE_EMERGENCY_NA': 0,
          'STATE_EMERGENCY_NONE': 1,
          'STATE_EMERGENCY_TRIGGER': 2,
          'STATE_EMERGENCY_RECOVERABLE': 3,
        },
      ),
    },
  )
  srp = types.ModuleType('sros_sdk_py.srp')
  srp.RequestFailedError = RequestFailedError

  std_srvs = types.ModuleType('std_srvs')
  std_srvs_srv = types.ModuleType('std_srvs.srv')
  std_srvs_srv.SetBool = SetBool
  ros_bridge = types.ModuleType('sr_amr_control.utils.ros_bridge')
  ros_bridge.ros_to_srp_unit = lambda value: value * 1000.0

  return {
    'geometry_msgs': geometry_msgs,
    'geometry_msgs.msg': geometry_msgs_msg,
    'sensor_msgs': sensor_msgs,
    'sensor_msgs.msg': sensor_msgs_msg,
    'rclpy': rclpy,
    'rclpy.callback_groups': callback_groups,
    'rclpy.node': node_module,
    'sros_sdk_py': sros_sdk_py,
    'sros_sdk_py.main_pb2': main_pb2,
    'sros_sdk_py.srp': srp,
    'std_srvs': std_srvs,
    'std_srvs.srv': std_srvs_srv,
    'sr_amr_control.utils.ros_bridge': ros_bridge,
  }


def _load_remote_controller():
  package = importlib.import_module('sr_amr_control')
  utils_package = importlib.import_module('sr_amr_control.utils')
  original_remote_control_module = sys.modules.pop(
    'sr_amr_control.remote_control', None
  )
  had_remote_control_module = original_remote_control_module is not None
  original_ros_bridge_module = sys.modules.pop('sr_amr_control.utils.ros_bridge', None)
  had_ros_bridge_module = original_ros_bridge_module is not None
  original_remote_control = getattr(package, 'remote_control', None)
  had_remote_control = hasattr(package, 'remote_control')
  original_ros_bridge = getattr(utils_package, 'ros_bridge', None)
  had_ros_bridge = hasattr(utils_package, 'ros_bridge')

  with mock.patch.dict(sys.modules, _install_ros_stubs()):
    try:
      from sr_amr_control.remote_control import RemoteController
    finally:
      if had_remote_control:
        package.remote_control = original_remote_control
      elif hasattr(package, 'remote_control'):
        delattr(package, 'remote_control')

      if had_ros_bridge:
        utils_package.ros_bridge = original_ros_bridge
      elif hasattr(utils_package, 'ros_bridge'):
        delattr(utils_package, 'ros_bridge')

      if had_remote_control_module:
        sys.modules['sr_amr_control.remote_control'] = original_remote_control_module
      if had_ros_bridge_module:
        sys.modules['sr_amr_control.utils.ros_bridge'] = original_ros_bridge_module

  return RemoteController


class FakeLogger:
  def __init__(self):
    self.errors = []
    self.infos = []
    self.warnings = []

  def error(self, message):
    self.errors.append(message)

  def info(self, message):
    self.infos.append(message)

  def warning(self, message):
    self.warnings.append(message)


class FakeNode:
  def __init__(self):
    self.logger = FakeLogger()
    self.parameters = {}

  def create_service(self, *args, **kwargs):
    return (args, kwargs)

  def create_subscription(self, *args, **kwargs):
    return (args, kwargs)

  def declare_parameter(self, name, default_value=None, descriptor=None):
    value = self.parameters.get(name, default_value)
    return types.SimpleNamespace(value=value)

  def get_logger(self):
    return self.logger


class FakeSrpClient:
  def __init__(self):
    self.remote_control_requests = []
    self.oba_requests = []
    self.speed_requests = []
    self.emergency_stop_calls = []
    self.release_emergency_stop_calls = []
    self.system_state = None

  def set_remote_control(self, enabled):
    self.remote_control_requests.append(enabled)

  def set_remote_control_oba(self, enabled):
    self.oba_requests.append(enabled)

  def set_remote_control_speed(self, linear_x, linear_y, angular_z):
    self.speed_requests.append((linear_x, linear_y, angular_z))

  def emergency_stop(self):
    self.emergency_stop_calls.append(True)

  def release_emergency_stop(self):
    self.release_emergency_stop_calls.append(True)

  def get_current_system_state(self):
    return self.system_state


class FakeStateChecker:
  def __init__(self, remote_statuses):
    self.remote_statuses = list(remote_statuses)

  def check_remote_control_status(self, enabled):
    if not self.remote_statuses:
      return False
    return self.remote_statuses.pop(0) == enabled


def make_cmd_vel(linear_x=0.5, linear_y=-0.25, angular_z=0.1):
  msg = Twist()
  msg.linear.x = linear_x
  msg.linear.y = linear_y
  msg.angular.z = angular_z
  return msg


def make_joy(left_x=0.0, left_y=0.0, buttons=None):
  axes = [0.0, 0.0, 0.0, 0.0, left_x, left_y, 0.0, 0.0]
  return Joy(axes=axes, buttons=buttons)


class FakeSystemState:
  def __init__(self, emergency_state=1, operation_state=None):
    self.emergency_state = emergency_state
    self.operation_state = operation_state


class TestRemoteController(unittest.TestCase):
  def test_successful_enable_opens_cmd_vel_gate(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._state_checker = FakeStateChecker([True])

    response = controller._handle_remote_control_enabled(
      SetBool.Request(data=True), SetBool.Response()
    )
    controller._handle_cmd_vel(make_cmd_vel())

    self.assertTrue(response.success)
    self.assertTrue(controller._remote_control_enabled)
    self.assertEqual(srp_client.remote_control_requests, [True])
    self.assertEqual(srp_client.oba_requests, [False])
    self.assertEqual(srp_client.speed_requests, [(500, -250, 100)])

  def test_successful_disable_closes_cmd_vel_gate(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._remote_control_enabled = True
    controller._state_checker = FakeStateChecker([False])

    response = controller._handle_remote_control_enabled(
      SetBool.Request(data=False), SetBool.Response()
    )
    controller._handle_cmd_vel(make_cmd_vel())

    self.assertTrue(response.success)
    self.assertFalse(controller._remote_control_enabled)
    self.assertEqual(srp_client.remote_control_requests, [False])
    self.assertEqual(srp_client.oba_requests, [])
    self.assertEqual(srp_client.speed_requests, [])

  def test_timeout_preserves_previous_cmd_vel_gate(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._TIMEOUT_SEC = 0
    controller._remote_control_enabled = False
    controller._state_checker = FakeStateChecker([])

    response = controller._handle_remote_control_enabled(
      SetBool.Request(data=True), SetBool.Response()
    )
    controller._handle_cmd_vel(make_cmd_vel())

    self.assertFalse(response.success)
    self.assertFalse(controller._remote_control_enabled)
    self.assertEqual(srp_client.remote_control_requests, [True])
    self.assertEqual(srp_client.oba_requests, [False])
    self.assertEqual(srp_client.speed_requests, [])

  def test_joy_centered_maps_to_stop(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._remote_control_enabled = True

    controller._handle_button(make_joy())

    self.assertEqual(srp_client.speed_requests, [(0, 0, 0)])

  def test_joy_forward_is_linear(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._remote_control_enabled = True

    controller._handle_button(make_joy(left_y=1.0))

    self.assertEqual(srp_client.speed_requests, [(500, 0, 0)])

  def test_joy_backward_is_linear(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._remote_control_enabled = True

    controller._handle_button(make_joy(left_y=-1.0))

    self.assertEqual(srp_client.speed_requests, [(-500, 0, 0)])

  def test_joy_left_is_angular(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._remote_control_enabled = True

    controller._handle_button(make_joy(left_x=-1.0))

    self.assertEqual(srp_client.speed_requests, [(0, 0, 800)])

  def test_joy_right_is_angular(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._remote_control_enabled = True

    controller._handle_button(make_joy(left_x=1.0))

    self.assertEqual(srp_client.speed_requests, [(0, 0, -800)])

  def test_joy_deadzone_blocks_motion(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._remote_control_enabled = True

    controller._handle_button(make_joy(left_x=0.1, left_y=0.0))

    self.assertEqual(srp_client.speed_requests, [(0, 0, 0)])

  def test_joy_uses_dominant_axis(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._remote_control_enabled = True

    controller._handle_button(make_joy(left_x=0.4, left_y=0.6))

    self.assertEqual(srp_client.speed_requests, [(250, 0, 0)])

  def test_x_button_triggers_emergency_stop(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._remote_control_enabled = True

    controller._handle_button(make_joy(buttons=[1, 0]))

    self.assertEqual(srp_client.remote_control_requests, [False])
    self.assertEqual(srp_client.emergency_stop_calls, [True])
    self.assertFalse(controller._remote_control_enabled)

  def test_x_button_triggers_emergency_stop_even_if_exit_manual_fails(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()

    def fail_set_remote_control(enabled):
      raise RequestFailedError(70001)

    srp_client.set_remote_control = fail_set_remote_control
    controller = remote_controller(FakeNode(), srp_client)

    controller._handle_button(make_joy(buttons=[1, 0]))

    self.assertEqual(srp_client.emergency_stop_calls, [True])
    self.assertFalse(controller._remote_control_enabled)

  def test_x_button_only_triggers_on_rising_edge(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)

    controller._handle_button(make_joy(buttons=[1, 0]))
    controller._handle_button(make_joy(buttons=[1, 0]))

    self.assertEqual(srp_client.emergency_stop_calls, [True])

  def test_y_button_releases_estop_and_enables_remote_control(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    srp_client.system_state = FakeSystemState(emergency_state=1)  # STATE_EMERGENCY_NONE
    controller._state_checker = FakeStateChecker([True])

    controller._handle_button(make_joy(buttons=[0, 1]))

    self.assertEqual(srp_client.release_emergency_stop_calls, [True])
    self.assertEqual(srp_client.remote_control_requests, [True])
    self.assertEqual(srp_client.oba_requests, [False])
    self.assertTrue(controller._remote_control_enabled)

  def test_y_button_only_triggers_on_rising_edge(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    srp_client.system_state = FakeSystemState(emergency_state=1)
    controller._state_checker = FakeStateChecker([True])

    controller._handle_button(make_joy(buttons=[0, 1]))
    controller._handle_button(make_joy(buttons=[0, 1]))

    self.assertEqual(srp_client.release_emergency_stop_calls, [True])
    self.assertEqual(srp_client.remote_control_requests, [True])

  def test_y_button_does_not_enable_when_estop_still_active(self):
    remote_controller = _load_remote_controller()
    srp_client = FakeSrpClient()
    controller = remote_controller(FakeNode(), srp_client)
    controller._wait_until_emergency_released = lambda timeout=2.0: False

    controller._handle_button(make_joy(buttons=[0, 1]))

    self.assertEqual(srp_client.release_emergency_stop_calls, [True])
    self.assertEqual(srp_client.remote_control_requests, [])
    self.assertFalse(controller._remote_control_enabled)


if __name__ == '__main__':
  unittest.main()
