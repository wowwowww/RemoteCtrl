import importlib
import sys
import types
import unittest
from unittest import mock


class Twist:
  def __init__(self):
    self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


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
    {'OperationState': type('OperationState', (), {'OPERATION_MANUAL': 'manual'})},
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

  def error(self, message):
    self.errors.append(message)

  def info(self, message):
    self.infos.append(message)


class FakeNode:
  def __init__(self):
    self.logger = FakeLogger()

  def create_service(self, *args, **kwargs):
    return (args, kwargs)

  def create_subscription(self, *args, **kwargs):
    return (args, kwargs)

  def get_logger(self):
    return self.logger


class FakeSrpClient:
  def __init__(self):
    self.remote_control_requests = []
    self.oba_requests = []
    self.speed_requests = []

  def set_remote_control(self, enabled):
    self.remote_control_requests.append(enabled)

  def set_remote_control_oba(self, enabled):
    self.oba_requests.append(enabled)

  def set_remote_control_speed(self, linear_x, linear_y, angular_z):
    self.speed_requests.append((linear_x, linear_y, angular_z))


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
    self.assertEqual(srp_client.oba_requests, [True])
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
    self.assertEqual(srp_client.oba_requests, [True])
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
    self.assertEqual(srp_client.oba_requests, [True])
    self.assertEqual(srp_client.speed_requests, [])


if __name__ == '__main__':
  unittest.main()
