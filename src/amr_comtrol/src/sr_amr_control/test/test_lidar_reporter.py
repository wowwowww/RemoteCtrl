import importlib
import math
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
  sys.path.insert(0, str(PACKAGE_ROOT))


class Header:
  def __init__(self):
    self.stamp = None
    self.frame_id = ''


class LaserScan:
  def __init__(self):
    self.header = Header()
    self.angle_min = 0.0
    self.angle_max = 0.0
    self.angle_increment = 0.0
    self.time_increment = 0.0
    self.scan_time = 0.0
    self.range_min = 0.0
    self.range_max = 0.0
    self.ranges = []
    self.intensities = []


class Point:
  def __init__(self):
    self.x = 0.0
    self.y = 0.0
    self.z = 0.0


class PoseStamped:
  pass


class Quaternion:
  pass


class Pose:
  pass


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


class FakePublisher:
  def __init__(self):
    self.published = []

  def publish(self, msg):
    self.published.append(msg)


class FakeClock:
  def now(self):
    return self

  def to_msg(self):
    return 'stamp'


class FakeLogger:
  def __init__(self):
    self.errors = []
    self.infos = []
    self.debugs = []

  def error(self, message):
    self.errors.append(message)

  def info(self, message):
    self.infos.append(message)

  def debug(self, message):
    self.debugs.append(message)


class FakeNode:
  def __init__(self):
    self.publishers = {}
    self.publisher_specs = []
    self.services = []
    self.logger = FakeLogger()

  def create_publisher(self, msg_type, topic, qos):
    publisher = FakePublisher()
    self.publishers[topic] = publisher
    self.publisher_specs.append((msg_type, topic, qos))
    return publisher

  def create_service(self, srv_type, name, callback, **kwargs):
    self.services.append((srv_type, name, callback, kwargs))
    return callback

  def get_clock(self):
    return FakeClock()

  def get_logger(self):
    return self.logger


class FakeProtobuf:
  def __init__(self):
    self.commands = []
    self._laser_point_callback = None
    self.original_handler_calls = 0

  def _send_command_msg(self, seq, cmd):
    self.commands.append((seq, cmd))

  def _handleRecvResponseMsg(self, msg):
    self.original_handler_calls += 1


class FakeSrp:
  def __init__(self):
    self._protobuf = FakeProtobuf()
    self.laser_callback = None

  def set_laser_point_callback(self, callback):
    self.laser_callback = callback
    self._protobuf._laser_point_callback = callback

  def _run_sync_threadsafe(self, fun, *args):
    return fun(1, *args)


class FakeSrpClient:
  def __init__(self):
    self._srp = FakeSrp()

  def set_laser_point_callback(self, callback):
    self._srp.set_laser_point_callback(callback)

  def set_laser_point_upload(self, enabled: bool):
    command = 42 if enabled else 43
    return self._srp._run_sync_threadsafe(
      self._srp._protobuf._send_command_msg, command
    )


class FakeTime:
  def __init__(self, *args, **kwargs):
    pass


class FakeBuffer:
  def can_transform(self, *args, **kwargs):
    return True

  def lookup_transform(self, *args, **kwargs):
    raise RuntimeError('not needed in this test')


class FakeTransformListener:
  def __init__(self, *args, **kwargs):
    pass


def _install_ros_stubs():
  geometry_msgs = types.ModuleType('geometry_msgs')
  geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
  geometry_msgs_msg.Point = Point
  geometry_msgs_msg.PoseStamped = PoseStamped
  geometry_msgs_msg.Quaternion = Quaternion

  rclpy = types.ModuleType('rclpy')
  rclpy.ok = lambda: True
  callback_groups = types.ModuleType('rclpy.callback_groups')
  callback_groups.MutuallyExclusiveCallbackGroup = type(
    'MutuallyExclusiveCallbackGroup',
    (),
    {'__init__': lambda self, *args, **kwargs: None},
  )
  node_module = types.ModuleType('rclpy.node')
  node_module.Node = object
  publisher_module = types.ModuleType('rclpy.publisher')
  publisher_module.Publisher = object
  time_module = types.ModuleType('rclpy.time')
  time_module.Time = FakeTime

  sros_sdk_py = types.ModuleType('sros_sdk_py')
  sros_sdk_py.SrpClient = object
  main_pb2 = types.ModuleType('sros_sdk_py.main_pb2')
  main_pb2.CMD_ENABLE_AUTO_UPLOAD_LASER_POINT = 42
  main_pb2.CMD_DISABLE_AUTO_UPLOAD_LASER_POINT = 43
  main_pb2.ResponseResult = types.SimpleNamespace(RESPONSE_OK=2, RESPONSE_PROCESSING=1)
  main_pb2.Response = types.SimpleNamespace(RESPONSE_LASER_POINTS=6)
  main_pb2.Pose = Pose
  srp = types.ModuleType('sros_sdk_py.srp')
  srp.RequestFailedError = RequestFailedError

  transforms3d = types.ModuleType('transforms3d')
  transforms3d_euler = types.ModuleType('transforms3d.euler')
  transforms3d_euler.euler2quat = lambda *args: (1.0, 0.0, 0.0, 0.0)

  std_srvs = types.ModuleType('std_srvs')
  std_srvs_srv = types.ModuleType('std_srvs.srv')
  std_srvs_srv.SetBool = SetBool

  sensor_msgs = types.ModuleType('sensor_msgs')
  sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
  sensor_msgs_msg.LaserScan = LaserScan

  tf2_ros = types.ModuleType('tf2_ros')
  tf2_ros.Buffer = FakeBuffer
  tf2_ros.TransformListener = FakeTransformListener

  return {
    'geometry_msgs': geometry_msgs,
    'geometry_msgs.msg': geometry_msgs_msg,
    'rclpy': rclpy,
    'rclpy.callback_groups': callback_groups,
    'rclpy.node': node_module,
    'rclpy.publisher': publisher_module,
    'rclpy.time': time_module,
    'sros_sdk_py': sros_sdk_py,
    'sros_sdk_py.main_pb2': main_pb2,
    'sros_sdk_py.srp': srp,
    'transforms3d': transforms3d,
    'transforms3d.euler': transforms3d_euler,
    'std_srvs': std_srvs,
    'std_srvs.srv': std_srvs_srv,
    'sensor_msgs': sensor_msgs,
    'sensor_msgs.msg': sensor_msgs_msg,
    'tf2_ros': tf2_ros,
  }


def _load_lidar_reporter():
  package = importlib.import_module('sr_amr_control')
  original_module = sys.modules.pop('sr_amr_control.lidar_reporter', None)
  had_module = original_module is not None
  original_attr = getattr(package, 'lidar_reporter', None)
  had_attr = hasattr(package, 'lidar_reporter')

  with mock.patch.dict(sys.modules, _install_ros_stubs()):
    try:
      from sr_amr_control.lidar_reporter import LidarReporter
    finally:
      if had_attr:
        package.lidar_reporter = original_attr
      elif hasattr(package, 'lidar_reporter'):
        delattr(package, 'lidar_reporter')
      if had_module:
        sys.modules['sr_amr_control.lidar_reporter'] = original_module

  return LidarReporter


def make_sensor_points(name, uuid, xs, ys, zs=None, reliabilitys=None):
  return types.SimpleNamespace(
    sensor_name=name,
    sensor_uuid=uuid,
    xs=xs,
    ys=ys,
    zs=zs or [],
    reliabilitys=reliabilitys or [],
  )


def make_laser_points():
  return types.SimpleNamespace(
    xs=[],
    ys=[],
    reliabilitys=[7],
    xs1=[],
    ys1=[],
    reliabilitys1=[8],
    loc_laser_points=[
      make_sensor_points('front_lidar', 111, [1000, 0], [0, 2000], reliabilitys=[9, 6]),
      make_sensor_points('rear_lidar', 112, [-1000], [0], reliabilitys=[10]),
    ],
    oba_laser_points=[],
    ext_laser_points=[],
    ext_laser_3d_points=[],
  )


class TestLidarReporter(unittest.TestCase):
  def test_disabled_default_does_not_enable_sdk_or_publish_points(self):
    lidar_reporter = _load_lidar_reporter()
    node = FakeNode()
    client = FakeSrpClient()

    reporter = lidar_reporter(
      node,
      client,
      'base_link',
      front_frame_id='base_link',
      rear_frame_id='base_link',
      enabled=False,
    )

    self.assertEqual(
      [(spec[1], spec[2]) for spec in node.publisher_specs],
      [('front/scan', 10), ('rear/scan', 10)],
    )
    self.assertEqual(node.services[0][1], 'lidar_enabled')
    self.assertEqual(client._srp._protobuf.commands, [])

    client._srp.laser_callback(make_laser_points())

    self.assertFalse(reporter.enabled)
    self.assertEqual(node.publishers['front/scan'].published, [])
    self.assertEqual(node.publishers['rear/scan'].published, [])

  def test_toggle_enable_publishes_front_and_rear_laser_scans(self):
    lidar_reporter = _load_lidar_reporter()
    node = FakeNode()
    client = FakeSrpClient()
    reporter = lidar_reporter(
      node,
      client,
      'base_link',
      front_frame_id='base_link',
      rear_frame_id='base_link',
      enabled=False,
    )

    response = reporter._handle_lidar_enabled(
      SetBool.Request(data=True), SetBool.Response()
    )
    client._srp.laser_callback(make_laser_points())

    self.assertTrue(response.success)
    self.assertTrue(reporter.enabled)
    self.assertEqual(client._srp._protobuf.commands, [(1, 42)])

    front_scan = node.publishers['front/scan'].published[0]
    rear_scan = node.publishers['rear/scan'].published[0]
    self.assertEqual(front_scan.header.stamp, 'stamp')
    self.assertEqual(front_scan.header.frame_id, 'base_link')
    self.assertEqual(rear_scan.header.stamp, 'stamp')
    self.assertEqual(rear_scan.header.frame_id, 'base_link')
    self.assertEqual(front_scan.angle_min, -math.pi)
    self.assertEqual(front_scan.angle_max, math.pi)
    self.assertAlmostEqual(front_scan.angle_increment, math.radians(1.0))
    self.assertEqual(front_scan.range_min, 0.0)
    self.assertEqual(front_scan.range_max, 100.0)

    zero_degree_index = int(
      round((0.0 - front_scan.angle_min) / front_scan.angle_increment)
    )
    ninety_degree_index = int(
      round((math.pi / 2 - front_scan.angle_min) / front_scan.angle_increment)
    )
    rear_index = int(round((math.pi - rear_scan.angle_min) / rear_scan.angle_increment))
    self.assertEqual(front_scan.ranges[zero_degree_index], 1.0)
    self.assertEqual(front_scan.intensities[zero_degree_index], 9.0)
    self.assertEqual(front_scan.ranges[ninety_degree_index], 2.0)
    self.assertEqual(front_scan.intensities[ninety_degree_index], 6.0)
    self.assertEqual(rear_scan.ranges[rear_index], 1.0)
    self.assertEqual(rear_scan.intensities[rear_index], 10.0)

  def test_toggle_disable_stops_publishing_and_sends_disable_command(self):
    lidar_reporter = _load_lidar_reporter()
    node = FakeNode()
    client = FakeSrpClient()
    reporter = lidar_reporter(
      node,
      client,
      'base_link',
      front_frame_id='base_link',
      rear_frame_id='base_link',
      enabled=True,
    )

    response = reporter._handle_lidar_enabled(
      SetBool.Request(data=False), SetBool.Response()
    )
    client._srp.laser_callback(make_laser_points())

    self.assertTrue(response.success)
    self.assertFalse(reporter.enabled)
    self.assertEqual(client._srp._protobuf.commands, [(1, 42), (1, 43)])
    self.assertEqual(node.publishers['front/scan'].published, [])
    self.assertEqual(node.publishers['rear/scan'].published, [])


if __name__ == '__main__':
  unittest.main()
