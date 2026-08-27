import importlib
import math
import sys
import types
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
  sys.path.insert(0, str(PACKAGE_ROOT))


class Point:
  def __init__(self):
    self.x = 0.0
    self.y = 0.0
    self.z = 0.0


class Quaternion:
  def __init__(self):
    self.x = 0.0
    self.y = 0.0
    self.z = 0.0
    self.w = 1.0


class Pose:
  def __init__(self):
    self.position = Point()
    self.orientation = Quaternion()


class PoseStamped:
  def __init__(self):
    self.pose = Pose()


geometry_msgs = types.ModuleType('geometry_msgs')
geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
geometry_msgs_msg.PoseStamped = PoseStamped
geometry_msgs_msg.Quaternion = Quaternion

sros_sdk_py = types.ModuleType('sros_sdk_py')
main_pb2 = types.ModuleType('sros_sdk_py.main_pb2')
main_pb2.Pose = object

transforms3d = types.ModuleType('transforms3d')
transforms3d_euler = types.ModuleType('transforms3d.euler')


def euler2quat(roll, pitch, yaw):
  cy = math.cos(yaw * 0.5)
  sy = math.sin(yaw * 0.5)
  cp = math.cos(pitch * 0.5)
  sp = math.sin(pitch * 0.5)
  cr = math.cos(roll * 0.5)
  sr = math.sin(roll * 0.5)
  return (
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  )


transforms3d_euler.euler2quat = euler2quat

sys.modules.setdefault('geometry_msgs', geometry_msgs)
sys.modules.setdefault('geometry_msgs.msg', geometry_msgs_msg)
sys.modules.setdefault('sros_sdk_py', sros_sdk_py)
sys.modules.setdefault('sros_sdk_py.main_pb2', main_pb2)
sys.modules.setdefault('transforms3d', transforms3d)
sys.modules.setdefault('transforms3d.euler', transforms3d_euler)

to_ros_pose = importlib.import_module('sr_amr_control.utils.ros_bridge').to_ros_pose


class TestToROSPose(unittest.TestCase):
  def test_to_ros_pose(self):
    # Create a mock protobuf pose
    protobuf_pose = type(
      'Pose',
      (object,),
      {'x': 1000, 'y': 2000, 'z': 3000, 'yaw': 1571, 'pitch': 0, 'roll': 0},
    )()

    # Call the function
    ros_pose = to_ros_pose(protobuf_pose)

    # Assert the values
    self.assertEqual(ros_pose.pose.position.x, 1.0)
    self.assertEqual(ros_pose.pose.position.y, 2.0)
    self.assertEqual(ros_pose.pose.position.z, 3.0)
    self.assertAlmostEqual(ros_pose.pose.orientation.x, 0.0)
    self.assertAlmostEqual(ros_pose.pose.orientation.y, 0.0)
    self.assertAlmostEqual(ros_pose.pose.orientation.z, 0.707, places=3)
    self.assertAlmostEqual(ros_pose.pose.orientation.w, 0.707, places=3)


if __name__ == '__main__':
  unittest.main()
