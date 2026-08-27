import math

from geometry_msgs.msg import PoseStamped, Quaternion
from sros_sdk_py.main_pb2 import Pose
from transforms3d.euler import euler2quat


def srp_to_ros_unit(n: float) -> float:
  return n / 1000.0


def ros_to_srp_unit(n: float) -> float:
  return n * 1000.0


def to_ros_pose(protobuf_pose: Pose) -> PoseStamped:
  ros_pose = PoseStamped()
  ros_pose.pose.position.x = srp_to_ros_unit(protobuf_pose.x)
  ros_pose.pose.position.y = srp_to_ros_unit(protobuf_pose.y)
  ros_pose.pose.position.z = srp_to_ros_unit(protobuf_pose.z)
  quat = euler2quat(
    srp_to_ros_unit(protobuf_pose.roll),
    srp_to_ros_unit(protobuf_pose.pitch),
    srp_to_ros_unit(protobuf_pose.yaw),  # 注意参数顺序变为 yaw→pitch→roll
  )
  ros_pose.pose.orientation.w = quat[0]
  ros_pose.pose.orientation.x = quat[1]
  ros_pose.pose.orientation.y = quat[2]
  ros_pose.pose.orientation.z = quat[3]
  return ros_pose


def yaw_from_quaternion(q: Quaternion) -> float:
  """Extract yaw radians from a geometry_msgs/Quaternion."""
  siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
  cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(siny_cosp, cosy_cosp)
