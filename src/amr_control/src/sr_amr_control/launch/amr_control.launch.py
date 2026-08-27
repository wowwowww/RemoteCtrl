from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  declared_args = [
    DeclareLaunchArgument(
      'connect_ip',default_value='192.168.71.50'
    ),
    DeclareLaunchArgument('connect_username', default_value=''),
    DeclareLaunchArgument('connect_passwd', default_value=''),
    DeclareLaunchArgument('frame_id', default_value='base_link'),
    DeclareLaunchArgument('parent_frame_id', default_value='map'),
    DeclareLaunchArgument('odom_frame_id', default_value='odom'),
    DeclareLaunchArgument('odom_topic', default_value='odom'),
    DeclareLaunchArgument('odom_use_integration', default_value='true'),
    DeclareLaunchArgument('lidar_points_frame_id', default_value='map'),
    DeclareLaunchArgument(
      'lidar_front_frame_id', default_value='right_front_laser_link'
    ),
    DeclareLaunchArgument(
      'lidar_rear_frame_id', default_value='left_behind_laser_link'
    ),
    DeclareLaunchArgument('lidar', default_value='false'),
  ]

  connect_ip = LaunchConfiguration('connect_ip')
  connect_username = LaunchConfiguration('connect_username')
  connect_passwd = LaunchConfiguration('connect_passwd')
  frame_id = LaunchConfiguration('frame_id')
  parent_frame_id = LaunchConfiguration('parent_frame_id')
  odom_frame_id = LaunchConfiguration('odom_frame_id')
  odom_topic = LaunchConfiguration('odom_topic')
  odom_use_integration = ParameterValue(
    LaunchConfiguration('odom_use_integration'), value_type=bool
  )
  lidar_points_frame_id = LaunchConfiguration('lidar_points_frame_id')
  lidar_front_frame_id = LaunchConfiguration('lidar_front_frame_id')
  lidar_rear_frame_id = LaunchConfiguration('lidar_rear_frame_id')
  lidar = ParameterValue(LaunchConfiguration('lidar'), value_type=bool)

  control_node = Node(
    package='sr_amr_control',
    executable='control_node',
    namespace='sr_amr_control',
    output='screen',
    parameters=[
      {
        'connect_ip': connect_ip,
        'connect_username': connect_username,
        'connect_passwd': connect_passwd,
        'frame_id': frame_id,
        'parent_frame_id': parent_frame_id,
        'odom_frame_id': odom_frame_id,
        'odom_topic': odom_topic,
        'odom_use_integration': odom_use_integration,
        'lidar_points_frame_id': lidar_points_frame_id,
        'lidar_front_frame_id': lidar_front_frame_id,
        'lidar_rear_frame_id': lidar_rear_frame_id,
        'lidar': lidar,
      }
    ],
  )

  return LaunchDescription(declared_args + [control_node])
