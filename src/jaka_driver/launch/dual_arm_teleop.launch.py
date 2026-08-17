import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Forwarded to RC_ctrl.launch.py so the arm IPs can be overridden from here,
    # e.g.  ros2 launch jaka_driver dual_arm_teleop.launch.py robot_left_ip:=192.168.71.36
    robot_left_ip_arg = DeclareLaunchArgument(
        'robot_left_ip', default_value='192.168.71.37',
        description='IP address of the left JAKA robot')
    robot_right_ip_arg = DeclareLaunchArgument(
        'robot_right_ip', default_value='192.168.71.36',
        description='IP address of the right JAKA robot')

    # quest_reader publishes /rc_ctrl/button + /rc_ctrl/{left,right}_target from
    # the VR handles; rc_ctrl_node connects to the arms and servos on those topics.
    # Both are required — this file starts them together.
    quest_reader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('quest_vr'), 'launch', 'quest_reader.launch.py'])
        ])
    )
    rc_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('jaka_driver'), 'launch', 'RC_ctrl.launch.py'])
        ]),
        launch_arguments={
            'robot_left_ip': LaunchConfiguration('robot_left_ip'),
            'robot_right_ip': LaunchConfiguration('robot_right_ip'),
        }.items(),
    )

    return LaunchDescription([
        robot_left_ip_arg,
        robot_right_ip_arg,
        quest_reader_launch,
        rc_ctrl_launch,
    ])
