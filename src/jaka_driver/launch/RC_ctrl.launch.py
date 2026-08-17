import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_left_ip_arg = DeclareLaunchArgument(
        'robot_left_ip',
        default_value='192.168.71.37',
        description='IP address of the left JAKA robot'
    )
    robot_right_ip_arg = DeclareLaunchArgument(
        'robot_right_ip',
        default_value='192.168.71.36',
        description='IP address of the right JAKA robot'
    )
    rc_ctrl_arg = DeclareLaunchArgument(
        'rc_ctrl',
        default_value='true',
        description='Enable or disable RC control'
    )
    left_target_topic_arg = DeclareLaunchArgument(
        'left_target_topic',
        default_value='/rc_ctrl/left_target',
        description='Topic carrying the left handle target pose (PoseStamped)'
    )
    right_target_topic_arg = DeclareLaunchArgument(
        'right_target_topic',
        default_value='/rc_ctrl/right_target',
        description='Topic carrying the right handle target pose (PoseStamped)'
    )
    button_topic_arg = DeclareLaunchArgument(
        'button_topic',
        default_value='/rc_ctrl/button',
        description='Topic carrying the VR button/grip state (sensor_msgs/Joy)'
    )

    # Create a node for the RC control
    rc_ctrl_node = Node(
        package='jaka_driver',
        executable='rc_ctrl_node',
        name='rc_ctrl_node',
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('rc_ctrl')),
        parameters=[{
            'robot_left_ip': LaunchConfiguration('robot_left_ip'),
            'robot_right_ip': LaunchConfiguration('robot_right_ip'),
            'left_target_topic': LaunchConfiguration('left_target_topic'),
            'right_target_topic': LaunchConfiguration('right_target_topic'),
            'button_topic': LaunchConfiguration('button_topic')
        }]
    )

    return LaunchDescription([
        robot_left_ip_arg,
        robot_right_ip_arg,
        rc_ctrl_arg,
        left_target_topic_arg,
        right_target_topic_arg,
        button_topic_arg,
        rc_ctrl_node,
        LogInfo(msg="RC control node launched.")
    ])