from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    command_arg = DeclareLaunchArgument(
        'command', default_value='',
        description='读取眼镜数据的子进程命令（空格分隔，如 "python3 /path/to/read.py"）')
    glasses_frame_arg = DeclareLaunchArgument(
        'glasses_frame', default_value='quest',
        description='眼镜坐标系 frame 名')
    robot_base_frame_arg = DeclareLaunchArgument(
        'robot_base_frame', default_value='world',
        description='机器人基坐标系 frame 名')
    left_hand_key_arg = DeclareLaunchArgument(
        'left_hand_key', default_value='l',
        description='左手柄在 transforms 里的 key')
    right_hand_key_arg = DeclareLaunchArgument(
        'right_hand_key', default_value='r',
        description='右手柄在 transforms 里的 key')
    adb_tag_arg = DeclareLaunchArgument(
        'adb_tag', default_value='wE9ryARX',
        description='logcat 过滤标签')
    apk_activity_arg = DeclareLaunchArgument(
        'apk_activity', default_value='com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity',
        description='要启动的眼镜 APK activity')
    start_apk_arg = DeclareLaunchArgument(
        'start_apk', default_value='true',
        description='启动时是否先 am start 拉起 APK')
    left_topic_arg = DeclareLaunchArgument(
        'left_target_topic', default_value='/rc_ctrl/left_target',
        description='左目标 PoseStamped topic')
    right_topic_arg = DeclareLaunchArgument(
        'right_target_topic', default_value='/rc_ctrl/right_target',
        description='右目标 PoseStamped topic')
    rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='125.0',
        description='发布/变换频率（Hz）')

    quest_reader_node = Node(
        package='quest_vr',
        executable='quest_reader',
        name='quest_reader',
        output='screen',
        parameters=[{
            'command': LaunchConfiguration('command'),
            'glasses_frame': LaunchConfiguration('glasses_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'left_hand_key': LaunchConfiguration('left_hand_key'),
            'right_hand_key': LaunchConfiguration('right_hand_key'),
            'adb_tag': LaunchConfiguration('adb_tag'),
            'apk_activity': LaunchConfiguration('apk_activity'),
            'start_apk': ParameterValue(LaunchConfiguration('start_apk'), value_type=bool),
            'left_target_topic': LaunchConfiguration('left_target_topic'),
            'right_target_topic': LaunchConfiguration('right_target_topic'),
            'publish_rate': ParameterValue(LaunchConfiguration('publish_rate'), value_type=float),
            # 静态 TF：眼镜原点在机器人基坐标系下的位姿（标定后填）
            'tf_translation': [0.0, 0.0, 0.0],
            'tf_rotation': [1.0, 0.0, 0.0, 0.0],
        }],
    )

    return LaunchDescription([
        command_arg,
        glasses_frame_arg,
        robot_base_frame_arg,
        left_hand_key_arg,
        right_hand_key_arg,
        adb_tag_arg,
        apk_activity_arg,
        start_apk_arg,
        left_topic_arg,
        right_topic_arg,
        rate_arg,
        quest_reader_node,
    ])
