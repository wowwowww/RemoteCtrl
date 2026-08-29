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
    gripper_force_arg = DeclareLaunchArgument(
        'gripper_force', default_value='20', description='PGI force percentage (20..100)')
    gripper_speed_arg = DeclareLaunchArgument(
        'gripper_speed', default_value='100', description='PGI speed percentage (1..100)')
    gripper_baudrate_arg = DeclareLaunchArgument(
        'gripper_baudrate', default_value='115200',
        description='PGI RS485 baud rate (default 115200)')
    gripper_rs485_channel_arg = DeclareLaunchArgument(
        'gripper_rs485_channel', default_value='1',
        description='JAKA SDK RS485 channel: 0=RS485H/TIO channel 1, 1=RS485L/TIO channel 2')
    gripper_slave_id_arg = DeclareLaunchArgument(
        'gripper_slave_id', default_value='1', description='PGI Modbus slave ID')
    gripper_open_position_arg = DeclareLaunchArgument(
        'gripper_open_position', default_value='1000',
        description='PGI open target in per mille (0..1000)')
    gripper_closed_position_arg = DeclareLaunchArgument(
        'gripper_closed_position', default_value='0',
        description='PGI closed target in per mille (0..1000)')
    gripper_linear_arg = DeclareLaunchArgument(
        'gripper_linear', default_value='true',
        description='Use linear (analog axes) control for gripper by default')
    gripper_trigger_deadband_arg = DeclareLaunchArgument(
        'gripper_trigger_deadband', default_value='0.02',
        description='Minimal change in analog trigger to send a new Grip()')
    gripper_initialize_arg = DeclareLaunchArgument(
        'gripper_initialize', default_value='true',
        description='Initialize PGI grippers during node startup')
    gripper_initialize_command_arg = DeclareLaunchArgument(
        'gripper_initialize_command', default_value='1',
        description='PGI initialization command: 1 or 165 (0xA5)')
    gripper_initialize_delay_arg = DeclareLaunchArgument(
        'gripper_initialize_delay_ms', default_value='2000',
        description='Wait time after PGI initialization command')
    gripper_configure_tio_arg = DeclareLaunchArgument(
        'gripper_configure_tio', default_value='false',
        description='Configure TIO via SDK; false uses the JAKA App saved configuration')
    gripper_enable_tio_power_arg = DeclareLaunchArgument(
        'gripper_enable_tio_power', default_value='true',
        description='Enable JAKA TIO voltage output for the grippers')
    gripper_tio_voltage_arg = DeclareLaunchArgument(
        'gripper_tio_voltage', default_value='0',
        description='JAKA TIO output voltage: 0=24 V, 1=12 V')
    gripper_use_button_arg = DeclareLaunchArgument(
        'gripper_use_button', default_value='false',
        description='Use only Joy.buttons; default also accepts digital trigger fallback')
    gripper_button_index_arg = DeclareLaunchArgument(
        'gripper_button_index', default_value='-1',
        description='Joy.buttons index; -1 selects left LTr=10 or right RTr=11')
    use_rpy_ctrl_arg = DeclareLaunchArgument(
        'use_rpy_ctrl', default_value='false',
        description='true = teleop controls orientation too; false = position only')
    enable_y_limit_arg = DeclareLaunchArgument(
        'enable_y_limit', default_value='true',
        description='true = clamp arm TCP y (left > -110 mm, right < 110 mm)')
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Enable debug logging for rc_ctrl_node')
    use_wifi_arg = DeclareLaunchArgument(
        'use_wifi', default_value='true',
        description='Whether quest_reader auto-switches to wireless adb (falls back to USB)')
    vr_ip_arg = DeclareLaunchArgument(
        'vr_ip', default_value='192.168.1.104',
        description='Quest wireless IP address')
    vr_port_arg = DeclareLaunchArgument(
        'vr_port', default_value='5555',
        description='Quest adb wireless listen port')
    vr_serial_arg = DeclareLaunchArgument(
        'vr_serial', default_value='2G97C5ZH5Q0279',
        description='Quest adb serial (for multi-device)')
    vr_mac_arg = DeclareLaunchArgument(
        'vr_mac', default_value='78-C4-FA-CC-88-23',
        description='Quest wireless MAC, used to auto-resolve its wireless IP (empty disables)')
    vr_subnet_arg = DeclareLaunchArgument(
        'vr_subnet', default_value='192.168.1.0/24',
        description='Subnet to ping-probe when resolving IP from MAC')

    # quest_reader publishes /rc_ctrl/button + /rc_ctrl/{left,right}_target from
    # the VR handles; rc_ctrl_node connects to the arms and servos on those topics.
    # Both are required — this file starts them together.
    quest_reader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('quest_vr'), 'launch', 'quest_reader.launch.py'])
        ]),
        launch_arguments={
            'use_wifi': LaunchConfiguration('use_wifi'),
            'vr_ip': LaunchConfiguration('vr_ip'),
            'vr_port': LaunchConfiguration('vr_port'),
            'vr_serial': LaunchConfiguration('vr_serial'),
            'vr_mac': LaunchConfiguration('vr_mac'),
            'vr_subnet': LaunchConfiguration('vr_subnet'),
        }.items(),
    )
    rc_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('jaka_driver'), 'launch', 'RC_ctrl.launch.py'])
        ]),
        launch_arguments={
            'robot_left_ip': LaunchConfiguration('robot_left_ip'),
            'robot_right_ip': LaunchConfiguration('robot_right_ip'),
            'gripper_force': LaunchConfiguration('gripper_force'),
            'gripper_speed': LaunchConfiguration('gripper_speed'),
            'gripper_baudrate': LaunchConfiguration('gripper_baudrate'),
            'gripper_rs485_channel': LaunchConfiguration('gripper_rs485_channel'),
            'gripper_slave_id': LaunchConfiguration('gripper_slave_id'),
            'gripper_open_position': LaunchConfiguration('gripper_open_position'),
            'gripper_closed_position': LaunchConfiguration('gripper_closed_position'),
            'gripper_linear': LaunchConfiguration('gripper_linear'),
            'gripper_trigger_deadband': LaunchConfiguration('gripper_trigger_deadband'),
            'gripper_initialize': LaunchConfiguration('gripper_initialize'),
            'gripper_initialize_command': LaunchConfiguration('gripper_initialize_command'),
            'gripper_initialize_delay_ms': LaunchConfiguration('gripper_initialize_delay_ms'),
            'gripper_configure_tio': LaunchConfiguration('gripper_configure_tio'),
            'gripper_enable_tio_power': LaunchConfiguration('gripper_enable_tio_power'),
            'gripper_tio_voltage': LaunchConfiguration('gripper_tio_voltage'),
            'gripper_use_button': LaunchConfiguration('gripper_use_button'),
            'gripper_button_index': LaunchConfiguration('gripper_button_index'),
            'use_rpy_ctrl': LaunchConfiguration('use_rpy_ctrl'),
            'enable_y_limit': LaunchConfiguration('enable_y_limit'),
            'debug': LaunchConfiguration('debug'),
        }.items(),
    )

    return LaunchDescription([
        robot_left_ip_arg,
        robot_right_ip_arg,
        gripper_force_arg,
        gripper_speed_arg,
        gripper_baudrate_arg,
        gripper_rs485_channel_arg,
        gripper_slave_id_arg,
        gripper_open_position_arg,
        gripper_closed_position_arg,
        gripper_linear_arg,
        gripper_trigger_deadband_arg,
        gripper_initialize_arg,
        gripper_initialize_command_arg,
        gripper_initialize_delay_arg,
        gripper_configure_tio_arg,
        gripper_enable_tio_power_arg,
        gripper_tio_voltage_arg,
        gripper_use_button_arg,
        gripper_button_index_arg,
        use_rpy_ctrl_arg,
        enable_y_limit_arg,
        debug_arg,
        use_wifi_arg,
        vr_ip_arg,
        vr_port_arg,
        vr_serial_arg,
        vr_mac_arg,
        vr_subnet_arg,
        quest_reader_launch,
        rc_ctrl_launch,
    ])
