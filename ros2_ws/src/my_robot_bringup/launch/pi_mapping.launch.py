from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('my_robot_bringup')

    pi_bringup_launch = f'{bringup_share}/launch/pi_bringup.launch.py'
    cmd_vel_bridge_config = f'{bringup_share}/config/teleop/cmd_vel_to_mavros.yaml'
    manual_control_bridge_config = (
        f'{bringup_share}/config/teleop/cmd_vel_to_manual_control.yaml'
    )
    rc_override_bridge_config = (
        f'{bringup_share}/config/teleop/cmd_vel_to_rc_override.yaml'
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyACM0:115200',
        description='ArduPilot FCU URL for MAVROS.',
    )
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='Optional GCS forwarding URL for MAVROS.',
    )
    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='MAVLink target system id.',
    )
    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='MAVLink target component id.',
    )
    camera_profile_arg = DeclareLaunchArgument(
        'camera_profile',
        default_value='640x480x15',
        description='RealSense color and depth profile.',
    )
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Start the RealSense D435 on the Raspberry Pi.',
    )
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Start the RPLIDAR node on the Raspberry Pi.',
    )
    enable_pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='false',
        description='Enable RealSense point cloud publication.',
    )
    align_depth_arg = DeclareLaunchArgument(
        'align_depth',
        default_value='false',
        description='Align depth to the color stream.',
    )
    lidar_serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/ttyUSB0',
        description='USB serial port used by the RPLIDAR.',
    )
    lidar_serial_baudrate_arg = DeclareLaunchArgument(
        'lidar_serial_baudrate',
        default_value='115200',
        description='Serial baudrate used by the RPLIDAR.',
    )
    lidar_frame_id_arg = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='laser_link',
        description='Frame published in the LaserScan header.',
    )
    lidar_scan_mode_arg = DeclareLaunchArgument(
        'lidar_scan_mode',
        default_value='Standard',
        description='RPLIDAR scan mode exposed by the driver.',
    )
    lidar_inverted_arg = DeclareLaunchArgument(
        'lidar_inverted',
        default_value='false',
        description='Invert the lidar rotation direction if required.',
    )
    lidar_angle_compensate_arg = DeclareLaunchArgument(
        'lidar_angle_compensate',
        default_value='true',
        description='Enable angle compensation in the RPLIDAR driver.',
    )
    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm',
        default_value='false',
        description='Enable the automatic GUIDED + arm safety supervisor.',
    )
    auto_disarm_arg = DeclareLaunchArgument(
        'auto_disarm',
        default_value='false',
        description='Enable automatic disarm on inactivity or sensor loss.',
    )
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='mavros_cmd_vel',
        description='Command bridge mode: mavros_cmd_vel, rc_override or manual_control.',
    )
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='1.0',
        description='Maximum linear speed accepted from /cmd_vel in m/s.',
    )
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='0.8',
        description='Maximum yaw rate accepted from /cmd_vel in rad/s.',
    )
    command_timeout_arg = DeclareLaunchArgument(
        'command_timeout',
        default_value='0.35',
        description='Timeout before forcing zero command in seconds.',
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='15.0',
        description='Publishing rate toward MAVROS in Hz.',
    )

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pi_bringup_launch),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
            'tgt_component': LaunchConfiguration('tgt_component'),
            'camera_profile': LaunchConfiguration('camera_profile'),
            'use_camera': LaunchConfiguration('use_camera'),
            'use_lidar': LaunchConfiguration('use_lidar'),
            'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
            'align_depth': LaunchConfiguration('align_depth'),
            'lidar_serial_port': LaunchConfiguration('lidar_serial_port'),
            'lidar_serial_baudrate': LaunchConfiguration('lidar_serial_baudrate'),
            'lidar_frame_id': LaunchConfiguration('lidar_frame_id'),
            'lidar_scan_mode': LaunchConfiguration('lidar_scan_mode'),
            'lidar_inverted': LaunchConfiguration('lidar_inverted'),
            'lidar_angle_compensate': LaunchConfiguration('lidar_angle_compensate'),
            'auto_arm': LaunchConfiguration('auto_arm'),
            'auto_disarm': LaunchConfiguration('auto_disarm'),
        }.items(),
    )

    cmd_vel_bridge_node = Node(
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('control_mode'), '" == "mavros_cmd_vel"',
            ])
        ),
        package='my_robot_bridge',
        executable='cmd_vel_to_mavros',
        name='cmd_vel_to_mavros',
        output='screen',
        parameters=[
            cmd_vel_bridge_config,
            {
                'max_linear_speed': ParameterValue(
                    LaunchConfiguration('max_linear_speed'),
                    value_type=float,
                ),
                'max_angular_speed': ParameterValue(
                    LaunchConfiguration('max_angular_speed'),
                    value_type=float,
                ),
                'command_timeout': ParameterValue(
                    LaunchConfiguration('command_timeout'),
                    value_type=float,
                ),
                'publish_rate': ParameterValue(
                    LaunchConfiguration('publish_rate'),
                    value_type=float,
                ),
            },
        ],
    )

    rc_override_bridge_node = Node(
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('control_mode'), '" == "rc_override"',
            ])
        ),
        package='my_robot_bridge',
        executable='cmd_vel_to_rc_override',
        name='cmd_vel_to_rc_override',
        output='screen',
        parameters=[
            rc_override_bridge_config,
            {
                'max_linear_speed': ParameterValue(
                    LaunchConfiguration('max_linear_speed'),
                    value_type=float,
                ),
                'max_angular_speed': ParameterValue(
                    LaunchConfiguration('max_angular_speed'),
                    value_type=float,
                ),
                'timeout_sec': ParameterValue(
                    LaunchConfiguration('command_timeout'),
                    value_type=float,
                ),
                'publish_rate': ParameterValue(
                    LaunchConfiguration('publish_rate'),
                    value_type=float,
                ),
            },
        ],
    )

    manual_control_bridge_node = Node(
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('control_mode'), '" == "manual_control"',
            ])
        ),
        package='my_robot_bridge',
        executable='cmd_vel_to_manual_control',
        name='cmd_vel_to_manual_control',
        output='screen',
        parameters=[
            manual_control_bridge_config,
            {
                'max_linear_speed': ParameterValue(
                    LaunchConfiguration('max_linear_speed'),
                    value_type=float,
                ),
                'max_angular_speed': ParameterValue(
                    LaunchConfiguration('max_angular_speed'),
                    value_type=float,
                ),
                'timeout_sec': ParameterValue(
                    LaunchConfiguration('command_timeout'),
                    value_type=float,
                ),
                'publish_rate': ParameterValue(
                    LaunchConfiguration('publish_rate'),
                    value_type=float,
                ),
            },
        ],
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        camera_profile_arg,
        use_camera_arg,
        use_lidar_arg,
        enable_pointcloud_arg,
        align_depth_arg,
        lidar_serial_port_arg,
        lidar_serial_baudrate_arg,
        lidar_frame_id_arg,
        lidar_scan_mode_arg,
        lidar_inverted_arg,
        lidar_angle_compensate_arg,
        auto_arm_arg,
        auto_disarm_arg,
        control_mode_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        command_timeout_arg,
        publish_rate_arg,
        base_bringup,
        cmd_vel_bridge_node,
        rc_override_bridge_node,
        manual_control_bridge_node,
    ])
