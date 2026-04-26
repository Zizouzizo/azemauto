from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('my_robot_bringup')
    description_share = get_package_share_directory('my_robot_description')

    xacro_file = f'{description_share}/urdf/azemauto.urdf.xacro'
    mavros_config = f'{bringup_share}/config/mavros/mavros_params.yaml'
    realsense_config = f'{bringup_share}/config/realsense/realsense_d435.yaml'
    lidar_config = f'{bringup_share}/config/lidar/rplidar_a2.yaml'
    auto_arm_config = f'{bringup_share}/config/safety/auto_arm_disarm.yaml'

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyAMA0:115200',
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

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            mavros_config,
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'tgt_system': ParameterValue(
                    LaunchConfiguration('tgt_system'),
                    value_type=int,
                ),
                'tgt_component': ParameterValue(
                    LaunchConfiguration('tgt_component'),
                    value_type=int,
                ),
            },
        ],
    )

    bridge_node = Node(
        package='my_robot_bridge',
        executable='mavros_bridge',
        name='mavros_bridge',
        output='screen',
    )

    auto_arm_disarm_node = Node(
        condition=IfCondition(LaunchConfiguration('auto_arm')),
        package='my_robot_bridge',
        executable='auto_arm_disarm',
        name='auto_arm_disarm',
        output='screen',
        parameters=[
            auto_arm_config,
            {
                'auto_arm_enabled': ParameterValue(
                    LaunchConfiguration('auto_arm'),
                    value_type=bool,
                ),
                'auto_disarm_enabled': ParameterValue(
                    LaunchConfiguration('auto_disarm'),
                    value_type=bool,
                ),
            },
        ],
    )

    lidar_node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[
            lidar_config,
            {
                'serial_port': LaunchConfiguration('lidar_serial_port'),
                'serial_baudrate': ParameterValue(
                    LaunchConfiguration('lidar_serial_baudrate'),
                    value_type=int,
                ),
                'frame_id': LaunchConfiguration('lidar_frame_id'),
                'scan_mode': LaunchConfiguration('lidar_scan_mode'),
                'inverted': ParameterValue(
                    LaunchConfiguration('lidar_inverted'),
                    value_type=bool,
                ),
                'angle_compensate': ParameterValue(
                    LaunchConfiguration('lidar_angle_compensate'),
                    value_type=bool,
                ),
            },
        ],
    )

    realsense_node = Node(
        condition=IfCondition(LaunchConfiguration('use_camera')),
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='sensors',
        name='camera',
        output='screen',
        parameters=[
            realsense_config,
            {
                'rgb_camera.profile': LaunchConfiguration('camera_profile'),
                'depth_module.profile': LaunchConfiguration('camera_profile'),
                'pointcloud.enable': ParameterValue(
                    LaunchConfiguration('enable_pointcloud'),
                    value_type=bool,
                ),
                'align_depth.enable': ParameterValue(
                    LaunchConfiguration('align_depth'),
                    value_type=bool,
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
        robot_state_publisher_node,
        mavros_node,
        bridge_node,
        auto_arm_disarm_node,
        lidar_node,
        realsense_node,
    ])
