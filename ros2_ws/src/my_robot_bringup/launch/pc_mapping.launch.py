from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('my_robot_bringup')
    description_share = get_package_share_directory('my_robot_description')
    rviz_share = get_package_share_directory('my_robot_rviz')

    xacro_file = f'{description_share}/urdf/azemauto.urdf.xacro'
    depth_to_scan_config = f'{bringup_share}/config/mapping/depth_to_scan.yaml'
    slam_config = f'{bringup_share}/config/mapping/slam_toolbox_mapping.yaml'
    teleop_joy_config = f'{bringup_share}/config/teleop/xbox_teleop.yaml'
    rviz_config = f'{rviz_share}/rviz/azemauto_mapping.rviz'

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    start_description_arg = DeclareLaunchArgument(
        'start_description',
        default_value='false',
        description='Start robot_state_publisher locally on the PC.',
    )
    use_joy_arg = DeclareLaunchArgument(
        'use_joy',
        default_value='false',
        description='Start joystick teleoperation nodes on the PC.',
    )
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Assume /scan is provided by the lidar running on the Pi.',
    )
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Keep RealSense visualization topics available in RViz.',
    )
    use_static_odom_tf_arg = DeclareLaunchArgument(
        'use_static_odom_tf',
        default_value='true',
        description='Publish a temporary odom->base_link transform when no odometry is available.',
    )
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='Joystick device id for joy_node.',
    )

    robot_state_publisher_node = Node(
        condition=IfCondition(LaunchConfiguration('start_description')),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    depth_to_scan_node = Node(
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('use_lidar'),
                '" == "false" and "',
                LaunchConfiguration('use_camera'),
                '" == "true"',
            ])
        ),
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[depth_to_scan_config],
        remappings=[
            ('depth', '/sensors/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/sensors/camera/depth/camera_info'),
            ('scan', '/scan/raw'),
        ],
    )

    scan_timestamp_bridge_node = Node(
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('use_lidar'),
                '" == "false" and "',
                LaunchConfiguration('use_camera'),
                '" == "true"',
            ])
        ),
        package='my_robot_bridge',
        executable='scan_timestamp_bridge',
        name='scan_timestamp_bridge',
        output='screen',
        parameters=[
            {
                'input_scan_topic': '/scan/raw',
                'output_scan_topic': '/scan',
                'output_frame_id': 'laser_link',
            },
        ],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': False}],
    )

    joy_node = Node(
        condition=IfCondition(LaunchConfiguration('use_joy')),
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {
                'device_id': ParameterValue(
                    LaunchConfiguration('joy_dev'),
                    value_type=int,
                ),
                'deadzone': 0.2,
                'autorepeat_rate': 20.0,
            },
        ],
    )

    teleop_twist_joy_node = Node(
        condition=IfCondition(LaunchConfiguration('use_joy')),
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[teleop_joy_config, {'publish_stamped_twist': False}],
        remappings=[('/cmd_vel', '/cmd_vel')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    static_odom_tf_node = Node(
        condition=IfCondition(LaunchConfiguration('use_static_odom_tf')),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fallback_odom_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )

    return LaunchDescription([
        start_description_arg,
        use_joy_arg,
        use_lidar_arg,
        use_camera_arg,
        use_static_odom_tf_arg,
        joy_dev_arg,
        robot_state_publisher_node,
        depth_to_scan_node,
        scan_timestamp_bridge_node,
        slam_toolbox_node,
        joy_node,
        teleop_twist_joy_node,
        rviz_node,
        static_odom_tf_node,
    ])
