from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('my_robot_bringup')
    description_share = get_package_share_directory('my_robot_description')
    rviz_share = get_package_share_directory('my_robot_rviz')

    xacro_file = f'{description_share}/urdf/azemauto.urdf.xacro'
    localization_config = f'{bringup_share}/config/localization/amcl_localization.yaml'
    depth_to_scan_config = f'{bringup_share}/config/mapping/depth_to_scan.yaml'
    rviz_config = f'{rviz_share}/rviz/azemauto_localization.rviz'

    default_map_file = PathJoinSubstitution(
        [EnvironmentVariable('HOME'), 'azemauto_maps', 'azemauto_site_01.yaml']
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file,
        description='Absolute path to the occupancy grid map yaml file.',
    )
    start_description_arg = DeclareLaunchArgument(
        'start_description',
        default_value='false',
        description='Start robot_state_publisher locally on the PC.',
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
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Autostart lifecycle nodes for localization.',
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
            ('scan', '/scan'),
        ],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            localization_config,
            {
                'yaml_filename': LaunchConfiguration('map_file'),
            },
        ],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[localization_config],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            localization_config,
            {
                'autostart': ParameterValue(
                    LaunchConfiguration('autostart'),
                    value_type=bool,
                ),
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        map_file_arg,
        start_description_arg,
        use_lidar_arg,
        use_camera_arg,
        autostart_arg,
        robot_state_publisher_node,
        depth_to_scan_node,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz_node,
    ])
