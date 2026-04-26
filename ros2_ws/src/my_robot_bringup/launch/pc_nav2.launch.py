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
    nav2_config = f'{bringup_share}/config/nav2/nav2_ackermann.yaml'
    depth_to_scan_config = f'{bringup_share}/config/mapping/depth_to_scan.yaml'
    nav_to_pose_bt = f'{bringup_share}/behavior_trees/navigate_to_pose_ackermann.xml'
    nav_through_poses_bt = (
        f'{bringup_share}/behavior_trees/navigate_through_poses_ackermann.xml'
    )
    rviz_config = f'{rviz_share}/rviz/azemauto_nav2.rviz'

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
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_config,
        description='Nav2 parameter file for the real rover.',
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
        description='Autostart localization and navigation lifecycle nodes.',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz with a Nav2-oriented configuration.',
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
            LaunchConfiguration('params_file'),
            {'yaml_filename': LaunchConfiguration('map_file')},
        ],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    lifecycle_manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'autostart': ParameterValue(
                    LaunchConfiguration('autostart'),
                    value_type=bool,
                ),
                'node_names': ['map_server', 'amcl'],
            },
        ],
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'default_nav_to_pose_bt_xml': nav_to_pose_bt,
                'default_nav_through_poses_bt_xml': nav_through_poses_bt,
            },
        ],
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
        ],
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'autostart': ParameterValue(
                    LaunchConfiguration('autostart'),
                    value_type=bool,
                ),
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    'collision_monitor',
                ],
            },
        ],
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        map_file_arg,
        params_file_arg,
        start_description_arg,
        use_lidar_arg,
        use_camera_arg,
        autostart_arg,
        use_rviz_arg,
        robot_state_publisher_node,
        depth_to_scan_node,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization_node,
        controller_server_node,
        smoother_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        collision_monitor_node,
        lifecycle_manager_navigation_node,
        rviz_node,
    ])
