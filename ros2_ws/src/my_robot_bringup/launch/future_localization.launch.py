from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('my_robot_bringup')

    ekf_config = (
        f'{bringup_share}/config/future/robot_localization/ekf_template.yaml'
    )
    navsat_transform_config = (
        f'{bringup_share}/config/future/robot_localization/'
        'navsat_transform_template.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_transform_config],
        remappings=[
            ('imu/data', '/sensors/imu/data'),
            ('gps/fix', '/sensors/gps/fix'),
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    return LaunchDescription([
        ekf_node,
        navsat_transform_node,
    ])
