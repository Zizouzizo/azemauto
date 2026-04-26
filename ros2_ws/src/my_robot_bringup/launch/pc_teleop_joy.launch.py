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
    rviz_share = get_package_share_directory('my_robot_rviz')

    xacro_file = f'{description_share}/urdf/azemauto.urdf.xacro'
    teleop_joy_config = f'{bringup_share}/config/teleop/xbox_teleop.yaml'
    rviz_config = f'{rviz_share}/rviz/azemauto.rviz'

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    start_description_arg = DeclareLaunchArgument(
        'start_description',
        default_value='false',
        description='Start robot_state_publisher locally on the PC.',
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

    joy_node = Node(
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

    return LaunchDescription([
        start_description_arg,
        joy_dev_arg,
        robot_state_publisher_node,
        joy_node,
        teleop_twist_joy_node,
        rviz_node,
    ])
