from setuptools import setup


package_name = 'my_robot_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azem',
    maintainer_email='azem@example.com',
    description='Bridge node for azemauto.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavros_bridge = my_robot_bridge.mavros_bridge_node:main',
            'cmd_vel_to_mavros = my_robot_bridge.cmd_vel_to_mavros_node:main',
            'cmd_vel_to_rc_override = my_robot_bridge.cmd_vel_to_rc_override_node:main',
            'cmd_vel_to_manual_control = my_robot_bridge.cmd_vel_to_manual_control_node:main',
            'auto_arm_disarm = my_robot_bridge.auto_arm_disarm_node:main',
            'scan_timestamp_bridge = my_robot_bridge.scan_timestamp_bridge_node:main',
        ],
    },
)
