from glob import glob
from setuptools import setup


package_name = 'my_robot_bringup'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config/mavros', glob('config/mavros/*.yaml')),
        ('share/' + package_name + '/config/realsense', glob('config/realsense/*.yaml')),
        ('share/' + package_name + '/config/lidar', glob('config/lidar/*.yaml')),
        ('share/' + package_name + '/config/network', glob('config/network/*.xml')),
        ('share/' + package_name + '/config/safety', glob('config/safety/*.yaml')),
        ('share/' + package_name + '/config/mapping', glob('config/mapping/*.yaml')),
        ('share/' + package_name + '/config/localization', glob('config/localization/*.yaml')),
        ('share/' + package_name + '/config/nav2', glob('config/nav2/*.yaml')),
        ('share/' + package_name + '/config/teleop', glob('config/teleop/*.yaml')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
        (
            'share/' + package_name + '/config/future/robot_localization',
            glob('config/future/robot_localization/*.yaml'),
        ),
        (
            'share/' + package_name + '/config/future/nav2',
            glob('config/future/nav2/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azem',
    maintainer_email='azem@example.com',
    description='Launch and configuration package for azemauto.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
