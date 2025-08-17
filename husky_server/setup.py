from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'husky_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps', 'map_pleinlaan9_1'), glob('maps/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'service = husky_server.ros2_husky_server:main',
        'husky_control = husky_server.husky_control:main',
        'laser_scan_node = husky_server.laser_scan_node:main',
        'uwb_node = husky_server.uwb_node:main',
        'spawn_node = husky_server.spawn_node:main',
        'find_location = husky_server.find_location:main',
        'uwb_proximity_pose = husky_server.uwb_proximity_pose:main',
        'uwb_RSSI_pose = husky_server.uwb_RSSI_pose:main',
        'slam_pose = husky_server.slam_pose:main',
        'ground_truth_pose_publisher = husky_server.ground_truth_pose_publisher:main',
        'uwb_ranging = husky_server.uwb_ranging:main',
        'uwb_odom_ranging = husky_server.uwb_odom_ranging:main',
        'odom_to_base_link = husky_server.odom_to_base_link:main',
        'odom_pose = husky_server.odom_pose:main',
        ],
    },
)
