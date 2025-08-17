from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tb4_code'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'spawn_node = tb4_code.spawn_node:main',
        'uwb_rssi_pose = tb4_code.uwb_RSSI_pose:main',
        'slam_pose =tb4_code.slam_pose:main',
        'uwb_ranging_pose = tb4_code.uwb_ranging_pose:main',
        'uwb_odom_real = tb4_code.uwb_odom_real:main',
        'odom_to_base_link = tb4_code.odom_to_base_link:main',
        'real_RSSI = tb4_code.uwb_RSSI_real:main',
        'real_ranging = tb4_code.uwb_ranging_real:main',
        'error_on_range = tb4_code.error_on_range:main',
        'odom_pose = tb4_code.odom_pose:main',
        'plot_graph = tb4_code.plot_graph:main',
        ],
    },
)
