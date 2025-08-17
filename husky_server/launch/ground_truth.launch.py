import os

from PIL.ImageChops import screen
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_pose_bridge',
        arguments=['/model/a200_0957/robot/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        output = 'screen'
    )

    ground_truth_pose_node = Node(
        package='husky_server',
        executable='ground_truth_pose_publisher',
        name='ground_truth_pose_publisher',
        output = 'screen',
    )

    return LaunchDescription([
        gz_bridge_node,
        ground_truth_pose_node
    ])