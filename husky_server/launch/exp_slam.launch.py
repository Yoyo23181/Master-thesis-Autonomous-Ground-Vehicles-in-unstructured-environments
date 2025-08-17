import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    namespace = 'a200_0957'
    setup_path = os.path.join(os.getenv('HOME'), 'clearpath/')

    robot_description_param = {
        'robot_description': Command([
            PathJoinSubstitution([
                setup_path, 'robot.urdf.xacro'
            ])
        ]
        )
    }

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('clearpath_viz'), '/launch/view_navigation.launch.py'
        ]),
        launch_arguments={'namespace': namespace}.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('husky_server'), '/launch/slam.launch.py'
        ]),
        launch_arguments={
            'setup_path': setup_path,
            'use_sim_time': 'true'
        }.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[robot_description_param],
        output='screen'
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('husky_server'), '/launch/nav2_simu.launch.py'
        ]),
        launch_arguments={
            'setup_path': setup_path,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        # navigation_launch,
        slam_launch,
        # robot_state_publisher_node,
        TimerAction(period=5.0, actions=[visualization_launch])
    ])
