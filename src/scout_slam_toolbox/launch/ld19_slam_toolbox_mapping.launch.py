#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('scout_slam_toolbox')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    use_sim_time = LaunchConfiguration('use_sim_time')

    can_parameters = {
        'interface_name': 'can0',
        'send_control': True,
        'use_cmd_vel': True,
        'receive_system_state': False,
        'receive_motion_feedback': True,
        'receive_wheel_odometry': False,
        'publish_odom': True,
        'log_frames': False,
        'odom_frame': 'odom',
        'base_frame': 'base_link',
        'use_sim_time': use_sim_time,
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.18',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_laser',
            ],
            output='screen',
        ),
        Node(
            package='scout_can',
            executable='scout_can_node',
            name='scout_can_slam_toolbox',
            output='screen',
            parameters=[can_parameters],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': os.path.join(
                    package_share, 'config', 'mapper_params_online_async.yaml'
                ),
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
