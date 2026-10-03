#!/usr/bin/env python3
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('scout_bringup')
    with open(os.path.join(package_share, 'config', 'scout_bringup_config.yaml'),
              encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)

    use_sim_time = LaunchConfiguration('use_sim_time')
    can_parameters = dict(config['scout_can'])
    can_parameters.update({
        'send_control': True,
        'use_cmd_vel': True,
        'receive_system_state': False,
        'receive_motion_feedback': True,
        'receive_wheel_odometry': False,
        'publish_odom': True,
        'log_frames': False,
        'use_sim_time': use_sim_time,
    })

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=str(config['common']['use_sim_time']).lower(),
        ),
        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
             output='screen'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.18',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link', '--child-frame-id', 'base_laser',
            ],
            output='screen',
        ),
        Node(package='scout_can', executable='scout_can_node',
             name='scout_can_vehicle', parameters=[can_parameters], output='screen'),
    ])
