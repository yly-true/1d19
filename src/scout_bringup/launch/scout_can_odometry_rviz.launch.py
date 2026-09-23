#!/usr/bin/env python3
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('scout_bringup')
    with open(
        os.path.join(package_share, 'config', 'scout_bringup_config.yaml'),
        encoding='utf-8',
    ) as config_file:
        config = yaml.safe_load(config_file)

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    can_parameters = dict(config['scout_can'])
    can_parameters['send_control'] = True
    can_parameters['receive_system_state'] = False
    can_parameters['receive_motion_feedback'] = True
    can_parameters['receive_wheel_odometry'] = False
    can_parameters['log_frames'] = False
    can_parameters['publish_odom'] = True
    can_parameters['use_sim_time'] = use_sim_time

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=str(config['common']['use_sim_time']).lower(),
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value=str(config['common']['rviz']).lower(),
        ),
        Node(
            package='scout_can',
            executable='scout_can_node',
            name='scout_can_odom_from_0x221',
            output='screen',
            parameters=[can_parameters],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            arguments=[
                '-d', os.path.join(package_share, 'rviz', 'scout_can_odometry.rviz')
            ],
        ),
    ])
