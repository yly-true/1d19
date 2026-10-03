#!/usr/bin/env python3
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('scout_bringup')
    with open(
        os.path.join(package_share, 'config', 'scout_bringup_config.yaml'),
        encoding='utf-8',
    ) as config_file:
        config = yaml.safe_load(config_file)

    common_config = config['common']
    mapping_config = config['mapping']

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=str(common_config['use_sim_time']).lower(),
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value=str(common_config['rviz']).lower(),
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value=str(mapping_config['resolution']),
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=str(mapping_config['publish_period_sec']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, 'launch', 'ld19_vehicle.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 使用 LD19 激光建图，同时使用车辆 /odom 提供运动先验。
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory',
                os.path.join(package_share, 'config'),
                '-configuration_basename',
                'ld19_cartographer_2d.lua',
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-resolution', resolution,
                '-publish_period_sec', publish_period_sec,
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            arguments=[
                '-d', os.path.join(package_share, 'rviz', 'ld19_mapping.rviz')
            ],
        ),
    ])
