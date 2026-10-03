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
    nav2_share = get_package_share_directory('nav2_bringup')
    with open(
        os.path.join(package_share, 'config', 'scout_bringup_config.yaml'),
        encoding='utf-8',
    ) as config_file:
        config = yaml.safe_load(config_file)

    common_config = config['common']
    navigation_config = config['navigation']
    can_parameters = dict(config['scout_can'])
    configured_map = navigation_config['map']
    map_default = (
        configured_map
        if os.path.isabs(configured_map)
        else os.path.join(package_share, 'maps', configured_map)
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')
    # CAN 车辆里程计：接收 0x221 发布 /odom，并把 Nav2 的 /cmd_vel 转成 0x111。
    can_parameters['send_control'] = True
    can_parameters['use_cmd_vel'] = True
    can_parameters['receive_system_state'] = False
    can_parameters['receive_motion_feedback'] = True
    can_parameters['receive_wheel_odometry'] = False
    can_parameters['publish_odom'] = True
    can_parameters['log_frames'] = False
    can_parameters['use_sim_time'] = use_sim_time

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=str(common_config['use_sim_time']).lower(),
        ),
        DeclareLaunchArgument(
            'map',
            default_value=map_default,
            description='Saved 2D map YAML file',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value=str(common_config['rviz']).lower(),
        ),

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
            name='scout_can_navigation',
            output='screen',
            parameters=[can_parameters],
        ),

        # Nav2 localization: map_server + AMCL；不启动 SLAM。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml,
                'params_file': os.path.join(
                    package_share, 'config', 'nav2_navigation_params.yaml'
                ),
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'use_composition': 'False',
                'use_respawn': 'False',
                'slam': 'False',
                'use_localization': 'True',
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            arguments=[
                '-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')
            ],
        ),
    ])
