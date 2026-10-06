#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


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
    configured_map = navigation_config['map']
    map_default = (
        configured_map
        if os.path.isabs(configured_map)
        else os.path.join(package_share, 'maps', configured_map)
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, 'launch', 'ld19_vehicle.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
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
                '-d', os.path.join(package_share, 'rviz', 'nav2_navigation_light.rviz')
            ],
        ),
    ])
