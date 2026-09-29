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

    common_config = config['common']
    mapping_config = config['mapping']
    can_parameters = dict(config['scout_can'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    # 车辆 CAN 节点：接收 0x221 发布 /odom，并把 /cmd_vel 转成 0x111。
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

        # LD19 驱动：发布 sensor_msgs/msg/LaserScan 到 /scan。
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            output='screen',
        ),

        # 固定安装关系：车辆基座 -> LD19。
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
            name='scout_can_mapping',
            output='screen',
            parameters=[can_parameters],
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
