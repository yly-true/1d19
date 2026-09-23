#!/usr/bin/env python3
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('scout_bringup')
    with open(
        os.path.join(package_share, 'config', 'scout_bringup_config.yaml'),
        encoding='utf-8',
    ) as config_file:
        config = yaml.safe_load(config_file)

    def create_can_node(context):
        mode = LaunchConfiguration('mode').perform(context)
        mode_parameters = {
            'send_control': {
                'send_control': True,
                'receive_system_state': False,
                'receive_motion_feedback': False,
                'receive_wheel_odometry': False,
            },
            'system_status': {
                'send_control': False,
                'receive_system_state': True,
                'receive_motion_feedback': False,
                'receive_wheel_odometry': False,
            },
            'motion_feedback': {
                'send_control': False,
                'receive_system_state': False,
                'receive_motion_feedback': True,
                'receive_wheel_odometry': False,
            },
            'wheel_odometry': {
                'send_control': False,
                'receive_system_state': False,
                'receive_motion_feedback': False,
                'receive_wheel_odometry': True,
            },
        }
        if mode not in mode_parameters:
            valid_modes = ', '.join(mode_parameters)
            raise RuntimeError(f'Unknown CAN mode: {mode}. Use one of: {valid_modes}')

        can_parameters = dict(config['scout_can'])
        can_parameters.update(mode_parameters[mode])
        can_parameters['publish_odom'] = False
        can_parameters['log_frames'] = False
        can_parameters['use_sim_time'] = LaunchConfiguration('use_sim_time')

        return [Node(
            package='scout_can',
            executable='scout_can_node',
            name=f'scout_can_{mode}',
            output='screen',
            parameters=[can_parameters],
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=str(config['common']['use_sim_time']).lower(),
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='send_control',
            description=(
                'CAN function: send_control, system_status, '
                'motion_feedback, or wheel_odometry'
            ),
        ),
        OpaqueFunction(function=create_can_node),
    ])
