#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    trajectory_logger = Node(
        package='cinematographic_adjustment_agent',
        executable='trajectory_data_logger',
        name='trajectory_data_logger',
        output='screen',
        parameters=[{
            'focus_max_velocity_mps': 1.0,
            'default_max_velocity_mps': 2.0,
        }]
    )
    
    return LaunchDescription([
        trajectory_logger,
    ])
