#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    drone_id = os.getenv("DRONE_ID", "0")
    odom_topic = f"/drone_{drone_id}_visual_slam/odom"
    baseline_topic = f"/drone_{drone_id}_planning/bspline"
    cinematic_topic = f"/drone_{drone_id}_planning/cinematic_bspline"
    
    trajectory_logger = Node(
        package='cinematographic_adjustment_agent',
        executable='trajectory_data_logger',
        name='trajectory_data_logger',
        output='screen',
        parameters=[{
            'focus_max_velocity_mps': 0.8,
            'default_max_velocity_mps': 1.1,
            'odom_topic': odom_topic,
            'baseline_bspline_topic': baseline_topic,
            'cinematic_bspline_topic': cinematic_topic,
        }]
    )
    
    return LaunchDescription([
        trajectory_logger,
    ])
