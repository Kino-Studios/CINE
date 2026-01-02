#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
import sys
from pathlib import Path

# Add repo root to sys.path to find 'configs'
here = Path(__file__).resolve()
for parent in [here] + list(here.parents):
    if (parent / "configs").is_dir():
        python_constants_path = parent / "configs" / "python-constants"
        if str(python_constants_path) not in sys.path:
            sys.path.insert(0, str(python_constants_path))
        break

from constants import SimulationParams


def generate_launch_description():
    drone_id = SimulationParams.DRONE_ID
    odom_topic = f"/drone_{drone_id}_visual_slam/odom"
    baseline_topic = f"/drone_{drone_id}_planning/bspline"
    cinematic_topic = f"/drone_{drone_id}_planning/cinematic_bspline"
    
    trajectory_logger = Node(
        package='cinematographic_adjustment_agent',
        executable='trajectory_data_logger',
        name='trajectory_data_logger',
        output='screen',
        parameters=[{
            'focus_max_velocity_mps': 0.7,
            'default_max_velocity_mps': 1.55,
            'odom_topic': odom_topic,
            'baseline_bspline_topic': baseline_topic,
            'cinematic_bspline_topic': cinematic_topic,
        }]
    )
    
    return LaunchDescription([
        trajectory_logger,
    ])
