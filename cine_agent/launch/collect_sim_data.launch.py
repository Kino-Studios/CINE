from cine_config.constants import SimulationParams
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drone_id = SimulationParams.DRONE_ID
    odom_topic = f'/drone_{drone_id}_visual_slam/odom'
    baseline_topic = f'/drone_{drone_id}_planning/bspline'
    cinematic_topic = f'/drone_{drone_id}_planning/cinematic_bspline'

    trajectory_logger = Node(
        package='cine_agent',
        executable='trajectory_data_logger',
        name='trajectory_data_logger',
        output='screen',
        parameters=[
            {
                'odom_topic': odom_topic,
                'baseline_bspline_topic': baseline_topic,
                'cinematic_bspline_topic': cinematic_topic,
            }
        ],
    )

    return LaunchDescription(
        [
            trajectory_logger,
        ]
    )
