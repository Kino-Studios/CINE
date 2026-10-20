from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='cine_depth',
                executable='depth_publisher_node',
                name='depth_publisher',
                output='screen',
                parameters=[{'use_sim_time': False}],
            )
        ]
    )
