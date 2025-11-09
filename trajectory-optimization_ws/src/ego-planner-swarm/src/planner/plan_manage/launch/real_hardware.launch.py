from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    depth_topic = LaunchConfiguration('depth_topic', default='/depth/image_raw')
    odom_topic = LaunchConfiguration('odom_topic', default='/odometry')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='/camera_pose')
    
    return LaunchDescription([
        DeclareLaunchArgument('depth_topic', default_value='/depth/image_raw'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry'),
        DeclareLaunchArgument('camera_pose_topic', default_value='/camera_pose'),
        
        Node(
            package='ego_planner',
            executable='ego_planner_node',
            name='ego_planner_node',
            output='screen',
            parameters=[{
                'depth_topic': depth_topic,
                'odom_topic': odom_topic,
                'camera_pose_topic': camera_pose_topic,
                'use_sim_time': False,
                'flight_type': 2,
                'target_x': 5.0,
                'target_y': 0.0,
                'target_z': 1.0
            }]
        ),
    ])
