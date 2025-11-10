from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ego_planner',
            executable='ego_planner_node',
            name='ego_planner_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'fsm/flight_type': 1,
                'fsm/thresh_replan_time': 2.0,
                'fsm/thresh_no_replan_meter': 0.01,
                'fsm/planning_horizon': 5.0,
                'fsm/planning_horizen_time': 3.0,
                'fsm/emergency_time': 1.0,
                'fsm/realworld_experiment': True,
                'fsm/fail_safe': True,
                'fsm/waypoint_num': 0
            }],
            remappings=[
                ('/grid_map/odom', '/imu_propagate'),
                ('/odom_world', '/imu_propagate'),
                ('/grid_map/depth', '/depth/image_raw'),
                ('/grid_map/pose', '/camera_pose'),
            ]
        ),
    ])