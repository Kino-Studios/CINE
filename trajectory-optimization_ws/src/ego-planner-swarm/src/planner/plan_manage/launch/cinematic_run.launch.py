import os
import numpy as np
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_cinematic_waypoints():
    """Generate simple straight-line cinematographic waypoints - guaranteed obstacle-free"""
    # Simple path: start area (-15, 0) moving toward goal area (1.5, -3.0)
    # Create 8 waypoints along a smooth path
    waypoints = [
        (-14.0, 0.0, 1.0),
        (-12.0, -0.5, 1.0),
        (-10.0, -1.0, 1.0),
        (-8.0, -1.5, 1.0),
        (-5.0, -2.0, 1.0),
        (-2.0, -2.5, 1.0),
        (0.5, -2.8, 1.0),
        (1.5, -3.0, 1.0),
    ]
    return waypoints

def generate_cinematic_waypoints_UNUSED():
    """Original arc waypoints - kept for reference"""
    center_x, center_y = 0.5, -1.0
    radius = 0.4
    z_height = 1.0
    num_points = 16
    start_angle = -np.pi/2
    end_angle = -np.pi/2 + 2*np.pi/3
    angles = np.linspace(start_angle, end_angle, num_points)
    
    waypoints = []
    for angle in angles:
        x = center_x + radius * np.cos(angle)
        y = center_y + radius * np.sin(angle)
        waypoints.append((x, y, z_height))
    
    return waypoints

def generate_launch_description():
    # Generate cinematographic waypoints
    waypoints = generate_cinematic_waypoints()
    
    # Basic parameters
    obj_num = LaunchConfiguration('obj_num', default=10)
    drone_id = LaunchConfiguration('drone_id', default=0)
    
    map_size_x = LaunchConfiguration('map_size_x', default = 50.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 25.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 2.0)
    
    odom_topic = LaunchConfiguration('odom_topic', default = 'odometry')
    depth_topic = LaunchConfiguration('depth_topic', default = 'depth/image_raw')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default = 'camera_pose')
    cloud_topic = LaunchConfiguration('cloud_topic', default = '/map_generator/global_cloud')
    
    # Declare arguments
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num)
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id)
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x)
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y)
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z)
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic)
    depth_topic_cmd = DeclareLaunchArgument('depth_topic', default_value=depth_topic)
    camera_pose_topic_cmd = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic)
    cloud_topic_cmd = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic)

    use_mockamap = LaunchConfiguration('use_mockamap', default=False)
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap)
    
    use_dynamic = LaunchConfiguration('use_dynamic', default=False)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic)
    
    use_cinematic_bspline = LaunchConfiguration('use_cinematic_bspline', default=True)
    use_cinematic_bspline_cmd = DeclareLaunchArgument('use_cinematic_bspline', default_value='true')
    
    # Map generator - NO OBSTACLES for clean cinematographic trajectory
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 26.0},
            {'map/y_size': 20.0},
            {'map/z_size': 3.0},
            {'map/resolution': 0.1},
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 0},  # DISABLED obstacles
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 0},  # DISABLED obstacles
            {'ObstacleShape/radius_l': 0.7},
            {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 0.8},
            {'ObstacleShape/theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 0.8}
        ],
        condition = UnlessCondition(use_mockamap)
    )
    
    # Build launch arguments for waypoints
    waypoint_args = {
        'flight_type': str(2),  # PRESET_TARGET mode
        'point_num': str(len(waypoints)),
    }
    
    # Add all 16 waypoints
    for i, (x, y, z) in enumerate(waypoints):
        waypoint_args[f'point{i}_x'] = str(x)
        waypoint_args[f'point{i}_y'] = str(y)
        waypoint_args[f'point{i}_z'] = str(z)
    
    # Include advanced parameters with cinematographic waypoints
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            'camera_pose_topic': camera_pose_topic,
            'depth_topic': depth_topic,
            'cloud_topic': cloud_topic,
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(2.5),
            'max_acc': str(7.0),
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': 'False',
            **waypoint_args  # Add cinematographic waypoints
        }.items()
    )
    
    # Trajectory server with cinematic mode enabled
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('planning/cinematic_bspline', ['drone_', drone_id, '_planning/cinematic_bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0},
            {'traj_server/use_cinematic_bspline': use_cinematic_bspline}
        ]
    )
    
    # Simulator
    simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': str(-15.0),
            'init_y_': str(0.0),
            'init_z_': str(0.1),
            'odometry_topic': odom_topic
        }.items()
    )
    
    ld = LaunchDescription()
    
    # Add all actions
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(depth_topic_cmd)
    ld.add_action(camera_pose_topic_cmd)
    ld.add_action(cloud_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(use_mockamap_cmd)
    ld.add_action(use_cinematic_bspline_cmd)
    
    ld.add_action(map_generator_node)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(simulator_include)
    
    return ld
