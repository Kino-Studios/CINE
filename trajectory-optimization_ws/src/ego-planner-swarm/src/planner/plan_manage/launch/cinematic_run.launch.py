import os
import sys
import numpy as np
from pathlib import Path

here = Path(__file__).resolve()
for parent in [here] + list(here.parents):
    if (parent / "configs").is_dir():
        python_constants_path = parent / "configs" / "python-constants"
        if str(python_constants_path) not in sys.path:
            sys.path.insert(0, str(python_constants_path))
        break

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

from constants import MLPParams, SimulationParams

def generate_cinematic_waypoints():
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
    waypoints = generate_cinematic_waypoints()

    obj_num = LaunchConfiguration('obj_num', default=10)
    drone_id = LaunchConfiguration('drone_id', default=0)

    map_size_x = LaunchConfiguration('map_size_x', default = 50.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 25.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 2.0)

    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')
    depth_topic = LaunchConfiguration('depth_topic', default = 'depth/image_raw')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default = 'camera_pose')
    cloud_topic = LaunchConfiguration('cloud_topic', default = '/map_generator/global_cloud')

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

    use_dynamic = LaunchConfiguration('use_dynamic', default=True)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic)

    use_cinematic_bspline = LaunchConfiguration('use_cinematic_bspline', default=True)
    use_cinematic_bspline_cmd = DeclareLaunchArgument('use_cinematic_bspline', default_value='true')

    use_mlp_correction = LaunchConfiguration('use_mlp_correction', default=str(MLPParams.USE_MLP_CORRECTION).lower())
    use_mlp_correction_cmd = DeclareLaunchArgument('use_mlp_correction', default_value=str(MLPParams.USE_MLP_CORRECTION).lower())

    mlp_weights_path = LaunchConfiguration('mlp_weights_path', default=MLPParams.MLP_WEIGHTS_PATH)
    mlp_weights_path_cmd = DeclareLaunchArgument('mlp_weights_path', default_value=MLPParams.MLP_WEIGHTS_PATH)

    use_simulation = LaunchConfiguration('use_simulation', default='true' if SimulationParams.USE_SIMULATION else 'false')
    use_simulation_cmd = DeclareLaunchArgument('use_simulation', default_value='true' if SimulationParams.USE_SIMULATION else 'false')

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
            {'map/obs_num': 0},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 0},
            {'ObstacleShape/radius_l': 0.7},
            {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 0.8},
            {'ObstacleShape/theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 0.8}
        ],
        condition = IfCondition(PythonExpression([
            '"true" if "', use_simulation, '" == "true" and "', use_mockamap, '" == "false" else "false"'
        ]))
    )

    waypoint_args = {
        'flight_type': str(2),
        'point_num': str(len(waypoints)),
    }

    for i, (x, y, z) in enumerate(waypoints):
        waypoint_args[f'point{i}_x'] = str(x)
        waypoint_args[f'point{i}_y'] = str(y)
        waypoint_args[f'point{i}_z'] = str(z)

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
            'realworld_experiment': PythonExpression(["'false' if '", use_simulation, "' == 'true' else 'true'"]),
            'use_sim_time': use_simulation,
            **waypoint_args
        }.items()
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        condition=UnlessCondition(use_mlp_correction),
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

    traj_server_node_mlp = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        condition=IfCondition(use_mlp_correction),
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('planning/cinematic_bspline', '/mlp/corrected_bspline')
        ],
        parameters=[
            {'traj_server/time_forward': 1.0},
            {'traj_server/use_cinematic_bspline': use_cinematic_bspline}
        ]
    )

    mlp_correction_node = Node(
        package='mlp_correction',
        executable='mlp_node',
        name='mlp_correction_node',
        output='screen',
        condition=IfCondition(use_mlp_correction),
        parameters=[
            {'mlp_weights_path': mlp_weights_path}
        ]
    )

    cinematic_intent_node = Node(
        package="cinematographic_adjustment_agent",
        executable="cinematographic_adjustment_node",
        name="cinematic_intent_node",
        output="screen",
        condition=IfCondition(use_cinematic_bspline),
        parameters=[
            {'waypoints_topic': '/move_base_simple/goal'},
            {'odom_topic': PythonExpression(["'/drone_' + str(", drone_id, ") + '_", odom_topic, "'"])},
            {'depth_topic': depth_topic},
            {'center_topic': '/object_detection/center_point'},
            {'bbox_topic': '/object_detection/bounding_box'},
            {'dimensions_topic': '/object_detection/dimensions'},
            {'image_width': 640},
            {'image_height': 480},
            {'camera_fx': 387.229248046875},
            {'camera_fy': 387.229248046875},
            {'camera_cx': 320.5},
            {'camera_cy': 240.5},
        ],
    )

    cinematic_bspline_filter_node = Node(
        package="cinematographic_adjustment_agent",
        executable="cinematic_bspline_filter",
        name="cinematic_bspline_filter",
        output="screen",
        condition=IfCondition(use_cinematic_bspline),
        parameters=[
            {'bspline_in_topic': PythonExpression(["'/drone_' + str(", drone_id, ") + '_planning/bspline'"])},
            {'bspline_out_topic': PythonExpression(["'/drone_' + str(", drone_id, ") + '_planning/cinematic_bspline'"])},
        ],
    )

    simulated_object_detector_node = Node(
        package="cinematographic_adjustment_agent",
        executable="simulated_object_detector",
        name="simulated_object_detector",
        output="screen",
        condition=IfCondition(PythonExpression([
            '"true" if "', use_simulation, '" == "true" and "', use_cinematic_bspline, '" == "true" else "false"'
        ])),
        remappings=[
            ('depth/image_raw', depth_topic),
        ],
        parameters=[
            {'cx': 320.5},
            {'cy': 240.5},
            {'fx': 387.229248046875},
            {'fy': 387.229248046875},
            {'width': 640},
            {'height': 480},
        ],
    )

    simulated_focus_publisher_node = Node(
        package="cinematographic_adjustment_agent",
        executable="simulated_focus_publisher",
        name="simulated_focus_publisher",
        output="screen",
        condition=IfCondition(use_simulation),
        parameters=[
            {'odom_topic': PythonExpression(["'/drone_' + str(", drone_id, ") + '_", odom_topic, "'"])},
            {'target_xyz': SimulationParams.TARGET_XYZ},
            {'focus_depth_m': SimulationParams.FOCUS_DEPTH_M},
        ],
    )

    simulated_light_publisher_node = Node(
        package="cinematographic_adjustment_agent",
        executable="simulated_light_publisher",
        name="simulated_light_publisher",
        output="screen",
        condition=IfCondition(use_simulation),
        parameters=[
            {'odom_topic': PythonExpression(["'/drone_' + str(", drone_id, ") + '_", odom_topic, "'"])},
        ],
    )

    cost_function_calculator_node = Node(
        package="cinematographic_adjustment_agent",
        executable="cost_function_calculator",
        name="cost_function_calculator",
        output="screen",
        condition=UnlessCondition(use_simulation),
        parameters=[
            # TODO: verify /imu_propagate is the correct odometry source for this hardware setup
            {'odom_topic': '/imu_propagate'},
            {'depth_topic': depth_topic},
            {'center_topic': '/object_detection/center_point'},
            {'bbox_topic': '/object_detection/bounding_box'},
            {'dimensions_topic': '/object_detection/dimensions'},
            {'light_vector_topic': '/cinematography/light_vector'},
            {'camera_fx': 387.229248046875},
            {'camera_fy': 387.229248046875},
            {'camera_cx': 320.5},
            {'camera_cy': 240.5},
        ],
    )

    light_vector_analysis_node = Node(
        package="cinematographic_adjustment_agent",
        executable="light_vector_analysis",
        name="light_vector_analysis",
        output="screen",
        condition=UnlessCondition(use_simulation),
        parameters=[
            {'image_topic': '/camera_publisher/image_raw'},
            # TODO: verify /imu_propagate is the correct odometry source for this hardware setup
            {'odom_topic': '/imu_propagate'},
            {'light_vector_topic': '/cinematography/light_vector'},
            {'camera_fx': 387.229248046875},
            {'camera_fy': 387.229248046875},
            {'camera_cx': 320.5},
            {'camera_cy': 240.5},
        ],
    )

    simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        condition=IfCondition(use_simulation),
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
    ld.add_action(use_mlp_correction_cmd)
    ld.add_action(mlp_weights_path_cmd)
    ld.add_action(use_simulation_cmd)

    ld.add_action(map_generator_node)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(traj_server_node_mlp)
    ld.add_action(mlp_correction_node)
    ld.add_action(simulated_object_detector_node)
    ld.add_action(simulated_focus_publisher_node)
    ld.add_action(simulated_light_publisher_node)
    ld.add_action(cost_function_calculator_node)
    ld.add_action(light_vector_analysis_node)
    ld.add_action(cinematic_intent_node)
    ld.add_action(cinematic_bspline_filter_node)
    ld.add_action(simulator_include)

    return ld
