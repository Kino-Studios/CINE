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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

from constants import CinematicConstraints, CinematographyParams

def generate_launch_description():
    # 定义参数的 LaunchConfiguration
    obj_num = LaunchConfiguration('obj_num', default=10)
    drone_id = LaunchConfiguration('drone_id', default=0)
    
    map_size_x = LaunchConfiguration('map_size_x', default = 50.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 25.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 5.0)
    
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')
    depth_topic = LaunchConfiguration('depth_topic', default = 'depth/image_raw')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default = 'camera_pose')
    cloud_topic = LaunchConfiguration('cloud_topic', default = '/map_generator/global_cloud')
    
    # 声明全局参数
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')
    depth_topic_cmd = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth topic')
    camera_pose_topic_cmd = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic')
    cloud_topic_cmd = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud topic for obstacles')

    # 地图属性以及是否使用动力学仿真
    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap 
    
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')
    
    use_dynamic = LaunchConfiguration('use_dynamic', default=True)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')
    
    use_cinematic_bspline = LaunchConfiguration('use_cinematic_bspline', default=True)
    use_cinematic_intent = LaunchConfiguration('use_cinematic_intent', default='true')
    use_cinematic_bspline_cmd = DeclareLaunchArgument('use_cinematic_bspline', default_value='true', description='Use cinematographic B-spline filter')
    use_cinematic_intent_cmd = DeclareLaunchArgument('use_cinematic_intent', default_value=use_cinematic_intent, description='Enable cinematographic intent node')
    use_simulated_light = LaunchConfiguration('use_simulated_light', default='true')
    simulated_light_use_target_world = LaunchConfiguration('simulated_light_use_target_world', default='true')
    use_simulated_light_cmd = DeclareLaunchArgument('use_simulated_light', default_value=use_simulated_light, description='Enable simulated light vector publisher')
    simulated_light_use_target_world_cmd = DeclareLaunchArgument(
        'simulated_light_use_target_world',
        default_value=simulated_light_use_target_world,
        description='Use /cinematography/target_world as the simulated light source'
    )

    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs', default='false')
    use_distinctive_trajs_cmd = DeclareLaunchArgument(
        'use_distinctive_trajs',
        default_value=use_distinctive_trajs,
        description='Enable multi-trajectory search'
    )

    params = CinematographyParams()
    constraints = CinematicConstraints()
    default_max_vel = LaunchConfiguration('default_max_vel', default=constraints.DEFAULT_MAX_VELOCITY_MPS)
    default_max_acc = LaunchConfiguration('default_max_acc', default=constraints.DEFAULT_MAX_ACCEL_MPS2)
    focus_max_vel = LaunchConfiguration('focus_max_vel', default=constraints.FOCUS_MAX_VELOCITY_MPS)
    focus_max_acc = LaunchConfiguration('focus_max_acc', default=constraints.FOCUS_MAX_ACCEL_MPS2)
    vel_damping = LaunchConfiguration('vel_damping', default=constraints.VEL_DAMPING)
    acc_damping = LaunchConfiguration('acc_damping', default=constraints.ACC_DAMPING)
    default_speed_scale = LaunchConfiguration('default_speed_scale', default=constraints.DEFAULT_SPEED_SCALE)
    focus_speed_scale = LaunchConfiguration('focus_speed_scale', default=constraints.FOCUS_SPEED_SCALE)
    target_distance_m = LaunchConfiguration('target_distance_m', default=constraints.TARGET_DISTANCE_M)
    target_distance_tol_m = LaunchConfiguration('target_distance_tol_m', default=constraints.TARGET_DISTANCE_TOL_M)
    target_distance_gate_m = LaunchConfiguration('target_distance_gate_m', default=constraints.TARGET_DISTANCE_GATE_M)
    yawdot_max = LaunchConfiguration('yawdot_max', default=constraints.YAWDOT_MAX)
    light_avoid_gain = LaunchConfiguration('light_avoid_gain', default=constraints.LIGHT_AVOID_GAIN)
    light_influence_distance_m = LaunchConfiguration('light_influence_distance_m', default=constraints.LIGHT_INFLUENCE_DISTANCE_M)
    light_avoid_speed_threshold = LaunchConfiguration(
        'light_avoid_speed_threshold', default=constraints.LIGHT_AVOID_SPEED_THRESHOLD_MPS)
    light_avoid_blend_base = LaunchConfiguration('light_avoid_blend_base', default=constraints.LIGHT_AVOID_BLEND_BASE)
    light_avoid_max_blend = LaunchConfiguration('light_avoid_max_blend', default=constraints.LIGHT_AVOID_MAX_BLEND)
    light_avoid_transition_s = LaunchConfiguration(
        'light_avoid_transition_s', default=constraints.LIGHT_AVOID_TRANSITION_S)
    light_cooldown_after_framing_s = LaunchConfiguration(
        'light_cooldown_after_framing_s', default=constraints.LIGHT_COOLDOWN_AFTER_FRAMING_S)
    light_distance_blend_gain = LaunchConfiguration(
        'light_distance_blend_gain', default=constraints.LIGHT_DISTANCE_BLEND_GAIN)
    light_rotation_damping = LaunchConfiguration(
        'light_rotation_damping', default=constraints.LIGHT_ROTATION_DAMPING)
    light_yaw_smooth_alpha = LaunchConfiguration(
        'light_yaw_smooth_alpha', default=constraints.LIGHT_YAW_SMOOTH_ALPHA)
    light_disable_near_finish_s = LaunchConfiguration(
        'light_disable_near_finish_s', default=constraints.LIGHT_DISABLE_NEAR_FINISH_S)
    use_target_yaw = LaunchConfiguration('use_target_yaw', default=str(constraints.USE_TARGET_YAW).lower())
    use_light_avoid_yaw = LaunchConfiguration('use_light_avoid_yaw', default='true')
    debug_light_yaw = LaunchConfiguration('debug_light_yaw', default='true')
    debug_light_period_s = LaunchConfiguration('debug_light_period_s', default=0.5)

    default_max_vel_cmd = DeclareLaunchArgument('default_max_vel', default_value=default_max_vel, description='Baseline max velocity')
    default_max_acc_cmd = DeclareLaunchArgument('default_max_acc', default_value=default_max_acc, description='Baseline max acceleration')
    focus_max_vel_cmd = DeclareLaunchArgument('focus_max_vel', default_value=focus_max_vel, description='Framing max velocity')
    focus_max_acc_cmd = DeclareLaunchArgument('focus_max_acc', default_value=focus_max_acc, description='Framing max acceleration')
    vel_damping_cmd = DeclareLaunchArgument('vel_damping', default_value=vel_damping, description='Velocity damping blend')
    acc_damping_cmd = DeclareLaunchArgument('acc_damping', default_value=acc_damping, description='Acceleration damping blend')
    default_speed_scale_cmd = DeclareLaunchArgument('default_speed_scale', default_value=default_speed_scale, description='Baseline speed scale')
    focus_speed_scale_cmd = DeclareLaunchArgument('focus_speed_scale', default_value=focus_speed_scale, description='Framing speed scale')
    target_distance_m_cmd = DeclareLaunchArgument('target_distance_m', default_value=target_distance_m, description='Target framing distance')
    target_distance_tol_m_cmd = DeclareLaunchArgument('target_distance_tol_m', default_value=target_distance_tol_m, description='Target framing distance tolerance')
    target_distance_gate_m_cmd = DeclareLaunchArgument('target_distance_gate_m', default_value=target_distance_gate_m, description='Framing distance gate')
    yawdot_max_cmd = DeclareLaunchArgument('yawdot_max', default_value=yawdot_max, description='Max yaw rate in rad/s')
    light_avoid_gain_cmd = DeclareLaunchArgument('light_avoid_gain', default_value=light_avoid_gain, description='Light avoidance yaw blend')
    light_influence_distance_m_cmd = DeclareLaunchArgument('light_influence_distance_m', default_value=light_influence_distance_m, description='Light avoidance influence radius')
    light_avoid_speed_threshold_cmd = DeclareLaunchArgument('light_avoid_speed_threshold', default_value=light_avoid_speed_threshold, description='Speed below which light avoidance can run')
    light_avoid_blend_base_cmd = DeclareLaunchArgument('light_avoid_blend_base', default_value=light_avoid_blend_base, description='Minor blend floor for light yaw')
    light_avoid_max_blend_cmd = DeclareLaunchArgument('light_avoid_max_blend', default_value=light_avoid_max_blend, description='Maximum blend factor for light yaw')
    light_avoid_transition_s_cmd = DeclareLaunchArgument('light_avoid_transition_s', default_value=light_avoid_transition_s, description='Time to ramp light avoidance after framing')
    light_cooldown_after_framing_s_cmd = DeclareLaunchArgument('light_cooldown_after_framing_s', default_value=light_cooldown_after_framing_s, description='Cooldown after framing before light avoidance engages')
    light_distance_blend_gain_cmd = DeclareLaunchArgument('light_distance_blend_gain', default_value=light_distance_blend_gain, description='Closer range blend boost')
    light_rotation_damping_cmd = DeclareLaunchArgument('light_rotation_damping', default_value=light_rotation_damping, description='Damping factor applied to light yaw change')
    light_yaw_smooth_alpha_cmd = DeclareLaunchArgument('light_yaw_smooth_alpha', default_value=light_yaw_smooth_alpha, description='Low-pass alpha applied to light yaw output')
    light_disable_near_finish_s_cmd = DeclareLaunchArgument('light_disable_near_finish_s', default_value=light_disable_near_finish_s, description='Disable light rotation when near trajectory end')
    use_target_yaw_cmd = DeclareLaunchArgument('use_target_yaw', default_value=use_target_yaw, description='Use target-facing yaw during framing')
    use_light_avoid_yaw_cmd = DeclareLaunchArgument('use_light_avoid_yaw', default_value=use_light_avoid_yaw, description='Enable light avoidance yaw during framing')
    debug_light_yaw_cmd = DeclareLaunchArgument('debug_light_yaw', default_value=debug_light_yaw, description='Log light/yaw debug info')
    debug_light_period_s_cmd = DeclareLaunchArgument('debug_light_period_s', default_value=debug_light_period_s, description='Light yaw debug period (s)')

    # map
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 45.0},
            {'map/y_size': 45.0},
            {'map/z_size': map_size_z},
            {'map/resolution': 0.1},
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 25},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 25},
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

    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[
            ('/mock_map', '/map_generator/global_cloud')
        ],
        parameters=[
            {'seed': 127},
            {'update_freq': 0.5},
            {'resolution': 0.1},
            {'x_length': PythonExpression(['int(', map_size_x, ')'])},
            {'y_length': PythonExpression(['int(', map_size_y, ')'])},
            {'z_length': PythonExpression(['int(', map_size_z, ')'])},
            {'type': 1},
            {'complexity': 0.05},
            {'fill': 0.12},
            {'fractal': 1},
            {'attenuation': 0.1}
        ],
        condition = IfCondition(use_mockamap)
    )
    
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
            'max_vel': default_max_vel,
            'max_acc': default_max_acc,
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': use_distinctive_trajs,
            'flight_type': str(1),
            'point_num': str(4),
            'point0_x': str(15.0),
            'point0_y': str(0.0),
            'point0_z': str(1.0),
            
            'point1_x': str(-15.0),
            'point1_y': str(0.0),
            'point1_z': str(1.0),
            
            'point2_x': str(15.0),
            'point2_y': str(0.0),
            'point2_z': str(1.0),
            
            'point3_x': str(-15.0),
            'point3_y': str(0.0),
            'point3_z': str(1.0),
            
            'point4_x': str(15.0),
            'point4_y': str(0.0),
            'point4_z': str(1.0),
        }.items()
    )
    
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
            {'traj_server/use_cinematic_bspline': use_cinematic_bspline},
            {'traj_server/default_speed_scale': default_speed_scale},
            {'traj_server/focus_speed_scale': focus_speed_scale},
            {'traj_server/default_max_vel': default_max_vel},
            {'traj_server/default_max_acc': default_max_acc},
            {'traj_server/focus_max_vel': focus_max_vel},
            {'traj_server/focus_max_acc': focus_max_acc},
            {'traj_server/target_distance_m': target_distance_m},
            {'traj_server/target_distance_tol_m': target_distance_tol_m},
            {'traj_server/target_distance_gate_m': target_distance_gate_m},
            {'traj_server/yawdot_max': yawdot_max},
            {'traj_server/light_avoid_gain': light_avoid_gain},
            {'traj_server/light_influence_distance_m': light_influence_distance_m},
            {'traj_server/light_avoid_speed_threshold': light_avoid_speed_threshold},
            {'traj_server/light_avoid_blend_base': light_avoid_blend_base},
            {'traj_server/light_avoid_max_blend': light_avoid_max_blend},
            {'traj_server/light_avoid_transition_s': light_avoid_transition_s},
            {'traj_server/light_avoid_cooldown_after_framing_s': light_cooldown_after_framing_s},
            {'traj_server/light_disable_near_finish_s': light_disable_near_finish_s},
            {'traj_server/light_distance_blend_gain': light_distance_blend_gain},
            {'traj_server/light_rotation_damping': light_rotation_damping},
            {'traj_server/light_yaw_smooth_alpha': light_yaw_smooth_alpha},
            {'traj_server/use_target_yaw': use_target_yaw},
            {'traj_server/use_light_avoid_yaw': use_light_avoid_yaw},
            {'traj_server/debug_light_yaw': debug_light_yaw},
            {'traj_server/debug_light_period_s': debug_light_period_s},
            {'traj_server/vel_damping': vel_damping},
            {'traj_server/acc_damping': acc_damping},
        ]
    )

    cinematic_intent_node = Node(
        package="cinematographic_adjustment_agent",
        executable="cinematographic_adjustment_node",
        name="cinematic_intent_node",
        output="screen",
        condition=IfCondition(use_cinematic_intent),
        parameters=[
            {'waypoints_topic': '/move_base_simple/goal'},
            {'odom_topic': PythonExpression(["'/drone_' + str(", drone_id, ") + '_", odom_topic, "'"])},
            {'depth_topic': depth_topic},
            {'center_topic': '/object_detection/center_point'},
            {'bbox_topic': '/object_detection/bounding_box'},
            {'dimensions_topic': '/object_detection/dimensions'},
            {'image_width': 1152},
            {'image_height': 648},
            {'camera_fx': 750.99681791},
            {'camera_fy': 748.9227188},
            {'camera_cx': 578.86898934},
            {'camera_cy': 326.60855341},
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
        condition=IfCondition(use_cinematic_bspline),
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

    simulated_light_node = Node(
        package="cinematographic_adjustment_agent",
        executable="simulated_light_publisher",
        name="simulated_light_publisher",
        output="screen",
        condition=IfCondition(use_simulated_light),
        parameters=[
            {'frame_id': 'world'},
            {'use_odom': True},
            {'odom_topic': PythonExpression(["'/drone_' + str(", drone_id, ") + '_", odom_topic, "'"])},
            {'use_target_world_as_source': simulated_light_use_target_world},
            {'light_source_relative_to_start': True},
            {'light_source_offset_xyz': [0.31, -1.625, 0.85]},
            {'publish_costs': True},
        ],
    )
    
    simulator_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
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
    ld.add_action(use_cinematic_intent_cmd)
    ld.add_action(use_simulated_light_cmd)
    ld.add_action(simulated_light_use_target_world_cmd)
    ld.add_action(use_distinctive_trajs_cmd)
    ld.add_action(default_max_vel_cmd)
    ld.add_action(default_max_acc_cmd)
    ld.add_action(focus_max_vel_cmd)
    ld.add_action(focus_max_acc_cmd)
    ld.add_action(default_speed_scale_cmd)
    ld.add_action(focus_speed_scale_cmd)
    ld.add_action(vel_damping_cmd)
    ld.add_action(acc_damping_cmd)
    ld.add_action(target_distance_m_cmd)
    ld.add_action(target_distance_tol_m_cmd)
    ld.add_action(target_distance_gate_m_cmd)
    ld.add_action(yawdot_max_cmd)
    ld.add_action(light_avoid_gain_cmd)
    ld.add_action(use_target_yaw_cmd)
    ld.add_action(use_light_avoid_yaw_cmd)
    ld.add_action(debug_light_yaw_cmd)
    ld.add_action(debug_light_period_s_cmd)

    # 添加 Map Generator 节点
    ld.add_action(map_generator_node)
    ld.add_action(mockamap_node)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(simulated_object_detector_node)
    ld.add_action(simulated_light_node)
    ld.add_action(cinematic_intent_node)
    ld.add_action(cinematic_bspline_filter_node)
    ld.add_action(simulator_include)

    return ld
