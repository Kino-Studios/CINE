import ast
from pathlib import Path

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'gazebo_sim.launch.py'


def _launched_node_names():
    tree = ast.parse(LAUNCH_FILE.read_text())

    return {
        keyword.value.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'Node'
        for keyword in node.keywords
        if keyword.arg == 'name' and isinstance(keyword.value, ast.Constant)
    }


def test_simulator_starts_contract_nodes_under_their_contract_names():
    # The live contract check credits a topic only to the node name CINE-Interfaces lists.
    assert {
        'depth_publisher_node',
        'camera_pose_publisher',
        'planner_setpoint_adapter',
        'vins_estimator',
        'vio_publisher',
        'bb_extractor',
        'mavros_setpoints',
    } <= _launched_node_names()


def _mavros_node_parameters():
    tree = ast.parse(LAUNCH_FILE.read_text())

    return [
        dictionary
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, 'id', None) == 'Node'
        and any(
            keyword.arg == 'package'
            and isinstance(keyword.value, ast.Constant)
            and keyword.value.value == 'mavros'
            for keyword in node.keywords
        )
        for keyword in node.keywords
        if keyword.arg == 'parameters'
        for dictionary in ast.walk(keyword.value)
        if isinstance(dictionary, ast.Dict)
    ]


def test_mavros_node_runs_on_simulator_time():
    # MAVROS stamps its IMU and odometry with its own clock. Without use_sim_time they carry
    # wall time and the visual-inertial estimator never initializes.
    stamped_with_sim_time = [
        True
        for dictionary in _mavros_node_parameters()
        for key, value in zip(dictionary.keys, dictionary.values, strict=True)
        if getattr(key, 'value', None) == 'use_sim_time' and getattr(value, 'value', None) is True
    ]

    assert stamped_with_sim_time == [True]


def test_mavros_node_keeps_the_names_of_the_nodes_it_hosts():
    # mavros_node hosts two nodes, mavros_router and mavros. A launch-level name renames every
    # node in the process, so both would be called mavros. node.launch sets no name either.
    renamed = [
        keyword.arg
        for node in ast.walk(ast.parse(LAUNCH_FILE.read_text()))
        if isinstance(node, ast.Call)
        and getattr(node.func, 'id', None) == 'Node'
        and any(
            keyword.arg == 'package'
            and isinstance(keyword.value, ast.Constant)
            and keyword.value.value == 'mavros'
            for keyword in node.keywords
        )
        for keyword in node.keywords
        if keyword.arg == 'name'
    ]

    assert renamed == []


def test_simulator_requests_the_imu_rate_on_simulator_time():
    tree = ast.parse(LAUNCH_FILE.read_text())

    requester_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, 'id', None) == 'Node'
        and any(
            keyword.arg == 'executable'
            and getattr(keyword.value, 'value', None) == 'imu_rate_requester'
            for keyword in node.keywords
        )
    ]

    assert [ast.unparse(call) for call in requester_calls if 'use_sim_time' in ast.unparse(call)]


def test_mavros_waits_out_heartbeat_gaps_at_the_slowest_real_time_factor():
    # The autopilot sends one heartbeat per simulated second, but MAVROS times the link with the
    # wall clock. At real-time factor 0.08 heartbeats arrive 12.5 s apart, past the stock 10 s,
    # so the link flapped and the flight starter quit. The override must follow apm_config.yaml,
    # which also sets conn_timeout, because a later parameter file wins.
    mavros_parameter_lists = [
        keyword.value
        for node in ast.walk(ast.parse(LAUNCH_FILE.read_text()))
        if isinstance(node, ast.Call)
        and getattr(node.func, 'id', None) == 'Node'
        and any(
            keyword.arg == 'package'
            and isinstance(keyword.value, ast.Constant)
            and keyword.value.value == 'mavros'
            for keyword in node.keywords
        )
        for keyword in node.keywords
        if keyword.arg == 'parameters'
    ]
    last_parameters = mavros_parameter_lists[0].elts[-1]

    assert ast.unparse(last_parameters) == "{'conn_timeout': MAVROS_HEARTBEAT_TIMEOUT_S}"


def _node_call(executable):
    return next(
        (
            node
            for node in ast.walk(ast.parse(LAUNCH_FILE.read_text()))
            if isinstance(node, ast.Call)
            and getattr(node.func, 'id', None) == 'Node'
            and any(
                keyword.arg == 'executable' and getattr(keyword.value, 'value', None) == executable
                for keyword in node.keywords
            )
        ),
        None,
    )


def _node_parameters(node_call):
    parameters = next(
        keyword.value for keyword in node_call.keywords if keyword.arg == 'parameters'
    )
    dictionary = next(element for element in parameters.elts if isinstance(element, ast.Dict))

    return {key.value: ast.unparse(value) for key, value in zip(dictionary.keys, dictionary.values)}


def _cinematic_run_launch_arguments():
    include = next(
        node
        for node in ast.walk(ast.parse(LAUNCH_FILE.read_text()))
        if isinstance(node, ast.Call)
        and getattr(node.func, 'id', None) == 'IncludeLaunchDescription'
        and 'cinematic_run_launch' in ast.unparse(node)
    )
    arguments = next(
        keyword.value for keyword in include.keywords if keyword.arg == 'launch_arguments'
    )
    dictionary = next(node for node in ast.walk(arguments) if isinstance(node, ast.Dict))

    return {
        key.value: ast.unparse(value)
        for key, value in zip(dictionary.keys, dictionary.values)
        if isinstance(key, ast.Constant)
    }


def test_mission_starts_the_cinematic_waypoint_generator_with_unscaled_waypoints():
    node_call = _node_call('cinematic_waypoint_generator')

    assert node_call is not None
    assert _node_parameters(node_call) == {
        'waypoints_file': 'str(_mission_waypoints_file(share_directory))',
        'odom_topic': 'odometry.odometry_topic',
        'publish_subject_position': 'False',
        'subject_wait_timeout_s': '30.0',
        'start_trigger_topic': "'/traj_start_trigger'",
        'waypoint_xy_scale': '1.0',
        'waypoint_y_extra_scale': '1.0',
        'waypoint_z_scale': '1.0',
        'world_bounds_min': '[-10.0, -10.0, 0.0]',
        'world_bounds_max': '[10.0, 20.0, 3.5]',
        'use_sim_time': 'True',
        'simulate_object': 'False',
        'camera_fx': "float(DETECTION_CAMERA['detection_camera_fx'])",
        'camera_fy': "float(DETECTION_CAMERA['detection_camera_fy'])",
        'camera_cx': "float(DETECTION_CAMERA['detection_camera_cx'])",
        'camera_cy': "float(DETECTION_CAMERA['detection_camera_cy'])",
        'image_width': "int(DETECTION_CAMERA['detection_image_width'])",
        'image_height': "int(DETECTION_CAMERA['detection_image_height'])",
        'camera_mount_forward_m': "float(DETECTION_CAMERA['detection_camera_mount_forward_m'])",
    }


def test_mission_no_longer_starts_a_filming_cue_node():
    assert _node_call('filming_cue') is None


def test_mission_runs_planner_in_manual_target_mode():
    assert _cinematic_run_launch_arguments()['flight_type'] == "'1'"
