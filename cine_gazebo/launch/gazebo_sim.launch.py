"""Starts the Gazebo world with the CINE drone, ArduPilot SITL and the Pi's software.

Each launch renders the drone model (`airframe:=iris` or `source_one`) and world into
RENDER_DIRECTORY, with camera and IMU noise from CINE-Sensing's `gazebo` profile and wind from
config/wind_profiles.yaml. ArduCopter SITL runs in lockstep through ardupilot_gazebo's JSON
interface; MAVROS and the CINE-Sensing nodes run as on the Pi; sitl_flight_starter arms and
takes off. `mission:=true` adds the depth source (`depth_source`), the ego-planner mission
(cinematic_run.launch.py), odometry_relay under nav_source sim, and the filming cue from the
cinematic waypoint generator. Nodes run on the bridged /clock; `ros2 bag record` writes each run
to /bags/<date>_<nav_source>_<world>_<wind>, with run.json, the scorer's record, beside it.
"""

import os
import shutil
import subprocess
import sys
import threading
from datetime import datetime
from pathlib import Path

import cine_sensing_config
from ament_index_python.packages import get_package_share_directory
from cine_bringup.mission_waypoints import load_mission_waypoints
from cine_gazebo.depth_source import depth_routing
from cine_gazebo.flight_starter_logic import VALID_NAV_SOURCES
from cine_gazebo.model_rendering import (
    CAMERA_TOPIC,
    DEPTH_TOPIC,
    DepthCameraSpec,
    camera_spec_from_vins_camera,
    imu_noise_from_vins,
    load_wind_profiles,
    render_model,
    render_world,
)
from cine_gazebo.odometry_routing import mission_odometry
from cine_gazebo.recording import bag_directory_name, recorded_topics
from cine_gazebo.run_metadata import (
    build_run_metadata,
    directory_sha1,
    file_sha1,
    git_state_from_environment,
    parse_seed,
    write_run_metadata_when_directory_exists,
)
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

RENDER_DIRECTORY = Path('/tmp/cine_gazebo')
# airframe -> the model directory under models/ whose template is rendered. Both land in
# RENDER_DIRECTORY / WORLD_MODEL_NAME (`model://cine_iris`, also bridge.yaml's pose topic name),
# so the world and the bridge need no change per airframe.
AIRFRAME_MODELS = {'iris': 'cine_iris', 'source_one': 'cine_source_one'}
WORLD_MODEL_NAME = 'cine_iris'
SIMULATION_PROFILE = 'gazebo'
SUBJECT_POSE_RATE_HZ = 30.0
# Where docker/gazebo-sim/Dockerfile builds SITL and fetch_weights.py puts the weights.
ARDUPILOT_DIRECTORY = Path('/opt/ardupilot')
ARDUCOPTER_BINARY = ARDUPILOT_DIRECTORY / 'build' / 'sitl' / 'bin' / 'arducopter'
# The defaults sim_vehicle.py loads for `-f gazebo-iris` (Tools/autotest/pysim/vehicleinfo.py).
STOCK_PARAMETER_FILES = (
    ARDUPILOT_DIRECTORY / 'Tools' / 'autotest' / 'default_params' / 'copter.parm',
    ARDUPILOT_DIRECTORY / 'Tools' / 'autotest' / 'default_params' / 'gazebo-iris.parm',
)
SITL_DIRECTORY = RENDER_DIRECTORY / 'sitl'
VINS_CONFIG_PATH = RENDER_DIRECTORY / 'vins_config.yaml'
WEIGHTS_DIRECTORY = Path('/weights')
MAVROS_FCU_URL = 'tcp://127.0.0.1:5760'
# MAVROS times the heartbeat on the wall clock while the autopilot sends one per simulated
# second, so the gap is 1 / real-time factor. 60 s holds the link down to factor 0.017; runs
# have measured 0.07 at worst.
MAVROS_HEARTBEAT_TIMEOUT_S = 60.0
BAGS_DIRECTORY = Path('/bags')
# How long run.json waits for `ros2 bag record` to create its output directory.
RUN_METADATA_WAIT_S = 120.0
RECORDED_LAUNCH_ARGUMENTS = (
    'headless',
    'world',
    'wind',
    'nav_source',
    'airframe',
    'mission',
    'depth_source',
    'scenario',
    'seed',
)
PLANNER_DRONE_ID = 0
DEPTH_MODEL_DIRECTORY = WEIGHTS_DIRECTORY / 'depth-anything' / 'DA3METRIC-LARGE'
# The Gazebo camera is 1152x648 with this focal length (the gazebo profile's camera); the
# planner's buffer takes 640x360, so the depth node resizes to that and the planner is told
# the intrinsics at that size: scaled_intrinsics(497.5985575, 497.5985575, 576, 324, 1152, 640).
SIMULATED_CAMERA_FOCAL_PX = 497.5985575
DEPTH_OUTPUT_WIDTH = 640
DEPTH_OUTPUT_HEIGHT = 360
# The depth camera renders at the depth node's output size. 10 Hz is assumed, not measured: a
# third of the RGB render cost and still several map updates inside PLANNER_ODOM_DEPTH_TIMEOUT.
SIMULATED_DEPTH_CAMERA = DepthCameraSpec(
    width=DEPTH_OUTPUT_WIDTH,
    height=DEPTH_OUTPUT_HEIGHT,
    rate_hz=10.0,
)
PLANNER_INTRINSICS = {
    'planner_fx': '276.444',
    'planner_fy': '276.444',
    'planner_cx': '320.0',
    'planner_cy': '180.0',
}
# The cost calculator and the waypoint generator read detection pixels in the full-resolution
# camera frame; the cost calculator scales them to the depth image itself.
DETECTION_CAMERA = {
    'detection_camera_fx': '497.5985575',
    'detection_camera_fy': '497.5985575',
    'detection_camera_cx': '576.0',
    'detection_camera_cy': '324.0',
    'detection_image_width': '1152',
    'detection_image_height': '648',
}
# The planner's map is centred on the origin in x and y (the drone spawns there, and the
# EKF origin is the takeoff point) and spans z from -0.01 m up; the ceiling is an occupied
# layer. Together they contain the world's obstacles (x -8.8 to 9.8, y 0 to 23.8) and every
# waypoint in config/mission_waypoints.yaml (z up to 3.0) with at least 1 m to spare.
PLANNER_MAP_SIZE_X = 30.0
PLANNER_MAP_SIZE_Y = 56.0
PLANNER_MAP_SIZE_Z = 5.0
PLANNER_VIRTUAL_CEILING = 4.5
# The planner stops the drone when its occupancy map goes this long without a depth and
# pose update. Upstream's 1.0 s is shorter than the 1.7 s simulated gap measured between
# map updates while the depth network runs on CPU, so the simulator allows 3.0 s.
PLANNER_ODOM_DEPTH_TIMEOUT = 3.0


def _camera_rate_hz():
    return float(cine_sensing_config.load('camera')['camera_publisher']['ros__parameters']['fps'])


def _render_model(share_directory, airframe, depth_camera):
    if airframe not in AIRFRAME_MODELS:
        raise ValueError(f"Unknown airframe '{airframe}'; choose one of {sorted(AIRFRAME_MODELS)}")

    camera = camera_spec_from_vins_camera(
        cine_sensing_config.load('vins_camera'),
        _camera_rate_hz(),
    )
    imu_noise = imu_noise_from_vins(cine_sensing_config.load('vins')['vins_config'])
    template_directory = share_directory / 'models' / AIRFRAME_MODELS[airframe]
    model_directory = RENDER_DIRECTORY / WORLD_MODEL_NAME
    model_directory.mkdir(parents=True, exist_ok=True)
    rendered = render_model(
        (template_directory / 'model.sdf.template').read_text(),
        camera,
        imu_noise,
        depth_camera,
    )
    (model_directory / 'model.sdf').write_text(rendered)
    shutil.copyfile(template_directory / 'model.config', model_directory / 'model.config')
    # Replace rather than merge, so switching airframes never leaves the other model's
    # meshes behind under the shared WORLD_MODEL_NAME directory.
    shutil.rmtree(model_directory / 'meshes', ignore_errors=True)

    if (template_directory / 'meshes').is_dir():
        shutil.copytree(template_directory / 'meshes', model_directory / 'meshes')


def _render_world(share_directory, world_name, wind_name):
    wind_profiles = load_wind_profiles(share_directory / 'config' / 'wind_profiles.yaml')
    if wind_name not in wind_profiles:
        raise ValueError(f"Unknown wind '{wind_name}'; choose one of {sorted(wind_profiles)}")

    template_path = share_directory / 'worlds' / f'{world_name}.sdf.template'
    if not template_path.is_file():
        raise FileNotFoundError(f"No world '{world_name}': {template_path} does not exist")

    world_path = RENDER_DIRECTORY / f'{world_name}.sdf'
    world_path.write_text(render_world(template_path.read_text(), wind_profiles[wind_name]))

    return world_path


def _start_gazebo(context):
    share_directory = Path(get_package_share_directory('cine_gazebo'))
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    world_name = LaunchConfiguration('world').perform(context)
    wind_name = LaunchConfiguration('wind').perform(context)

    depth = _depth_routing(context)
    _render_model(
        share_directory,
        LaunchConfiguration('airframe').perform(context),
        SIMULATED_DEPTH_CAMERA if depth.renders_simulator_depth else None,
    )
    world_path = _render_world(share_directory, world_name, wind_name)

    server_flags = '-s -r' if headless else '-r'
    gz_sim_launch = Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_sim_launch)),
            launch_arguments={
                'gz_args': f'{server_flags} {world_path}',
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(
            package='cine_gazebo',
            executable='subject_pose_publisher',
            name='subject_pose_publisher',
            parameters=[
                {
                    'world_file': str(world_path),
                    'publish_rate_hz': SUBJECT_POSE_RATE_HZ,
                    'use_sim_time': True,
                },
            ],
            output='screen',
        ),
    ]


def _position_parameter_file(share_directory, nav_source):
    if nav_source not in VALID_NAV_SOURCES:
        raise ValueError(f"Unknown nav_source '{nav_source}'; choose one of {VALID_NAV_SOURCES}")

    parameter_file = share_directory / 'config' / 'ardupilot' / f'{nav_source}_position.parm'
    if not parameter_file.is_file():
        raise FileNotFoundError(
            f"nav_source '{nav_source}' needs {parameter_file}, which does not exist",
        )

    return parameter_file


def _airframe_parameter_files(share_directory, airframe):
    if airframe == 'iris':
        return ()

    return (share_directory / 'config' / 'ardupilot' / f'{airframe}.parm',)


def _start_vehicle(context):
    share_directory = Path(get_package_share_directory('cine_gazebo'))
    nav_source = LaunchConfiguration('nav_source').perform(context)
    airframe = LaunchConfiguration('airframe').perform(context)
    parameter_file = _position_parameter_file(share_directory, nav_source)
    # Later files win. The airframe's file comes last so its values override the stock
    # Iris defaults; the position file sets none of the same parameters.
    parameter_files = (
        *STOCK_PARAMETER_FILES,
        parameter_file,
        *_airframe_parameter_files(share_directory, airframe),
    )
    defaults = ','.join(str(path) for path in parameter_files)
    # SITL keeps its parameters and logs in its working directory.
    SITL_DIRECTORY.mkdir(parents=True, exist_ok=True)
    mavros_launch_directory = Path(get_package_share_directory('mavros')) / 'launch'

    return [
        # No MAVProxy: MAVROS is the only client of SERIAL0. --wipe drops parameters a
        # previous run saved, so the files above are always what SITL flies with.
        ExecuteProcess(
            cmd=[
                str(ARDUCOPTER_BINARY),
                '--model',
                'JSON',
                '--speedup',
                '1',
                '--slave',
                '0',
                '--defaults',
                defaults,
                '--wipe',
                '-I0',
            ],
            cwd=str(SITL_DIRECTORY),
            name='arducopter',
            output='screen',
        ),
        # Started as a Node, not through apm.launch: that file declares no use_sim_time
        # argument, so MAVROS kept stamping its IMU and odometry with wall time. The
        # parameters are the ones apm.launch and node.launch give it.
        Node(
            package='mavros',
            executable='mavros_node',
            parameters=[
                {
                    'fcu_url': MAVROS_FCU_URL,
                    'gcs_url': 'udp://@',
                    'tgt_system': 1,
                    'tgt_component': 1,
                    'fcu_protocol': 'v2.0',
                    'use_sim_time': True,
                },
                str(mavros_launch_directory / 'apm_pluginlists.yaml'),
                str(mavros_launch_directory / 'apm_config.yaml'),
                {'conn_timeout': MAVROS_HEARTBEAT_TIMEOUT_S},
            ],
            output='screen',
        ),
        # The drone's launch starts the same node: one IMU rate, from the JSON constants.
        Node(
            package='cine_sensing_imu',
            executable='imu_rate_requester',
            name='imu_rate_requester',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='cine_gazebo',
            executable='sitl_flight_starter',
            name='sitl_flight_starter',
            parameters=[{'nav_source': nav_source, 'use_sim_time': True}],
            output='screen',
        ),
    ]


def _mission_enabled(context):
    return LaunchConfiguration('mission').perform(context).lower() == 'true'


def _depth_routing(context):
    return depth_routing(
        LaunchConfiguration('depth_source').perform(context),
        _mission_enabled(context),
    )


def _mission_waypoints_file(share_directory):
    return share_directory / 'config' / 'mission_waypoints.yaml'


def _run_metadata(context):
    # Imported here because cine_config.constants reads the JSON config tree when imported,
    # which would break loading this launch file wherever that tree is missing.
    from cine_config.constants import UserRequestedParams

    from cine_config import config_root

    share_directory = Path(get_package_share_directory('cine_gazebo'))
    waypoints_file = _mission_waypoints_file(share_directory)
    waypoints = load_mission_waypoints(str(waypoints_file))

    return build_run_metadata(
        scenario=LaunchConfiguration('scenario').perform(context),
        world=LaunchConfiguration('world').perform(context),
        wind=LaunchConfiguration('wind').perform(context),
        seed=parse_seed(LaunchConfiguration('seed').perform(context)),
        nav_source=LaunchConfiguration('nav_source').perform(context),
        waypoints=waypoints,
        waypoints_sha1=file_sha1(waypoints_file),
        focus_target_m=UserRequestedParams.DESIRED_FOCUS_DISTANCE_M,
        git=git_state_from_environment(os.environ),
        launch_arguments={
            name: LaunchConfiguration(name).perform(context) for name in RECORDED_LAUNCH_ARGUMENTS
        },
        config_sha1=directory_sha1(config_root()),
    )


def _write_run_metadata_in_background(bag_directory, metadata):
    def write():
        try:
            write_run_metadata_when_directory_exists(
                bag_directory,
                metadata,
                timeout_s=RUN_METADATA_WAIT_S,
            )
        except OSError as exception:
            # The run still flies and records; only its scorer inputs are missing.
            print(f'[gazebo_sim] run.json was not written: {exception}', file=sys.stderr)

    # Daemon: a launch that is shutting down must not wait for the poll.
    threading.Thread(target=write, name='run_metadata_writer', daemon=True).start()

    return []


def _start_recording(context):
    bag_name = bag_directory_name(
        datetime.now(),
        LaunchConfiguration('nav_source').perform(context),
        LaunchConfiguration('world').perform(context),
        LaunchConfiguration('wind').perform(context),
    )
    # The planner's topic is remapped without a leading slash; `ros2 bag record` wants
    # absolute names.
    topics = [
        topic if topic.startswith('/') else f'/{topic}'
        for topic in recorded_topics(PLANNER_DRONE_ID)
    ]

    # Recording starts for every run, the smoke flight included. A topic nobody
    # publishes yet is recorded as soon as it appears. `ros2 bag record` has no
    # `use_sim_time:=true` parameter form; --use-sim-time is its flag for the same thing.
    recorder = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '--use-sim-time',
            '--output',
            str(BAGS_DIRECTORY / bag_name),
            *topics,
        ],
        name='bag_recorder',
        output='screen',
    )

    if not _mission_enabled(context):
        return [recorder]

    # The recorder refuses an existing output directory, so run.json is written after it starts.
    metadata = _run_metadata(context)

    return [
        recorder,
        RegisterEventHandler(
            OnProcessStart(
                target_action=recorder,
                on_start=[
                    OpaqueFunction(
                        function=lambda _context: _write_run_metadata_in_background(
                            BAGS_DIRECTORY / bag_name,
                            metadata,
                        ),
                    ),
                ],
            ),
        ),
    ]


def _require_pinned_weights():
    # Ultralytics downloads a bare model name it cannot find, which would replace the
    # hash-checked weights with an unpinned file; fail instead.
    model_name = cine_sensing_config.load('user_params')['ModelDetectionConstants']['MODEL_PATH']
    weights_file = WEIGHTS_DIRECTORY / model_name
    if not weights_file.is_file():
        raise RuntimeError(
            f'{weights_file} is missing; fetch the pinned weights with fetch_weights.py '
            'instead of letting Ultralytics download an unpinned copy',
        )


def _require_depth_weights():
    if not DEPTH_MODEL_DIRECTORY.exists():
        raise RuntimeError(
            f'{DEPTH_MODEL_DIRECTORY} is missing; fetch the pinned weights with '
            'fetch_weights.py instead of letting Hugging Face download an unpinned copy',
        )


def _virtual_env_site_packages():
    # Same reason as cine_depth's depth_publisher.launch.py: the node's shebang is the system
    # Python, which cannot see torch and depth_anything_3 installed in the workspace venv.
    virtual_env = os.environ.get('VIRTUAL_ENV')
    if not virtual_env:
        return {}

    python_version = f'python{sys.version_info.major}.{sys.version_info.minor}'
    site_packages = Path(virtual_env) / 'lib' / python_version / 'site-packages'
    existing_path = os.environ.get('PYTHONPATH', '')

    return {'PYTHONPATH': os.pathsep.join(filter(None, [str(site_packages), existing_path]))}


def _start_mission(context):
    if not _mission_enabled(context):
        return []

    depth = _depth_routing(context)
    if depth.starts_depth_network:
        _require_depth_weights()

    share_directory = Path(get_package_share_directory('cine_gazebo'))
    odometry = mission_odometry(LaunchConfiguration('nav_source').perform(context))
    cinematic_run_launch = (
        Path(get_package_share_directory('cine_bringup')) / 'launch' / 'cinematic_run.launch.py'
    )

    relay_nodes = (
        [
            Node(
                package='cine_gazebo',
                executable='odometry_relay',
                name='odometry_relay',
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        ]
        if odometry.starts_relay
        else []
    )

    depth_nodes = []

    if depth.starts_depth_network:
        depth_nodes.append(
            Node(
                package='cine_depth',
                executable='depth_publisher_node',
                name='depth_publisher_node',
                parameters=[
                    {
                        'model_id': str(DEPTH_MODEL_DIRECTORY),
                        'output_width': DEPTH_OUTPUT_WIDTH,
                        'output_height': DEPTH_OUTPUT_HEIGHT,
                        'focal_length_px': SIMULATED_CAMERA_FOCAL_PX,
                        'use_sim_time': True,
                    },
                ],
                additional_env={'PYTHONNOUSERSITE': '1', **_virtual_env_site_packages()},
                output='screen',
            ),
        )

    if depth.renders_simulator_depth:
        depth_nodes.append(
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                name='depth_image_bridge',
                arguments=[f'/{DEPTH_TOPIC}'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        )

    return [
        *relay_nodes,
        *depth_nodes,
        Node(
            package='cine_agent',
            executable='cinematic_waypoint_generator',
            name='cinematic_waypoint_generator',
            parameters=[
                {
                    'waypoints_file': str(_mission_waypoints_file(share_directory)),
                    'odom_topic': odometry.odometry_topic,
                    'publish_subject_position': False,
                    'subject_wait_timeout_s': 30.0,
                    'start_trigger_topic': '/traj_start_trigger',
                    'waypoint_xy_scale': 1.0,
                    'waypoint_y_extra_scale': 1.0,
                    'waypoint_z_scale': 1.0,
                    'world_bounds_min': [-10.0, -10.0, 0.0],
                    'world_bounds_max': [10.0, 20.0, 3.5],
                    'use_sim_time': True,
                    'simulate_object': False,
                    'camera_fx': float(DETECTION_CAMERA['detection_camera_fx']),
                    'camera_fy': float(DETECTION_CAMERA['detection_camera_fy']),
                    'camera_cx': float(DETECTION_CAMERA['detection_camera_cx']),
                    'camera_cy': float(DETECTION_CAMERA['detection_camera_cy']),
                    'image_width': int(DETECTION_CAMERA['detection_image_width']),
                    'image_height': int(DETECTION_CAMERA['detection_image_height']),
                },
            ],
            output='screen',
        ),
        Node(
            package='cine_agent',
            executable='camera_pose_publisher',
            name='camera_pose_publisher',
            parameters=[
                {
                    'drone_id': PLANNER_DRONE_ID,
                    'odometry_topic': odometry.odometry_topic,
                    'use_sim_time': True,
                },
            ],
            output='screen',
        ),
        Node(
            package='cine_agent',
            executable='planner_setpoint_adapter',
            name='planner_setpoint_adapter',
            parameters=[{'drone_id': PLANNER_DRONE_ID, 'use_sim_time': True}],
            output='screen',
        ),
        GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(cinematic_run_launch)),
                    launch_arguments={
                        'use_simulation': 'false',
                        'use_sim_time': 'true',
                        # The correction node needs onnxruntime and pandas, which are not
                        # dependencies of this simulator.
                        'use_mlp_correction': 'false',
                        'odometry_topic': odometry.odometry_topic,
                        'cost_calculator_odometry_topic': odometry.cost_calculator_odometry_topic,
                        'waypoints_file': str(_mission_waypoints_file(share_directory)),
                        'drone_id': str(PLANNER_DRONE_ID),
                        'depth_topic': '/depth/image_raw',
                        'map_size_x': str(PLANNER_MAP_SIZE_X),
                        'map_size_y': str(PLANNER_MAP_SIZE_Y),
                        'map_size_z': str(PLANNER_MAP_SIZE_Z),
                        'planner_virtual_ceil_height': str(PLANNER_VIRTUAL_CEILING),
                        'planner_odom_depth_timeout': str(PLANNER_ODOM_DEPTH_TIMEOUT),
                        **PLANNER_INTRINSICS,
                        **DETECTION_CAMERA,
                        # It would publish a second /cinematography/costs stream and its own goals.
                        'start_cinematic_intent_node': 'false',
                        # The generator's goals on /move_base_simple/goal steer the planner
                        # only in manual-target mode.
                        'flight_type': '1',
                    }.items(),
                ),
            ],
            scoped=True,
        ),
    ]


def _start_sensing(context):
    nav_source = LaunchConfiguration('nav_source').perform(context)
    _require_pinned_weights()
    # Same generator complete_vio.launch.py runs; it reads CINE_SENSING_PROFILE, set above.
    generator_script = (
        Path(get_package_share_directory('vins')) / 'launch' / 'generate_vins_config.py'
    )
    RENDER_DIRECTORY.mkdir(parents=True, exist_ok=True)
    subprocess.run(['python3', str(generator_script), str(VINS_CONFIG_PATH)], check=True)
    imu_bridge_launch = (
        Path(get_package_share_directory('cine_sensing_imu')) / 'launch' / 'imu_bridge.launch.py'
    )

    # complete_vio.launch.py is not reused: it also starts the Pi camera driver, which
    # Gazebo's camera replaces.

    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(imu_bridge_launch))),
        Node(
            package='vins',
            executable='vins_node',
            name='vins_estimator',
            arguments=[str(VINS_CONFIG_PATH)],
            output='screen',
        ),
        Node(
            package='cine_sensing_imu',
            executable='vio_publisher',
            name='vio_publisher',
            parameters=[{'forward_to_autopilot': nav_source == 'vins'}],
            output='screen',
        ),
        # The gazebo profile names the weights by file name only; Ultralytics looks for a
        # bare name in the working directory, checked above, before downloading it.
        Node(
            package='cine_sensing_yolo',
            executable='yolo_camera',
            name='yolo_camera',
            cwd=str(WEIGHTS_DIRECTORY),
            output='screen',
        ),
        Node(
            package='cine_sensing_yolo',
            executable='bb_extractor',
            name='bb_extractor',
            # bb_extractor loads the same bare model name.
            cwd=str(WEIGHTS_DIRECTORY),
            output='screen',
        ),
        Node(
            package='cine_sensing_setpoint',
            executable='mavros_setpoints',
            name='mavros_setpoints',
            output='screen',
        ),
    ]


def generate_launch_description():
    bridge_config = Path(get_package_share_directory('cine_gazebo')) / 'config' / 'bridge.yaml'

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description='true runs the Gazebo server only, without the window.',
            ),
            DeclareLaunchArgument(
                'world',
                default_value='cine_field',
                description='World to load: worlds/<world>.sdf.template in this package.',
            ),
            DeclareLaunchArgument(
                'wind',
                default_value='calm',
                description='Wind profile from config/wind_profiles.yaml.',
            ),
            DeclareLaunchArgument(
                'nav_source',
                default_value='sim',
                description=(
                    'sim flies on the simulated GPS; vins hands navigation to VINS in flight. '
                    'Loads config/ardupilot/<nav_source>_position.parm.'
                ),
            ),
            DeclareLaunchArgument(
                'airframe',
                default_value='iris',
                description=(
                    'iris flies the stock Iris airframe; source_one flies the real TBS Source '
                    'One with its mass, inertia and thrust, and loads '
                    'config/ardupilot/source_one.parm after the other parameter files.'
                ),
            ),
            DeclareLaunchArgument(
                'mission',
                default_value='true',
                description=(
                    'true starts the depth source and the planner mission, which '
                    'sitl_flight_starter triggers once the drone is up; false flies the '
                    'smoke flight only.'
                ),
            ),
            DeclareLaunchArgument(
                'depth_source',
                default_value='gazebo',
                description=(
                    'gazebo feeds the planner exact depth from a depth camera in the drone '
                    'model; network runs the depth network on the simulated RGB image, as the '
                    'drone does. Read only when mission is true.'
                ),
            ),
            DeclareLaunchArgument(
                'scenario',
                default_value='cine_field_mission',
                description=(
                    'Name written to run.json as the scenario, which the scorer pairs runs on '
                    'with the world, the waypoints, the wind and the seed.'
                ),
            ),
            DeclareLaunchArgument(
                'seed',
                default_value='',
                description=(
                    'Integer written to run.json as the seed; empty writes null. Nothing in '
                    'the simulator consumes it yet: it labels the run for pairing.'
                ),
            ),
            # Every node started after this, including those in included launch files,
            # runs on the bridged /clock.
            SetParameter(name='use_sim_time', value=True),
            # Set before rendering: the camera is read through this profile, and later
            # CINE-Sensing nodes started from this launch inherit it.
            SetEnvironmentVariable(
                cine_sensing_config.PROFILE_ENVIRONMENT_VARIABLE,
                SIMULATION_PROFILE,
            ),
            AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(RENDER_DIRECTORY), prepend=True),
            OpaqueFunction(function=_start_gazebo),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gazebo_bridge',
                parameters=[{'config_file': str(bridge_config), 'use_sim_time': True}],
                output='screen',
            ),
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                name='camera_image_bridge',
                arguments=[f'/{CAMERA_TOPIC}'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
            OpaqueFunction(function=_start_vehicle),
            OpaqueFunction(function=_start_sensing),
            OpaqueFunction(function=_start_mission),
            OpaqueFunction(function=_start_recording),
        ],
    )
