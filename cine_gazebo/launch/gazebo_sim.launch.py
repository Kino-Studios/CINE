"""Starts the Gazebo world with the CINE drone, ArduPilot SITL and the Pi's software.

The drone model (`airframe:=iris` or `source_one`) and the world are rendered from their templates into RENDER_DIRECTORY on
every launch: the camera and IMU noise from the `gazebo` profile of CINE-Sensing's config,
the wind from config/wind_profiles.yaml. ArduCopter SITL runs in lockstep with Gazebo through
ardupilot_gazebo's JSON interface, MAVROS and the CINE-Sensing nodes run as on the Pi
with the `gazebo` profile, and sitl_flight_starter arms and takes off. With `mission:=true`
the depth node and the ego-planner mission (cine_bringup's cinematic_run.launch.py, flying
config/mission_waypoints.yaml) start too, and sitl_flight_starter's trigger starts the
mission. Every ROS node runs on sim time from the bridged /clock, and `ros2 bag record` writes every run to
/bags/<date>_<nav_source>_<world>_<wind>.
"""

import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import cine_sensing_config
from ament_index_python.packages import get_package_share_directory
from cine_gazebo.flight_starter_logic import VALID_NAV_SOURCES
from cine_gazebo.model_rendering import (
    CAMERA_TOPIC,
    camera_spec_from_vins_camera,
    imu_noise_from_vins,
    load_wind_profiles,
    render_model,
    render_world,
)
from cine_gazebo.recording import bag_directory_name, recorded_topics
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

RENDER_DIRECTORY = Path('/tmp/cine_gazebo')
# airframe -> the model directory under models/ whose template is rendered. Either way the
# result lands in RENDER_DIRECTORY / WORLD_MODEL_NAME, the model the world template spawns
# (`model://cine_iris`, also the name in bridge.yaml's pose topic), so the world and the
# bridge need no change per airframe.
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
BAGS_DIRECTORY = Path('/bags')
PLANNER_DRONE_ID = 0
DEPTH_MODEL_DIRECTORY = WEIGHTS_DIRECTORY / 'depth-anything' / 'DA3METRIC-LARGE'
# The Gazebo camera is 1152x648 with this focal length (the gazebo profile's camera); the
# planner's buffer takes 640x360, so the depth node resizes to that and the planner is told
# the intrinsics at that size: scaled_intrinsics(497.5985575, 497.5985575, 576, 324, 1152, 640).
SIMULATED_CAMERA_FOCAL_PX = 497.5985575
DEPTH_OUTPUT_WIDTH = 640
DEPTH_OUTPUT_HEIGHT = 360
PLANNER_INTRINSICS = {
    'planner_fx': '276.444',
    'planner_fy': '276.444',
    'planner_cx': '320.0',
    'planner_cy': '180.0',
}
# The planner's map is centred on the origin in x and y (the drone spawns there, and the
# EKF origin is the takeoff point) and spans z from -0.01 m up; the ceiling is an occupied
# layer. Together they contain the world's obstacles (x -8.8 to 9.8, y 0 to 23.8) and every
# waypoint in config/mission_waypoints.yaml (z up to 3.0) with at least 1 m to spare.
PLANNER_MAP_SIZE_X = 30.0
PLANNER_MAP_SIZE_Y = 56.0
PLANNER_MAP_SIZE_Z = 5.0
PLANNER_VIRTUAL_CEILING = 4.5
ODOMETRY_TOPICS = {'sim': '/mavros/local_position/odom', 'vins': '/vio/odometry'}


def _camera_rate_hz():
    return float(cine_sensing_config.load('camera')['camera_publisher']['ros__parameters']['fps'])


def _render_model(share_directory, airframe):
    if airframe not in AIRFRAME_MODELS:
        raise ValueError(f"Unknown airframe '{airframe}'; choose one of {sorted(AIRFRAME_MODELS)}")

    camera = camera_spec_from_vins_camera(
        cine_sensing_config.load('vins_camera'), _camera_rate_hz()
    )
    imu_noise = imu_noise_from_vins(cine_sensing_config.load('vins')['vins_config'])
    template_directory = share_directory / 'models' / AIRFRAME_MODELS[airframe]
    model_directory = RENDER_DIRECTORY / WORLD_MODEL_NAME
    model_directory.mkdir(parents=True, exist_ok=True)
    rendered = render_model(
        (template_directory / 'model.sdf.template').read_text(), camera, imu_noise
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

    _render_model(share_directory, LaunchConfiguration('airframe').perform(context))
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
                }
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
            f"nav_source '{nav_source}' needs {parameter_file}, which does not exist"
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
    mavros_script = (
        Path(get_package_share_directory('cine_sensing_imu')) / 'launch' / 'start_mavros.sh'
    )

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
        ExecuteProcess(
            cmd=['bash', str(mavros_script)],
            additional_env={'FCU_URL': MAVROS_FCU_URL, 'USE_SIM_TIME': 'true'},
            name='mavros',
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

    return [
        # Recording starts for every run, the smoke flight included. A topic nobody
        # publishes yet is recorded as soon as it appears. `ros2 bag record` has no
        # `use_sim_time:=true` parameter form; --use-sim-time is its flag for the same thing.
        ExecuteProcess(
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
            'instead of letting Ultralytics download an unpinned copy'
        )


def _require_depth_weights():
    if not DEPTH_MODEL_DIRECTORY.exists():
        raise RuntimeError(
            f'{DEPTH_MODEL_DIRECTORY} is missing; fetch the pinned weights with '
            'fetch_weights.py instead of letting Hugging Face download an unpinned copy'
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
    if LaunchConfiguration('mission').perform(context).lower() != 'true':
        return []

    _require_depth_weights()
    share_directory = Path(get_package_share_directory('cine_gazebo'))
    odometry_topic = ODOMETRY_TOPICS[LaunchConfiguration('nav_source').perform(context)]
    cinematic_run_launch = (
        Path(get_package_share_directory('cine_bringup')) / 'launch' / 'cinematic_run.launch.py'
    )

    return [
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
                }
            ],
            additional_env={'PYTHONNOUSERSITE': '1', **_virtual_env_site_packages()},
            output='screen',
        ),
        Node(
            package='cine_agent',
            executable='camera_pose_publisher',
            name='camera_pose_publisher',
            parameters=[
                {
                    'drone_id': PLANNER_DRONE_ID,
                    'odometry_topic': odometry_topic,
                    'use_sim_time': True,
                }
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
                        'odometry_topic': odometry_topic,
                        'waypoints_file': str(
                            share_directory / 'config' / 'mission_waypoints.yaml'
                        ),
                        'drone_id': str(PLANNER_DRONE_ID),
                        'depth_topic': '/depth/image_raw',
                        'map_size_x': str(PLANNER_MAP_SIZE_X),
                        'map_size_y': str(PLANNER_MAP_SIZE_Y),
                        'map_size_z': str(PLANNER_MAP_SIZE_Z),
                        'planner_virtual_ceil_height': str(PLANNER_VIRTUAL_CEILING),
                        **PLANNER_INTRINSICS,
                    }.items(),
                )
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
                    'true starts the depth node and the planner mission, which '
                    'sitl_flight_starter triggers once the drone is up; false flies the '
                    'smoke flight only.'
                ),
            ),
            # Every node started after this, including those in included launch files,
            # runs on the bridged /clock.
            SetParameter(name='use_sim_time', value=True),
            # Set before rendering: the camera is read through this profile, and later
            # CINE-Sensing nodes started from this launch inherit it.
            SetEnvironmentVariable(
                cine_sensing_config.PROFILE_ENVIRONMENT_VARIABLE, SIMULATION_PROFILE
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
        ]
    )
