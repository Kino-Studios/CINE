from __future__ import annotations

import ast
import glob
import logging
import os
import shutil
import signal
import subprocess
import tempfile
import time
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from pathlib import Path
from threading import Event
from typing import Any

import rclpy
from cine_config.constants import SimulationParams, WaypointParams
from rclpy.node import Node

_logger = logging.getLogger(__name__)

_ROS_PACKAGE = 'cine_agent'

_SAFETY_CLEANUP_TARGETS = [
    'trajectory_data_logger',
    'simulated_focus_publisher',
    'cinematic_waypoint_generator',
    'cinematic_bspline_filter',
    'simulated_light_publisher',
    'cost_function_calculator',
    'simulated_object_detector',
    'traj_server',
    'ego_planner_node',
    'so3_quadrotor_simulator',
    'pcl_render_node',
    'random_forest',
]


@dataclass
class CollectionParams:
    odom_topic: str = ''
    baseline_topic: str = ''
    cinematic_topic: str = ''
    frame_id: str = 'world'
    target_xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    focus_depth_m: float = 1.0
    waypoints_topic: str = ''
    waypoints_file: str = ''
    drone_ns: str = ''


def _find_workspace_root(start: Path) -> Path:
    for parent in [start] + list(start.parents):
        if (parent / 'tools' / 'start_simulation.sh').is_file():
            return parent / 'tools'

    raise RuntimeError('Unable to locate tools/ (start_simulation.sh missing).')


def _cleanup_cyclonedds() -> None:
    for pattern in ['/tmp/cyclonedds*', os.path.expanduser('~/.local/share/cyclonedds*')]:
        for path in glob.glob(pattern):
            try:
                if os.path.isdir(path):
                    shutil.rmtree(path)
                else:
                    os.remove(path)
            except OSError:
                pass


def _ensure_display(env: dict[str, str]) -> None:
    if env.get('DISPLAY'):
        return
    if Path('/etc/resolv.conf').is_file():
        try:
            with open('/etc/resolv.conf', encoding='utf-8') as handle:
                for line in handle:
                    if line.startswith('nameserver'):
                        parts = line.split()
                        if len(parts) >= 2:
                            env['DISPLAY'] = f'{parts[1]}:0'

                            return
        except OSError:
            pass
    env['DISPLAY'] = ':0'


def _prepare_environment(logging_severity: str, ros_log_dir: Path | None = None) -> None:
    os.environ.setdefault('ROS_DOMAIN_ID', '0')
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    _ensure_display(os.environ)
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ['RCUTILS_LOGGING_SEVERITY'] = logging_severity.upper()
    if ros_log_dir is not None:
        os.environ['ROS_LOG_DIR'] = str(ros_log_dir)


def _parse_env_file(path: Path) -> None:
    try:
        contents = path.read_text(encoding='utf-8')
    except OSError:
        return
    for line in contents.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        if stripped.startswith('export '):
            stripped = stripped[len('export ') :].strip()
        if '=' not in stripped:
            continue
        key, value = stripped.split('=', 1)
        os.environ[key.strip()] = value.strip().strip('"').strip("'")


def _parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value

    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _parse_target_xyz(value: Any) -> Any:
    if isinstance(value, (list, tuple)):
        return [float(v) for v in value]
    if isinstance(value, str):
        text = value.strip()
        if text.startswith('[') and text.endswith(']'):
            try:
                parsed = ast.literal_eval(text)
            except (ValueError, SyntaxError):
                return value
            if isinstance(parsed, (list, tuple)):
                return [float(v) for v in parsed]

    return value


def _format_ros_value(value: Any) -> str:
    if isinstance(value, (list, tuple)):
        return '[' + ','.join(str(v) for v in value) + ']'

    return str(value)


def _resolve_param(
    node: Node,
    name: str,
    default: Any,
    env_key: str | None = None,
    parser: Callable[[Any], Any] | None = None,
) -> Any:
    value = node.get_parameter(name).value
    if value is None or value == '':
        if env_key:
            env_value = os.environ.get(env_key)
            if env_value not in (None, ''):
                value = env_value
            else:
                value = default
        else:
            value = default
    if parser is None:
        return value
    try:
        return parser(value)
    except (ValueError, TypeError, SyntaxError):
        return value


def _ros2_node_list() -> list[str]:
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            env=os.environ,
            check=False,
        )
        if result.returncode != 0:
            if result.stderr:
                _logger.warning('ros2 node list error: %s', result.stderr.strip())

            return []

        return [line.strip() for line in result.stdout.splitlines() if line.strip()]
    except (OSError, subprocess.SubprocessError) as error:
        _logger.warning('failed to run ros2 node list: %s', error)

        return []


def _wait_for_node(node_name: str, timeout_s: float = 30.0) -> bool:
    start_time = time.time()
    canonical_name = node_name if node_name.startswith('/') else '/' + node_name
    while time.time() - start_time < timeout_s:
        nodes = _ros2_node_list()
        if canonical_name in nodes or node_name in nodes:
            return True
        print('.', end='', flush=True)
        time.sleep(1.0)
    print()

    return False


def _start_process(cmd: Iterable[str], label: str) -> subprocess.Popen:
    _logger.info('launching %s: %s', label, ' '.join(cmd))

    return subprocess.Popen(list(cmd), env=os.environ, start_new_session=True)


def _cleanup_procs(procs: list[subprocess.Popen | None]) -> None:
    for proc in procs:
        if proc is None or proc.poll() is not None:
            continue
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
            proc.wait(timeout=3)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGTERM)
                proc.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    pgid = os.getpgid(proc.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


def _safety_cleanup() -> None:
    _logger.info('performing final safety cleanup...')
    for target in _SAFETY_CLEANUP_TARGETS:
        subprocess.run(['pkill', '-f', target], capture_output=True)


def _ros2_run_cmd(
    executable: str, drone_ns: str, extra_params: dict[str, str] | None = None
) -> list[str]:
    cmd = ['ros2', 'run', _ROS_PACKAGE, executable, '--ros-args']
    for key, val in (extra_params or {}).items():
        cmd.extend(['-p', f'{key}:={val}'])
    cmd.extend(['-r', f'__ns:={drone_ns}'])

    return cmd


def _declare_common_params(node: Node, defaults: dict[str, Any]) -> None:
    for name, default_value in defaults.items():
        node.declare_parameter(name, default_value)


def _resolve_common_params(
    node: Node, drone_ns: str, default_topics: dict[str, str]
) -> CollectionParams:
    return CollectionParams(
        odom_topic=_resolve_param(node, 'odom_topic', default_topics['odom']),
        baseline_topic=_resolve_param(node, 'baseline_bspline_topic', default_topics['baseline']),
        cinematic_topic=_resolve_param(
            node, 'cinematic_bspline_topic', default_topics['cinematic']
        ),
        frame_id=_resolve_param(node, 'frame_id', WaypointParams.FRAME_ID, env_key='FRAME_ID'),
        target_xyz=_resolve_param(
            node,
            'target_xyz',
            SimulationParams.TARGET_XYZ,
            env_key='TARGET_XYZ',
            parser=_parse_target_xyz,
        ),
        focus_depth_m=float(
            _resolve_param(
                node, 'focus_depth_m', SimulationParams.FOCUS_DEPTH_M, env_key='FOCUS_DEPTH_M'
            )
        ),
        waypoints_topic=node.get_parameter('waypoints_topic').value or '',
        waypoints_file=node.get_parameter('waypoints_file').value or '',
        drone_ns=drone_ns,
    )


def _make_default_topics(drone_ns: str) -> dict[str, str]:
    return {
        'odom': f'{drone_ns}_visual_slam/odom',
        'baseline': f'{drone_ns}_planning/bspline',
        'cinematic': f'{drone_ns}_planning/cinematic_bspline',
    }


def _launch_simulation(
    workspace_root: Path, drone_id: int, mode_label: str
) -> tuple[subprocess.Popen | None, Any | None, bool]:
    ego_node_name = f'drone_{drone_id}_ego_planner_node'
    if ego_node_name in _ros2_node_list():
        return None, None, True

    start_script = workspace_root / 'start_simulation.sh'
    log_handle = tempfile.NamedTemporaryFile(
        mode='w', prefix=f'ego_planner_{mode_label}_', suffix='.log', delete=False
    )
    log_path = Path(log_handle.name)
    sim_proc = subprocess.Popen(
        [str(start_script), mode_label],
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        env=os.environ,
    )
    _logger.info('starting simulation...')

    if not _wait_for_node(ego_node_name, timeout_s=30):
        _logger.error('simulation failed to start; check %s', log_path)
        sim_proc.terminate()
        log_handle.close()

        return sim_proc, log_handle, False

    _logger.info('simulation initialized successfully! log: %s', log_path)

    return sim_proc, log_handle, True


def _launch_focus_publisher(params: CollectionParams) -> subprocess.Popen:
    return _start_process(
        _ros2_run_cmd(
            'simulated_focus_publisher',
            params.drone_ns,
            {
                'odom_topic': params.odom_topic,
                'frame_id': params.frame_id,
                'target_offset_xyz': _format_ros_value(params.target_xyz),
                'target_xyz': _format_ros_value(params.target_xyz),
                'focus_depth_m': str(params.focus_depth_m),
            },
        ),
        'simulated_focus_publisher',
    )


def _launch_logger(params: CollectionParams, mode: str) -> subprocess.Popen:
    cmd = [
        'ros2',
        'run',
        _ROS_PACKAGE,
        'trajectory_data_logger',
        '--mode',
        mode,
        '--ros-args',
        '-p',
        f'odom_topic:={params.odom_topic}',
        '-p',
        f'baseline_bspline_topic:={params.baseline_topic}',
        '-p',
        f'cinematic_bspline_topic:={params.cinematic_topic}',
        '-r',
        f'__ns:={params.drone_ns}',
    ]

    return _start_process(cmd, 'trajectory_data_logger')


def _launch_waypoint_generator(
    params: CollectionParams, extra_params: dict[str, str] | None = None
) -> subprocess.Popen:
    ros_params = {'odom_topic': params.odom_topic}
    if extra_params:
        ros_params.update(extra_params)
    cmd = _ros2_run_cmd('cinematic_waypoint_generator', params.drone_ns, ros_params)
    if params.waypoints_topic:
        cmd.extend(['-p', f'waypoints_topic:={params.waypoints_topic}'])
    if params.waypoints_file:
        cmd.extend(['-p', f'waypoints_file:={params.waypoints_file}'])

    return _start_process(cmd, 'cinematic_waypoint_generator')


def _run_event_loop(procs: list[subprocess.Popen | None]) -> None:
    stop_event = Event()

    def _handle_signal(signum, frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    while not stop_event.is_set():
        for proc in procs:
            if proc is not None and proc.poll() is not None:
                stop_event.set()
                break
        time.sleep(0.5)


def _run_collection(
    node_name: str,
    mode_label: str,
    extra_node_params: dict[str, Any] | None = None,
    env_file: Path | None = None,
    ros_log_dir: Path | None = None,
    build_procs_fn: Callable[[CollectionParams, Node], list[subprocess.Popen | None]] | None = None,
    args: list[str] | None = None,
) -> None:
    workspace_root = _find_workspace_root(Path(__file__).resolve())

    if env_file and env_file.is_file():
        _parse_env_file(env_file)

    logging_severity = (
        os.environ.get('LOGGING_SEVERITY') or SimulationParams.LOGGING_SEVERITY or 'DEBUG'
    )
    _prepare_environment(logging_severity, ros_log_dir)
    _cleanup_cyclonedds()
    time.sleep(2)

    rclpy.init(args=args)
    node = Node(node_name)

    drone_id = SimulationParams.DRONE_ID
    drone_ns = f'/drone_{drone_id}'
    default_topics = _make_default_topics(drone_ns)

    base_params: dict[str, Any] = {
        'odom_topic': default_topics['odom'],
        'baseline_bspline_topic': default_topics['baseline'],
        'cinematic_bspline_topic': default_topics['cinematic'],
        'frame_id': 'world',
        'target_xyz': [0.0, 0.0, 0.0],
        'focus_depth_m': 1.0,
        'waypoints_topic': '',
        'waypoints_file': '',
    }
    if extra_node_params:
        base_params.update(extra_node_params)
    _declare_common_params(node, base_params)

    params = _resolve_common_params(node, drone_ns, default_topics)

    sim_proc, sim_log_handle, sim_ok = _launch_simulation(workspace_root, drone_id, mode_label)
    if not sim_ok:
        node.destroy_node()
        rclpy.shutdown()

        return

    child_procs: list[subprocess.Popen | None] = []
    if build_procs_fn:
        child_procs = build_procs_fn(params, node)

    try:
        _run_event_loop(child_procs)
    finally:
        _logger.info('stopping data collection.')
        _cleanup_procs(child_procs + [sim_proc])
        if sim_log_handle:
            sim_log_handle.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _safety_cleanup()


def run_baseline_collection(args: list[str] | None = None) -> None:
    def _build_procs(params: CollectionParams, node: Node) -> list[subprocess.Popen | None]:
        logger = _launch_logger(params, 'baseline')
        time.sleep(2)
        focus = _launch_focus_publisher(params)
        time.sleep(2)
        waypoints = _launch_waypoint_generator(params, extra_params={'baseline_mode': 'True'})

        return [logger, focus, waypoints]

    _run_collection(
        node_name='baseline_collection',
        mode_label='baseline',
        extra_node_params={'baseline_mode': True},
        build_procs_fn=_build_procs,
        args=args,
    )


def run_cinematic_collection(args: list[str] | None = None) -> None:
    workspace_root = _find_workspace_root(Path(__file__).resolve())
    env_file = workspace_root / 'collect_constrained_data.env'
    ros_log_dir = workspace_root.parent / 'ros_logs'
    ros_log_dir.mkdir(parents=True, exist_ok=True)

    def _build_procs(params: CollectionParams, node: Node) -> list[subprocess.Popen | None]:
        use_simulated_focus = _resolve_param(
            node,
            'use_simulated_focus',
            SimulationParams.USE_SIMULATED_FOCUS,
            env_key='USE_SIMULATED_FOCUS',
            parser=_parse_bool,
        )
        use_simulated_light = _resolve_param(
            node,
            'use_simulated_light',
            SimulationParams.USE_SIMULATED_LIGHT,
            env_key='USE_SIMULATED_LIGHT',
            parser=_parse_bool,
        )

        procs: list[subprocess.Popen | None] = []

        filter_proc = _start_process(
            _ros2_run_cmd('cinematic_bspline_filter', params.drone_ns),
            'cinematic_bspline_filter',
        )
        procs.append(filter_proc)
        time.sleep(2)

        if use_simulated_focus:
            procs.append(_launch_focus_publisher(params))
            time.sleep(2)
        else:
            detector = _start_process(
                _ros2_run_cmd('simulated_object_detector', params.drone_ns),
                'simulated_object_detector',
            )
            procs.append(detector)
            time.sleep(2)

            costs = _start_process(
                _ros2_run_cmd(
                    'cost_function_calculator',
                    params.drone_ns,
                    {'odom_topic': params.odom_topic, 'frame_id': params.frame_id},
                ),
                'cost_function_calculator',
            )
            procs.append(costs)
            time.sleep(2)

        if use_simulated_light:
            light = _start_process(
                _ros2_run_cmd(
                    'simulated_light_publisher',
                    params.drone_ns,
                    {'frame_id': params.frame_id},
                ),
                'simulated_light_publisher',
            )
            procs.append(light)
            time.sleep(1)

        procs.append(_launch_logger(params, 'constrained'))
        time.sleep(2)

        procs.append(_launch_waypoint_generator(params, extra_params={'frame_id': params.frame_id}))

        return procs

    _run_collection(
        node_name='cinematic_collection',
        mode_label='constrained',
        extra_node_params={
            'use_simulated_focus': True,
            'use_simulated_light': True,
        },
        env_file=env_file,
        ros_log_dir=ros_log_dir,
        build_procs_fn=_build_procs,
        args=args,
    )
