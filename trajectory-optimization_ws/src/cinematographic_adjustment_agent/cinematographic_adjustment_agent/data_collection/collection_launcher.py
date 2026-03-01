from __future__ import annotations

import ast
import glob
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from threading import Event
from typing import Any, Callable, Iterable, Optional

import rclpy
from rclpy.node import Node


def _add_config_path() -> None:
    here = Path(__file__).resolve()
    for parent in [here] + list(here.parents):
        configs_dir = parent / "configs"
        if configs_dir.is_dir():
            python_constants_path = configs_dir / "python-constants"
            if str(python_constants_path) not in sys.path:
                sys.path.insert(0, str(python_constants_path))
            return
    raise RuntimeError("Unable to locate configs/python-constants for constants import.")


_add_config_path()
from constants import SimulationParams, WaypointParams  # noqa: E402


def _find_workspace_root(start: Path) -> Path:
    for parent in [start] + list(start.parents):
        if (parent / "start_simulation.sh").is_file():
            return parent
    raise RuntimeError("Unable to locate trajectory-optimization_ws (start_simulation.sh missing).")


def _cleanup_cyclonedds() -> None:
    for pattern in ["/tmp/cyclonedds*", os.path.expanduser("~/.local/share/cyclonedds*")]:
        for path in glob.glob(pattern):
            try:
                if os.path.isdir(path):
                    shutil.rmtree(path)
                else:
                    os.remove(path)
            except OSError:
                pass


def _ensure_display(env: dict[str, str]) -> None:
    if env.get("DISPLAY"):
        return
    if Path("/etc/resolv.conf").is_file():
        try:
            with open("/etc/resolv.conf", "r", encoding="utf-8") as handle:
                for line in handle:
                    if line.startswith("nameserver"):
                        parts = line.split()
                        if len(parts) >= 2:
                            env["DISPLAY"] = f"{parts[1]}:0"
                            return
        except OSError:
            pass
    env["DISPLAY"] = ":0"


def _prepare_environment(logging_severity: str, ros_log_dir: Optional[Path] = None) -> None:
    os.environ.setdefault("ROS_DOMAIN_ID", "0")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    _ensure_display(os.environ)
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    os.environ["RCUTILS_LOGGING_SEVERITY"] = logging_severity.upper()
    if ros_log_dir is not None:
        os.environ["ROS_LOG_DIR"] = str(ros_log_dir)


def _parse_env_file(path: Path) -> None:
    try:
        contents = path.read_text(encoding="utf-8")
    except OSError:
        return
    for line in contents.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("export "):
            stripped = stripped[len("export ") :].strip()
        if "=" not in stripped:
            continue
        key, value = stripped.split("=", 1)
        os.environ[key.strip()] = value.strip().strip('"').strip("'")


def _parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _parse_target_xyz(value: Any) -> Any:
    if isinstance(value, (list, tuple)):
        return [float(v) for v in value]
    if isinstance(value, str):
        text = value.strip()
        if text.startswith("[") and text.endswith("]"):
            try:
                parsed = ast.literal_eval(text)
            except (ValueError, SyntaxError):
                return value
            if isinstance(parsed, (list, tuple)):
                return [float(v) for v in parsed]
    return value


def _format_ros_value(value: Any) -> str:
    if isinstance(value, (list, tuple)):
        return "[" + ",".join(str(v) for v in value) + "]"
    return str(value)


def _resolve_param(
    node: Node,
    name: str,
    default: Any,
    env_key: Optional[str] = None,
    parser: Optional[Callable[[Any], Any]] = None,
) -> Any:
    value = node.get_parameter(name).value
    if value is None or value == "":
        if env_key:
            env_value = os.environ.get(env_key)
            if env_value not in (None, ""):
                value = env_value
            else:
                value = default
        else:
            value = default
    if parser is None:
        return value
    try:
        return parser(value)
    except Exception:
        return value


def _ros2_node_list() -> list[str]:
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            env=os.environ,
            check=False,
        )
        if result.returncode != 0:
            if result.stderr:
                print(f"\n[collection] ros2 node list error: {result.stderr.strip()}")
            return []
        return [line.strip() for line in result.stdout.splitlines() if line.strip()]
    except Exception as e:
        print(f"\n[collection] failed to run ros2 node list: {e}")
        return []


def _wait_for_node(node_name: str, timeout_s: float = 30.0) -> bool:
    start_time = time.time()
    # Normalize node name to have a leading slash for comparison with ros2 node list
    canonical_name = node_name if node_name.startswith("/") else "/" + node_name
    while time.time() - start_time < timeout_s:
        nodes = _ros2_node_list()
        if canonical_name in nodes or node_name in nodes:
            return True
        print(".", end="", flush=True)
        time.sleep(1.0)
    print()
    return False


def _start_process(cmd: Iterable[str], label: str) -> subprocess.Popen:
    print(f"[collection] launching {label}: {' '.join(cmd)}")
    return subprocess.Popen(list(cmd), env=os.environ, start_new_session=True)


def _cleanup_procs(procs: list[subprocess.Popen | None]) -> None:
    for proc in procs:
        if proc is None or proc.poll() is not None:
            continue
        try:
            # Kill the entire process group
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
    print("[collection] performing final safety cleanup...")

    targets = [
        "trajectory_data_logger",
        "simulated_focus_publisher",
        "cinematic_waypoint_generator",
        "cinematic_bspline_filter",
        "simulated_light_publisher",
        "cost_function_calculator",
        "simulated_object_detector",
        "traj_server",
        "ego_planner_node",
        "so3_quadrotor_simulator",
        "pcl_render_node",
        "random_forest",
    ]

    for target in targets:
        subprocess.run(["pkill", "-f", target], capture_output=True)


def run_baseline_collection(args: Optional[list[str]] = None) -> None:
    workspace_root = _find_workspace_root(Path(__file__).resolve())

    logging_severity = os.environ.get("LOGGING_SEVERITY") or SimulationParams.LOGGING_SEVERITY or "DEBUG"
    _prepare_environment(logging_severity)
    _cleanup_cyclonedds()
    time.sleep(2)

    rclpy.init(args=args)
    node = Node("baseline_collection")

    drone_id = SimulationParams.DRONE_ID
    drone_ns = f"/drone_{drone_id}"
    default_odom_topic = f"{drone_ns}_visual_slam/odom"
    default_baseline_topic = f"{drone_ns}_planning/bspline"
    default_cinematic_topic = f"{drone_ns}_planning/cinematic_bspline"

    node.declare_parameter("odom_topic", default_odom_topic)
    node.declare_parameter("baseline_bspline_topic", default_baseline_topic)
    node.declare_parameter("cinematic_bspline_topic", default_cinematic_topic)
    node.declare_parameter("frame_id", "world")
    node.declare_parameter("target_xyz", [0.0, 0.0, 0.0])
    node.declare_parameter("focus_depth_m", 1.0)
    node.declare_parameter("waypoints_topic", "")
    node.declare_parameter("waypoints_file", "")
    node.declare_parameter("baseline_mode", True)

    odom_topic = _resolve_param(node, "odom_topic", default_odom_topic)
    baseline_topic = _resolve_param(node, "baseline_bspline_topic", default_baseline_topic)
    cinematic_topic = _resolve_param(node, "cinematic_bspline_topic", default_cinematic_topic)
    waypoints_topic = node.get_parameter("waypoints_topic").value or ""
    waypoints_file = node.get_parameter("waypoints_file").value or ""
    frame_id = _resolve_param(node, "frame_id", WaypointParams.FRAME_ID, env_key="FRAME_ID")
    target_xyz = _resolve_param(
        node, "target_xyz", SimulationParams.TARGET_XYZ, env_key="TARGET_XYZ", parser=_parse_target_xyz
    )
    focus_depth_m = float(
        _resolve_param(node, "focus_depth_m", SimulationParams.FOCUS_DEPTH_M, env_key="FOCUS_DEPTH_M")
    )

    sim_proc: Optional[subprocess.Popen] = None
    sim_log_handle: Optional[Any] = None
    ego_node_name = f"drone_{drone_id}_ego_planner_node"
    if ego_node_name not in _ros2_node_list():
        start_script = workspace_root / "start_simulation.sh"
        sim_log_file = tempfile.NamedTemporaryFile(
            delete=False, mode="w", prefix="ego_planner_baseline_", suffix=".log"
        )
        sim_log_path = Path(sim_log_file.name)
        sim_log_handle = sim_log_file
        sim_proc = subprocess.Popen(
            [str(start_script), "baseline"],
            stdout=sim_log_handle,
            stderr=subprocess.STDOUT,
            env=os.environ,
        )
        print("starting simulation...")
        if not _wait_for_node(ego_node_name, timeout_s=30):
            print(f"simulation failed to start; check {sim_log_path}")
            if sim_proc:
                sim_proc.terminate()
            if sim_log_handle:
                sim_log_handle.close()
            node.destroy_node()
            rclpy.shutdown()
            return
        print(f"simulation initialized successfully! log: {sim_log_path}")

    logger_proc = _start_process(
        [
            "ros2",
            "run",
            "cinematographic_adjustment_agent",
            "trajectory_data_logger",
            "--mode",
            "baseline",
            "--ros-args",
            "-p", f"odom_topic:={odom_topic}",
            "-p", f"baseline_bspline_topic:={baseline_topic}",
            "-p", f"cinematic_bspline_topic:={cinematic_topic}",
            "-r", f"__ns:={drone_ns}",
        ],
        "trajectory_data_logger",
    )
    time.sleep(2)

    focus_proc = _start_process(
        [
            "ros2",
            "run",
            "cinematographic_adjustment_agent",
            "simulated_focus_publisher",
            "--ros-args",
            "-p", f"odom_topic:={odom_topic}",
            "-p", f"frame_id:={frame_id}",
            "-p", f"target_offset_xyz:={_format_ros_value(target_xyz)}",
            "-p", f"target_xyz:={_format_ros_value(target_xyz)}",
            "-p", f"focus_depth_m:={focus_depth_m}",
            "-r", f"__ns:={drone_ns}",
        ],
        "simulated_focus_publisher",
    )
    time.sleep(2)

    waypoint_cmd = [
        "ros2",
        "run",
        "cinematographic_adjustment_agent",
        "cinematic_waypoint_generator",
        "--ros-args",
        "-p", f"odom_topic:={odom_topic}",
        "-p", "baseline_mode:=True",
        "-r", f"__ns:={drone_ns}",
    ]
    if waypoints_topic:
        waypoint_cmd.extend(["-p", f"waypoints_topic:={waypoints_topic}"])
    if waypoints_file:
        waypoint_cmd.extend(["-p", f"waypoints_file:={waypoints_file}"])

    waypoint_proc = _start_process(waypoint_cmd, "cinematic_waypoint_generator")

    stop_event = Event()

    def _handle_signal(signum, frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        while not stop_event.is_set():
            for proc in [logger_proc, focus_proc, waypoint_proc]:
                if proc.poll() is not None:
                    stop_event.set()
                    break
            time.sleep(0.5)
    finally:
        print("stopping data collection.")
        _cleanup_procs([logger_proc, focus_proc, waypoint_proc, sim_proc])

        if sim_log_handle:
            sim_log_handle.close()

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        _safety_cleanup()


def run_cinematic_collection(args: Optional[list[str]] = None) -> None:
    workspace_root = _find_workspace_root(Path(__file__).resolve())

    env_file = workspace_root / "collect_constrained_data.env"
    if env_file.is_file():
        _parse_env_file(env_file)

    logging_severity = os.environ.get("LOGGING_SEVERITY") or SimulationParams.LOGGING_SEVERITY or "DEBUG"
    ros_log_dir = workspace_root.parent / "ros_logs"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    _prepare_environment(logging_severity, ros_log_dir)
    _cleanup_cyclonedds()
    time.sleep(2)

    rclpy.init(args=args)
    node = Node("cinematic_collection")

    drone_id = SimulationParams.DRONE_ID
    drone_ns = f"/drone_{drone_id}"
    default_odom_topic = f"{drone_ns}_visual_slam/odom"
    default_baseline_topic = f"{drone_ns}_planning/bspline"
    default_cinematic_topic = f"{drone_ns}_planning/cinematic_bspline"

    node.declare_parameter("odom_topic", default_odom_topic)
    node.declare_parameter("baseline_bspline_topic", default_baseline_topic)
    node.declare_parameter("cinematic_bspline_topic", default_cinematic_topic)
    node.declare_parameter("frame_id", "world")
    node.declare_parameter("target_xyz", [0.0, 0.0, 0.0])
    node.declare_parameter("focus_depth_m", 1.0)
    node.declare_parameter("use_simulated_focus", True)
    node.declare_parameter("use_simulated_light", True)
    node.declare_parameter("waypoints_topic", "")
    node.declare_parameter("waypoints_file", "")

    odom_topic = _resolve_param(node, "odom_topic", default_odom_topic)
    baseline_topic = _resolve_param(node, "baseline_bspline_topic", default_baseline_topic)
    cinematic_topic = _resolve_param(node, "cinematic_bspline_topic", default_cinematic_topic)
    waypoints_topic = node.get_parameter("waypoints_topic").value or ""
    waypoints_file = node.get_parameter("waypoints_file").value or ""
    frame_id = _resolve_param(node, "frame_id", WaypointParams.FRAME_ID, env_key="FRAME_ID")
    target_xyz = _resolve_param(
        node, "target_xyz", SimulationParams.TARGET_XYZ, env_key="TARGET_XYZ", parser=_parse_target_xyz
    )
    focus_depth_m = float(
        _resolve_param(node, "focus_depth_m", SimulationParams.FOCUS_DEPTH_M, env_key="FOCUS_DEPTH_M")
    )
    use_simulated_focus = _resolve_param(
        node,
        "use_simulated_focus",
        SimulationParams.USE_SIMULATED_FOCUS,
        env_key="USE_SIMULATED_FOCUS",
        parser=_parse_bool,
    )
    use_simulated_light = _resolve_param(
        node,
        "use_simulated_light",
        SimulationParams.USE_SIMULATED_LIGHT,
        env_key="USE_SIMULATED_LIGHT",
        parser=_parse_bool,
    )

    sim_proc: Optional[subprocess.Popen] = None
    sim_log_handle: Optional[Any] = None
    ego_node_name = f"drone_{drone_id}_ego_planner_node"
    if ego_node_name not in _ros2_node_list():
        start_script = workspace_root / "start_simulation.sh"
        sim_log_file = tempfile.NamedTemporaryFile(
            delete=False, mode="w", prefix="ego_planner_constrained_", suffix=".log"
        )
        sim_log_path = Path(sim_log_file.name)
        sim_log_handle = sim_log_file
        sim_proc = subprocess.Popen(
            [str(start_script), "constrained"],
            stdout=sim_log_handle,
            stderr=subprocess.STDOUT,
            env=os.environ,
        )
        print("starting simulation...")
        if not _wait_for_node(ego_node_name, timeout_s=30):
            print(f"simulation failed to start; check {sim_log_path}")
            if sim_proc:
                sim_proc.terminate()
            if sim_log_handle:
                sim_log_handle.close()
            node.destroy_node()
            rclpy.shutdown()
            return
        print(f"simulation initialized successfully! log: {sim_log_path}")

    filter_proc = _start_process(
        [
            "ros2", "run", "cinematographic_adjustment_agent", "cinematic_bspline_filter",
            "--ros-args", "-r", f"__ns:={drone_ns}",
        ],
        "cinematic_bspline_filter",
    )
    time.sleep(2)

    detector_proc: Optional[subprocess.Popen] = None
    costs_proc: Optional[subprocess.Popen] = None
    focus_proc: Optional[subprocess.Popen] = None
    if use_simulated_focus:
        focus_proc = _start_process(
            [
                "ros2",
                "run",
                "cinematographic_adjustment_agent",
                "simulated_focus_publisher",
                "--ros-args",
                "-p", f"odom_topic:={odom_topic}",
                "-p", f"frame_id:={frame_id}",
                "-p", f"target_offset_xyz:={_format_ros_value(target_xyz)}",
                "-p", f"target_xyz:={_format_ros_value(target_xyz)}",
                "-p", f"focus_depth_m:={focus_depth_m}",
                "-r", f"__ns:={drone_ns}",
            ],
            "simulated_focus_publisher",
        )
        time.sleep(2)
    else:
        detector_proc = _start_process(
             [
                 "ros2", "run", "cinematographic_adjustment_agent", "simulated_object_detector",
                 "--ros-args", "-r", f"__ns:={drone_ns}",
             ],
             "simulated_object_detector",
         )
        time.sleep(2)

        costs_proc = _start_process(
            [
                "ros2",
                "run",
                "cinematographic_adjustment_agent",
                "cost_function_calculator",
                "--ros-args",
                "-p", f"odom_topic:={odom_topic}",
                "-p", f"frame_id:={frame_id}",
                "-r", f"__ns:={drone_ns}",
            ],
            "cost_function_calculator",
        )
        time.sleep(2)

    light_proc: Optional[subprocess.Popen] = None
    if use_simulated_light:
        light_proc = _start_process(
            [
                "ros2",
                "run",
                "cinematographic_adjustment_agent",
                "simulated_light_publisher",
                "--ros-args",
                "-p", f"frame_id:={frame_id}",
                "-r", f"__ns:={drone_ns}",
            ],
            "simulated_light_publisher",
        )
        time.sleep(1)

    logger_proc = _start_process(
        [
            "ros2",
            "run",
            "cinematographic_adjustment_agent",
            "trajectory_data_logger",
            "--mode",
            "constrained",
            "--ros-args",
            "-p", f"odom_topic:={odom_topic}",
            "-p", f"baseline_bspline_topic:={baseline_topic}",
            "-p", f"cinematic_bspline_topic:={cinematic_topic}",
            "-r", f"__ns:={drone_ns}",
        ],
        "trajectory_data_logger",
    )
    time.sleep(2)

    waypoint_cmd = [
        "ros2",
        "run",
        "cinematographic_adjustment_agent",
        "cinematic_waypoint_generator",
        "--ros-args",
        "-p", f"odom_topic:={odom_topic}",
        "-p", f"frame_id:={frame_id}",
        "-r", f"__ns:={drone_ns}",
    ]
    if waypoints_topic:
        waypoint_cmd.extend(["-p", f"waypoints_topic:={waypoints_topic}"])
    if waypoints_file:
        waypoint_cmd.extend(["-p", f"waypoints_file:={waypoints_file}"])

    waypoint_proc = _start_process(waypoint_cmd, "cinematic_waypoint_generator")

    stop_event = Event()

    def _handle_signal(signum, frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    procs = [
        logger_proc,
        waypoint_proc,
        filter_proc,
        focus_proc,
        detector_proc,
        costs_proc,
        light_proc,
    ]
    try:
        while not stop_event.is_set():
            for proc in procs:
                if proc is not None and proc.poll() is not None:
                    stop_event.set()
                    break
            time.sleep(0.5)
    finally:
        print("stopping data collection.")
        _cleanup_procs(procs + [sim_proc])

        if sim_log_handle:
            sim_log_handle.close()

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        _safety_cleanup()
