"""Runs one simulator flight inside the Modal container and saves everything it produced.

The stack is started through the image's own entrypoint, exactly as `docker compose up`
starts it, so the cloud run builds, fetches weights and launches the same way a local
run does. This module adds what an unattended run needs around that: a display, a build
capped to the memory, a memory watchdog and deadline that stop the stack with SIGINT so
rosbag2 can close its bag, samples of load and real-time factor, scoring with cine_eval,
and one result.json that says what happened.
"""

import codecs
import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import threading
import time
import traceback
from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path

from sim_plan import (
    ALLOWANCE_SECONDS,
    CYCLONEDDS_LOOPBACK_CONFIG,
    LOOPBACK,
    XVFB_DISPLAY,
    BuildParallelism,
    OutcomeFacts,
    SimRunPlan,
    StatsSample,
    bounded_log,
    build_parallelism,
    cgroup_memory_headroom_gib,
    classify_outcome,
    colcon_defaults_yaml,
    estimate_cost,
    memory_available_gib,
    parse_crashes,
    read_stats_output,
    record_stats_reading,
    stack_environment,
    summarize_factors,
    summarize_intervals,
    time_budget,
)

LAUNCH_MARKER = b'[INFO] [launch]'
# Lines worth repeating in Modal's live log; the full output goes to launch.log.
ECHOED_LINE = re.compile(
    r'process has died|^Summary: |Failed\s+<<<|^(PASS|FAIL):|\[launch\]: All log files',
)
DROPPED_ENVIRONMENT = ('PYTHONPATH', 'PYTHONHOME')
DROPPED_ENVIRONMENT_PREFIX = 'MODAL_'
LOG_TAIL_CHARS = 6000
OUTPUT_CHARS = 4000
CRASH_MARKER = b'process has died'
# smoke_flight.py prints its verdict just before it interrupts its own launch, so in a
# check run with no stop from the driver, a death after this line is part of shutdown.
SMOKE_VERDICT = re.compile(rb'^(PASS|FAIL): ')
# smoke_flight.py stops its own launch with SIGINT, SIGTERM and SIGKILL, waiting up to
# 30 s after each; the driver's SIGINT must outlast all three or the launch is orphaned.
CHECK_INTERRUPT_GRACE_SECONDS = 120.0
# ps, top, three gz stats messages and eight `ros2 topic hz` windows of 12 s each.
DEEP_SAMPLE_SECONDS = 150
SAMPLER_JOIN_SECONDS = 45
REINDEX_SECONDS = 600
# Below this much time before the internal deadline, a step is skipped rather than started.
MINIMUM_STEP_SECONDS = 60
KILLED_OUTPUT_WAIT_SECONDS = 10
DISPLAY_STOP_SECONDS = 5
EARLIER_ATTEMPTS = 'earlier_attempts'
# The deep sample lists the busiest processes and the rate of every topic the flight
# depends on, once, halfway through the flight.
SAMPLED_TOPICS = (
    '/clock',
    '/cinematography/translational_position_setpoints',
    '/mavros/setpoint_position/local',
    '/cinematography/costs',
    '/cinematography/subject_position',
    '/cinematography/framing_active',
    '/cinematography/drone_odometry',
    '/object_detection/center_point',
    '/gazebo/drone_pose',
)
GZ_STATS_COMMAND = 'timeout 8 gz topic -e -t /stats -n 1'


@dataclass(frozen=True)
class ContainerPaths:
    entrypoint: str
    work: Path
    bags: Path
    runs: Path
    repo_root: Path
    colcon_base: Path
    ros_setup: Path
    eval_python: str
    weights: Path
    xvfb: str | None


@dataclass(frozen=True)
class Timing:
    poll_seconds: float = 2.0
    interrupt_grace_seconds: float = 60.0
    terminate_grace_seconds: float = 30.0
    sample_seconds: float = 30.0
    # Below this much free memory the stack is stopped while its bag can still close.
    memory_stop_gib: float = 1.0


DEFAULT_TIMING = Timing()


@dataclass(frozen=True)
class FlightLimits:
    # None runs until the stack exits by itself (the smoke check).
    flight_seconds: float | None
    deadline: float
    memory_stop_gib: float
    poll_seconds: float
    interrupt_grace_seconds: float
    terminate_grace_seconds: float


@dataclass
class FlightReport:
    launch_started: bool = False
    launch_seconds: float | None = None
    flight_seconds: float | None = None
    stack_exit_code: int | None = None
    stopped_by: str | None = None
    ended_by_signal: str | None = None
    final_exit_code: int | None = None
    log_offset_at_stop: int | None = None
    lowest_headroom_gib: float | None = None


@dataclass(frozen=True)
class BagState:
    path: Path
    complete: bool


@dataclass(frozen=True)
class LogScan:
    died_before_stop: str
    died_after_stop: str
    tail: str


@dataclass(frozen=True)
class ProbeStep:
    name: str
    script: str
    timeout_seconds: int
    environment: Mapping[str, str]
    frame_path: str = ''


def child_environment(base: Mapping[str, str], overrides: Mapping[str, str]) -> dict[str, str]:
    """The stack's environment: Modal's own Python paths and settings never reach ROS."""

    environment = {
        name: value
        for name, value in base.items()
        if name not in DROPPED_ENVIRONMENT and not name.startswith(DROPPED_ENVIRONMENT_PREFIX)
    }
    environment.update(overrides)

    return environment


def lowest_headroom(readings: list[float | None]) -> float | None:
    known = [reading for reading in readings if reading is not None]

    return min(known) if known else None


def _read_text(path: str) -> str:
    try:
        return Path(path).read_text()
    except OSError:
        return ''


def memory_readings() -> dict[str, float | None]:
    """What the kernel and the cgroup each say is left; gVisor may report either."""

    return {
        'meminfo_available_gib': memory_available_gib(_read_text('/proc/meminfo')),
        'cgroup_headroom_gib': cgroup_memory_headroom_gib(
            _read_text('/sys/fs/cgroup/memory.current'),
            _read_text('/sys/fs/cgroup/memory.max'),
            _read_text('/sys/fs/cgroup/memory.stat'),
        ),
    }


def read_memory_headroom_gib() -> float | None:
    return lowest_headroom(list(memory_readings().values()))


def scan_log(
    log_path: Path,
    offset_at_stop: int | None,
    tail_bytes: int = LOG_TAIL_CHARS,
) -> LogScan:
    """Reads the log a line at a time, since a long flight's log can outgrow the memory left.

    Only the crash lines and the tail are kept; a crash line is counted after the stop
    when it starts at or past the log's size at the moment the stop was decided.
    """

    if not log_path.exists():
        return LogScan('', '', '')

    before_stop = []
    after_stop = []
    position = 0

    with log_path.open('rb') as handle:
        for line in handle:
            if offset_at_stop is None and SMOKE_VERDICT.match(line):
                offset_at_stop = position

            if CRASH_MARKER in line:
                text = line.decode(errors='replace')

                if offset_at_stop is None or position < offset_at_stop:
                    before_stop.append(text)
                else:
                    after_stop.append(text)

            position += len(line)

        handle.seek(max(position - tail_bytes, 0))
        tail = handle.read().decode(errors='replace')

    return LogScan(''.join(before_stop), ''.join(after_stop), tail)


def _signal_group(process_id: int, signal_number: int) -> None:
    try:
        os.killpg(process_id, signal_number)
    except (ProcessLookupError, PermissionError):
        pass


def stop_process_group(
    process: subprocess.Popen,
    interrupt_grace: float,
    terminate_grace: float,
) -> str:
    """SIGINT first so every node shuts down cleanly, then SIGTERM, then SIGKILL."""

    for signal_number, grace in (
        (signal.SIGINT, interrupt_grace),
        (signal.SIGTERM, terminate_grace),
    ):
        _signal_group(process.pid, signal_number)

        try:
            process.wait(grace)
        except subprocess.TimeoutExpired:
            continue

        _signal_group(process.pid, signal.SIGKILL)

        return signal_number.name

    _signal_group(process.pid, signal.SIGKILL)
    process.wait()

    return 'SIGKILL'


class _LogFollower:
    """Reads the growing launch log, spots the launch, and echoes the lines that matter."""

    def __init__(self, path: Path):
        self.handle = path.open('rb')
        self.partial = b''
        self.launched = False

    def poll(self) -> None:
        chunk = self.handle.read()
        if not chunk:
            return

        lines = (self.partial + chunk).split(b'\n')
        self.partial = lines.pop()

        for line in lines:
            if LAUNCH_MARKER in line:
                self.launched = True

            text = line.decode(errors='replace')
            if ECHOED_LINE.search(text):
                print(f'[stack] {text}', flush=True)

    def close(self) -> None:
        self.poll()

        if LAUNCH_MARKER in self.partial:
            self.launched = True

        self.handle.close()


def fly(
    command: list[str],
    environment: Mapping[str, str],
    log_path: Path,
    limits: FlightLimits,
    read_headroom: Callable[[], float | None] = read_memory_headroom_gib,
    on_midflight: Callable[[], None] | None = None,
    midflight_join_seconds: float = DEEP_SAMPLE_SECONDS + 30,
) -> FlightReport:
    report = FlightReport()
    started = time.monotonic()
    launched_at = None
    midflight_thread = None
    follower = None

    with log_path.open('wb') as log_handle:
        process = subprocess.Popen(
            command,
            env=dict(environment),
            stdin=subprocess.DEVNULL,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    try:
        follower = _LogFollower(log_path)

        while True:
            exit_code = process.poll()
            follower.poll()
            now = time.monotonic()

            if launched_at is None and follower.launched:
                launched_at = now
                report.launch_started = True
                report.launch_seconds = now - started

            if exit_code is not None:
                report.stack_exit_code = exit_code
                report.final_exit_code = exit_code
                report.flight_seconds = None if launched_at is None else now - launched_at
                _signal_group(process.pid, signal.SIGKILL)

                break

            headroom = read_headroom()
            report.lowest_headroom_gib = lowest_headroom([report.lowest_headroom_gib, headroom])
            flown = None if launched_at is None else now - launched_at

            # Judged on the current reading and only after launch: a compile spike before
            # it has no bag at risk, and must not linger in the lowest-ever figure.
            if (
                launched_at is not None
                and headroom is not None
                and headroom < limits.memory_stop_gib
            ):
                report.stopped_by = 'memory'
            elif now >= limits.deadline:
                report.stopped_by = 'deadline'
            elif (
                flown is not None
                and limits.flight_seconds is not None
                and flown >= limits.flight_seconds
            ):
                report.stopped_by = 'flight_time'

            if report.stopped_by is not None:
                # The flight ends here; the shutdown that follows is not flight time.
                report.flight_seconds = flown
                report.log_offset_at_stop = log_path.stat().st_size
                report.ended_by_signal = stop_process_group(
                    process,
                    limits.interrupt_grace_seconds,
                    limits.terminate_grace_seconds,
                )
                report.final_exit_code = process.returncode

                break

            midflight_due = (
                flown is not None
                and limits.flight_seconds is not None
                and flown >= limits.flight_seconds / 2
            )
            if midflight_due and midflight_thread is None and on_midflight is not None:
                midflight_thread = threading.Thread(target=on_midflight, daemon=True)
                midflight_thread.start()

            time.sleep(limits.poll_seconds)
    finally:
        if process.poll() is None:
            stop_process_group(
                process,
                limits.interrupt_grace_seconds,
                limits.terminate_grace_seconds,
            )

        if follower is not None:
            follower.close()

    if report.launch_started is False and follower.launched:
        report.launch_started = True

    if midflight_thread is not None:
        midflight_thread.join(midflight_join_seconds)

    return report


def find_bag(bags_directory: Path) -> BagState | None:
    if not bags_directory.is_dir():
        return None

    candidates = [path for path in bags_directory.iterdir() if path.is_dir()]
    if not candidates:
        return None

    newest = max(candidates, key=lambda path: (path.stat().st_mtime, path.name))

    return BagState(newest, (newest / 'metadata.yaml').exists())


def storage_id_for(bag_directory: Path) -> str | None:
    suffixes = {path.suffix for path in bag_directory.iterdir() if path.is_file()}
    if '.mcap' in suffixes:
        return 'mcap'

    if '.db3' in suffixes:
        return 'sqlite3'

    return None


def gate_label(output: str) -> str | None:
    for line in reversed(output.splitlines()):
        try:
            parsed = json.loads(line)
        except ValueError:
            continue

        if isinstance(parsed, dict) and isinstance(parsed.get('label'), str):
            return parsed['label']

    return None


def _ros_check(topic_type: str, publish: str, echo: str) -> str:
    return (
        'command -v ros2 >/dev/null || { echo "ros2 is not on PATH"; exit 127; }\n'
        f'ros2 topic pub -r 5 /cine_probe {topic_type} "{publish}" >/dev/null 2>&1 &\n'
        'publisher=$!\n'
        f'timeout 25 ros2 topic echo --once /cine_probe {topic_type} {echo}\n'
        'status=$?\n'
        'kill "$publisher" 2>/dev/null\n'
        'exit "$status"\n'
    )


GZ_TRANSPORT_CHECK = (
    'command -v gz >/dev/null || { echo "gz is not on PATH"; exit 127; }\n'
    'timeout 20 gz topic -e -t /cine_probe -n 1 &\n'
    'echo_pid=$!\n'
    'sleep 3\n'
    'for attempt in 1 2 3 4 5; do\n'
    '  gz topic -t /cine_probe -m gz.msgs.StringMsg -p \'data:"probe"\'\n'
    '  sleep 1\n'
    'done\n'
    'wait "$echo_pid"\n'
)

# The image's Gazebo comes from ros-jazzy-gz-sim-vendor, which installs its example worlds
# under /opt/ros; /usr/share/gz is where a plain Gazebo install would put them.
GAZEBO_WORLD_SEARCH_ROOTS = ('/opt/ros', '/usr/share/gz')


def render_check(search_roots: Sequence[str]) -> str:
    """Renders Gazebo's stock camera world through Xvfb and Mesa, as the drone camera will be."""

    roots = ' '.join(shlex.quote(root) for root in search_roots)

    return (
        'command -v gz >/dev/null || { echo "gz is not on PATH"; exit 127; }\n'
        f"world=$(find {roots} -path '*/worlds/camera_sensor.sdf' 2>/dev/null | head -1)\n"
        f'[ -n "$world" ] || {{ echo "camera_sensor.sdf not found under {roots}"; exit 2; }}\n'
        'echo "world: $world"\n' + RENDER_FROM_WORLD
    )


RENDER_FRAME_PATH = '/tmp/render_frame.txt'

RENDER_FROM_WORLD = (
    'gz sim -s -r "$world" >/tmp/render_probe.log 2>&1 &\n'
    'server=$!\n'
    f'timeout 90 gz topic -e -t /camera -n 1 >{RENDER_FRAME_PATH}\n'
    'status=$?\n'
    'kill "$server" 2>/dev/null; wait "$server" 2>/dev/null\n'
    f'echo "frame: $(wc -c <{RENDER_FRAME_PATH}) bytes"; head -c 300 {RENDER_FRAME_PATH}; echo\n'
    'tail -5 /tmp/render_probe.log\n'
    'exit "$status"\n'
)

BLANK_FRAME_EXIT_CODE = 3
_FRAME_SIZE_FIELD = re.compile(r'^(width|height|step): (\d+)$', re.MULTILINE)
_FRAME_DATA_FIELD = re.compile(r'^data: "((?:[^"\\]|\\.)*)"', re.MULTILINE)


@dataclass(frozen=True)
class FrameVerdict:
    passed: bool
    distinct_colors: int
    summary: str


def judge_frame(gz_text: str) -> FrameVerdict:
    """Reads the image printed by `gz topic -e` and passes it only if it holds two colors.

    A frame can arrive filled with one color when the scene failed to draw, so its arrival
    alone proves nothing about rendering.
    """

    sizes = {name: int(value) for name, value in _FRAME_SIZE_FIELD.findall(gz_text)}
    data_match = _FRAME_DATA_FIELD.search(gz_text)
    if data_match is None or sizes.keys() != {'width', 'height', 'step'} or not sizes['width']:
        return FrameVerdict(False, 0, 'no image received')

    width, height, step = sizes['width'], sizes['height'], sizes['step']
    pixels = codecs.escape_decode(data_match.group(1).encode('latin-1'))[0]
    if len(pixels) != height * step:
        summary = f'frame {width}x{height} carried {len(pixels)} of {height * step} bytes'

        return FrameVerdict(False, 0, summary)

    bytes_per_pixel = step // width
    colors = {
        pixels[row * step + offset : row * step + offset + bytes_per_pixel]
        for row in range(height)
        for offset in range(0, width * bytes_per_pixel, bytes_per_pixel)
    }
    plural = '' if len(colors) == 1 else 's'
    summary = f'frame {width}x{height}, {len(colors)} distinct color{plural}'

    return FrameVerdict(len(colors) > 1, len(colors), summary)


def apply_frame_check(step: ProbeStep, outcome: dict) -> dict:
    """Fails a step whose command succeeded but whose saved frame is blank or cut short."""

    if not step.frame_path or outcome['exit_code'] != 0:
        return outcome

    frame_file = Path(step.frame_path)
    gz_text = frame_file.read_text(errors='replace') if frame_file.exists() else ''
    verdict = judge_frame(gz_text)
    checked = {**outcome, 'output': f'{outcome["output"]}frame check: {verdict.summary}\n'}

    if not verdict.passed:
        checked['exit_code'] = BLANK_FRAME_EXIT_CODE

    return checked


SYSTEM_CHECK = (
    'echo "cpus: $(nproc)"\n'
    "grep -m1 'model name' /proc/cpuinfo\n"
    'head -3 /proc/meminfo\n'
    'echo "cgroup memory.max: $(cat /sys/fs/cgroup/memory.max 2>&1)"\n'
    'echo "cgroup memory.current: $(cat /sys/fs/cgroup/memory.current 2>&1)"\n'
    'df -h / /tmp 2>&1\n'
    'python3 --version\n'
    'uname -a\n'
)


def probe_steps(weights_directory: str) -> list[ProbeStep]:
    loopback_dds = {'CYCLONEDDS_URI': CYCLONEDDS_LOOPBACK_CONFIG}
    string_check = _ros_check('std_msgs/msg/String', '{data: probe}', '--field data')
    render = render_check(GAZEBO_WORLD_SEARCH_ROOTS)

    return [
        ProbeStep('system', SYSTEM_CHECK, 30, {}),
        ProbeStep(
            'display',
            f'test -S /tmp/.X11-unix/X{XVFB_DISPLAY[1:]} && echo "display ready"',
            10,
            {},
        ),
        ProbeStep('dds_default', string_check, 40, {}),
        ProbeStep('dds_loopback', string_check, 40, loopback_dds),
        ProbeStep('gz_transport_default', GZ_TRANSPORT_CHECK, 35, {}),
        ProbeStep('gz_transport_loopback', GZ_TRANSPORT_CHECK, 35, {'GZ_IP': LOOPBACK}),
        ProbeStep(
            'render',
            render,
            150,
            {'DISPLAY': XVFB_DISPLAY},
            frame_path=RENDER_FRAME_PATH,
        ),
        ProbeStep(
            'weights',
            f'python3 /opt/cine/fetch_weights.py --weights-dir {weights_directory}',
            900,
            {},
        ),
    ]


def run_captured(command: list[str], environment: Mapping[str, str], timeout: float) -> dict:
    """Runs a command in its own process group and kills the whole group when it is done.

    Whatever the command started in the background dies with it, so a probe step or a
    sample never leaves a publisher or a Gazebo server running into the next one. On a
    timeout the output printed so far is kept and the exit code is 124, as timeout(1) uses.
    """

    started = time.monotonic()
    note = ''

    try:
        process = subprocess.Popen(
            command,
            env=dict(environment),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except OSError as exception:
        return {'exit_code': 127, 'seconds': 0.0, 'output': str(exception)}

    try:
        output, _ = process.communicate(timeout=timeout)
        exit_code = process.returncode
    except subprocess.TimeoutExpired as expired:
        finished_exit_code = process.poll()
        _signal_group(process.pid, signal.SIGKILL)

        try:
            output, _ = process.communicate(timeout=KILLED_OUTPUT_WAIT_SECONDS)
        except subprocess.TimeoutExpired:
            output = expired.output or b''

        if finished_exit_code is None:
            exit_code = 124
            note = f'\ntimed out after {timeout:g} s'
        else:
            # The command itself finished; something it left behind held its output open.
            exit_code = finished_exit_code
            note = f'\nbackground processes still held the output after {timeout:g} s'
    finally:
        _signal_group(process.pid, signal.SIGKILL)

    return {
        'exit_code': exit_code,
        'seconds': round(time.monotonic() - started, 1),
        'output': (output or b'').decode(errors='replace') + note,
    }


def run_probe(paths: ContainerPaths, base_environment: Mapping[str, str]) -> list[dict]:
    results = []

    for step in probe_steps(str(paths.weights)):
        environment = child_environment(base_environment, step.environment)
        print(f'[probe] {step.name} (up to {step.timeout_seconds} s)', flush=True)
        outcome = run_captured(
            [paths.entrypoint, 'bash', '-c', step.script],
            environment,
            step.timeout_seconds + 15,
        )
        outcome['output'] = bounded_log(outcome['output'], OUTPUT_CHARS)
        outcome = apply_frame_check(step, outcome)
        print(
            f'[probe] {step.name}: exit {outcome["exit_code"]} after {outcome["seconds"]} s',
            flush=True,
        )
        results.append({'name': step.name, **outcome})

    return results


def udp_counters_text(snmp_path: Path = Path('/proc/net/snmp')) -> str:
    try:
        lines = snmp_path.read_text().splitlines()
    except OSError:
        return ''

    return ''.join(line + '\n' for line in lines if line.startswith('Udp:'))


def deep_sample_script(udp_counters_at_start: str = '') -> str:
    lines = [
        'echo "=== ps"; ps -eo pcpu,rss,args --sort=-pcpu | head -25 | cut -c1-200',
        'echo "=== top"; top -b -n 2 -d 3 -o %CPU | awk \'/^top -/{n++} n==2\' | head -30 | cut -c1-200',
        f'echo "=== gz stats"; {GZ_STATS_COMMAND.replace("-n 1", "-n 3")} | grep -E "real_time_factor"',
        # ArduPilot and Gazebo exchange lock-step packets on UDP port 9002. Modal runs gVisor,
        # whose /proc/net/udp reports queues and drops as 0 (proves the sockets exist, says
        # nothing on drops); the /proc/net/snmp counters are real but cumulative, so the
        # sampler-start reading is the baseline.
        'echo "=== udp sockets (gVisor reports drops and queues as 0)"; cat /proc/net/udp',
        'echo "=== udp counters at sampler start (snmp)"; '
        f'printf %s {shlex.quote(udp_counters_at_start)}',
        'echo "=== udp counters mid-flight (snmp; subtract the sampler-start counters)"; '
        'grep "^Udp:" /proc/net/snmp',
    ]
    lines += [
        f'echo "=== hz {topic}"; timeout 12 ros2 topic hz --window 50 {topic} 2>&1 | tail -2'
        for topic in SAMPLED_TOPICS
    ]

    return '\n'.join(lines) + '\n'


class _Sampler:
    """Every few seconds, appends load, memory headroom and Gazebo's real-time factor."""

    def __init__(
        self,
        paths: ContainerPaths,
        environment: Mapping[str, str],
        output: Path,
        interval: float,
    ):
        self.paths = paths
        self.environment = environment
        self.output = output
        self.interval = interval
        self.stop_event = threading.Event()
        self.factors: list[float] = []
        self.stats_samples: list[StatsSample] = []
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def _loop(self) -> None:
        started = time.monotonic()

        while not self.stop_event.is_set():
            stats = run_captured(
                [self.paths.entrypoint, 'bash', '-c', GZ_STATS_COMMAND],
                self.environment,
                20,
            )
            reading = read_stats_output(stats['output'])
            self.factors += reading.step_factors

            record_stats_reading(self.stats_samples, reading)
            readings = memory_readings()
            sample = {
                'seconds': round(time.monotonic() - started, 1),
                'load_1min': os.getloadavg()[0],
                'headroom_gib': lowest_headroom(list(readings.values())),
                **readings,
                **reading.row_fields,
            }

            with self.output.open('a') as handle:
                handle.write(json.dumps(sample) + '\n')

            self.stop_event.wait(self.interval)

    def start(self) -> None:
        self.output.touch()
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(SAMPLER_JOIN_SECONDS)


def _start_display(xvfb: str | None, warnings: list[str]) -> subprocess.Popen | None:
    if xvfb is None:
        return None

    try:
        display = subprocess.Popen(
            [xvfb, XVFB_DISPLAY, '-screen', '0', '1280x720x24', '-nolisten', 'tcp'],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except OSError as exception:
        warnings.append(f'Xvfb did not start ({exception}); the drone camera may not render')

        return None

    socket = Path(f'/tmp/.X11-unix/X{XVFB_DISPLAY[1:]}')
    waited_until = time.monotonic() + 10
    while not socket.exists() and time.monotonic() < waited_until and display.poll() is None:
        time.sleep(0.2)

    if not socket.exists():
        warnings.append(
            'Xvfb started but its display socket never appeared; the drone camera may not render',
        )

    return display


def _seconds_left(finish_by: float) -> float:
    return finish_by - time.monotonic()


def _stop_display(display: subprocess.Popen) -> None:
    _signal_group(display.pid, signal.SIGTERM)

    try:
        display.wait(DISPLAY_STOP_SECONDS)
    except subprocess.TimeoutExpired:
        _signal_group(display.pid, signal.SIGKILL)
        display.wait()


def _reindex_bag(
    paths: ContainerPaths,
    bag: BagState,
    environment: Mapping[str, str],
    finish_by: float,
    warnings: list[str],
) -> bool:
    storage_id = storage_id_for(bag.path)
    if storage_id is None:
        warnings.append(f'bag {bag.path.name} has no data files to reindex')

        return False

    if _seconds_left(finish_by) < MINIMUM_STEP_SECONDS:
        warnings.append(f'bag {bag.path.name} was not reindexed: the run is out of time')

        return False

    outcome = run_captured(
        [paths.entrypoint, 'ros2', 'bag', 'reindex', str(bag.path), '-s', storage_id],
        environment,
        min(REINDEX_SECONDS, _seconds_left(finish_by)),
    )
    if outcome['exit_code'] != 0 or not (bag.path / 'metadata.yaml').exists():
        warnings.append(
            f'reindexing bag {bag.path.name} failed: {bounded_log(outcome["output"], 500)}',
        )

        return False

    return True


def _eval_step(paths: ContainerPaths, arguments: list[str], timeout: float) -> dict:
    environment = {
        'PATH': '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
        'HOME': os.environ.get('HOME', '/root'),
        'CINE_CONFIG_ROOT': str(paths.repo_root / 'cine_config' / 'config'),
        'PYTHONPATH': f'{paths.repo_root / "cine_eval"}:{paths.repo_root / "cine_config"}',
        'PYTHONDONTWRITEBYTECODE': '1',
    }
    script = f'source {paths.ros_setup} >/dev/null 2>&1; exec "$0" -m cine_eval "$@"'
    command = ['bash', '-c', script, paths.eval_python, *arguments, '--json']

    return run_captured(command, environment, timeout)


def score_bag(
    paths: ContainerPaths,
    bag_path: Path,
    eval_directory: Path,
    finish_by: float,
) -> tuple[dict, str | None]:
    """extract, score and gate in turn; a failed or skipped step stops the chain.

    No step starts within MINIMUM_STEP_SECONDS of finish_by, and none runs past it.
    """

    scoring = {}
    eval_directory.mkdir(parents=True, exist_ok=True)
    steps = (
        ('extract', ['extract', str(bag_path), '--out', str(eval_directory)]),
        ('score', ['score', str(eval_directory)]),
        ('gate', ['gate', str(eval_directory)]),
    )
    for name, arguments in steps:
        left = _seconds_left(finish_by)
        if left < MINIMUM_STEP_SECONDS:
            print(f'[eval] {name} skipped: {left:.0f} s left before the deadline', flush=True)
            scoring[name] = {'exit_code': None, 'skipped': 'out of time'}

            return scoring, None

        print(f'[eval] {name}', flush=True)
        scoring[name] = _eval_step(paths, arguments, min(ALLOWANCE_SECONDS['scoring'] / 2, left))
        # The label is read before trimming, which could cut a long JSON line in half.
        full_output = scoring[name]['output']
        scoring[name]['output'] = bounded_log(full_output, OUTPUT_CHARS)

        # gate exits 1 for REVIEW, which is a verdict, not a failure.
        if name == 'gate':
            return scoring, gate_label(full_output)

        if scoring[name]['exit_code'] != 0:
            return scoring, None

    return scoring, None


def _copy_tree(source: Path, destination: Path, warnings: list[str]) -> None:
    try:
        shutil.copytree(source, destination, dirs_exist_ok=True, symlinks=False)
    except (OSError, shutil.Error) as exception:
        warnings.append(f'copying {source} to {destination} failed: {exception}')


def flight_limits(plan: SimRunPlan, timing: Timing, started: float) -> FlightLimits:
    budget = time_budget(plan)
    reserved = ALLOWANCE_SECONDS['shutdown']

    if plan.mode == 'mission':
        reserved += ALLOWANCE_SECONDS['scoring']

    return FlightLimits(
        flight_seconds=None if plan.mode == 'check' else float(plan.flight_seconds),
        deadline=started + max(budget.internal_deadline_seconds - reserved, 0),
        memory_stop_gib=timing.memory_stop_gib,
        poll_seconds=timing.poll_seconds,
        interrupt_grace_seconds=(
            max(timing.interrupt_grace_seconds, CHECK_INTERRUPT_GRACE_SECONDS)
            if plan.mode == 'check'
            else timing.interrupt_grace_seconds
        ),
        terminate_grace_seconds=timing.terminate_grace_seconds,
    )


def _fly_stack(
    plan: SimRunPlan,
    paths: ContainerPaths,
    timing: Timing,
    environment: Mapping[str, str],
    run_directory: Path,
    started: float,
    result: dict,
) -> None:
    warnings = result['warnings']
    sampler = _Sampler(paths, environment, run_directory / 'samples.jsonl', timing.sample_seconds)

    udp_counters_at_start = udp_counters_text()

    def deep_sample() -> None:
        outcome = run_captured(
            [paths.entrypoint, 'bash', '-c', deep_sample_script(udp_counters_at_start)],
            environment,
            DEEP_SAMPLE_SECONDS,
        )
        (run_directory / 'deep_sample.txt').write_text(outcome['output'])

    sampler.start()

    try:
        report = fly(
            [paths.entrypoint],
            environment,
            run_directory / 'launch.log',
            flight_limits(plan, timing, started),
            on_midflight=deep_sample,
        )
    finally:
        sampler.stop()

    finish_by = started + time_budget(plan).internal_deadline_seconds
    scan = scan_log(run_directory / 'launch.log', report.log_offset_at_stop)
    crashes = parse_crashes(scan.died_before_stop)
    result['crashes'] = [asdict(crash) for crash in crashes]
    result['shutdown_exits'] = [asdict(crash) for crash in parse_crashes(scan.died_after_stop)]
    result['flight'] = asdict(report)
    result['timings'].update(
        {'launch_seconds': report.launch_seconds, 'flight_seconds': report.flight_seconds},
    )
    result['memory'] = {'lowest_headroom_gib': report.lowest_headroom_gib}
    result['real_time_factor'] = summarize_intervals(sampler.stats_samples)
    result['step_real_time_factor'] = summarize_factors(sampler.factors)
    result['log_tail'] = scan.tail

    if not report.launch_started:
        _copy_tree(
            paths.colcon_base / 'log' / 'latest_build',
            run_directory / 'colcon_log',
            warnings,
        )

    bag = find_bag(paths.bags)
    reindexed = False

    if bag is not None and not bag.complete:
        reindexed = _reindex_bag(paths, bag, environment, finish_by, warnings)
        bag = BagState(bag.path, reindexed)

    if bag is None:
        result['bag'] = None
    else:
        result['bag'] = {'name': bag.path.name, 'complete': bag.complete, 'reindexed': reindexed}

    result['status'] = classify_outcome(
        OutcomeFacts(
            mode=plan.mode,
            launch_started=report.launch_started,
            stack_exit_code=report.stack_exit_code,
            stopped_by=report.stopped_by,
            bag_complete=bag is not None and bag.complete,
            crash_count=len(crashes),
        ),
    )
    # The bag copy has no time limit; a container killed during it still leaves this.
    _write_result(run_directory, result)

    if bag is not None:
        _copy_tree(bag.path, run_directory / 'bag', warnings)

    if plan.mode == 'mission' and bag is not None and bag.complete:
        # Scored from the local copy; reading the bag back off the volume would be slower.
        result['scoring'], result['gate'] = score_bag(
            paths,
            bag.path,
            run_directory / 'eval',
            finish_by,
        )


def _write_result(run_directory: Path, result: dict) -> None:
    try:
        result['files'] = sorted(
            {
                str(path.relative_to(run_directory))
                for path in run_directory.rglob('*')
                if path.is_file()
            }
            | {'result.json'},
        )
        (run_directory / 'result.json').write_text(json.dumps(result, indent=2))
    except OSError as exception:
        # The result is still returned to the caller and printed by the client.
        print(f'result.json could not be written: {exception}', flush=True)


def _set_aside_earlier_attempt(run_directory: Path) -> None:
    """Moves a preempted attempt's files aside, since Modal reruns it under the same id."""

    if not run_directory.is_dir():
        return

    leftovers = [path for path in run_directory.iterdir() if path.name != EARLIER_ATTEMPTS]
    if not leftovers:
        return

    attempts = run_directory / EARLIER_ATTEMPTS
    attempt_count = len(list(attempts.iterdir())) if attempts.is_dir() else 0
    attempt = attempts / str(attempt_count + 1)
    attempt.mkdir(parents=True)

    for path in leftovers:
        path.rename(attempt / path.name)


def _new_result(plan: SimRunPlan, parallelism: BuildParallelism) -> dict:
    return {
        'run_id': plan.run_id,
        'mode': plan.mode,
        'status': 'driver_error',
        'plan': asdict(plan),
        'parallelism': asdict(parallelism),
        'budget': asdict(time_budget(plan)),
        'timings': {},
        'crashes': [],
        'shutdown_exits': [],
        'memory': {'lowest_headroom_gib': None},
        'real_time_factor': None,
        'step_real_time_factor': None,
        'bag': None,
        'scoring': None,
        'gate': None,
        'warnings': [],
    }


def run_container(
    plan: SimRunPlan,
    paths: ContainerPaths,
    timing: Timing = DEFAULT_TIMING,
    commit: Callable[[], None] = lambda: None,
) -> dict:
    started = time.monotonic()
    parallelism = build_parallelism(plan.cpu_cores, plan.memory_gib)
    result = _new_result(plan, parallelism)
    # Written straight to the runs volume, so whatever finished before an out-of-memory
    # kill or a preemption is already saved by the volume's background commits.
    run_directory = paths.runs / plan.run_id
    display = None

    try:
        _set_aside_earlier_attempt(run_directory)

        for directory in (
            run_directory,
            paths.work,
            paths.bags,
            paths.colcon_base,
            paths.weights,
        ):
            directory.mkdir(parents=True, exist_ok=True)

        (run_directory / 'plan.json').write_text(json.dumps(asdict(plan), indent=2))
        defaults_path = paths.work / 'colcon-defaults.yaml'
        defaults_path.write_text(colcon_defaults_yaml(parallelism))
        environment = child_environment(
            os.environ,
            stack_environment(plan, parallelism, str(defaults_path)),
        )
        display = _start_display(paths.xvfb, result['warnings'])

        if plan.mode == 'probe':
            result['probe'] = run_probe(paths, environment)
            result['status'] = 'probe_done'
        else:
            _fly_stack(plan, paths, timing, environment, run_directory, started, result)
    except Exception:
        result['status'] = 'driver_error'
        result['driver_error'] = traceback.format_exc()
        print(result['driver_error'], flush=True)
    finally:
        if display is not None:
            _stop_display(display)

        elapsed = time.monotonic() - started
        result['timings']['total_seconds'] = round(elapsed, 1)
        estimate = estimate_cost(plan)
        result['cost'] = {
            'usd_per_hour': round(estimate.usd_per_hour, 4),
            'elapsed_usd': round(estimate.usd_per_hour * elapsed / 3600, 4),
        }
        _write_result(run_directory, result)

        # Broad on purpose: a failed commit must not turn a finished run into a
        # remote error; the client falls back to downloading what was committed.
        try:
            commit()
        except Exception as exception:
            print(f'The runs volume could not be committed: {exception}', flush=True)

    return result
