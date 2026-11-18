"""Smoke flight: launch the Gazebo stack, let the flight starter take off, fly 2 m north.

Run inside the gazebo-sim image (the entrypoint runs it when CHECK=true):
    python3 cine_gazebo/test/smoke_flight.py
It launches gazebo_sim.launch.py with mission:=false, waits for sitl_flight_starter to
exit 0 (with NAV_SOURCE=vins that happens only after the starter has switched the autopilot
to VINS and confirmed the hold, so the setpoint is published after the switch), publishes one
setpoint 2 m north of the takeoff point at takeoff altitude through the Pi's setpoint relay,
and exits 0 when both the autopilot's estimate and Gazebo's true
pose come within 0.5 m of it inside 30 s of sim time. HEADLESS, NAV_SOURCE and WIND are
passed through to the launch.

The pass criterion, `reached_setpoint`, is pure so it is unit tested without ROS; rclpy
is imported only when the flight runs. The file name does not start with test_, so
pytest never collects it.
"""

import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass

from cine_gazebo.flight_starter_logic import StarterConfig

AUTOPILOT_SOURCE = 'autopilot'
GROUND_TRUTH_SOURCE = 'ground_truth'
REQUIRED_SOURCES = (AUTOPILOT_SOURCE, GROUND_TRUTH_SOURCE)

NORTH_OFFSET_M = 2.0
TOLERANCE_M = 0.5
DEADLINE_S = 30.0
SETPOINT_TOPIC = '/cinematography/translational_position_setpoints'
AUTOPILOT_ODOMETRY_TOPIC = '/mavros/local_position/odom'
GROUND_TRUTH_POSE_TOPIC = '/gazebo/drone_pose'

# Wall-clock bounds, so a simulator that never starts cannot hang the check forever.
STARTER_WAIT_WALL_S = 900.0
RELAY_DISCOVERY_WALL_S = 30.0
FLIGHT_WALL_S = 300.0
LAUNCH_SHUTDOWN_WALL_S = 30.0
SPIN_TIMEOUT_S = 0.1

# ros2 launch reports each process's exit in these forms (launch.actions.ExecuteLocal).
STARTER_FINISHED_PATTERN = re.compile(r'\[sitl_flight_starter-\d+\]: process has finished cleanly')
STARTER_DIED_PATTERN = re.compile(
    r'\[sitl_flight_starter-\d+\]: process has died \[pid \d+, exit code (-?\d+)'
)


@dataclass(frozen=True)
class PositionSample:
    # Seconds of sim time since the setpoint was published.
    time_s: float
    source: str
    # Metres from this source's own takeoff point, east-north-up.
    position: tuple[float, float, float]


def closest_distances(samples, target):
    """Return each required source's closest approach to `target`; inf if it never reported."""
    closest = dict.fromkeys(REQUIRED_SOURCES, math.inf)
    for sample in samples:
        if sample.source in closest:
            closest[sample.source] = min(closest[sample.source], math.dist(sample.position, target))

    return closest


def reached_setpoint(samples, target, tolerance_m, deadline_s):
    """True when every required source came within `tolerance_m` of `target` by `deadline_s`."""
    in_time = [sample for sample in samples if sample.time_s <= deadline_s]
    closest = closest_distances(in_time, target)

    return all(closest[source] <= tolerance_m for source in REQUIRED_SOURCES)


def _format_distances(distances):
    return ', '.join(f'{source} {distance:.2f} m' for source, distance in distances.items())


class _LaunchWatcher:
    """Echoes the launch's output and notes how sitl_flight_starter exited."""

    def __init__(self, process):
        self.process = process
        self.starter_exited = threading.Event()
        self.starter_exit_code = None
        self._thread = threading.Thread(target=self._read_output, daemon=True)
        self._thread.start()

    def _read_output(self):
        for line in self.process.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            if self.starter_exited.is_set():
                continue
            if STARTER_FINISHED_PATTERN.search(line):
                self.starter_exit_code = 0
                self.starter_exited.set()
                continue
            died = STARTER_DIED_PATTERN.search(line)
            if died:
                self.starter_exit_code = int(died.group(1))
                self.starter_exited.set()


def _start_launch():
    command = [
        'ros2',
        'launch',
        'cine_gazebo',
        'gazebo_sim.launch.py',
        'mission:=false',
        f'headless:={os.environ.get("HEADLESS", "true")}',
        f'nav_source:={os.environ.get("NAV_SOURCE", "sim")}',
        f'wind:={os.environ.get("WIND", "calm")}',
    ]
    print('Starting:', ' '.join(command), flush=True)

    return subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def _stop_launch(process):
    if process.poll() is not None:
        return
    for stop_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        os.killpg(process.pid, stop_signal)
        try:
            process.wait(timeout=LAUNCH_SHUTDOWN_WALL_S)

            return
        except subprocess.TimeoutExpired:
            continue


def _fly(watcher):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )

    takeoff_altitude_m = StarterConfig(nav_source='sim').takeoff_altitude_m
    target = (0.0, NORTH_OFFSET_M, takeoff_altitude_m)

    rclpy.init()
    node = rclpy.create_node(
        'smoke_flight', parameter_overrides=[Parameter('use_sim_time', value=True)]
    )
    takeoff_points = {}
    latest_autopilot_pose = None
    samples = []
    setpoint_published_s = None

    def now_s():
        return node.get_clock().now().nanoseconds * 1e-9

    def record(source, point):
        # The first pose each source reports is taken before arming: the starter waits
        # for the EKF's local position before it asks for GUIDED and arms.
        takeoff_points.setdefault(source, point)
        if setpoint_published_s is None:
            return
        takeoff_point = takeoff_points[source]
        samples.append(
            PositionSample(
                time_s=now_s() - setpoint_published_s,
                source=source,
                position=tuple(
                    coordinate - origin
                    for coordinate, origin in zip(point, takeoff_point, strict=True)
                ),
            )
        )

    def on_autopilot_odometry(message):
        nonlocal latest_autopilot_pose
        latest_autopilot_pose = message.pose.pose
        position = message.pose.pose.position
        record(AUTOPILOT_SOURCE, (position.x, position.y, position.z))

    def on_ground_truth_pose(message):
        position = message.pose.position
        record(GROUND_TRUTH_SOURCE, (position.x, position.y, position.z))

    node.create_subscription(
        Odometry, AUTOPILOT_ODOMETRY_TOPIC, on_autopilot_odometry, qos_profile_sensor_data
    )
    node.create_subscription(
        PoseStamped, GROUND_TRUTH_POSE_TOPIC, on_ground_truth_pose, qos_profile_sensor_data
    )
    # Same QoS as the relay's subscription in cine_sensing_setpoint/mavros_setpoints.py.
    relay_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    setpoint_publisher = node.create_publisher(PoseStamped, SETPOINT_TOPIC, relay_qos)

    try:
        wall_deadline = time.monotonic() + STARTER_WAIT_WALL_S
        while not watcher.starter_exited.is_set():
            if watcher.process.poll() is not None:
                print('FAIL: the launch exited before sitl_flight_starter finished', flush=True)

                return 1
            if time.monotonic() > wall_deadline:
                print(
                    f'FAIL: sitl_flight_starter did not exit within {STARTER_WAIT_WALL_S:g} s '
                    'of wall time',
                    flush=True,
                )

                return 1
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)
        if watcher.starter_exit_code != 0:
            print(f'FAIL: sitl_flight_starter exited {watcher.starter_exit_code}', flush=True)

            return 1

        missing = [source for source in REQUIRED_SOURCES if source not in takeoff_points]
        if missing or latest_autopilot_pose is None:
            print(f'FAIL: no takeoff point from {missing}', flush=True)

            return 1

        wall_deadline = time.monotonic() + RELAY_DISCOVERY_WALL_S
        while setpoint_publisher.get_subscription_count() == 0:
            if time.monotonic() > wall_deadline:
                print(f'FAIL: nothing subscribes to {SETPOINT_TOPIC}', flush=True)

                return 1
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)

        takeoff_point = takeoff_points[AUTOPILOT_SOURCE]
        setpoint = PoseStamped()
        setpoint.header.stamp = node.get_clock().now().to_msg()
        setpoint.header.frame_id = 'map'
        setpoint.pose.position.x = takeoff_point[0] + target[0]
        setpoint.pose.position.y = takeoff_point[1] + target[1]
        setpoint.pose.position.z = takeoff_point[2] + target[2]
        # Keep the current heading, so the check measures translation only.
        setpoint.pose.orientation = latest_autopilot_pose.orientation
        setpoint_publisher.publish(setpoint)
        setpoint_published_s = now_s()
        print(
            f'Published setpoint {target} m from the takeoff point at sim time '
            f'{setpoint_published_s:.1f} s',
            flush=True,
        )

        wall_deadline = time.monotonic() + FLIGHT_WALL_S
        while now_s() - setpoint_published_s <= DEADLINE_S:
            if reached_setpoint(samples, target, TOLERANCE_M, DEADLINE_S):
                break
            if time.monotonic() > wall_deadline:
                print(f'Sim time stalled; stopping after {FLIGHT_WALL_S:g} s of wall time')
                break
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)

        distances = _format_distances(closest_distances(samples, target))
        if reached_setpoint(samples, target, TOLERANCE_M, DEADLINE_S):
            print(f'PASS: closest distance to the setpoint: {distances}', flush=True)

            return 0
        print(
            f'FAIL: not within {TOLERANCE_M:g} m inside {DEADLINE_S:g} s of sim time; '
            f'closest distance to the setpoint: {distances}',
            flush=True,
        )

        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    process = _start_launch()
    try:
        return _fly(_LaunchWatcher(process))
    finally:
        _stop_launch(process)


if __name__ == '__main__':
    sys.exit(main())
