"""The waypoint generator node behaves for the Gazebo simulator, checked against real rclpy.

Each check runs in a fresh interpreter because test_cost_calculator.py replaces rclpy in
sys.modules for the rest of the session. The package source goes first on PYTHONPATH so
the check sees this tree rather than the last colcon install.
"""

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    shutil.which('ros2') is None,
    reason='needs ROS 2; runs inside cine-gazebo-sim:dev',
)

PACKAGE_SOURCE = Path(__file__).resolve().parents[1]
MISSION_FILE = PACKAGE_SOURCE.parent / 'cine_gazebo' / 'config' / 'mission_waypoints.yaml'

HARNESS = """
import os
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

from cine_agent.waypoint_automation.cinematic_waypoint_generator import (
    CinematicWaypointGenerator,
)

SUBJECT_TOPIC = '/cinematography/subject_position'
MISSION_PARAMETERS = [
    ('waypoints_file', os.environ['MISSION_FILE']),
    ('waypoint_xy_scale', '1.0'),
    ('waypoint_y_extra_scale', '1.0'),
    ('waypoint_z_scale', '1.0'),
    ('world_bounds_min', '[-12.0,-12.0,0.0]'),
    ('world_bounds_max', '[12.0,16.0,4.0]'),
]
LONG_ROUTE_PARAMETERS = [
    (name, os.environ['LONG_ROUTE_FILE'] if name == 'waypoints_file' else value)
    for name, value in MISSION_PARAMETERS
]
SUBJECT_BESIDE_ROUTE_MIDDLE = (3.0, 7.0, 0.0)
FRAMING_TOPIC = '/cinematography/framing_active'


def start(parameters=()):
    arguments = ['--ros-args']
    for name, value in parameters:
        arguments += ['-p', f'{name}:={value}']
    rclpy.init(args=arguments)
    node = CinematicWaypointGenerator()
    probe = Node('probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)

    return node, probe, executor


def spin_until(executor, condition, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if condition():
            return True
        executor.spin_once(timeout_sec=0.05)

    return condition()


def spin_for(executor, seconds):
    spin_until(executor, lambda: False, seconds)


def odometry_at(x, y, z):
    message = Odometry()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.position.z = z
    message.pose.pose.orientation.w = 1.0

    return message


def subject_at(x, y, z):
    message = PointStamped()
    message.header.frame_id = 'world'
    message.point.x = x
    message.point.y = y
    message.point.z = z

    return message


def collect(probe, message_type, topic, qos=10):
    received = []
    probe.create_subscription(message_type, topic, received.append, qos)

    return received


def publisher_for(probe, message_type, topic, executor):
    publisher = probe.create_publisher(message_type, topic, 10)
    assert spin_until(executor, lambda: publisher.get_subscription_count() >= 1, 5)

    return publisher


def shut_down(node, probe, executor):
    executor.shutdown()
    probe.destroy_node()
    node.destroy_node()
    rclpy.shutdown()
"""

SWITCHED_OFF_SUBJECT_POSITION = """
node, probe, executor = start(LONG_ROUTE_PARAMETERS + [('publish_subject_position', 'false')])
received = collect(probe, PointStamped, SUBJECT_TOPIC)
odometry_pub = publisher_for(probe, Odometry, node.odom_topic, executor)
odometry_pub.publish(odometry_at(0.0, 0.0, 0.0))
assert spin_until(executor, lambda: node.start_pos is not None, 5)
subject_pub = publisher_for(probe, PointStamped, SUBJECT_TOPIC, executor)
subject_pub.publish(subject_at(*SUBJECT_BESIDE_ROUTE_MIDDLE))
assert spin_until(executor, lambda: node.target_world is not None, 5)
spin_for(executor, 0.5)
assert len(received) == 1, f'expected 1 message, got {len(received)}'
shut_down(node, probe, executor)
"""

LATE_SUBSCRIBER_CUE = """
node, probe, executor = start()
node._publish_framing_active(True)
spin_for(executor, 0.2)
latched = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
received = collect(probe, Bool, FRAMING_TOPIC, latched)
assert spin_until(executor, lambda: len(received) > 0, 3), 'late subscriber got nothing'
assert received[0].data is True
shut_down(node, probe, executor)
"""

HOLD_GOALS_FOR_ESTIMATE = """
node, probe, executor = start(LONG_ROUTE_PARAMETERS + [('subject_wait_timeout_s', '30.0')])
goals = collect(probe, Path, node.route_topic)
odometry_pub = publisher_for(probe, Odometry, node.odom_topic, executor)
odometry_pub.publish(odometry_at(0.0, 0.0, 0.0))
assert spin_until(executor, lambda: node.start_pos is not None, 5)
spin_for(executor, node.STARTUP_DELAY + 1.0)
assert goals == [], f'goals published without an estimate: {len(goals)}'
subject_pub = publisher_for(probe, PointStamped, SUBJECT_TOPIC, executor)
subject_pub.publish(subject_at(*SUBJECT_BESIDE_ROUTE_MIDDLE))
assert spin_until(executor, lambda: len(goals) > 0, 5), 'no goal after the estimate arrived'
assert node.dynamic_waypoints is not None
shut_down(node, probe, executor)
"""

HOLD_GOALS_FOR_START_TRIGGER = """
node, probe, executor = start(LONG_ROUTE_PARAMETERS + [('start_trigger_topic', '/traj_start_trigger')])
goals = collect(probe, Path, node.route_topic)
odometry_pub = publisher_for(probe, Odometry, node.odom_topic, executor)
odometry_pub.publish(odometry_at(0.0, 0.0, 0.0))
assert spin_until(executor, lambda: node.start_pos is not None, 5)
spin_for(executor, node.STARTUP_DELAY + 1.0)
assert goals == [], f'goals published before the start trigger: {len(goals)}'
trigger_pub = publisher_for(probe, PoseStamped, '/traj_start_trigger', executor)
trigger_pub.publish(PoseStamped())
assert spin_until(executor, lambda: len(goals) > 0, 5), 'no goal after the start trigger'
shut_down(node, probe, executor)
"""

FLY_STATIC_ROUTE_AFTER_TIMEOUT = """
node, probe, executor = start(LONG_ROUTE_PARAMETERS + [('subject_wait_timeout_s', '0.5')])
warnings = []


class RecordingLogger:
    def warn(self, message, *args, **kwargs):
        warnings.append(message)

    def __getattr__(self, name):
        return lambda *args, **kwargs: None


node.get_logger = lambda: RecordingLogger()
goals = collect(probe, Path, node.route_topic)
odometry_pub = publisher_for(probe, Odometry, node.odom_topic, executor)
odometry_pub.publish(odometry_at(0.0, 0.0, 0.0))
assert spin_until(executor, lambda: len(goals) > 0, 10), 'no goal after the wait timed out'
assert any('subject estimate' in message for message in warnings), warnings
shut_down(node, probe, executor)
"""

FOUR_POINT_MISSION_AROUND_SUBJECT = """
node, probe, executor = start(MISSION_PARAMETERS)
goals = collect(probe, Path, node.route_topic)
odometry_pub = publisher_for(probe, Odometry, node.odom_topic, executor)
odometry_pub.publish(odometry_at(0.0, 0.0, 0.0))
assert spin_until(executor, lambda: node.start_pos is not None, 5)
subject_pub = publisher_for(probe, PointStamped, SUBJECT_TOPIC, executor)
subject_pub.publish(subject_at(0.0, 10.45, 1.0))
assert spin_until(executor, lambda: len(goals) > 0, 10), 'no goal'
assert node.dynamic_waypoints is not None
spin_for(executor, 1.0)
shut_down(node, probe, executor)
"""

FILE_WAYPOINTS_AT_ORIGIN = """
node, probe, executor = start(MISSION_PARAMETERS)
odometry_pub = publisher_for(probe, Odometry, node.odom_topic, executor)
odometry_pub.publish(odometry_at(0.0, 0.0, 0.0))
assert spin_until(executor, lambda: node.start_pos is not None, 5)
assert len(node.static_waypoints) == 4
for waypoint in node.static_waypoints:
    assert node._rel_to_world(waypoint) == waypoint, waypoint
shut_down(node, probe, executor)
"""


LONG_ROUTE_POINTS = [[0, 2, 2], [0, 4, 2], [0, 6, 2], [0, 8, 2], [0, 10, 2], [0, 12, 2]]


def _run_against_package_source(script, long_route_file):
    environment = dict(
        os.environ,
        MISSION_FILE=str(MISSION_FILE),
        LONG_ROUTE_FILE=str(long_route_file),
        PYTHONPATH=os.pathsep.join([str(PACKAGE_SOURCE), os.environ.get('PYTHONPATH', '')]),
    )

    return subprocess.run(
        [sys.executable, '-c', HARNESS + script],
        env=environment,
        capture_output=True,
        text=True,
        timeout=120,
    )


def _assert_passes(script, tmp_path):
    long_route_file = tmp_path / 'long_route.yaml'
    long_route_file.write_text(json.dumps(LONG_ROUTE_POINTS))
    result = _run_against_package_source(script, long_route_file)

    assert result.returncode == 0, result.stderr[-3000:]


def test_generator_does_not_republish_subject_position_when_switched_off(tmp_path):
    _assert_passes(SWITCHED_OFF_SUBJECT_POSITION, tmp_path)


def test_late_subscriber_receives_current_filming_cue(tmp_path):
    _assert_passes(LATE_SUBSCRIBER_CUE, tmp_path)


def test_generator_holds_goals_until_subject_estimate_arrives(tmp_path):
    _assert_passes(HOLD_GOALS_FOR_ESTIMATE, tmp_path)


def test_generator_holds_goals_until_start_trigger(tmp_path):
    _assert_passes(HOLD_GOALS_FOR_START_TRIGGER, tmp_path)


def test_generator_logs_and_flies_static_route_after_subject_wait_timeout(tmp_path):
    _assert_passes(FLY_STATIC_ROUTE_AFTER_TIMEOUT, tmp_path)


def test_file_waypoints_are_unchanged_when_drone_starts_at_origin(tmp_path):
    _assert_passes(FILE_WAYPOINTS_AT_ORIGIN, tmp_path)


def test_generator_builds_route_around_subject_with_the_four_point_mission(tmp_path):
    _assert_passes(FOUR_POINT_MISSION_AROUND_SUBJECT, tmp_path)
