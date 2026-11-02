"""WaypointSelector sweeps distinct candidates and publishes them for RViz.

The pure-Python CI job has no ROS, and test_cost_calculator replaces rclpy and geometry_msgs
in sys.modules during collection, so a real node is not available here. Small fakes stand in
for the ROS modules only while the selector is imported, which leaves sys.modules as found.
"""

import math
import sys
from dataclasses import dataclass, field
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

CANDIDATES_TOPIC = '/cinematography/candidates'
CLOCK_STAMP = 'clock-stamp'


@dataclass
class FakePoint:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class FakeColor:
    r: float = 0.0
    g: float = 0.0
    b: float = 0.0
    a: float = 0.0


@dataclass
class FakeScale:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class FakeOrientation:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class FakePose:
    position: FakePoint = field(default_factory=FakePoint)
    orientation: FakeOrientation = field(default_factory=FakeOrientation)


@dataclass
class FakeHeader:
    stamp: object = None
    frame_id: str = ''


@dataclass
class FakeMarker:
    SPHERE = 2
    SPHERE_LIST = 7
    ADD = 0

    header: FakeHeader = field(default_factory=FakeHeader)
    ns: str = ''
    id: int = 0
    type: int = 0
    action: int = 0
    pose: FakePose = field(default_factory=FakePose)
    scale: FakeScale = field(default_factory=FakeScale)
    color: FakeColor = field(default_factory=FakeColor)
    points: list = field(default_factory=list)


class FakePublisher:
    def __init__(self, message_type, queue_size):
        self.message_type = message_type
        self.queue_size = queue_size
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.publishers = {}

    def create_publisher(self, message_type, topic, queue_size):
        publisher = FakePublisher(message_type, queue_size)
        self.publishers[topic] = publisher

        return publisher

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: CLOCK_STAMP))


def _fake_module(name, **attributes):
    module = ModuleType(name)
    module.__dict__.update(attributes)

    return module


_FAKE_ROS_MODULES = {
    'rclpy': _fake_module('rclpy'),
    'rclpy.node': _fake_module('rclpy.node', Node=object),
    'geometry_msgs': _fake_module('geometry_msgs'),
    'geometry_msgs.msg': _fake_module('geometry_msgs.msg', Point=FakePoint),
    'visualization_msgs': _fake_module('visualization_msgs'),
    'visualization_msgs.msg': _fake_module('visualization_msgs.msg', Marker=FakeMarker),
}

with patch.dict(sys.modules, _FAKE_ROS_MODULES):
    from cine_agent.setpoint_adjustment.controllers.waypoint_selector import WaypointSelector


def _config():
    return SimpleNamespace(
        params=SimpleNamespace(ANGULAR_VELOCITY_RPS=0.2),
        candidate_angle_samples=7,
        candidate_angle_window_rad=0.6,
        candidate_radius_samples=5,
        candidate_radius_range_m=[0.8, 2.0],
    )


def _cost_favouring_positive_x(x, _y, _z):
    return -x


def _cost_rejecting_everything(_x, _y, _z):
    return None


def _select(radius_m=1.5, cost_of_position=_cost_favouring_positive_x):
    node = FakeNode()
    evaluated_positions = []

    def recording_cost(x, y, z, *_target_and_drone):
        evaluated_positions.append((x, y, z))

        return cost_of_position(x, y, z)

    selector = WaypointSelector(node, _config())
    best = selector.select_next_waypoint(
        center_xy=(0.0, 0.0),
        radius_m=radius_m,
        altitude_m=1.5,
        target_world=np.zeros(3),
        drone_position=np.zeros(3),
        cost_estimation_func=recording_cost,
        dt=0.0,
    )

    return SimpleNamespace(node=node, evaluated_positions=evaluated_positions, best=best)


def _distinct_radii(positions):
    return sorted({round(math.hypot(x, y), 6) for x, y, _ in positions})


def _point_tuples(marker):
    return [(point.x, point.y, point.z) for point in marker.points]


def _published_markers(selection):
    return selection.node.publishers[CANDIDATES_TOPIC].messages


def test_radius_inside_range_sweeps_five_distinct_radii():
    selection = _select(radius_m=1.5)

    assert _distinct_radii(selection.evaluated_positions) == pytest.approx(
        [1.05, 1.2875, 1.525, 1.7625, 2.0]
    )


def test_every_one_of_the_35_candidates_is_a_distinct_position():
    selection = _select(radius_m=1.5)

    assert len(set(selection.evaluated_positions)) == 35


def test_radius_below_range_falls_back_to_the_configured_range():
    selection = _select(radius_m=0.2)

    assert _distinct_radii(selection.evaluated_positions) == pytest.approx(
        [0.8, 1.1, 1.4, 1.7, 2.0]
    )


def test_radius_above_range_falls_back_to_the_configured_range():
    selection = _select(radius_m=3.0)

    assert _distinct_radii(selection.evaluated_positions) == pytest.approx(
        [0.8, 1.1, 1.4, 1.7, 2.0]
    )


def test_selector_creates_a_marker_publisher_with_queue_ten():
    selection = _select()

    publisher = selection.node.publishers[CANDIDATES_TOPIC]

    assert publisher.message_type is FakeMarker
    assert publisher.queue_size == 10


def test_candidate_marker_holds_all_35_evaluated_points():
    selection = _select()

    candidate_marker = _published_markers(selection)[0]

    assert len(candidate_marker.points) == 35
    assert _point_tuples(candidate_marker) == selection.evaluated_positions


def test_candidate_marker_is_a_small_grey_sphere_list_in_the_world_frame():
    selection = _select()

    candidate_marker = _published_markers(selection)[0]

    assert candidate_marker.ns == 'candidates'
    assert candidate_marker.id == 0
    assert candidate_marker.type == FakeMarker.SPHERE_LIST
    assert candidate_marker.action == FakeMarker.ADD
    assert candidate_marker.header.frame_id == 'world'
    assert candidate_marker.header.stamp == CLOCK_STAMP
    assert candidate_marker.scale == FakeScale(0.08, 0.08, 0.08)
    assert candidate_marker.color == FakeColor(0.9, 0.9, 0.9, 0.9)


def test_winner_marker_sits_at_the_returned_waypoint():
    selection = _select()

    position = _published_markers(selection)[1].pose.position

    assert (position.x, position.y, position.z) == tuple(selection.best)
    assert selection.best == pytest.approx([2.0, 0.0, 1.5])


def test_winner_marker_is_a_larger_opaque_green_sphere_in_the_world_frame():
    selection = _select()

    winner_marker = _published_markers(selection)[1]

    assert winner_marker.ns == 'candidates'
    assert winner_marker.id == 1
    assert winner_marker.type == FakeMarker.SPHERE
    assert winner_marker.action == FakeMarker.ADD
    assert winner_marker.header.frame_id == 'world'
    assert winner_marker.header.stamp == CLOCK_STAMP
    assert winner_marker.scale == FakeScale(0.16, 0.16, 0.16)
    assert winner_marker.color == FakeColor(0.0, 1.0, 0.0, 1.0)
    assert winner_marker.pose.orientation.w == 1.0


def test_winner_marker_position_holds_plain_floats_not_numpy_values():
    selection = _select()

    position = _published_markers(selection)[1].pose.position

    assert type(position.x) is float
    assert type(position.y) is float
    assert type(position.z) is float


def test_rejected_candidates_are_still_shown_with_no_winner_marker():
    selection = _select(cost_of_position=_cost_rejecting_everything)

    markers = _published_markers(selection)

    assert selection.best is None
    assert len(markers) == 1
    assert len(markers[0].points) == 35
