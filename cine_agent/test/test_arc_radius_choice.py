"""The cost model picks the arc radius, and a moving subject rebuilds the arc only before filming.

A rebuilt route resumes where the drone is, within the same part of the route (before,
during or after filming), instead of at the route's first waypoint.

The pure-Python CI job has no ROS, and test_cost_calculator replaces rclpy and the message
packages in sys.modules during collection, so a real node is not available here. Small fakes
stand in for the ROS modules only while the mixins are imported, which leaves sys.modules as
found. The mixins and the cost model under them are the real code.
"""

import math
import re
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import numpy  # noqa: F401  patch.dict would drop a numpy first imported inside it, forcing a reload.
import pytest
from cine_config.constants import BsplineFilterParams, CinematographyParams, WaypointParams

SUBJECT = (0.0, 10.45, 1.0)
ANCHOR_START = (-5.1, 6.0, 3.0)
ANCHOR_END = (-5.1, 13.0, 3.0)
ROUTE = [
    (-5.1, 0.0, 3.0),
    ANCHOR_START,
    (-5.1, 10.0, 3.0),
    ANCHOR_END,
    (-5.1, 16.0, 3.0),
]
ARC_CHOICE_PATTERN = re.compile(
    r'Arc radius chosen: (?P<chosen>[\d.]+) m; candidates: (?P<rest>.*)',
)


class FakeMessage:
    SPHERE = 2
    LINE_STRIP = 4
    ADD = 0
    DELETE = 2

    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id='')
        self.point = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        self.scale = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.color = SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0)
        self.markers = []
        self.points = []
        self.data = None


def _fake_module(name, **attributes):
    module = ModuleType(name)
    module.__dict__.update(attributes)

    return module


_MESSAGE_NAMES = (
    'Point',
    'PointStamped',
    'PolygonStamped',
    'PoseStamped',
    'Vector3Stamped',
)
_FAKE_ROS_MODULES = {
    'rclpy': _fake_module('rclpy'),
    'cv2': _fake_module('cv2'),
    'geometry_msgs': _fake_module('geometry_msgs'),
    'geometry_msgs.msg': _fake_module(
        'geometry_msgs.msg',
        **{name: FakeMessage for name in _MESSAGE_NAMES},
    ),
    'nav_msgs': _fake_module('nav_msgs'),
    'nav_msgs.msg': _fake_module('nav_msgs.msg', Odometry=FakeMessage),
    'std_msgs': _fake_module('std_msgs'),
    'std_msgs.msg': _fake_module(
        'std_msgs.msg',
        Bool=FakeMessage,
        Float32=FakeMessage,
        Float32MultiArray=FakeMessage,
    ),
    'visualization_msgs': _fake_module('visualization_msgs'),
    'visualization_msgs.msg': _fake_module(
        'visualization_msgs.msg',
        Marker=FakeMessage,
        MarkerArray=FakeMessage,
    ),
}
with patch.dict(sys.modules, _FAKE_ROS_MODULES):
    from cine_agent.cost_functions.cost_calculator import (
        CostCalculator,
        ParameterNotDeclaredException,
    )
    from cine_agent.waypoint_automation.anchors.anchor_mixin import AnchorMixin
    from cine_agent.waypoint_automation.detection.detection_mixin import DetectionMixin
    from cine_agent.waypoint_automation.pathing.path_builder import PathBuilderMixin
    from cine_agent.waypoint_automation.sequencing.waypoint_sequencer import (
        WaypointSequencerMixin,
    )
    from cine_agent.waypoint_automation.utils.geometry_mixin import GeometryMixin
    from cine_agent.waypoint_automation.viz.visualization_mixin import VisualizationMixin


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class RecordingLogger:
    def __init__(self):
        self.info_messages = []

    def info(self, message, **kwargs):
        self.info_messages.append(message)

    def debug(self, message, **kwargs):
        pass

    def warn(self, message, **kwargs):
        pass

    def warning(self, message, **kwargs):
        pass


class ConstantCost:
    def __init__(self, cost):
        self.cost = cost

    def estimate_position_cost_at_position(self, *args, **kwargs):
        return self.cost


class ArcPlanningNode(
    DetectionMixin,
    AnchorMixin,
    PathBuilderMixin,
    WaypointSequencerMixin,
    VisualizationMixin,
    GeometryMixin,
):
    """The generator's planning mixins with their state, but no rclpy node underneath."""

    def __init__(self, enable_cost_optimization=True):
        self.parameters = {}
        self.logger = RecordingLogger()
        self.frame_id = 'world'
        self.static_waypoints = list(ROUTE)
        self.waypoint_xy_scale = 1.0
        self.waypoint_y_extra_scale = 1.0
        self.waypoint_z_scale = 1.0
        self.origin_offset = (0.0, 0.0, 0.0)
        self.start_pos = ROUTE[0]
        self.current_pos = ROUTE[0]
        self.current_yaw = 0.0
        self.world_bounds_min = (-20.0, -20.0, 0.0)
        self.world_bounds_max = (20.0, 30.0, 4.0)
        self.enable_z_clamp = False
        self.min_altitude_m = float(BsplineFilterParams.MIN_ALTITUDE_M)
        self.max_altitude_m = float(BsplineFilterParams.MAX_ALTITUDE_M)
        self.focus_altitude_m = float(BsplineFilterParams.FOCUS_ALTITUDE_M)
        self.min_orbit_radius_m = float(WaypointParams.MIN_ORBIT_RADIUS_M)
        self.min_focus_distance_m = float(CinematographyParams().MIN_FOCUS_DISTANCE_M)
        self.arc_waypoint_spacing_m = float(WaypointParams.ARC_WAYPOINT_SPACING_M)
        self.anchor_min_gap = int(WaypointParams.ANCHOR_MIN_GAP)
        self.bezier_blend_length_m = float(WaypointParams.BEZIER_BLEND_LENGTH_M)
        self.bezier_transition_points = int(WaypointParams.BEZIER_TRANSITION_POINTS)
        self.min_waypoint_separation_m = float(WaypointParams.MIN_WAYPOINT_SEPARATION_M)
        self.candidate_radius_samples = int(WaypointParams.CANDIDATE_RADIUS_SAMPLES)
        self.candidate_radius_scale_range = [
            float(value) for value in WaypointParams.CANDIDATE_RADIUS_RANGE_M
        ]
        self.enable_cost_optimization = enable_cost_optimization
        self.use_dynamic_waypoints_on_detection = True
        self.use_dynamic_waypoints = True
        self.pending_detection_merge = False
        self.baseline_mode = False
        self.simulate_object = False
        self.debug_arc = False
        self.fx = 500.0
        self.fy = 500.0
        self.cx = 320.0
        self.cy = 240.0
        self.image_width = 640
        self.image_height = 480
        self.detected_object = None
        self.target_world = None
        self.last_target_world = None
        self.focus_depth_m = None
        self.anchor_start_idx = None
        self.anchor_end_idx = None
        self.anchor_start_world = None
        self.anchor_end_world = None
        self.anchor_focus_distance = None
        self.anchor_source = None
        self.anchors_set = False
        self.anchor_marker_scale_m = 0.28
        self.anchor_start_pub = FakePublisher()
        self.anchor_end_pub = FakePublisher()
        self.anchor_marker_pub = FakePublisher()
        self.anchor_distance_pub = FakePublisher()
        self._last_published_target_stamp = None
        self.viz_data = {}
        self.generated_arc_length = 0
        self.dynamic_waypoints = None
        self.last_cinematic_waypoints = None
        self.active_waypoints = None
        self.current_target_index = 0
        self.last_published_index = -1
        self.waypoints_sent = False
        self.framing_start_idx = None
        self.framing_end_idx = None
        self.framing_active = False
        self.sequence_initializations = 0
        self.cost_calculator = CostCalculator(self)

    def declare_parameter(self, name, default):
        self.parameters.setdefault(name, default)

        return SimpleNamespace(value=self.parameters[name])

    def get_parameter(self, name):
        if name not in self.parameters:
            raise ParameterNotDeclaredException(name)

        return SimpleNamespace(value=self.parameters[name])

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'stamp'))

    def _initialize_waypoint_sequence(self, resume_phase=None):
        self.sequence_initializations += 1


class RouteFlyingNode(ArcPlanningNode):
    """The planning node with the real sequence initializer, so a rebuild publishes goals."""

    _initialize_waypoint_sequence = WaypointSequencerMixin._initialize_waypoint_sequence

    def __init__(self):
        super().__init__()
        self.max_waypoint_jump_m = float(WaypointParams.MAX_WAYPOINT_JUMP_M)
        self.focus_entry_decel_waypoints = 0
        self.waypoint_radius_m = float(WaypointParams.WAYPOINT_RADIUS_M)
        self.min_goal_distance_m = float(WaypointParams.MIN_GOAL_DISTANCE_M)
        self.min_first_waypoint_distance_m = float(WaypointParams.MIN_FIRST_WAYPOINT_DISTANCE_M)
        self.PROGRESS_EPS_M = 0.05
        self.WAYPOINT_LOOKAHEAD = int(WaypointParams.WAYPOINT_LOOKAHEAD)
        self.dynamic_waypoint_lookahead = int(WaypointParams.DYNAMIC_WAYPOINT_LOOKAHEAD)
        self.light_avoidance_enabled = False
        self.goal_pub = FakePublisher()
        self.framing_active_pub = FakePublisher()
        self.baseline_framing_pub = FakePublisher()
        self.baseline_framing_active = False
        self.sent_waypoints_history = []
        self.last_progress_time = None
        self.last_progress_dist = None
        self.stuck_attempts = 0


def _node_flying_the_route(passed_waypoint):
    """A node that started the route and is halfway from `passed_waypoint` to the next one.

    The route runs along +y; its filming range is waypoints 7-12 of 0-18.
    """

    node = RouteFlyingNode()
    node.target_world = SUBJECT
    node.generate_cinematographic_waypoints()
    node._initialize_waypoint_sequence()
    passed = node.active_waypoints[passed_waypoint]
    upcoming = node.active_waypoints[passed_waypoint + 1]
    node.current_pos = tuple(
        (passed_coordinate + upcoming_coordinate) / 2.0
        for passed_coordinate, upcoming_coordinate in zip(passed, upcoming)
    )
    node.current_target_index = passed_waypoint + 1
    node.goal_pub.messages.clear()

    return node


def _subject_estimate(position):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        point=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
    )


def _node_flying_towards_the_arc():
    node = ArcPlanningNode()
    node.target_world = SUBJECT
    node.generate_cinematographic_waypoints()
    node.active_waypoints = list(node.dynamic_waypoints)
    node.framing_start_idx = 3
    node.framing_end_idx = 7
    node.current_target_index = 1

    return node


def _logged_arc_choice(node):
    for message in node.logger.info_messages:
        match = ARC_CHOICE_PATTERN.match(message)
        if match:
            return match

    return None


def _candidate_costs(arc_choice):
    pairs = (entry.split('=') for entry in arc_choice.group('rest').split(', '))

    return {float(radius): float(cost) for radius, cost in pairs}


def test_candidate_radii_span_the_band_from_the_focus_minimum():
    node = ArcPlanningNode()

    assert node._candidate_arc_radii() == pytest.approx([1.0, 1.25, 1.5, 1.75, 2.0])


@pytest.mark.parametrize('radius', [1.0, 1.25, 1.5, 1.75, 2.0])
def test_every_candidate_arc_point_lies_at_its_radius_from_the_subject(radius):
    node = ArcPlanningNode()

    arc = node._build_candidate_arc(ANCHOR_START, ANCHOR_END, SUBJECT, radius)

    assert arc

    for point in arc:
        assert math.dist(point, SUBJECT) == pytest.approx(radius, abs=1e-6)
        assert point[2] > SUBJECT[2]


def test_cheapest_candidate_is_kept_and_logged():
    node = ArcPlanningNode()
    node.target_world = SUBJECT

    node.generate_cinematographic_waypoints()

    arc_choice = _logged_arc_choice(node)
    assert arc_choice is not None
    costs = _candidate_costs(arc_choice)
    assert float(arc_choice.group('chosen')) == min(costs, key=costs.get)
    assert node.anchor_focus_distance == pytest.approx(
        max(math.dist(point, SUBJECT) for point in node.last_cinematic_waypoints),
    )


def test_cost_optimization_off_keeps_the_anchor_distance_arc():
    node = ArcPlanningNode(enable_cost_optimization=False)
    node.target_world = SUBJECT

    node.generate_cinematographic_waypoints()

    assert _logged_arc_choice(node) is None
    assert node.anchor_focus_distance == pytest.approx(math.dist(ANCHOR_START, SUBJECT))


def test_arc_score_is_mean_per_point_cost():
    node = ArcPlanningNode()
    node.cost_calculator = ConstantCost(3.0)
    subject = (0.0, 0.0, 1.0)
    short_arc = [(1.5, 0.0, 1.0), (0.0, 1.5, 1.0), (-1.5, 0.0, 1.0)]
    long_arc = [
        (1.5, 0.0, 1.0),
        (0.0, 1.5, 1.0),
        (-1.5, 0.0, 1.0),
        (0.0, -1.5, 1.0),
        (1.5, 0.0, 1.0),
        (0.0, 1.5, 1.0),
        (-1.5, 0.0, 1.0),
    ]

    assert node._score_arc_points(short_arc, subject) == pytest.approx(3.0)
    assert node._score_arc_points(long_arc, subject) == pytest.approx(3.0)


def test_arc_with_a_point_beyond_the_band_top_scores_infinite():
    node = ArcPlanningNode()
    subject = (0.0, 0.0, 1.0)
    arc_leaving_the_band = [(1.0, 0.0, 1.0), (0.0, 1.0, 1.0), (0.0, 2.5, 1.0)]

    assert node._score_arc_points(arc_leaving_the_band, subject) == math.inf


def test_estimate_moving_more_than_half_a_metre_before_entry_rebuilds_arc():
    node = _node_flying_towards_the_arc()
    moved_subject = (0.0, 11.05, 1.0)

    node.target_world_callback(_subject_estimate(moved_subject))

    assert node.sequence_initializations == 1
    assert node.arc_center_world == pytest.approx(moved_subject)


def test_estimate_moving_during_filming_leaves_arc_frozen():
    node = _node_flying_towards_the_arc()
    node.current_target_index = 4
    node.framing_active = True
    waypoints_before = list(node.dynamic_waypoints)
    focus_distance_before = node.anchor_focus_distance
    published_before = len(node.anchor_distance_pub.messages)

    node.target_world_callback(_subject_estimate((0.0, 11.05, 1.0)))

    assert node.sequence_initializations == 0
    assert node.dynamic_waypoints == waypoints_before
    assert node.anchor_focus_distance == focus_distance_before
    assert len(node.anchor_distance_pub.messages) == published_before


def test_rebuild_before_entry_sends_the_drone_forward_not_back_to_the_route_start():
    node = _node_flying_the_route(passed_waypoint=4)
    drone_y = node.current_pos[1]

    node.target_world_callback(_subject_estimate((0.0, 11.05, 1.0)))

    assert node.goal_pub.messages[0].pose.position.y > drone_y
    assert node.current_target_index <= node.framing_start_idx


def test_rebuild_before_entry_never_skips_into_the_arc_the_drone_is_beside():
    node = _node_flying_the_route(passed_waypoint=11)
    node.current_target_index = 5

    node.target_world_callback(_subject_estimate((0.0, 11.05, 1.0)))

    assert node.current_target_index <= node.framing_start_idx


def test_rebuild_during_filming_resumes_inside_the_arc():
    node = _node_flying_the_route(passed_waypoint=9)

    node._rebuild_active_with_detection()

    assert node.framing_start_idx <= node.current_target_index <= node.framing_end_idx
    assert node.framing_active


def test_rebuild_after_filming_does_not_fly_the_arc_again():
    node = _node_flying_the_route(passed_waypoint=15)

    node._rebuild_active_with_detection()

    assert node.current_target_index > node.framing_end_idx
