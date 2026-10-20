"""ROS 2 node that sequences and publishes cinematographic waypoints.

``CinematicWaypointGenerator`` owns the waypoint schedule for one drone.  It
maintains a list of static (pre-planned) waypoints and, when a subject is
detected, splices in dynamically generated arc waypoints around the subject
using cost-guided candidate selection.

Coordinate frames and units
----------------------------
All waypoint positions are in the world (ENU) frame, metres.  Yaw angles are
radians, measured from the positive-X axis.  Odometry is received as
``nav_msgs/Odometry``; orientation quaternion convention is [x, y, z, w].

Waypoint sources (in priority order)
--------------------------------------
1. File: path supplied via the ``waypoints_file`` ROS parameter.
2. Topic: ``PoseArray`` received on ``waypoints_topic``.
3. Hardcoded ``static_waypoints`` list (fallback, used in simulation).

When ``use_dynamic_waypoints_on_detection`` is true and a subject detection
arrives on the ``detections_topic`` (``Float32MultiArray``), the node generates
candidate arc positions around the detected subject and selects the one with the
lowest cinematographic cost (framing + focus distance + light avoidance).  The
anchor window (``anchor_start_index``, ``anchor_end_index``) marks the sub-path
along which cinematic framing is active.

``baseline_mode`` disables dynamic waypoint merging and subject tracking,
producing plain EGO-Planner setpoints for comparison data collection.
"""

from cine_agent.utils.paths import add_config_path

add_config_path()

import math

import rclpy
from constants import (
    BsplineFilterParams,
    CinematicConstraints,
    CinematographyParams,
    LightAvoidanceParams,
    WaypointParams,
)
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from ..cost_functions.cost_functions import CostCalculator
from .anchors.anchor_mixin import AnchorMixin
from .detection.detection_mixin import DetectionMixin
from .io.waypoint_io import WaypointIOMixin
from .pathing.path_builder import PathBuilderMixin
from .sequencing.waypoint_sequencer import WaypointSequencerMixin
from .simulation.simulation_mixin import SimulationMixin
from .utils.geometry_mixin import GeometryMixin
from .viz.visualization_mixin import VisualizationMixin


class CinematicWaypointGenerator(
    Node,
    WaypointIOMixin,
    DetectionMixin,
    AnchorMixin,
    PathBuilderMixin,
    WaypointSequencerMixin,
    VisualizationMixin,
    SimulationMixin,
    GeometryMixin,
):
    def __init__(self):
        super().__init__('cinematic_waypoint_generator')
        self.params = CinematographyParams()
        self.constraints = CinematicConstraints()

        self.frame_id = str(self.declare_parameter('frame_id', WaypointParams.FRAME_ID).value)
        self.waypoints_topic = str(self.declare_parameter('waypoints_topic', '').value)
        self.waypoints_file = str(self.declare_parameter('waypoints_file', '').value)

        self.output_topic = str(
            self.declare_parameter('output_topic', self.params.WAYPOINTS_TOPIC).value
        )
        self.odom_topic = str(self.declare_parameter('odom_topic', self.params.ODOM_TOPIC).value)
        self.detections_topic = str(
            self.declare_parameter('detections_topic', '/object_detections').value
        )
        self.light_vector_topic = str(
            self.declare_parameter('light_vector_topic', self.params.LIGHT_VECTOR_TOPIC).value
        )
        self.enable_cost_optimization = bool(
            self.declare_parameter(
                'enable_cost_optimization', WaypointParams.ENABLE_COST_OPTIMIZATION
            ).value
        )
        self.light_avoidance_enabled = bool(
            self.declare_parameter('light_avoidance_enabled', LightAvoidanceParams.ENABLED).value
        )
        self.light_avoidance_gain = float(
            self.declare_parameter('light_avoid_gain', self.constraints.LIGHT_AVOID_GAIN).value
        )
        self.light_influence_distance_m = float(
            self.declare_parameter(
                'light_influence_distance_m',
                self.constraints.LIGHT_INFLUENCE_DISTANCE_M,
            ).value
        )
        self.use_dynamic_waypoints_on_detection = bool(
            self.declare_parameter(
                'use_dynamic_waypoints_on_detection',
                WaypointParams.USE_DYNAMIC_WAYPOINTS_ON_DETECTION,
            ).value
        )
        self.candidate_angle_window_rad = float(
            self.declare_parameter(
                'candidate_angle_window_rad', WaypointParams.CANDIDATE_ANGLE_WINDOW_RAD
            ).value
        )
        self.candidate_angle_samples = int(
            self.declare_parameter(
                'candidate_angle_samples', WaypointParams.CANDIDATE_ANGLE_SAMPLES
            ).value
        )
        self.candidate_radius_samples = int(
            self.declare_parameter(
                'candidate_radius_samples', WaypointParams.CANDIDATE_RADIUS_SAMPLES
            ).value
        )
        self.candidate_radius_scale_range = [
            float(v)
            for v in self.declare_parameter(
                'candidate_radius_range_m', WaypointParams.CANDIDATE_RADIUS_RANGE_M
            ).value
        ]

        self.goal_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.detection_sub = self.create_subscription(
            Float32MultiArray, self.detections_topic, self.detection_callback, 10
        )

        if self.waypoints_topic:
            self.waypoints_sub = self.create_subscription(
                PoseArray, self.waypoints_topic, self.waypoints_callback, 10
            )
        else:
            self.waypoints_sub = None

        self.anchor_distance_topic = str(
            self.declare_parameter(
                'anchor_focus_distance_topic',
                '/cinematography/anchor_focus_distance',
            ).value
        )
        self.anchor_distance_pub = self.create_publisher(Float32, self.anchor_distance_topic, 10)
        if self.light_avoidance_enabled:
            self.light_sub = self.create_subscription(
                Vector3Stamped,
                self.light_vector_topic,
                self.light_callback,
                10,
            )
        else:
            self.light_sub = None
        self.focus_depth_sub = self.create_subscription(
            Float32, '/cinematography/focus_depth_m', self.focus_depth_callback, 10
        )
        self.target_world_sub = self.create_subscription(
            PointStamped, '/cinematography/target_world', self.target_world_callback, 10
        )
        self.target_world_pub = self.create_publisher(
            PointStamped, '/cinematography/target_world', 10
        )
        self.target_world_timer = self.create_timer(0.1, self._publish_target_world)

        self.start_pos = None
        self.origin_offset = None
        self.current_yaw = 0.0
        self.waypoints_sent = False
        self.ready_time = None
        self.STARTUP_DELAY = 0.5
        self.WAYPOINT_TIMER_PERIOD_S = 0.05
        self.WAYPOINT_LOOKAHEAD = int(
            self.declare_parameter('waypoint_lookahead', WaypointParams.WAYPOINT_LOOKAHEAD).value
        )
        self.dynamic_waypoint_lookahead = int(
            self.declare_parameter(
                'dynamic_waypoint_lookahead', WaypointParams.DYNAMIC_WAYPOINT_LOOKAHEAD
            ).value
        )

        self.velocity_settled = False
        self.VELOCITY_THRESHOLD = 0.2
        self.current_vel = (0.0, 0.0, 0.0)
        self.current_pos = None

        self.object_detection_time = None
        self.DETECTION_DEBOUNCE = 2.0
        self.DISTANCE_THRESHOLD_M = float(
            self.declare_parameter(
                'distance_threshold_m', WaypointParams.DISTANCE_THRESHOLD_M
            ).value
        )
        self.PROGRESS_EPS_M = 0.05
        self.PROGRESS_TIMEOUT_S = float(
            self.declare_parameter('progress_timeout_s', WaypointParams.PROGRESS_TIMEOUT_S).value
        )
        self.min_goal_distance_m = float(
            self.declare_parameter('min_goal_distance_m', WaypointParams.MIN_GOAL_DISTANCE_M).value
        )
        self.min_first_waypoint_distance_m = float(
            self.declare_parameter(
                'min_first_waypoint_distance_m',
                WaypointParams.MIN_FIRST_WAYPOINT_DISTANCE_M,
            ).value
        )
        self.enable_progress_skip = bool(
            self.declare_parameter(
                'enable_progress_skip', WaypointParams.ENABLE_PROGRESS_SKIP
            ).value
        )

        self.image_width = int(self.declare_parameter('image_width', self.params.IMAGE_WIDTH).value)
        self.image_height = int(
            self.declare_parameter('image_height', self.params.IMAGE_HEIGHT).value
        )
        self.fx = float(self.declare_parameter('camera_fx', self.params.CAMERA_FX).value)
        self.fy = float(self.declare_parameter('camera_fy', self.params.CAMERA_FY).value)
        self.cx = float(self.declare_parameter('camera_cx', self.params.CAMERA_CX).value)
        self.cy = float(self.declare_parameter('camera_cy', self.params.CAMERA_CY).value)
        self.orbit_altitude_m = float(
            self.declare_parameter('orbit_altitude_m', self.params.CONSTANT_ALTITUDE_M).value
        )
        self.min_altitude_m = float(
            self.declare_parameter('min_altitude_m', BsplineFilterParams.MIN_ALTITUDE_M).value
        )
        self.max_altitude_m = float(
            self.declare_parameter('max_altitude_m', BsplineFilterParams.MAX_ALTITUDE_M).value
        )
        self.min_orbit_radius_m = float(
            self.declare_parameter('min_orbit_radius_m', WaypointParams.MIN_ORBIT_RADIUS_M).value
        )
        self.waypoint_radius_m = float(
            self.declare_parameter('waypoint_radius_m', WaypointParams.WAYPOINT_RADIUS_M).value
        )
        self.max_waypoint_jump_m = float(
            self.declare_parameter('max_waypoint_jump_m', WaypointParams.MAX_WAYPOINT_JUMP_M).value
        )
        self.min_waypoint_separation_m = float(
            self.declare_parameter(
                'min_waypoint_separation_m', WaypointParams.MIN_WAYPOINT_SEPARATION_M
            ).value
        )
        self.bezier_blend_length_m = float(
            self.declare_parameter(
                'bezier_blend_length_m', WaypointParams.BEZIER_BLEND_LENGTH_M
            ).value
        )
        self.bezier_transition_points = int(
            self.declare_parameter(
                'bezier_transition_points', WaypointParams.BEZIER_TRANSITION_POINTS
            ).value
        )
        self.bezier_min_speed_mps = float(
            self.declare_parameter(
                'bezier_min_speed_mps', WaypointParams.BEZIER_MIN_SPEED_MPS
            ).value
        )
        self.min_focus_distance_m = float(self.params.MIN_FOCUS_DISTANCE_M)
        self.focus_distance_tolerance_m = float(
            self.declare_parameter('focus_distance_tolerance_m', 0.05).value
        )
        self.focus_altitude_m = float(
            self.declare_parameter('focus_altitude_m', BsplineFilterParams.FOCUS_ALTITUDE_M).value
        )
        self.anchor_start_index = int(
            self.declare_parameter('anchor_start_index', WaypointParams.ANCHOR_START_INDEX).value
        )
        self.anchor_end_index = int(
            self.declare_parameter('anchor_end_index', WaypointParams.ANCHOR_END_INDEX).value
        )
        self.anchor_index_window = int(
            self.declare_parameter('anchor_index_window', WaypointParams.ANCHOR_INDEX_WINDOW).value
        )
        self.anchor_min_gap = int(
            self.declare_parameter('anchor_min_gap', WaypointParams.ANCHOR_MIN_GAP).value
        )
        self.arc_waypoint_spacing_m = float(
            self.declare_parameter(
                'arc_waypoint_spacing_m', WaypointParams.ARC_WAYPOINT_SPACING_M
            ).value
        )
        self.anchor_marker_scale_m = float(
            self.declare_parameter(
                'anchor_marker_scale_m', WaypointParams.ANCHOR_MARKER_SCALE_M
            ).value
        )
        self.target_marker_scale_m = float(
            self.declare_parameter(
                'target_marker_scale_m', WaypointParams.TARGET_MARKER_SCALE_M
            ).value
        )
        self.focus_entry_decel_waypoints = int(
            self.declare_parameter(
                'focus_entry_decel_waypoints',
                WaypointParams.FOCUS_ENTRY_DECEL_WAYPOINTS,
            ).value
        )
        self.waypoint_xy_scale = float(
            self.declare_parameter('waypoint_xy_scale', WaypointParams.WAYPOINT_XY_SCALE).value
        )
        self.waypoint_y_extra_scale = float(
            self.declare_parameter(
                'waypoint_y_extra_scale', WaypointParams.WAYPOINT_Y_EXTRA_SCALE
            ).value
        )
        self.waypoint_z_scale = float(
            self.declare_parameter('waypoint_z_scale', WaypointParams.WAYPOINT_Z_SCALE).value
        )
        self.enable_z_clamp = bool(
            self.declare_parameter('enable_z_clamp', WaypointParams.ENABLE_Z_CLAMP).value
        )
        self.z_ceiling_override_m = float(
            self.declare_parameter(
                'z_ceiling_override_m', WaypointParams.Z_CEILING_OVERRIDE_M
            ).value
        )
        self.world_bounds_min = [
            float(v)
            for v in self.declare_parameter(
                'world_bounds_min', BsplineFilterParams.WORLD_BOUNDS_MIN
            ).value
        ]
        self.world_bounds_max = [
            float(v)
            for v in self.declare_parameter(
                'world_bounds_max', BsplineFilterParams.WORLD_BOUNDS_MAX
            ).value
        ]
        self.virtual_ceiling_m = float(
            self.declare_parameter('virtual_ceiling_m', BsplineFilterParams.VIRTUAL_CEILING_M).value
        )
        self.simulate_object = bool(self.declare_parameter('simulate_object', False).value)
        self.simulated_object_world_offset = [
            float(v)
            for v in self.declare_parameter('simulated_object_world_offset', [0.3, 0.6, 0.0]).value
        ]
        self.simulated_object_side_offset_m = float(
            self.declare_parameter('simulated_object_side_offset_m', 0.6).value
        )
        self.simulated_object_radius_m = float(
            self.declare_parameter('simulated_object_radius_m', 0.8).value
        )
        self.simulated_object_outward_offset_m = float(
            self.declare_parameter('simulated_object_outward_offset_m', 1.0).value
        )

        self.max_altitude_m = min(self.max_altitude_m, self.virtual_ceiling_m)
        self.DISTANCE_THRESHOLD_M = max(self.waypoint_radius_m, 0.001)
        self.min_goal_distance_m = max(self.min_goal_distance_m, self.DISTANCE_THRESHOLD_M)
        self.min_first_waypoint_distance_m = max(
            self.min_first_waypoint_distance_m, self.DISTANCE_THRESHOLD_M
        )

        self.static_waypoints = [
            (0.54, 0.425, 0.28),
            (1.15, 1.2, 0.5),
            (0.91, 2.025, 0.85),
            (0.99, 2.79, 1.05),
            (1.62, 3.24, 1.25),
            (2.34, 3.375, 1.35),
            (3.05, 3.0, 1.4),
            (3.285, 2.43, 1.15),
            (3.0, 1.5, 0.8),
            (2.61, 0.63, 0.7),
            (2.43, -0.45, 0.65),
        ]

        if self.waypoints_file:
            self.get_logger().info(f'Loading base waypoints from file: {self.waypoints_file}')
            self._load_waypoints_from_file(self.waypoints_file)
        else:
            self.get_logger().info(
                f'Using default static waypoints (count: {len(self.static_waypoints)})'
            )

        self.dynamic_waypoints = None
        self.detected_object = None
        self.use_dynamic_waypoints = True
        self.active_waypoints = None
        self.current_target_index = 0
        self.last_published_index = -1
        self.simulated_detection_active = True
        self.pending_detection_merge = True

        self.stuck_progress_threshold_m = 0.25
        self.stuck_time_s = 0.8
        self.nudge_offset_xy_m = 0.5
        self.nudge_offset_z_m = 0.25
        self.max_stuck_nudges = 2
        self.last_progress_time = None
        self.last_progress_dist = None
        self.stuck_attempts = 0
        self.anchors_set = False
        self.anchor_start_idx: int | None = None
        self.anchor_end_idx: int | None = None
        self.anchor_start_world = None
        self.anchor_end_world = None
        self.anchor_source = 'none'
        self.anchor_focus_distance = None
        self.framing_active = False
        self.baseline_framing_active = False
        self.framing_start_idx = None
        self.framing_end_idx = None

        self.simulated_detection_pub = self.create_publisher(
            Float32MultiArray, '/object_detections', 10
        )
        if self.simulate_object:
            self.create_timer(0.5, self.publish_simulated_detection)
        self.anchor_start_topic = str(
            self.declare_parameter('anchor_start_topic', '/cinematography/anchor_start').value
        )
        self.anchor_end_topic = str(
            self.declare_parameter('anchor_end_topic', '/cinematography/anchor_end').value
        )
        self.anchor_start_pub = self.create_publisher(PointStamped, self.anchor_start_topic, 10)
        self.anchor_end_pub = self.create_publisher(PointStamped, self.anchor_end_topic, 10)
        self.anchor_marker_pub = self.create_publisher(
            MarkerArray, '/cinematography/anchor_markers', 10
        )
        self.target_marker_pub = self.create_publisher(Marker, '/cinematography/target_marker', 10)
        self.framing_active_pub = self.create_publisher(Bool, '/cinematography/framing_active', 10)
        self.baseline_framing_pub = self.create_publisher(
            Bool, '/cinematography/baseline_framing_active', 10
        )
        self.sent_waypoints_viz_pub = self.create_publisher(
            MarkerArray, '/cinematography/sent_waypoints_viz', 10
        )
        self.sent_waypoints_history = []
        self.debug_arc = True
        self.debug_arc_log_period_s = 0.5
        self.debug_arc_marker_scale = 0.04
        self.arc_marker_pub = self.create_publisher(Marker, '/cinematography/arc_marker', 10)
        self.last_arc_log_time = None
        self.enable_debug_viz = True
        self.viz_static_path = True
        self.viz_dynamic_path = True
        self.viz_active_path = True
        self.viz_transitions = True
        self.viz_arc_only = True
        self.transition_viz_pub = self.create_publisher(
            MarkerArray, '/cinematography/transition_viz', 10
        )
        self.debug_markers_pub = self.create_publisher(
            MarkerArray, '/cinematography/debug_markers', 10
        )
        self.debug_viz_timer = self.create_timer(0.2, self._publish_debug_visualization)
        self.sent_waypoints_timer = self.create_timer(0.1, self._publish_sent_waypoints_dots)
        self.viz_data = {
            'static_world': None,
            'arc_start': None,
            'arc_end': None,
            'cinematographic_waypoints': None,
            'transition_in': None,
            'transition_out': None,
            'combined_world': None,
            'start_idx': None,
            'end_idx': None,
        }

        self.last_cinematic_waypoints: list | None = None
        self.anchor_timer = self.create_timer(1.0, self._publish_anchor_points)
        self.target_marker_timer = self.create_timer(0.5, self._publish_target_marker)

        self.timer = self.create_timer(self.WAYPOINT_TIMER_PERIOD_S, self.check_and_send_waypoints)
        self.focus_depth_m: float | None = None
        self.target_world: tuple[float, float, float] | None = None
        self.last_target_world: tuple[float, float, float] | None = None
        self._last_published_target_stamp = None
        self.light_vector = None
        self.light_distance = None
        self.cost_calculator = CostCalculator(self)
        self._anchor_marker_seq = 0
        self.baseline_mode = bool(self.declare_parameter('baseline_mode', False).value)
        mode_str = 'baseline' if self.baseline_mode else 'cinematic'
        self.get_logger().info(f'Waypoint generator running in {mode_str} mode')
        if self.baseline_mode:
            self.use_dynamic_waypoints = False
            self.pending_detection_merge = False

    def odom_callback(self, msg: Odometry):
        self.current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        self.current_vel = (float(vx), float(vy), float(vz))
        velocity_mag = (vx**2 + vy**2 + vz**2) ** 0.5

        if self.start_pos is None:
            self.start_pos = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )
            self.origin_offset = self.start_pos
            self.ready_time = self.get_clock().now()
            if self.target_world is not None:
                self._set_anchors(target_world=self.target_world, obj_depth=self.focus_depth_m)

        w = msg.pose.pose.orientation.w
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.velocity_settled = velocity_mag < self.VELOCITY_THRESHOLD
        try:
            self.cost_calculator.update_odometry(msg)
        except AttributeError as error:
            self.get_logger().warn(f'odometry update skipped: malformed message ({error})')


def main(args=None):
    rclpy.init(args=args)
    node = CinematicWaypointGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
