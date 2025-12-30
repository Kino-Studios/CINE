import math

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from ..cost_functions.cost_calculator import CostCalculator

class CinematicWaypointGenerator(Node):
    def __init__(self):
        super().__init__("cinematic_waypoint_generator")

        self.frame_id = str(self.declare_parameter("frame_id", "world").value)
        self.output_topic = str(self.declare_parameter("output_topic", "/move_base_simple/goal").value)
        self.odom_topic = str(self.declare_parameter("odom_topic", "/drone_0_visual_slam/odom").value)
        self.detections_topic = str(self.declare_parameter("detections_topic", "/object_detections").value)
        self.light_vector_topic = str(self.declare_parameter("light_vector_topic", "/cinematography/light_vector").value)
        self.enable_cost_optimization = bool(self.declare_parameter("enable_cost_optimization", True).value)
        self.use_dynamic_waypoints_on_detection = bool(self.declare_parameter("use_dynamic_waypoints_on_detection", True).value)
        self.candidate_angle_window_rad = float(self.declare_parameter("candidate_angle_window_rad", 0.6).value)
        self.candidate_angle_samples = int(self.declare_parameter("candidate_angle_samples", 7).value)
        self.candidate_radius_samples = int(self.declare_parameter("candidate_radius_samples", 5).value)
        self.candidate_radius_scale_range = [float(v) for v in self.declare_parameter("candidate_radius_scale_range", [0.7, 1.3]).value]

        self.goal_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.detection_sub = self.create_subscription(Float32MultiArray, self.detections_topic, self.detection_callback, 10)
        self.light_sub = self.create_subscription(Vector3Stamped, self.light_vector_topic, self.light_callback, 10)
        self.focus_depth_sub = self.create_subscription(Float32, "/cinematography/focus_depth_m", self.focus_depth_callback, 10)
        self.target_world_sub = self.create_subscription(PointStamped, "/cinematography/target_world", self.target_world_callback, 10)

        self.start_pos = None
        self.origin_offset = None
        self.current_yaw = 0.0
        self.waypoints_sent = False
        self.ready_time = None
        self.STARTUP_DELAY = 0.5
        self.WAYPOINT_TIMER_PERIOD_S = 0.05 # planner must be rapidly fed to prevent stalling
        self.WAYPOINT_LOOKAHEAD = int(self.declare_parameter("waypoint_lookahead", 2).value)
        self.dynamic_waypoint_lookahead = int(self.declare_parameter("dynamic_waypoint_lookahead", 3).value)

        self.velocity_settled = False
        self.VELOCITY_THRESHOLD = 0.2
        self.current_vel = (0.0, 0.0, 0.0)
        self.current_pos = None

        self.object_detection_time = None
        self.DETECTION_DEBOUNCE = 2.0
        self.DISTANCE_THRESHOLD_M = float(self.declare_parameter("distance_threshold_m", 0.5).value)
        self.PROGRESS_EPS_M = 0.05
        self.PROGRESS_TIMEOUT_S = float(self.declare_parameter("progress_timeout_s", 4.0).value)
        self.min_goal_distance_m = float(self.declare_parameter("min_goal_distance_m", 0.5).value)
        self.min_first_waypoint_distance_m = float(
            self.declare_parameter("min_first_waypoint_distance_m", 0.5).value
        )
        self.enable_progress_skip = bool(self.declare_parameter("enable_progress_skip", False).value)

        self.image_width = int(self.declare_parameter("image_width", 1152).value)
        self.image_height = int(self.declare_parameter("image_height", 648).value)
        self.fx = float(self.declare_parameter("camera_fx", 387.229248046875).value)
        self.fy = float(self.declare_parameter("camera_fy", 387.229248046875).value)
        self.cx = float(self.declare_parameter("camera_cx", 320.5).value)
        self.cy = float(self.declare_parameter("camera_cy", 240.5).value)
        self.orbit_altitude_m = float(self.declare_parameter("orbit_altitude_m", 1.5).value)
        self.min_altitude_m = float(self.declare_parameter("min_altitude_m", 0.2).value)
        self.max_altitude_m = float(self.declare_parameter("max_altitude_m", 3.0).value)
        self.min_orbit_radius_m = float(self.declare_parameter("min_orbit_radius_m", 0.6).value)
        self.max_waypoint_jump_m = float(self.declare_parameter("max_waypoint_jump_m", 3.0).value)
        self.bezier_blend_length_m = float(self.declare_parameter("bezier_blend_length_m", 0.1).value)
        self.bezier_transition_points = int(self.declare_parameter("bezier_transition_points", 6).value)
        self.bezier_min_speed_mps = float(self.declare_parameter("bezier_min_speed_mps", 0.05).value)
        self.focus_distance_m = float(self.declare_parameter("focus_distance_m", 1.0).value)
        self.focus_distance_tolerance_m = float(
            self.declare_parameter("focus_distance_tolerance_m", 0.05).value
        )
        self.focus_altitude_m = float(self.declare_parameter("focus_altitude_m", -1.0).value)
        self.anchor_start_index = int(self.declare_parameter("anchor_start_index", -1).value)
        self.anchor_end_index = int(self.declare_parameter("anchor_end_index", -1).value)
        self.anchor_index_window = int(self.declare_parameter("anchor_index_window", 2).value)
        self.anchor_min_gap = int(self.declare_parameter("anchor_min_gap", 2).value)
        self.arc_waypoint_spacing_m = float(self.declare_parameter("arc_waypoint_spacing_m", 1.0).value)
        self.anchor_marker_scale_m = float(self.declare_parameter("anchor_marker_scale_m", 0.28).value)
        self.target_marker_scale_m = float(self.declare_parameter("target_marker_scale_m", 0.32).value)
        self.waypoint_xy_scale = float(self.declare_parameter("waypoint_xy_scale", 1.1).value)
        self.waypoint_y_extra_scale = float(self.declare_parameter("waypoint_y_extra_scale", 1.05).value)
        self.waypoint_z_scale = float(self.declare_parameter("waypoint_z_scale", 1.1).value)
        self.enable_z_clamp = bool(self.declare_parameter("enable_z_clamp", False).value)
        self.z_ceiling_override_m = float(self.declare_parameter("z_ceiling_override_m", -1.0).value)
        self.world_bounds_min = [float(v) for v in self.declare_parameter("world_bounds_min", [-10.0, -10.0, 0.0]).value]
        self.world_bounds_max = [float(v) for v in self.declare_parameter("world_bounds_max", [10.0, 10.0, 2.8]).value]
        self.virtual_ceiling_m = float(self.declare_parameter("virtual_ceiling_m", 2.8).value)
        self.simulate_fake_object = bool(self.declare_parameter("simulate_fake_object", False).value)
        self.fake_object_world_offset = [float(v) for v in self.declare_parameter("fake_object_world_offset", [0.3, 0.6, 0.0]).value]
        self.fake_object_side_offset_m = float(self.declare_parameter("fake_object_side_offset_m", 0.6).value)
        self.fake_object_radius_m = float(self.declare_parameter("fake_object_radius_m", 0.8).value)
        self.fake_object_outward_offset_m = float(self.declare_parameter("fake_object_outward_offset_m", 1.0).value)

        self.max_altitude_m = min(self.max_altitude_m, self.virtual_ceiling_m)

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
        self.anchor_source = "none"
        self.anchor_focus_distance = None
        self.framing_active = False
        self.framing_start_idx = None
        self.framing_end_idx = None

        self.fake_detection_pub = self.create_publisher(Float32MultiArray, "/object_detections", 10)
        if self.simulate_fake_object:
            self.create_timer(0.5, self.publish_simulated_detection)
        self.anchor_start_topic = str(
            self.declare_parameter("anchor_start_topic", "/cinematography/anchor_start").value
        )
        self.anchor_end_topic = str(
            self.declare_parameter("anchor_end_topic", "/cinematography/anchor_end").value
        )
        self.anchor_start_pub = self.create_publisher(PointStamped, self.anchor_start_topic, 10)
        self.anchor_end_pub = self.create_publisher(PointStamped, self.anchor_end_topic, 10)
        self.anchor_marker_pub = self.create_publisher(MarkerArray, "/cinematography/anchor_markers", 10)
        self.target_marker_pub = self.create_publisher(Marker, "/cinematography/target_marker", 10)
        self.framing_active_pub = self.create_publisher(Bool, "/cinematography/framing_active", 10)
        self.debug_arc = bool(self.declare_parameter("debug_arc", True).value)
        self.debug_arc_log_period_s = float(self.declare_parameter("debug_arc_log_period_s", 0.5).value)
        self.debug_arc_marker_scale = float(self.declare_parameter("debug_arc_marker_scale", 0.04).value)
        self.arc_marker_pub = self.create_publisher(Marker, "/cinematography/arc_marker", 10)
        self.last_arc_log_time = None
        self.enable_debug_viz = bool(self.declare_parameter("enable_debug_viz", True).value)
        self.viz_static_path = bool(self.declare_parameter("viz_static_path", True).value)
        self.viz_dynamic_path = bool(self.declare_parameter("viz_dynamic_path", True).value)
        self.viz_active_path = bool(self.declare_parameter("viz_active_path", True).value)
        self.viz_transitions = bool(self.declare_parameter("viz_transitions", True).value)
        self.viz_arc_only = bool(self.declare_parameter("viz_arc_only", True).value)
        self.viz_target_sphere = bool(self.declare_parameter("viz_target_sphere", True).value)
        self.viz_waypoint_numbers = bool(self.declare_parameter("viz_waypoint_numbers", False).value)
        self.debug_markers_pub = self.create_publisher(MarkerArray, "/cinematography/debug_markers", 10)
        self.debug_viz_timer = self.create_timer(0.2, self._publish_debug_visualization)
        self.viz_data = {
            "static_world": None,
            "arc_start": None,
            "arc_end": None,
            "cinematographic_waypoints": None,
            "transition_in": None,
            "transition_out": None,
            "combined_world": None,
            "start_idx": None,
            "end_idx": None,
        }
        self.anchor_timer = self.create_timer(1.0, self._publish_anchor_points)
        self.target_marker_timer = self.create_timer(0.5, self._publish_target_marker)

        self.timer = self.create_timer(self.WAYPOINT_TIMER_PERIOD_S, self.check_and_send_waypoints)
        self.focus_depth_m: float | None = None
        self.target_world: tuple[float, float, float] | None = None
        self.last_target_world: tuple[float, float, float] | None = None
        self.light_vector = None
        self.cost_calculator = CostCalculator(self)

    def _build_room_orbit(self):
        radius = max(self.min_orbit_radius_m, 1.0)
        radius = min(radius, 1.5)
        num_pts = 16
        pts = []

        for i in range(num_pts):
            t = 2.0 * math.pi * i / num_pts
            x = radius * math.cos(t)
            y = radius * math.sin(t)
            z = self.orbit_altitude_m + 0.6 * math.sin(t)
            z = max(self.min_altitude_m, min(self.max_altitude_m, z))
            pts.append((x, y, z))
        return pts

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
        except Exception:
            pass

    def light_callback(self, msg: Vector3Stamped):
        self.light_vector = (float(msg.vector.x), float(msg.vector.y), float(msg.vector.z))
        try:
            self.cost_calculator.update_light_vector(msg)
        except Exception:
            pass

    def focus_depth_callback(self, msg: Float32):
        try:
            self.focus_depth_m = float(getattr(msg, "data", None))
        except Exception:
            self.focus_depth_m = None

    def target_world_callback(self, msg: PointStamped):
        try:
            p = msg.point
            self.target_world = (float(p.x), float(p.y), float(p.z))
            self.last_target_world = self.target_world
            self._set_anchors(target_world=self.target_world, obj_depth=self.focus_depth_m)
        except Exception:
            self.target_world = None
            self.last_target_world = None

    def detection_callback(self, msg: Float32MultiArray):
        now = self.get_clock().now()
        if self.object_detection_time is not None:
            elapsed = (now - self.object_detection_time).nanoseconds / 1e9
            if elapsed < self.DETECTION_DEBOUNCE:
                return

        parsed = self._parse_detection(msg)
        if parsed is None:
            return

        self.detected_object = parsed
        self.object_detection_time = now
        self.simulated_detection_active = False
        self.use_dynamic_waypoints = True
        self.pending_detection_merge = True
        self._set_anchors(target_world=self.target_world, obj_depth=self.detected_object["depth"])
        self._rebuild_active_with_detection()

    def _set_anchors(self, target_world=None, obj_depth=None):
        if self.current_pos is None or self.origin_offset is None:
            return

        static_world = [self._rel_to_world(wp) for wp in self._scaled_static_waypoints()]
        if target_world is None and self.detected_object is not None:
            target_world, _ = self._compute_object_world(self.detected_object)

        if target_world is not None:
            desired_distance = self._resolve_focus_distance(obj_depth, use_anchor=False)
            start_idx, end_idx = self._select_anchor_pair(
                static_world,
                target_world,
                desired_distance,
            )
            dist_start = math.dist(static_world[start_idx], target_world)
            dist_end = math.dist(static_world[end_idx], target_world)
            if math.isfinite(dist_start) and math.isfinite(dist_end):
                self.anchor_focus_distance = max(dist_start, dist_end)
            else:
                self.anchor_focus_distance = None
            self.anchor_start_world = static_world[start_idx]
            self.anchor_end_world = static_world[end_idx]
            self.anchor_source = "static_pair"
        else:
            start_idx = 0
            if self.active_waypoints is not None and 0 <= int(self.current_target_index) < len(self.active_waypoints):
                start_idx = int(self.current_target_index)
            else:
                start_idx = min(
                    range(len(static_world)),
                    key=lambda i: (static_world[i][0] - self.current_pos[0]) ** 2
                    + (static_world[i][1] - self.current_pos[1]) ** 2
                    + (static_world[i][2] - self.current_pos[2]) ** 2,
                )
            end_idx = len(static_world) - 1
            self.anchor_start_world = static_world[start_idx]
            self.anchor_end_world = static_world[end_idx]
            self.anchor_source = "fallback"
            self.anchor_focus_distance = None

        self.anchor_start_idx = int(start_idx)
        self.anchor_end_idx = int(end_idx)
        self.anchors_set = True
        self._publish_anchor_points()

    def _rebuild_active_with_detection(self):
        if self.start_pos is None or self.origin_offset is None:
            return

        use_dynamic = bool(self.use_dynamic_waypoints_on_detection)
        if use_dynamic:
            self.generate_cinematographic_waypoints()
            self.use_dynamic_waypoints = True
            self.pending_detection_merge = False
        else:
            self.use_dynamic_waypoints = False
            self.pending_detection_merge = True

        self.active_waypoints = None
        self.waypoints_sent = False
        self.current_target_index = 0
        self.last_published_index = -1
        self._initialize_waypoint_sequence()

    def _estimate_waypoint_cost(self, candidate_world, target_world, reference_world):
        if not bool(self.enable_cost_optimization):
            return None
        if candidate_world is None or target_world is None or reference_world is None:
            return None
        if self.cost_calculator is None:
            return None

        return self.cost_calculator.estimate_full_cost_at_position(
            float(candidate_world[0]),
            float(candidate_world[1]),
            float(candidate_world[2]),
            float(target_world[0]),
            float(target_world[1]),
            float(target_world[2]),
            reference_drone_x=float(reference_world[0]),
            reference_drone_y=float(reference_world[1]),
            reference_drone_z=float(reference_world[2]),
        )

    def _parse_detection(self, msg: Float32MultiArray):
        if msg is None or msg.data is None:
            return None

        data = [float(x) for x in msg.data]
        if len(data) < 1:
            return None

        num_objects = int(round(data[0]))
        if num_objects >= 1 and len(data) >= 4:
            center_x = data[1]
            center_y = data[2]
            depth_m = data[3]
            if not (depth_m > 0.1 and math.isfinite(depth_m)):
                return None

            norm_x = None
            norm_y = None
            if len(data) >= 9:
                norm_x = data[7]
                norm_y = data[8]

            return {
                "center_x": center_x,
                "center_y": center_y,
                "depth": depth_m,
                "norm_x": norm_x,
                "norm_y": norm_y,
            }

        if len(data) >= 3:
            x_center = data[0]
            y_center = data[1]
            depth_m = data[2]
            if not (depth_m > 0.1 and math.isfinite(depth_m)):
                return None
            return {
                "center_x": x_center,
                "center_y": y_center,
                "depth": depth_m,
                "norm_x": x_center if 0.0 <= x_center <= 1.0 else None,
                "norm_y": y_center if 0.0 <= y_center <= 1.0 else None,
            }

        return None

    def _normalize3(self, v):
        x, y, z = float(v[0]), float(v[1]), float(v[2])
        n = (x * x + y * y + z * z) ** 0.5
        if n <= 1e-9:
            return (0.0, 0.0, 0.0), 0.0
        return (x / n, y / n, z / n), n

    def _sample_cubic_bezier(self, p0, p1, p2, p3, n_points):
        n_points = int(n_points)
        if n_points <= 0:
            return []

        out = []
        for i in range(1, n_points + 1):
            t = i / (n_points + 1)
            u = 1.0 - t
            b0 = u * u * u
            b1 = 3.0 * u * u * t
            b2 = 3.0 * u * t * t
            b3 = t * t * t
            out.append(
                (
                    b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
                    b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1],
                    b0 * p0[2] + b1 * p1[2] + b2 * p2[2] + b3 * p3[2],
                )
            )
        return out

    def _build_bezier_transition(self, p0, p3, prev_point=None, next_point=None):
        if self.bezier_transition_points <= 0 or self.bezier_blend_length_m <= 0.0:
            return []

        dx = p3[0] - p0[0]
        dy = p3[1] - p0[1]
        dz = p3[2] - p0[2]
        if (dx * dx + dy * dy + dz * dz) ** 0.5 < 1e-3:
            return []

        start_dir = (
            (p0[0] - prev_point[0], p0[1] - prev_point[1], p0[2] - prev_point[2])
            if prev_point is not None
            else (dx, dy, dz)
        )
        end_dir = (
            (next_point[0] - p3[0], next_point[1] - p3[1], next_point[2] - p3[2])
            if next_point is not None
            else (dx, dy, dz)
        )

        v0_dir, v0_mag = self._normalize3(start_dir)
        if v0_mag < 1e-6:
            v0_dir, _ = self._normalize3((dx, dy, dz))
        v3_dir, v3_mag = self._normalize3(end_dir)
        if v3_mag < 1e-6:
            v3_dir, _ = self._normalize3((dx, dy, dz))

        alpha = float(self.bezier_blend_length_m)
        beta = float(self.bezier_blend_length_m)
        p1 = (p0[0] + alpha * v0_dir[0], p0[1] + alpha * v0_dir[1], p0[2] + alpha * v0_dir[2])
        p2 = (p3[0] - beta * v3_dir[0], p3[1] - beta * v3_dir[1], p3[2] - beta * v3_dir[2])
        transition = self._sample_cubic_bezier(p0, p1, p2, p3, self.bezier_transition_points)
        return [self._clamp_waypoint_to_bounds(wp) for wp in transition]

    def _resolve_focus_distance(self, obj_depth, use_anchor=True):
        if use_anchor and self.anchor_focus_distance is not None and self.anchor_focus_distance > 0.0:
            return float(self.anchor_focus_distance)

        desired = float(self.focus_distance_m)
        if desired > 0.0:
            return desired
        if self.focus_depth_m is not None and self.focus_depth_m > 0.0:
            return float(self.focus_depth_m)
        if obj_depth is not None and float(obj_depth) > 0.0:
            return float(obj_depth)
        return float(self.min_orbit_radius_m)

    def _segment_circle_intersection(self, p0, p1, target_world, desired_distance):
        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])

        def _dist_at(t):
            x = p0[0] + t * (p1[0] - p0[0])
            y = p0[1] + t * (p1[1] - p0[1])
            z = p0[2] + t * (p1[2] - p0[2])
            dx = x - tx
            dy = y - ty
            dz = z - tz
            return math.sqrt(dx * dx + dy * dy + dz * dz), (x, y, z)

        d0, _ = _dist_at(0.0)
        d1, _ = _dist_at(1.0)
        f0 = d0 - desired_distance
        f1 = d1 - desired_distance

        if not (math.isfinite(f0) and math.isfinite(f1)):
            return None
        if abs(f0) <= 1e-6:
            _, p = _dist_at(0.0)
            return p
        if abs(f1) <= 1e-6:
            _, p = _dist_at(1.0)
            return p
        if f0 * f1 > 0.0:
            return None

        t0 = 0.0
        t1 = 1.0
        for _ in range(12):
            tm = 0.5 * (t0 + t1)
            dm, _ = _dist_at(tm)
            fm = dm - desired_distance
            if f0 * fm <= 0.0:
                t1 = tm
                f1 = fm
            else:
                t0 = tm
                f0 = fm
        _, p = _dist_at(0.5 * (t0 + t1))
        return p

    def _compute_anchor_crossings(self, static_world, target_world, desired_distance):
        n = len(static_world)
        if n < 2:
            return None

        desired = max(float(desired_distance), float(self.min_orbit_radius_m))
        distances = []
        for wp in static_world:
            dx = float(wp[0]) - float(target_world[0])
            dy = float(wp[1]) - float(target_world[1])
            dz = float(wp[2]) - float(target_world[2])
            distances.append(math.sqrt(dx * dx + dy * dy + dz * dz))

        closest_idx = int(min(range(n), key=lambda i: distances[i]))
        left_anchor = None
        left_idx = None
        for i in range(closest_idx, 0, -1):
            p0 = static_world[i - 1]
            p1 = static_world[i]
            hit = self._segment_circle_intersection(p0, p1, target_world, desired)
            if hit is not None:
                left_anchor = hit
                left_idx = i - 1
                break

        right_anchor = None
        right_idx = None
        for i in range(closest_idx, n - 1):
            p0 = static_world[i]
            p1 = static_world[i + 1]
            hit = self._segment_circle_intersection(p0, p1, target_world, desired)
            if hit is not None:
                right_anchor = hit
                right_idx = i + 1
                break

        if left_anchor is None or right_anchor is None:
            return None

        if left_idx is None or right_idx is None:
            return None
        if right_idx <= left_idx:
            return None

        return (left_idx, right_idx, left_anchor, right_anchor)

    def _select_anchor_pair(self, static_world, target_world, desired_distance):
        n = len(static_world)
        if n < 2:
            return 0, max(0, n - 1)

        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])
        desired = max(float(desired_distance), float(self.min_orbit_radius_m))
        distances = []
        for wp in static_world:
            dx = float(wp[0]) - tx
            dy = float(wp[1]) - ty
            dz = float(wp[2]) - tz
            distances.append(math.sqrt(dx * dx + dy * dy + dz * dz))

        closest_idx = int(min(range(n), key=lambda i: distances[i]))
        min_gap = max(1, int(self.anchor_min_gap))
        balance_weight = 1.5

        best_pair = (0, min_gap)
        best_cost = float("inf")
        for i in range(n - 1):
            if i > closest_idx:
                break
            di = distances[i]
            if not math.isfinite(di) or di < self.min_orbit_radius_m:
                continue
            for j in range(i + min_gap, n):
                if j < closest_idx:
                    continue
                dj = distances[j]
                if not math.isfinite(dj) or dj < self.min_orbit_radius_m:
                    continue
                cost = abs(di - desired) + abs(dj - desired) + balance_weight * abs(di - dj)
                if cost < best_cost:
                    best_cost = cost
                    best_pair = (i, j)

        return best_pair

    def _path_length_between(self, static_world, start_idx, end_idx):
        if not static_world or end_idx <= start_idx:
            return 0.0
        length = 0.0
        for i in range(start_idx, end_idx):
            a = static_world[i]
            b = static_world[i + 1]
            dx = float(b[0]) - float(a[0])
            dy = float(b[1]) - float(a[1])
            dz = float(b[2]) - float(a[2])
            length += math.sqrt(dx * dx + dy * dy + dz * dz)
        return float(length)

    def _resolve_framing_altitude_from_anchors(self, start_wp, end_wp, target_world=None, desired_distance=None):
        if self.focus_altitude_m > 0.0 and math.isfinite(self.focus_altitude_m):
            return float(self.focus_altitude_m)
        z = 0.5 * (float(start_wp[2]) + float(end_wp[2]))
        if target_world is not None and desired_distance is not None:
            tz = float(target_world[2])
            desired = float(desired_distance)
            if desired > 0.0:
                dz = z - tz
                if abs(dz) > desired:
                    z = tz + math.copysign(desired, dz)
        return float(z)

    def _build_anchor_arc(
        self,
        start_wp,
        end_wp,
        target_world,
        desired_distance,
        altitude_m,
        path_length_m,
        prev_wp=None,
        next_wp=None,
    ):
        if target_world is None:
            return []
        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])
        desired = max(float(desired_distance), float(self.min_orbit_radius_m))

        v0 = (float(start_wp[0]) - tx, float(start_wp[1]) - ty, float(start_wp[2]) - tz)
        v1 = (float(end_wp[0]) - tx, float(end_wp[1]) - ty, float(end_wp[2]) - tz)
        v0_len = math.sqrt(v0[0] * v0[0] + v0[1] * v0[1] + v0[2] * v0[2])
        v1_len = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2])
        if v0_len < 1e-6 or v1_len < 1e-6:
            return []

        v0n = (v0[0] / v0_len, v0[1] / v0_len, v0[2] / v0_len)
        v1n = (v1[0] / v1_len, v1[1] / v1_len, v1[2] / v1_len)

        dot = max(-1.0, min(1.0, v0n[0] * v1n[0] + v0n[1] * v1n[1] + v0n[2] * v1n[2]))
        theta = math.acos(dot)
        spacing = max(0.15, float(self.arc_waypoint_spacing_m))
        total_len = max(1e-6, theta * desired)
        num_total = max(7, int(math.ceil(total_len / spacing)) + 1)

        def _project_to_sphere(candidate):
            dx = float(candidate[0]) - tx
            dy = float(candidate[1]) - ty
            dz = float(candidate[2]) - tz
            n = math.sqrt(dx * dx + dy * dy + dz * dz)
            if n <= 1e-6:
                return candidate
            scale = desired / n
            return (tx + dx * scale, ty + dy * scale, tz + dz * scale)

        if theta < 1e-6:
            candidate = _project_to_sphere((tx + v0n[0] * desired, ty + v0n[1] * desired, tz + v0n[2] * desired))
            return [self._clamp_waypoint_to_bounds(candidate)]

        sin_theta = math.sin(theta)
        arc_pts = []
        for i in range(num_total):
            t = i / (num_total - 1)
            s0 = math.sin((1.0 - t) * theta) / sin_theta
            s1 = math.sin(t * theta) / sin_theta
            vx = s0 * v0n[0] + s1 * v1n[0]
            vy = s0 * v0n[1] + s1 * v1n[1]
            vz = s0 * v0n[2] + s1 * v1n[2]
            candidate = (tx + vx * desired, ty + vy * desired, tz + vz * desired)
            candidate = self._clamp_waypoint_to_bounds(candidate)
            candidate = _project_to_sphere(candidate)
            candidate = self._clamp_waypoint_to_bounds(candidate)
            arc_pts.append(candidate)
        return arc_pts

    def _score_arc_points(self, arc_pts, target_world):
        if target_world is None or self.cost_calculator is None:
            return float("inf")
        if not arc_pts or len(arc_pts) < 2:
            return float("inf")

        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])
        total = 0.0
        ref = arc_pts[0]
        for wp in arc_pts[1:]:
            cost = self.cost_calculator.estimate_full_cost_at_position(
                float(wp[0]),
                float(wp[1]),
                float(wp[2]),
                tx,
                ty,
                tz,
                reference_drone_x=float(ref[0]),
                reference_drone_y=float(ref[1]),
                reference_drone_z=float(ref[2]),
            )
            if cost is None or not math.isfinite(cost):
                return float("inf")
            total += float(cost)
            ref = wp
        return float(total)

    def _resolve_framing_altitude(self, base_pos):
        if self.focus_altitude_m > 0.0 and math.isfinite(self.focus_altitude_m):
            return float(self.focus_altitude_m)
        if base_pos is None:
            return float(self.orbit_altitude_m)
        return float(base_pos[2])

    def _enforce_focus_distance(self, candidate, target_world, desired_distance, altitude_m, tolerance_m):
        if target_world is None:
            return candidate

        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])
        x, y, _ = candidate
        z = float(altitude_m)
        dx = x - tx
        dy = y - ty
        dz = z - tz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if not math.isfinite(dist):
            return candidate

        desired = float(desired_distance)
        min_dist = math.sqrt(self.min_orbit_radius_m * self.min_orbit_radius_m + dz * dz)
        if desired < min_dist:
            desired = min_dist

        if abs(dist - desired) <= tolerance_m:
            return (x, y, z)

        xy_dist = math.hypot(dx, dy)
        if xy_dist < 1e-6:
            ref = self.current_pos or self.start_pos
            if ref is not None:
                dx = float(ref[0]) - tx
                dy = float(ref[1]) - ty
                xy_dist = math.hypot(dx, dy)
        if xy_dist < 1e-6:
            dx = math.cos(self.current_yaw)
            dy = math.sin(self.current_yaw)
            xy_dist = math.hypot(dx, dy)

        desired_xy = math.sqrt(max(desired * desired - dz * dz, 0.0))
        if desired_xy < self.min_orbit_radius_m:
            desired_xy = float(self.min_orbit_radius_m)
        scale = desired_xy / max(xy_dist, 1e-6)
        return (tx + dx * scale, ty + dy * scale, z)

    def _publish_anchor_points(self):
        if not self.anchors_set or self.anchor_start_idx is None or self.anchor_end_idx is None:
            return
        static_world = [self._rel_to_world(wp) for wp in self._scaled_static_waypoints()]
        if not static_world:
            return
        start_idx = int(max(0, min(self.anchor_start_idx, len(static_world) - 1)))
        end_idx = int(max(start_idx, min(self.anchor_end_idx, len(static_world) - 1)))
        start_wp = static_world[start_idx]
        end_wp = static_world[end_idx]
        now = self.get_clock().now().to_msg()
        start_msg = PointStamped()
        start_msg.header.stamp = now
        start_msg.header.frame_id = self.frame_id
        start_msg.point.x = float(start_wp[0])
        start_msg.point.y = float(start_wp[1])
        start_msg.point.z = float(start_wp[2])
        self.anchor_start_pub.publish(start_msg)
        end_msg = PointStamped()
        end_msg.header.stamp = now
        end_msg.header.frame_id = self.frame_id
        end_msg.point.x = float(end_wp[0])
        end_msg.point.y = float(end_wp[1])
        end_msg.point.z = float(end_wp[2])
        self.anchor_end_pub.publish(end_msg)
        self._publish_anchor_markers(start_wp, end_wp, now)

    def _publish_anchor_markers(self, start_wp, end_wp, now_msg):
        scale = max(0.01, float(self.anchor_marker_scale_m))
        markers = MarkerArray()

        start_marker = Marker()
        start_marker.header.stamp = now_msg
        start_marker.header.frame_id = self.frame_id
        start_marker.ns = "cinematography_anchors"
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.orientation.w = 1.0
        start_marker.pose.position.x = float(start_wp[0])
        start_marker.pose.position.y = float(start_wp[1])
        start_marker.pose.position.z = float(start_wp[2])
        start_marker.scale.x = scale
        start_marker.scale.y = scale
        start_marker.scale.z = scale
        start_marker.color.r = 1.0
        start_marker.color.g = 0.2
        start_marker.color.b = 0.2
        start_marker.color.a = 0.9
        markers.markers.append(start_marker)

        end_marker = Marker()
        end_marker.header.stamp = now_msg
        end_marker.header.frame_id = self.frame_id
        end_marker.ns = "cinematography_anchors"
        end_marker.id = 1
        end_marker.type = Marker.SPHERE
        end_marker.action = Marker.ADD
        end_marker.pose.orientation.w = 1.0
        end_marker.pose.position.x = float(end_wp[0])
        end_marker.pose.position.y = float(end_wp[1])
        end_marker.pose.position.z = float(end_wp[2])
        end_marker.scale.x = scale
        end_marker.scale.y = scale
        end_marker.scale.z = scale
        end_marker.color.r = 0.2
        end_marker.color.g = 0.2
        end_marker.color.b = 1.0
        end_marker.color.a = 0.9
        markers.markers.append(end_marker)

        self.anchor_marker_pub.publish(markers)

    def _publish_target_marker(self):
        target = self.target_world or self.last_target_world
        if target is None:
            return
        now = self.get_clock().now().to_msg()
        scale = max(0.01, float(self.target_marker_scale_m))
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.frame_id
        marker.ns = "cinematography_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float(target[0])
        marker.pose.position.y = float(target[1])
        marker.pose.position.z = float(target[2])
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = 0.8
        marker.color.g = 0.1
        marker.color.b = 0.9
        marker.color.a = 0.9
        self.target_marker_pub.publish(marker)

    def _debug_arc(self, arc_pts, target_world, desired_distance):
        if not self.debug_arc:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "cinematography_arc"
        marker.id = 0
        if not arc_pts:
            marker.action = Marker.DELETE
            self.arc_marker_pub.publish(marker)
            return

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = max(0.01, float(self.debug_arc_marker_scale))
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.9
        for wp in arc_pts:
            p = Point()
            p.x = float(wp[0])
            p.y = float(wp[1])
            p.z = float(wp[2])
            marker.points.append(p)
        self.arc_marker_pub.publish(marker)

        if target_world is None:
            return
        now = self.get_clock().now()
        if self.last_arc_log_time is not None:
            elapsed = (now - self.last_arc_log_time).nanoseconds / 1e9
            if elapsed < self.debug_arc_log_period_s:
                return
        self.last_arc_log_time = now

        desired = float(desired_distance)
        errors = []
        for wp in arc_pts:
            dist = math.dist(wp, target_world)
            if math.isfinite(dist):
                errors.append(abs(dist - desired))
        if not errors:
            return
        max_err = max(errors)
        mean_err = sum(errors) / len(errors)
        self.get_logger().info(
            f"Arc debug: points={len(arc_pts)} desired={desired:.3f} "
            f"max_err={max_err:.3f} mean_err={mean_err:.3f}"
        )

    def _publish_debug_visualization(self):
        if not self.enable_debug_viz:
            return

        markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = self.frame_id
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        now = self.get_clock().now().to_msg()

        if self.viz_static_path and self.viz_data["static_world"] is not None:
            markers.markers.extend(
                self._create_path_markers(
                    self.viz_data["static_world"],
                    "static_path",
                    (0.5, 0.5, 0.5, 0.4),
                    0.02,
                    now,
                    0,
                )
            )

        if self.viz_arc_only and self.viz_data["cinematographic_waypoints"] is not None:
            arc_pts = self.viz_data["cinematographic_waypoints"]
            markers.markers.extend(
                self._create_path_markers(
                    arc_pts,
                    "arc_path",
                    (0.0, 1.0, 0.0, 1.0),
                    0.06,
                    now,
                    100,
                )
            )
            for i, wp in enumerate(arc_pts):
                markers.markers.append(
                    self._create_waypoint_sphere(
                        wp,
                        200 + i,
                        "arc_waypoints",
                        (0.0, 1.0, 0.0, 0.8),
                        0.08,
                        now,
                    )
                )

        if self.viz_transitions:
            transition_in = self.viz_data["transition_in"] or []
            if transition_in:
                markers.markers.extend(
                    self._create_path_markers(
                        transition_in,
                        "transition_in",
                        (1.0, 1.0, 0.0, 0.8),
                        0.04,
                        now,
                        300,
                    )
                )
            transition_out = self.viz_data["transition_out"] or []
            if transition_out:
                markers.markers.extend(
                    self._create_path_markers(
                        transition_out,
                        "transition_out",
                        (1.0, 0.5, 0.0, 0.8),
                        0.04,
                        now,
                        400,
                    )
                )

        if self.viz_data["arc_start"] is not None:
            markers.markers.append(
                self._create_waypoint_sphere(
                    self.viz_data["arc_start"],
                    500,
                    "arc_anchors",
                    (1.0, 0.0, 0.0, 1.0),
                    0.12,
                    now,
                )
            )
            markers.markers.append(
                self._create_text_marker(
                    self.viz_data["arc_start"],
                    "ARC START",
                    501,
                    "arc_labels",
                    (1.0, 1.0, 1.0, 1.0),
                    0.15,
                    now,
                )
            )

        if self.viz_data["arc_end"] is not None:
            markers.markers.append(
                self._create_waypoint_sphere(
                    self.viz_data["arc_end"],
                    510,
                    "arc_anchors",
                    (0.0, 0.0, 1.0, 1.0),
                    0.12,
                    now,
                )
            )
            markers.markers.append(
                self._create_text_marker(
                    self.viz_data["arc_end"],
                    "ARC END",
                    511,
                    "arc_labels",
                    (1.0, 1.0, 1.0, 1.0),
                    0.15,
                    now,
                )
            )

        if self.viz_dynamic_path and self.viz_data["combined_world"] is not None:
            markers.markers.extend(
                self._create_path_markers(
                    self.viz_data["combined_world"],
                    "combined_path",
                    (0.0, 0.8, 1.0, 0.6),
                    0.03,
                    now,
                    600,
                )
            )

        if self.viz_active_path and self.active_waypoints is not None:
            origin = self.origin_offset if self.origin_offset is not None else (0.0, 0.0, 0.0)
            active_world = [
                (wp[0] + origin[0], wp[1] + origin[1], wp[2] + origin[2])
                for wp in self.active_waypoints
            ]
            markers.markers.extend(
                self._create_path_markers(
                    active_world,
                    "active_path",
                    (1.0, 0.0, 1.0, 1.0),
                    0.05,
                    now,
                    700,
                )
            )
            if 0 <= self.current_target_index < len(active_world):
                current_wp = active_world[self.current_target_index]
                markers.markers.append(
                    self._create_waypoint_sphere(
                        current_wp,
                        800,
                        "active_targets",
                        (1.0, 0.0, 1.0, 1.0),
                        0.15,
                        now,
                    )
                )
                markers.markers.append(
                    self._create_text_marker(
                        current_wp,
                        f"CURRENT #{self.current_target_index}",
                        801,
                        "active_labels",
                        (1.0, 1.0, 0.0, 1.0),
                        0.2,
                        now,
                    )
                )

        if self.viz_target_sphere and self.last_target_world is not None:
            focus_dist = self._resolve_focus_distance(None)
            if focus_dist > 0.0:
                markers.markers.append(
                    self._create_sphere_wireframe(
                        self.last_target_world,
                        focus_dist,
                        900,
                        "target_sphere",
                        (1.0, 0.0, 1.0, 0.3),
                        now,
                    )
                )

        if (
            self.viz_arc_only
            and self.viz_data["cinematographic_waypoints"] is not None
            and self.last_target_world is not None
        ):
            markers.markers.extend(
                self._create_distance_error_markers(
                    self.viz_data["cinematographic_waypoints"],
                    self.last_target_world,
                    self._resolve_focus_distance(None),
                    now,
                )
            )

        if self.viz_waypoint_numbers and self.active_waypoints is not None:
            origin = self.origin_offset if self.origin_offset is not None else (0.0, 0.0, 0.0)
            active_world = [
                (wp[0] + origin[0], wp[1] + origin[1], wp[2] + origin[2])
                for wp in self.active_waypoints
            ]
            for i, wp in enumerate(active_world):
                markers.markers.append(
                    self._create_text_marker(
                        wp,
                        str(i),
                        1000 + i,
                        "wp_numbers",
                        (1.0, 1.0, 1.0, 0.8),
                        0.1,
                        now,
                        offset_z=0.05,
                    )
                )

        self.debug_markers_pub.publish(markers)

    def _create_path_markers(self, waypoints, namespace, color, line_width, stamp, marker_id):
        markers = []

        line_marker = Marker()
        line_marker.header.stamp = stamp
        line_marker.header.frame_id = self.frame_id
        line_marker.ns = namespace
        line_marker.id = int(marker_id)
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = float(line_width)
        line_marker.color.r = float(color[0])
        line_marker.color.g = float(color[1])
        line_marker.color.b = float(color[2])
        line_marker.color.a = float(color[3])
        line_marker.pose.orientation.w = 1.0

        for wp in waypoints:
            p = Point()
            p.x = float(wp[0])
            p.y = float(wp[1])
            p.z = float(wp[2])
            line_marker.points.append(p)

        markers.append(line_marker)
        return markers

    def _create_waypoint_sphere(self, position, marker_id, namespace, color, scale, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(scale)
        marker.scale.y = float(scale)
        marker.scale.z = float(scale)
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        return marker

    def _create_text_marker(self, position, text, marker_id, namespace, color, scale, stamp, offset_z=0.2):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2]) + float(offset_z)
        marker.pose.orientation.w = 1.0
        marker.scale.z = float(scale)
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        marker.text = text
        return marker

    def _create_sphere_wireframe(self, center, radius, marker_id, namespace, color, stamp, num_segments=32):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        marker.pose.orientation.w = 1.0

        cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
        r = float(radius)

        for plane in ("xy", "xz", "yz"):
            for i in range(num_segments):
                angle1 = 2.0 * math.pi * i / num_segments
                angle2 = 2.0 * math.pi * (i + 1) / num_segments
                if plane == "xy":
                    p1 = Point(x=cx + r * math.cos(angle1), y=cy + r * math.sin(angle1), z=cz)
                    p2 = Point(x=cx + r * math.cos(angle2), y=cy + r * math.sin(angle2), z=cz)
                elif plane == "xz":
                    p1 = Point(x=cx + r * math.cos(angle1), y=cy, z=cz + r * math.sin(angle1))
                    p2 = Point(x=cx + r * math.cos(angle2), y=cy, z=cz + r * math.sin(angle2))
                else:
                    p1 = Point(x=cx, y=cy + r * math.cos(angle1), z=cz + r * math.sin(angle1))
                    p2 = Point(x=cx, y=cy + r * math.cos(angle2), z=cz + r * math.sin(angle2))
                marker.points.append(p1)
                marker.points.append(p2)

        return marker

    def _create_distance_error_markers(self, waypoints, target, desired_distance, stamp):
        markers = []
        if not waypoints or target is None:
            return markers

        tx, ty, tz = float(target[0]), float(target[1]), float(target[2])
        desired = float(desired_distance)
        for i, wp in enumerate(waypoints):
            dist = math.dist(wp, target)
            error = abs(dist - desired)
            if error < 0.05:
                color = (0.0, 1.0, 0.0, 0.8)
            elif error < 0.1:
                color = (1.0, 1.0, 0.0, 0.8)
            else:
                color = (1.0, 0.0, 0.0, 0.8)

            line = Marker()
            line.header.stamp = stamp
            line.header.frame_id = self.frame_id
            line.ns = "distance_errors"
            line.id = 1100 + i
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.01
            line.color.r = float(color[0])
            line.color.g = float(color[1])
            line.color.b = float(color[2])
            line.color.a = 0.3
            line.pose.orientation.w = 1.0
            p1 = Point(x=float(wp[0]), y=float(wp[1]), z=float(wp[2]))
            p2 = Point(x=tx, y=ty, z=tz)
            line.points.append(p1)
            line.points.append(p2)
            markers.append(line)

            if error > 0.05:
                markers.append(
                    self._create_text_marker(
                        wp,
                        f"Δ{error:.3f}m",
                        1200 + i,
                        "error_text",
                        color,
                        0.08,
                        stamp,
                        offset_z=-0.1,
                    )
                )

        return markers

    def generate_cinematographic_waypoints(self):
        if self.start_pos is None:
            return
        if self.detected_object is None:
            target_world = self.target_world or self.last_target_world
            if target_world is None and self.simulate_fake_object:
                base_pos = self.current_pos if self.current_pos is not None else self.start_pos
                if base_pos is not None:
                    offset = self.fake_object_world_offset
                    target_world = (
                        float(base_pos[0]) + float(offset[0]),
                        float(base_pos[1]) + float(offset[1]),
                        float(base_pos[2]) + float(offset[2]),
                    )
                    self.target_world = target_world
                    self.last_target_world = target_world
            if target_world is None:
                return
            depth = float(self.focus_depth_m) if self.focus_depth_m is not None else float(self.focus_distance_m)
            self.detected_object = {
                "center_x": float(self.cx),
                "center_y": float(self.cy),
                "depth": depth,
                "norm_x": 0.5,
                "norm_y": 0.5,
            }

        if self.anchor_start_idx is None or self.anchor_end_idx is None:
            self._set_anchors(target_world=self.target_world, obj_depth=self.detected_object["depth"])
        if self.anchor_start_idx is None or self.anchor_end_idx is None:
            return

        # Prefer globally computed focus depth if available.
        obj_depth = float(self.focus_depth_m) if self.focus_depth_m is not None else float(self.detected_object["depth"])
        center_x = float(self.detected_object["center_x"])
        center_y = float(self.detected_object["center_y"])
        norm_x = self.detected_object.get("norm_x", None)
        norm_y = self.detected_object.get("norm_y", None)

        if norm_x is not None and norm_y is not None:
            u = float(norm_x) * float(self.image_width)
            v = float(norm_y) * float(self.image_height)
        else:
            u = center_x
            v = center_y

        if self.fx > 0 and self.fy > 0:
            obj_x_m = (u - self.cx) / self.fx * obj_depth
            obj_y_m = (v - self.cy) / self.fy * obj_depth
        else:
            obj_x_m = 0.0
            obj_y_m = 0.0

        base_pos = self.current_pos if self.current_pos is not None else self.start_pos
        if base_pos is None:
            return

        target_world = self.target_world
        if target_world is None:
            target_world, _ = self._compute_object_world(self.detected_object)

        if target_world is None:
            center_x = float(self.detected_object["center_x"])
            center_y = float(self.detected_object["center_y"])
            norm_x = self.detected_object.get("norm_x", None)
            norm_y = self.detected_object.get("norm_y", None)

            if norm_x is not None and norm_y is not None:
                u = float(norm_x) * float(self.image_width)
                v = float(norm_y) * float(self.image_height)
            else:
                u = center_x
                v = center_y

            if self.fx > 0 and self.fy > 0:
                obj_x_m = (u - self.cx) / self.fx * obj_depth
                obj_y_m = (v - self.cy) / self.fy * obj_depth
            else:
                obj_x_m = 0.0
                obj_y_m = 0.0

            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            target_world = (
                base_pos[0] + cos_yaw * obj_x_m - sin_yaw * obj_y_m,
                base_pos[1] + sin_yaw * obj_x_m + cos_yaw * obj_y_m,
                base_pos[2],
            )
        self.last_target_world = target_world

        static_world = [self._rel_to_world(wp) for wp in self._scaled_static_waypoints()]
        start_idx = int(max(0, min(self.anchor_start_idx, len(static_world) - 2)))
        end_idx = int(max(start_idx + 1, min(self.anchor_end_idx, len(static_world) - 1)))
        self.viz_data["static_world"] = static_world
        self.viz_data["start_idx"] = start_idx
        self.viz_data["end_idx"] = end_idx

        start_wp = static_world[start_idx]
        end_wp = static_world[end_idx]

        anchor_dist_start = math.dist(start_wp, target_world)
        anchor_dist_end = math.dist(end_wp, target_world)
        anchor_distance = max(anchor_dist_start, anchor_dist_end)
        self.anchor_focus_distance = float(anchor_distance)
        focus_distance = max(float(anchor_distance), float(self.min_orbit_radius_m))
        base_z = self._resolve_framing_altitude_from_anchors(
            start_wp,
            end_wp,
            target_world=target_world,
            desired_distance=focus_distance,
        )
        base_z = max(self.min_altitude_m, min(self.max_altitude_m, base_z))
        path_length = self._path_length_between(static_world, start_idx, end_idx)

        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])
        dz = base_z - tz
        desired = focus_distance
        radius_xy = math.sqrt(max(desired * desired - dz * dz, 0.0))
        radius_xy = max(self.min_orbit_radius_m, radius_xy)

        def _project_to_circle(wp):
            dx = float(wp[0]) - tx
            dy = float(wp[1]) - ty
            xy_dist = math.hypot(dx, dy)
            if xy_dist < 1e-6:
                dx = 1.0
                dy = 0.0
                xy_dist = 1.0
            scale = radius_xy / xy_dist if radius_xy > 1e-6 else 0.0
            candidate = (tx + dx * scale, ty + dy * scale, base_z)
            candidate = self._clamp_waypoint_to_bounds(candidate)
            dist = math.dist(candidate, (tx, ty, tz))
            if math.isfinite(dist) and abs(dist - desired) > 1e-3:
                dx = float(candidate[0]) - tx
                dy = float(candidate[1]) - ty
                xy_dist = math.hypot(dx, dy)
                if xy_dist > 1e-6:
                    scale = radius_xy / xy_dist if radius_xy > 1e-6 else 0.0
                    candidate = (tx + dx * scale, ty + dy * scale, base_z)
                    candidate = self._clamp_waypoint_to_bounds(candidate)
            return candidate

        arc_start = _project_to_circle(start_wp)
        arc_end = _project_to_circle(end_wp)
        self.viz_data["arc_start"] = arc_start
        self.viz_data["arc_end"] = arc_end

        prev_static = static_world[start_idx - 1] if start_idx - 1 >= 0 else None
        next_static = static_world[end_idx + 1] if end_idx + 1 < len(static_world) else None

        cinematographic_waypoints = self._build_anchor_arc(
            arc_start,
            arc_end,
            target_world,
            focus_distance,
            base_z,
            path_length,
            prev_wp=prev_static,
            next_wp=next_static,
        )
        if cinematographic_waypoints:
            self.anchor_start_world = arc_start
            self.anchor_end_world = arc_end
        self.viz_data["cinematographic_waypoints"] = cinematographic_waypoints
        self._debug_arc(cinematographic_waypoints, target_world, desired)

        transition_in = []
        transition_out = []
        if cinematographic_waypoints:
            if prev_static is not None:
                prev_prev = static_world[start_idx - 2] if start_idx - 2 >= 0 else None
                next_dynamic = cinematographic_waypoints[1] if len(cinematographic_waypoints) > 1 else None
                transition_in = self._build_bezier_transition(
                    prev_static,
                    cinematographic_waypoints[0],
                    prev_point=prev_prev,
                    next_point=next_dynamic,
                )

            if next_static is not None:
                next_next = static_world[end_idx + 2] if end_idx + 2 < len(static_world) else None
                prev_dynamic = cinematographic_waypoints[-2] if len(cinematographic_waypoints) > 1 else None
                transition_out = self._build_bezier_transition(
                    cinematographic_waypoints[-1],
                    next_static,
                    prev_point=prev_dynamic,
                    next_point=next_next,
                )

        if not cinematographic_waypoints:
            combined_world = static_world
        else:
            combined_world = (
                static_world[:start_idx]
                + transition_in
                + cinematographic_waypoints
                + transition_out
                + static_world[end_idx + 1 :]
            )
        self.viz_data["transition_in"] = transition_in
        self.viz_data["transition_out"] = transition_out
        self.viz_data["combined_world"] = combined_world

        transition = []
        if (
            self.current_pos is not None
            and len(combined_world) >= 2
            and self.bezier_transition_points > 0
            and self.bezier_blend_length_m > 0.0
        ):
            p0 = self.current_pos
            p3 = combined_world[0]
            p_next = combined_world[1]

            v0_dir, v0_mag = self._normalize3(self.current_vel)
            if v0_mag < self.bezier_min_speed_mps:
                fallback = (p3[0] - p0[0], p3[1] - p0[1], p3[2] - p0[2])
                v0_dir, _ = self._normalize3(fallback)

            v3_dir, _ = self._normalize3((p_next[0] - p3[0], p_next[1] - p3[1], p_next[2] - p3[2]))
            alpha = float(self.bezier_blend_length_m)
            beta = float(self.bezier_blend_length_m)
            p1 = (p0[0] + alpha * v0_dir[0], p0[1] + alpha * v0_dir[1], p0[2] + alpha * v0_dir[2])
            p2 = (p3[0] - beta * v3_dir[0], p3[1] - beta * v3_dir[1], p3[2] - beta * v3_dir[2])

            transition = self._sample_cubic_bezier(p0, p1, p2, p3, self.bezier_transition_points)
            transition = [self._clamp_waypoint_to_bounds(wp) for wp in transition]

        if transition:
            combined_world = transition + combined_world

        # Store as relative to the recorded origin so publishing always does origin + relative.
        origin = self.origin_offset if self.origin_offset is not None else (0.0, 0.0, 0.0)
        self.dynamic_waypoints = [
            (wp[0] - origin[0], wp[1] - origin[1], wp[2] - origin[2]) for wp in combined_world
        ]

    def _clamp_waypoint_to_bounds(self, wp):
        x, y, z = wp
        ox, oy = (self.origin_offset or (0.0, 0.0, 0.0))[:2]
        x_min = self.world_bounds_min[0] + ox
        x_max = self.world_bounds_max[0] + ox
        y_min = self.world_bounds_min[1] + oy
        y_max = self.world_bounds_max[1] + oy

        x = max(x_min, min(x_max, float(x)))
        y = max(y_min, min(y_max, float(y)))
        original_z = float(z)
        if self.enable_z_clamp:
            ceiling = (
                float(self.z_ceiling_override_m)
                if self.z_ceiling_override_m > 0.0
                else min(self.world_bounds_max[2], self.virtual_ceiling_m)
            )
            z = max(self.world_bounds_min[2], min(ceiling, float(z)))
            z = max(self.min_altitude_m, min(self.max_altitude_m, z))
            if abs(z - original_z) > 1e-6:
                self.get_logger().debug(f"Z clamped from {original_z:.2f} to {z:.2f}")
        else:
            # Only enforce minimum altitude to avoid ground collisions.
            z = max(self.world_bounds_min[2], max(self.min_altitude_m, float(z)))
        return (x, y, z)

    def publish_simulated_detection(self):
        fake_detection = Float32MultiArray()
        fake_detection.data = [1.0, 320.0, 240.0, 1.5, 5000.0, 0.05, 0.9, 0.5, 0.5]
        self.fake_detection_pub.publish(fake_detection)
        if not self.simulate_fake_object:
            return
        if not self.simulated_detection_active and self.detected_object is not None:
            return
        parsed = self._parse_detection(fake_detection)
        if parsed is None:
            return
        now = self.get_clock().now()
        if self.object_detection_time is not None:
            elapsed = (now - self.object_detection_time).nanoseconds / 1e9
            if elapsed < self.DETECTION_DEBOUNCE:
                return
        self.detected_object = parsed
        self.object_detection_time = now
        self.simulated_detection_active = True
        self.use_dynamic_waypoints = True
        self.pending_detection_merge = True
        self._set_anchors(target_world=self.target_world, obj_depth=self.detected_object["depth"])
        if self.dynamic_waypoints is None or not self.use_dynamic_waypoints:
            self._rebuild_active_with_detection()

    def check_and_send_waypoints(self):
        if self.start_pos is not None and self.waypoints_sent:
            return

        if self.start_pos is not None and self.ready_time is not None:
            elapsed = (self.get_clock().now() - self.ready_time).nanoseconds / 1e9
            if elapsed >= self.STARTUP_DELAY or self.velocity_settled:
                if self.active_waypoints is None:
                    self._initialize_waypoint_sequence()
                self._step_waypoint_sequence()

    def _should_scale_static_waypoints(self):
        return (
            (not self.use_dynamic_waypoints)
            and not (self.pending_detection_merge and self.detected_object is not None)
            and not self.simulate_fake_object
        )

    def _scaled_static_waypoints(self):
        waypoints = list(self.static_waypoints)
        if self._should_scale_static_waypoints() and (
            self.waypoint_xy_scale != 1.0 or self.waypoint_y_extra_scale != 1.0
        ):
            sx = self.waypoint_xy_scale
            sy = self.waypoint_xy_scale * self.waypoint_y_extra_scale
            waypoints = [(wp[0] * sx, wp[1] * sy, wp[2]) for wp in waypoints]
        return waypoints

    def _initialize_waypoint_sequence(self):
        if self.use_dynamic_waypoints and self.dynamic_waypoints is None:
            if self.detected_object is not None or self.target_world is not None or self.last_target_world is not None:
                self.get_logger().info("Generating dynamic waypoints for active path")
                self.generate_cinematographic_waypoints()
            if self.dynamic_waypoints is None:
                self.get_logger().warn("Dynamic waypoints unavailable; falling back to static path")
                self.use_dynamic_waypoints = False

        if self.use_dynamic_waypoints and self.dynamic_waypoints is not None:
            waypoints_to_send = self.dynamic_waypoints
        else:
            waypoints_to_send = self._scaled_static_waypoints()

        if (not self.use_dynamic_waypoints) and self.pending_detection_merge and self.detected_object is not None:
            waypoints_to_send = self._merge_framing_with_static(waypoints_to_send, self.detected_object)
            self.pending_detection_merge = False
        elif self.simulate_fake_object and (not self.use_dynamic_waypoints):
            waypoints_to_send = self._insert_fake_object_orbit(waypoints_to_send)

        waypoints_to_send = self._deduplicate_waypoints(waypoints_to_send, min_separation=0.3)
        if not self.use_dynamic_waypoints:
            waypoints_to_send = self._apply_descent_tail(waypoints_to_send)

        self.active_waypoints = self._limit_consecutive_waypoint_jumps(waypoints_to_send)
        self.current_target_index = 0
        self.last_published_index = -1
        self.last_progress_time = None
        self.last_progress_dist = None
        self.waypoints_sent = False
        self._update_framing_indices()
        self._publish_framing_active(self._is_framing_idx(self.current_target_index))
        if self.current_pos is not None and self.active_waypoints:
            skipped = 0
            min_start_threshold = max(self._current_distance_threshold(), float(self.min_first_waypoint_distance_m))
            while self.current_target_index < len(self.active_waypoints):
                target_world = self._rel_to_world(self.active_waypoints[self.current_target_index])
                dist = self._distance_to(target_world)
                threshold = min_start_threshold
                if self.current_target_index == 0:
                    threshold = min(threshold, max(dist * 0.9, self.PROGRESS_EPS_M))
                if dist > threshold:
                    break
                self.current_target_index += 1
                skipped += 1
            if skipped > 0:
                self.last_published_index = self.current_target_index - 1
                self.get_logger().info(
                    f"Skipped {skipped} near waypoints at init; starting at "
                    f"{self.current_target_index} (min_start_threshold={min_start_threshold:.2f})"
                )
        self._prefill_waypoint_queue()

    def _limit_consecutive_waypoint_jumps(self, waypoints):
        if not waypoints or self.max_waypoint_jump_m <= 0:
            return waypoints

        out = [waypoints[0]]
        for wp in waypoints[1:]:
            prev = out[-1]
            dx = wp[0] - prev[0]
            dy = wp[1] - prev[1]
            dz = wp[2] - prev[2]
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5

            if dist <= self.max_waypoint_jump_m:
                out.append(wp)
                continue

            steps = max(1, int(math.ceil(dist / self.max_waypoint_jump_m)))
            for s in range(1, steps + 1):
                t = s / steps
                out.append((prev[0] + t * dx, prev[1] + t * dy, prev[2] + t * dz))
        return out

    def _apply_descent_tail(self, waypoints):
        if not waypoints or len(waypoints) < 3:
            return waypoints

        tail_count = max(3, int(len(waypoints) * 0.3))
        start_idx = max(0, len(waypoints) - tail_count)
        start_z = waypoints[start_idx][2]
        target_z = max(self.min_altitude_m, self.world_bounds_min[2])

        if start_z <= target_z + 1e-3:
            return waypoints

        adjusted = list(waypoints)
        for i in range(len(waypoints) - start_idx):
            t = (i + 1) / (len(waypoints) - start_idx)
            z = start_z + t * (target_z - start_z)
            wp = adjusted[start_idx + i]
            adjusted[start_idx + i] = (wp[0], wp[1], z)
        return adjusted

    def _deduplicate_waypoints(self, waypoints, min_separation=0.3):
        if not waypoints:
            return waypoints
        out = []
        prev = None
        for wp in waypoints:
            if prev is not None:
                dx = wp[0] - prev[0]
                dy = wp[1] - prev[1]
                dz = wp[2] - prev[2]
                if (dx * dx + dy * dy + dz * dz) ** 0.5 < min_separation:
                    continue
            out.append(wp)
            prev = wp
        return out

    def _compute_object_world(self, detection):
        if detection is None:
            return None
        obj_depth = float(detection["depth"])
        center_x = float(detection["center_x"])
        center_y = float(detection["center_y"])
        norm_x = detection.get("norm_x", None)
        norm_y = detection.get("norm_y", None)

        if norm_x is not None and norm_y is not None:
            u = float(norm_x) * float(self.image_width)
            v = float(norm_y) * float(self.image_height)
        else:
            u = center_x
            v = center_y

        if self.fx > 0 and self.fy > 0:
            obj_x_m = (u - self.cx) / self.fx * obj_depth
            obj_y_m = (v - self.cy) / self.fy * obj_depth
        else:
            obj_x_m = 0.0
            obj_y_m = 0.0

        base_pos = self.current_pos if self.current_pos is not None else self.start_pos
        if base_pos is None:
            return None, obj_depth

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        world_dx = cos_yaw * obj_x_m - sin_yaw * obj_y_m
        world_dy = sin_yaw * obj_x_m + cos_yaw * obj_y_m
        center_world = (base_pos[0] + world_dx, base_pos[1] + world_dy, base_pos[2])
        self.last_target_world = center_world
        return center_world, obj_depth

    def _build_orbit_segment(self, center_world, radius, z, num_pts=16, reference_world=None):
        radius = max(self.min_orbit_radius_m, float(radius))
        z = max(self.min_altitude_m, min(self.max_altitude_m, float(z)))
        num_pts = int(max(1, num_pts))

        reference = reference_world
        if reference is None:
            reference = self.current_pos if self.current_pos is not None else self.start_pos

        best_radius = float(radius)
        if bool(self.enable_cost_optimization) and reference is not None:
            radius_samples = int(max(1, self.candidate_radius_samples))
            r_scale_min, r_scale_max = [float(v) for v in self.candidate_radius_scale_range]
            r_scale_min, r_scale_max = (min(r_scale_min, r_scale_max), max(r_scale_min, r_scale_max))

            r0 = float(radius)
            r_min = max(self.min_orbit_radius_m, r0 * r_scale_min)
            r_max = max(r_min, r0 * r_scale_max)
            radii = [r_min + (r_max - r_min) * k / (radius_samples - 1) for k in range(radius_samples)] if radius_samples > 1 else [r0]

            best_total = float("inf")
            for rr in radii:
                total = 0.0
                ref = reference
                for i in range(num_pts):
                    t = 2.0 * math.pi * i / num_pts
                    candidate = self._clamp_waypoint_to_bounds((center_world[0] + rr * math.cos(t), center_world[1] + rr * math.sin(t), z))
                    c = self._estimate_waypoint_cost(candidate, center_world, ref)
                    if c is None or not math.isfinite(c):
                        total = float("inf")
                        break
                    total += float(c)
                    ref = candidate

                if total < best_total:
                    best_total = total
                    best_radius = float(rr)

        waypoints = []
        for i in range(num_pts):
            t = 2.0 * math.pi * i / num_pts
            x = center_world[0] + best_radius * math.cos(t)
            y = center_world[1] + best_radius * math.sin(t)
            waypoints.append(self._clamp_waypoint_to_bounds((x, y, z)))
        return waypoints

    def _merge_framing_with_static(self, rel_waypoints, detection):
        if not rel_waypoints or detection is None:
            return rel_waypoints

        origin = self.origin_offset if self.origin_offset is not None else (0.0, 0.0, 0.0)
        wps_world = [(wp[0] + origin[0], wp[1] + origin[1], wp[2] + origin[2]) for wp in rel_waypoints]

        center_world, obj_depth = self._compute_object_world(detection)
        if center_world is None:
            return rel_waypoints
        self.last_target_world = center_world

        # Anchor at the point on the nominal path closest to the detected object (world XY),
        # so the inserted orbit actually wraps the object rather than a fixed "max Y" heuristic.
        idx_anchor = min(
            range(len(wps_world)),
            key=lambda i: (wps_world[i][0] - center_world[0]) ** 2 + (wps_world[i][1] - center_world[1]) ** 2,
        )
        anchor_wp = wps_world[idx_anchor]
        focus_distance = self._resolve_focus_distance(obj_depth)
        dz = float(anchor_wp[2]) - float(center_world[2])
        radius_xy = math.sqrt(max(focus_distance * focus_distance - dz * dz, 0.0))
        orbit_radius = max(self.min_orbit_radius_m, radius_xy)
        orbit = self._build_orbit_segment(center_world, orbit_radius, anchor_wp[2], reference_world=anchor_wp)

        overlap_thresh = 0.35
        def too_close(wp):
            for o in orbit:
                dx = wp[0] - o[0]
                dy = wp[1] - o[1]
                dz = wp[2] - o[2]
                if (dx * dx + dy * dy + dz * dz) ** 0.5 < overlap_thresh:
                    return True
            return False

        head = [wp for wp in wps_world[: idx_anchor + 1] if not too_close(wp)]
        tail = [wp for wp in wps_world[idx_anchor + 1 :] if not too_close(wp)]
        merged_world = head + orbit + tail

        return [(wp[0] - origin[0], wp[1] - origin[1], wp[2] - origin[2]) for wp in merged_world]

    def _rel_to_world(self, target_rel):
        origin = self.origin_offset if self.origin_offset is not None else (0.0, 0.0, 0.0)
        target_world = (
            target_rel[0] + origin[0],
            target_rel[1] + origin[1],
            target_rel[2] + origin[2],
        )
        return self._clamp_waypoint_to_bounds(target_world)

    def _distance_to(self, target_world):
        dx = self.current_pos[0] - target_world[0]
        dy = self.current_pos[1] - target_world[1]
        dz = self.current_pos[2] - target_world[2]
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _closest_waypoint_index(self, target_world):
        if self.active_waypoints is None:
            return None
        best_idx = None
        best_dist_sq = float("inf")
        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])
        for i, wp_rel in enumerate(self.active_waypoints):
            wp_world = self._rel_to_world(wp_rel)
            dx = float(wp_world[0]) - tx
            dy = float(wp_world[1]) - ty
            dz = float(wp_world[2]) - tz
            dist_sq = dx * dx + dy * dy + dz * dz
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_idx = i
        return best_idx

    def _update_framing_indices(self):
        self.framing_start_idx = None
        self.framing_end_idx = None
        if self.active_waypoints is None:
            return
        if not self.anchors_set:
            return
        if self.anchor_start_world is None or self.anchor_end_world is None:
            return
        start_idx = self._closest_waypoint_index(self.anchor_start_world)
        end_idx = self._closest_waypoint_index(self.anchor_end_world)
        if start_idx is None or end_idx is None:
            return
        if end_idx < start_idx:
            start_idx, end_idx = end_idx, start_idx
        self.framing_start_idx = start_idx
        self.framing_end_idx = end_idx

    def _is_framing_idx(self, idx):
        if self.framing_start_idx is None or self.framing_end_idx is None:
            return False
        return self.framing_start_idx <= idx <= self.framing_end_idx

    def _current_distance_threshold(self):
        base = float(self.DISTANCE_THRESHOLD_M)
        min_goal = float(self.min_goal_distance_m)
        if self._is_framing_idx(self.current_target_index):
            spacing = float(self.arc_waypoint_spacing_m) if self.arc_waypoint_spacing_m > 0.0 else base
            return max(min_goal, min(base, 0.9 * spacing))
        return max(base, min_goal)

    def _current_target_reached(self):
        if self.active_waypoints is None or self.current_pos is None:
            return False
        if self.current_target_index >= len(self.active_waypoints):
            return True
        target_world = self._rel_to_world(self.active_waypoints[self.current_target_index])
        return self._distance_to(target_world) <= self._current_distance_threshold()

    def _publish_framing_active(self, active):
        active = bool(active)
        if self.framing_active == active:
            return
        msg = Bool()
        msg.data = active
        self.framing_active_pub.publish(msg)
        self.framing_active = active

    def _publish_waypoint(self, target_world, idx: int):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = float(target_world[0])
        goal.pose.position.y = float(target_world[1])
        goal.pose.position.z = float(target_world[2])
        # Face the object only while framing between anchors; otherwise keep current yaw.
        yaw = self.current_yaw
        if self._is_framing_idx(idx):
            look_at = self.last_target_world or self.target_world
            if look_at is not None:
                dx = float(look_at[0]) - float(target_world[0])
                dy = float(look_at[1]) - float(target_world[1])
                yaw = math.atan2(dy, dx)
        qw = math.cos(0.5 * yaw)
        qz = math.sin(0.5 * yaw)
        goal.pose.orientation.w = qw
        goal.pose.orientation.z = qz
        self.goal_pub.publish(goal)

        if idx == self.current_target_index:
            self.last_progress_time = self.get_clock().now()
            self.last_progress_dist = self._distance_to(target_world)
            self.stuck_attempts = 0

    def _prefill_waypoint_queue(self):
        if self.active_waypoints is None or self.current_pos is None:
            return
        lookahead = self.WAYPOINT_LOOKAHEAD
        if self.use_dynamic_waypoints and self.dynamic_waypoints is not None:
            lookahead = self.dynamic_waypoint_lookahead
        lookahead = max(1, int(lookahead))

        while (
            self.last_published_index < len(self.active_waypoints) - 1
            and (self.last_published_index - self.current_target_index + 1) < lookahead
        ):
            if self.last_published_index >= self.current_target_index and not self._current_target_reached():
                break
            next_idx = self.last_published_index + 1
            target_world = self._rel_to_world(self.active_waypoints[next_idx])
            self._publish_waypoint(target_world, next_idx)
            self.last_published_index = next_idx

    def _step_waypoint_sequence(self):
        if self.active_waypoints is None or self.current_pos is None:
            self._publish_framing_active(False)
            return

        if self.current_target_index >= len(self.active_waypoints):
            self.waypoints_sent = True
            self._publish_framing_active(False)
            return

        self._publish_framing_active(self._is_framing_idx(self.current_target_index))

        target_world = self._rel_to_world(self.active_waypoints[self.current_target_index])

        dx = self.current_pos[0] - target_world[0]
        dy = self.current_pos[1] - target_world[1]
        dz = self.current_pos[2] - target_world[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5

        now = self.get_clock().now()
        if self.last_progress_time is None:
            self.last_progress_time = now
            self.last_progress_dist = dist

        if dist < (self.last_progress_dist or 0.0) - self.PROGRESS_EPS_M:
            self.last_progress_time = now
            self.last_progress_dist = dist
        else:
            if self.enable_progress_skip:
                elapsed_stuck = (now - self.last_progress_time).nanoseconds / 1e9
                if elapsed_stuck >= self.PROGRESS_TIMEOUT_S and dist > self.DISTANCE_THRESHOLD_M * 0.5:
                    self.get_logger().warn(
                        f"No progress toward waypoint {self.current_target_index} for {elapsed_stuck:.1f}s; skipping it"
                    )
                    self.current_target_index += 1
                    self.last_progress_time = None
                    self.last_progress_dist = None
                    self.stuck_attempts = 0
                    self._prefill_waypoint_queue()
                    return

        if self.current_target_index <= 1:
            min_reach = max(self._current_distance_threshold(), float(self.min_first_waypoint_distance_m))
            if dist <= min_reach:
                self.get_logger().info(
                    f"Skipping close waypoint {self.current_target_index} at {dist:.2f}m "
                    f"(min_reach={min_reach:.2f})"
                )
                self.current_target_index += 1
                self.last_progress_time = None
                self.last_progress_dist = None
                self.stuck_attempts = 0
                self._prefill_waypoint_queue()
                if self.current_target_index >= len(self.active_waypoints):
                    self.waypoints_sent = True
                return

        if dist <= self._current_distance_threshold():
            self.current_target_index += 1
            self.last_progress_time = None
            self.last_progress_dist = None
            self.stuck_attempts = 0
            self._prefill_waypoint_queue()
            if self.current_target_index >= len(self.active_waypoints):
                self.waypoints_sent = True
            return

        self._prefill_waypoint_queue()
    def _build_fake_object_waypoints(self, obj_world, radius, base_z):
        radius = max(self.min_orbit_radius_m, radius)
        num_pts = 16
        waypoints = []
        z = max(self.min_altitude_m, min(self.max_altitude_m, base_z))
        for i in range(num_pts):
            t = 2.0 * math.pi * i / num_pts
            x = obj_world[0] + radius * math.cos(t)
            y = obj_world[1] + radius * math.sin(t)
            waypoints.append(self._clamp_waypoint_to_bounds((x, y, z)))
        return waypoints

    def _compute_fake_object_position(self, anchor_wp):
        """
        Place a fake object just to the right of the initial trajectory direction,
        with an optional absolute offset. This keeps the orbit circular and offset
        from the nominal path.
        """
        base_x, base_y, base_z = anchor_wp
        yaw = self.current_yaw
        # Right-hand side offset relative to heading
        dx_side = math.sin(yaw) * self.fake_object_side_offset_m
        dy_side = -math.cos(yaw) * self.fake_object_side_offset_m
        # Outward along Y (same sign as anchor Y) to keep it outside the path and force an inward curve.
        sign_y = 1.0 if base_y >= 0.0 else -1.0
        dy_out = sign_y * self.fake_object_outward_offset_m

        obj_world = (
            base_x + self.fake_object_world_offset[0] + dx_side,
            base_y + self.fake_object_world_offset[1] + dy_side + dy_out,
            base_z + self.fake_object_world_offset[2],
        )
        obj_world = (
            obj_world[0],
            obj_world[1],
            max(self.min_altitude_m, min(self.max_altitude_m, obj_world[2])),
        )
        return obj_world

    def _insert_fake_object_orbit(self, waypoints):
        if not waypoints:
            return waypoints

        # Pick the waypoint with the largest Y (outer-most on Y-axis).
        idx_max_y = max(range(len(waypoints)), key=lambda i: waypoints[i][1])
        anchor_wp = waypoints[idx_max_y]
        obj_world = self._compute_fake_object_position(anchor_wp)
        self.last_target_world = obj_world
        prev_wp = waypoints[idx_max_y - 1] if idx_max_y - 1 >= 0 else None
        next_wp = waypoints[idx_max_y + 1] if idx_max_y + 1 < len(waypoints) else None
        orbit = self._build_framing_arc(
            obj_world,
            anchor_wp,
            self.fake_object_radius_m,
            prev_wp=prev_wp,
            next_wp=next_wp,
        )
        return waypoints[: idx_max_y + 1] + orbit + waypoints[idx_max_y + 1 :]

    def _build_framing_arc(self, obj_world, anchor_wp, radius, prev_wp=None, next_wp=None):
        """
        Build the shortest circular arc between incoming and outgoing directions
        around the object, with no fixed span. The arc bends toward the object
        side and follows the minimal angle between the two headings.
        """
        cx, cy, _ = obj_world
        ax, ay, az = anchor_wp
        # Choose a radius that stays well inside the object->anchor line so we don’t bulge outward.
        dist_oa = math.hypot(ax - cx, ay - cy)
        desired_r = min(radius, 0.4 * dist_oa) if dist_oa > 1e-3 else radius
        radius = max(self.min_orbit_radius_m, desired_r)
        z = max(self.min_altitude_m, min(self.max_altitude_m, az))

        # Incoming/outgoing headings to bias the turn direction.
        def _dir_from(a, b):
            return (a[0] - b[0], a[1] - b[1])

        approach = _dir_from(anchor_wp, prev_wp) if prev_wp is not None else None
        depart = _dir_from(next_wp, anchor_wp) if next_wp is not None else None

        def _angle(v):
            return math.atan2(v[1], v[0])

        approach_ang = _angle(approach) if approach else None
        depart_ang = _angle(depart) if depart else approach_ang

        # Determine start/end angles around the object; default to anchor direction if missing.
        start_angle = math.atan2(ay - cy, ax - cx)
        end_angle = start_angle
        if next_wp is not None:
            end_angle = math.atan2(next_wp[1] - cy, next_wp[0] - cx)
        # Pick shortest angular direction.
        delta = (end_angle - start_angle + math.pi) % (2 * math.pi) - math.pi
        # If we have approach vector, ensure we bend toward the object side of it.
        if approach is not None:
            obj_vec = (cx - ax, cy - ay)
            cross = approach[0] * obj_vec[1] - approach[1] * obj_vec[0]
            turn_sign = 1.0 if cross > 0.0 else -1.0
            if delta * turn_sign < 0:
                # Flip to go the other way around the circle.
                delta = delta - math.copysign(2 * math.pi, delta)
        num_pts = 7

        arc_pts = []
        for i in range(num_pts):
            t = i / (num_pts - 1)
            ang = start_angle + t * delta
            x = cx + radius * math.cos(ang)
            y = cy + radius * math.sin(ang)
            arc_pts.append(self._clamp_waypoint_to_bounds((x, y, z)))
        return arc_pts

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


if __name__ == "__main__":
    main()
