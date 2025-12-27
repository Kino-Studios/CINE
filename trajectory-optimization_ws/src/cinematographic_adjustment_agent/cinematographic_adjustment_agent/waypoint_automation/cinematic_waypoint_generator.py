import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class CinematicWaypointGenerator(Node):
    def __init__(self):
        super().__init__("cinematic_waypoint_generator")

        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.declare_parameter("odom_topic", "/drone_0_visual_slam/odom")
        self.declare_parameter("detections_topic", "/object_detections")

        odom_topic = str(self.get_parameter("odom_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.detection_sub = self.create_subscription(Float32MultiArray, detections_topic, self.detection_callback, 10)

        self.start_pos = None
        self.origin_offset = None
        self.current_yaw = 0.0
        self.waypoints_sent = False
        self.ready_time = None
        self.STARTUP_DELAY = 0.5
        self.WAYPOINT_TIMER_PERIOD_S = 0.05 # planner must be rapidly fed to prevent stalling
        self.WAYPOINT_LOOKAHEAD = 3

        self.velocity_settled = False
        self.VELOCITY_THRESHOLD = 0.2
        self.current_vel = (0.0, 0.0, 0.0)
        self.current_pos = None

        self.object_detection_time = None
        self.DETECTION_DEBOUNCE = 2.0
        self.DISTANCE_THRESHOLD_M = 0.8
        self.PROGRESS_EPS_M = 0.05
        self.PROGRESS_TIMEOUT_S = 2.0

        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("camera_fx", 387.229248046875)
        self.declare_parameter("camera_fy", 387.229248046875)
        self.declare_parameter("camera_cx", 320.5)
        self.declare_parameter("camera_cy", 240.5)
        self.declare_parameter("orbit_altitude_m", 1.5)
        self.declare_parameter("min_altitude_m", 0.2)
        self.declare_parameter("max_altitude_m", 3.0)
        self.declare_parameter("min_orbit_radius_m", 0.6)
        self.declare_parameter("max_waypoint_jump_m", 3.0)
        self.declare_parameter("bezier_blend_length_m", 0.75)
        self.declare_parameter("bezier_transition_points", 6)
        self.declare_parameter("bezier_min_speed_mps", 0.05)
        self.declare_parameter("waypoint_xy_scale", 1.1)
        self.declare_parameter("waypoint_y_extra_scale", 1.05)
        self.declare_parameter("waypoint_z_scale", 1.1)
        self.declare_parameter("enable_z_clamp", False)
        self.declare_parameter("z_ceiling_override_m", -1.0)
        self.declare_parameter("world_bounds_min", [-10.0, -10.0, 0.0])
        self.declare_parameter("world_bounds_max", [10.0, 10.0, 2.8])
        self.declare_parameter("virtual_ceiling_m", 2.8)
        self.declare_parameter("simulate_fake_object", True)
        # Place a fake object slightly ahead of, and to the right of, the outer-most Y waypoint.
        self.declare_parameter("fake_object_world_offset", [0.3, 0.6, 0.0])
        self.declare_parameter("fake_object_side_offset_m", 0.6)
        self.declare_parameter("fake_object_radius_m", 0.8)
        self.declare_parameter("fake_object_outward_offset_m", 1.0)

        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.fx = float(self.get_parameter("camera_fx").value)
        self.fy = float(self.get_parameter("camera_fy").value)
        self.cx = float(self.get_parameter("camera_cx").value)
        self.cy = float(self.get_parameter("camera_cy").value)
        self.orbit_altitude_m = float(self.get_parameter("orbit_altitude_m").value)
        self.min_altitude_m = float(self.get_parameter("min_altitude_m").value)
        self.max_altitude_m = float(self.get_parameter("max_altitude_m").value)
        self.min_orbit_radius_m = float(self.get_parameter("min_orbit_radius_m").value)
        self.max_waypoint_jump_m = float(self.get_parameter("max_waypoint_jump_m").value)
        self.bezier_blend_length_m = float(self.get_parameter("bezier_blend_length_m").value)
        self.bezier_transition_points = int(self.get_parameter("bezier_transition_points").value)
        self.bezier_min_speed_mps = float(self.get_parameter("bezier_min_speed_mps").value)
        self.waypoint_xy_scale = float(self.get_parameter("waypoint_xy_scale").value)
        self.waypoint_y_extra_scale = float(self.get_parameter("waypoint_y_extra_scale").value)
        self.waypoint_z_scale = float(self.get_parameter("waypoint_z_scale").value)
        self.enable_z_clamp = bool(self.get_parameter("enable_z_clamp").value)
        self.z_ceiling_override_m = float(self.get_parameter("z_ceiling_override_m").value)
        self.world_bounds_min = [float(v) for v in self.get_parameter("world_bounds_min").value]
        self.world_bounds_max = [float(v) for v in self.get_parameter("world_bounds_max").value]
        self.virtual_ceiling_m = float(self.get_parameter("virtual_ceiling_m").value)
        self.max_altitude_m = min(self.max_altitude_m, self.virtual_ceiling_m)
        self.simulate_fake_object = bool(self.get_parameter("simulate_fake_object").value)
        self.fake_object_world_offset = [float(v) for v in self.get_parameter("fake_object_world_offset").value]
        self.fake_object_side_offset_m = float(self.get_parameter("fake_object_side_offset_m").value)
        self.fake_object_radius_m = float(self.get_parameter("fake_object_radius_m").value)
        self.fake_object_outward_offset_m = float(self.get_parameter("fake_object_outward_offset_m").value)

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
        self.use_dynamic_waypoints = False
        self.active_waypoints = None
        self.current_target_index = 0
        self.last_published_index = -1
        self.simulated_detection_active = False
        self.pending_detection_merge = False

        self.stuck_progress_threshold_m = 0.25
        self.stuck_time_s = 0.8
        self.nudge_offset_xy_m = 0.5
        self.nudge_offset_z_m = 0.25
        self.max_stuck_nudges = 2
        self.last_progress_time = None
        self.last_progress_dist = None
        self.stuck_attempts = 0

        self.fake_detection_pub = self.create_publisher(Float32MultiArray, "/object_detections", 10)

        self.timer = self.create_timer(self.WAYPOINT_TIMER_PERIOD_S, self.check_and_send_waypoints)

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

        w = msg.pose.pose.orientation.w
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.velocity_settled = velocity_mag < self.VELOCITY_THRESHOLD

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
        self.use_dynamic_waypoints = False
        self.pending_detection_merge = True
        self._rebuild_active_with_detection()

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

    def generate_cinematographic_waypoints(self):
        if self.detected_object is None or self.start_pos is None:
            return

        if self.simulated_detection_active:
            # Keep the relative pattern; translation to world happens at publish time.
            self.dynamic_waypoints = list(self.static_waypoints)
            self.object_detection_time = self.get_clock().now()
            self.publish_simulated_detection()
            return

        obj_depth = float(self.detected_object["depth"])
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

        circle_radius = max(float(self.min_orbit_radius_m), obj_depth * 0.4)

        num_waypoints = 16
        arc_span = 2 * math.pi / 3
        start_angle = -arc_span / 2

        cinematographic_waypoints = []
        circle_center_abs = None

        for i in range(num_waypoints):
            angle = start_angle + (arc_span * i / (num_waypoints - 1))
            dx_circle = circle_radius * math.cos(angle)
            dy_circle = circle_radius * math.sin(angle)

            thirds_offset_x_m = 0.0
            thirds_offset_y_m = 0.0
            if norm_x is not None and norm_y is not None:
                thirds_offset_x_m = (float(norm_x) - 0.5) * 0.15
                thirds_offset_y_m = (float(norm_y) - 0.5) * 0.15

            wp_x_m = obj_x_m + dx_circle + thirds_offset_x_m
            wp_y_m = obj_y_m + dy_circle + thirds_offset_y_m

            base_pos = self.current_pos if self.current_pos is not None else self.start_pos
            if base_pos is None:
                continue

            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            world_dx = cos_yaw * wp_x_m - sin_yaw * wp_y_m
            world_dy = sin_yaw * wp_x_m + cos_yaw * wp_y_m

            wp_z_m = self.orbit_altitude_m + 0.6 * math.sin(angle)
            wp_z_m = max(self.min_altitude_m, min(self.max_altitude_m, wp_z_m))
            abs_x = base_pos[0] + world_dx
            abs_y = base_pos[1] + world_dy
            abs_z = wp_z_m

            if circle_center_abs is None:
                circle_center_abs = (
                    base_pos[0] + cos_yaw * (obj_x_m + thirds_offset_x_m) - sin_yaw * (obj_y_m + thirds_offset_y_m),
                    base_pos[1] + sin_yaw * (obj_x_m + thirds_offset_x_m) + cos_yaw * (obj_y_m + thirds_offset_y_m),
                    wp_z_m,
                )

            cinematographic_waypoints.append(self._clamp_waypoint_to_bounds((abs_x, abs_y, abs_z)))

        transition = []
        if (
            self.current_pos is not None
            and circle_center_abs is not None
            and len(cinematographic_waypoints) >= 2
            and self.bezier_transition_points > 0
            and self.bezier_blend_length_m > 0.0
        ):
            p0 = self.current_pos
            p3 = cinematographic_waypoints[0]
            p_next = cinematographic_waypoints[1]

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
            cinematographic_waypoints = transition + cinematographic_waypoints

        # Store as relative to the recorded origin so publishing always does origin + relative.
        origin = self.origin_offset if self.origin_offset is not None else (0.0, 0.0, 0.0)
        self.dynamic_waypoints = [
            (wp[0] - origin[0], wp[1] - origin[1], wp[2] - origin[2]) for wp in cinematographic_waypoints
        ]

    def _clamp_waypoint_to_bounds(self, wp):
        x, y, z = wp
        # Shift XY bounds by the recorded origin so relative goals are not clipped back to the default box.
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

    def check_and_send_waypoints(self):
        if self.start_pos is not None and self.waypoints_sent:
            return

        if self.start_pos is not None and self.ready_time is not None:
            elapsed = (self.get_clock().now() - self.ready_time).nanoseconds / 1e9
            if elapsed >= self.STARTUP_DELAY or self.velocity_settled:
                if self.active_waypoints is None:
                    self._initialize_waypoint_sequence()
                self._step_waypoint_sequence()

    def _initialize_waypoint_sequence(self):
        if self.use_dynamic_waypoints and self.dynamic_waypoints is not None:
            waypoints_to_send = self.dynamic_waypoints
        else:
            waypoints_to_send = list(self.static_waypoints)

        if self.waypoint_xy_scale != 1.0 or self.waypoint_y_extra_scale != 1.0:
            sx = self.waypoint_xy_scale
            sy = self.waypoint_xy_scale * self.waypoint_y_extra_scale
            waypoints_to_send = [(wp[0] * sx, wp[1] * sy, wp[2]) for wp in waypoints_to_send]

        if self.pending_detection_merge and self.detected_object is not None:
            waypoints_to_send = self._merge_framing_with_static(waypoints_to_send, self.detected_object)
            self.pending_detection_merge = False
        elif self.simulate_fake_object:
            waypoints_to_send = self._insert_fake_object_orbit(waypoints_to_send)

        waypoints_to_send = self._deduplicate_waypoints(waypoints_to_send, min_separation=0.3)
        waypoints_to_send = self._apply_descent_tail(waypoints_to_send)

        self.active_waypoints = self._limit_consecutive_waypoint_jumps(waypoints_to_send)
        self.current_target_index = 0
        self.last_published_index = -1
        self.last_progress_time = None
        self.last_progress_dist = None
        self.waypoints_sent = False
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
        return center_world, obj_depth

    def _build_orbit_segment(self, center_world, radius, z, num_pts=16):
        radius = max(self.min_orbit_radius_m, radius)
        z = max(self.min_altitude_m, min(self.max_altitude_m, z))
        waypoints = []
        for i in range(num_pts):
            t = 2.0 * math.pi * i / num_pts
            x = center_world[0] + radius * math.cos(t)
            y = center_world[1] + radius * math.sin(t)
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

        idx_anchor = max(range(len(wps_world)), key=lambda i: wps_world[i][1])
        anchor_wp = wps_world[idx_anchor]
        orbit_radius = max(self.min_orbit_radius_m, obj_depth * 0.4)
        orbit = self._build_orbit_segment(center_world, orbit_radius, anchor_wp[2])

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

    def _publish_waypoint(self, target_world, idx: int):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "world"
        goal.pose.position.x = float(target_world[0])
        goal.pose.position.y = float(target_world[1])
        goal.pose.position.z = float(target_world[2])
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        if idx == self.current_target_index:
            self.last_progress_time = self.get_clock().now()
            self.last_progress_dist = self._distance_to(target_world)
            self.stuck_attempts = 0

    def _prefill_waypoint_queue(self):
        if self.active_waypoints is None or self.current_pos is None:
            return

        while (
            self.last_published_index < len(self.active_waypoints) - 1
            and (self.last_published_index - self.current_target_index + 1) < self.WAYPOINT_LOOKAHEAD
        ):
            next_idx = self.last_published_index + 1
            target_world = self._rel_to_world(self.active_waypoints[next_idx])
            self._publish_waypoint(target_world, next_idx)
            self.last_published_index = next_idx

    def _step_waypoint_sequence(self):
        if self.active_waypoints is None or self.current_pos is None:
            return

        if self.current_target_index >= len(self.active_waypoints):
            self.waypoints_sent = True
            self.timer.cancel()
            return

        self._prefill_waypoint_queue()

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

        if dist <= self.DISTANCE_THRESHOLD_M:
            self.current_target_index += 1
            self.last_progress_time = None
            self.last_progress_dist = None
            self.stuck_attempts = 0
            self._prefill_waypoint_queue()
            if self.current_target_index >= len(self.active_waypoints):
                self.waypoints_sent = True
                self.timer.cancel()

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
        orbit = self._build_fake_object_waypoints(obj_world, self.fake_object_radius_m, anchor_wp[2])
        return waypoints[: idx_max_y + 1] + orbit + waypoints[idx_max_y + 1 :]

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
