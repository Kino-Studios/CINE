#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class CinematicWaypointGenerator(Node):
    def __init__(self):
        super().__init__("cinematic_waypoint_generator")

        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.odom_sub = self.create_subscription(Odometry, "/drone_0_odometry", self.odom_callback, 10)
        self.visual_odom_sub = self.create_subscription(Odometry, "/drone_0_visual_odom", self.odom_callback, 10)
        self.detection_sub = self.create_subscription(Float32MultiArray, "/object_detections", self.detection_callback, 10)

        self.start_pos = None
        self.waypoints_sent = False
        self.ready_time = None
        self.STARTUP_DELAY = 5.0

        self.velocity_settled = False
        self.VELOCITY_THRESHOLD = 0.01
        self.current_vel = (0.0, 0.0, 0.0)
        self.current_pos = None

        self.object_detection_time = None
        self.DETECTION_DEBOUNCE = 2.0
        self.DISTANCE_THRESHOLD_M = 0.5

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
        self.declare_parameter("max_waypoint_jump_m", 1.0)
        self.declare_parameter("bezier_blend_length_m", 0.75)
        self.declare_parameter("bezier_transition_points", 6)
        self.declare_parameter("bezier_min_speed_mps", 0.05)
        self.declare_parameter("world_bounds_min", [-10.0, -10.0, 0.0])
        self.declare_parameter("world_bounds_max", [10.0, 10.0, 5.0])

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
        self.world_bounds_min = [float(v) for v in self.get_parameter("world_bounds_min").value]
        self.world_bounds_max = [float(v) for v in self.get_parameter("world_bounds_max").value]

        self.static_waypoints = [
            (0.1, 0.3, 0.3),
            (0.15, 0.8, 0.32),
            (0.2, 1.3, 0.35),
            (0.25, 1.8, 0.38),
            (0.3, 2.3, 0.42),
            (0.35, 2.8, 0.48),
            (0.4, 3.2, 0.58),
            (0.45, 3.5, 0.7),
            (0.5, 3.6, 0.8),
            (0.5, 3.5, 0.82),
            (0.48, 3.2, 0.78),
            (0.45, 2.8, 0.7),
            (0.4, 2.3, 0.58),
            (0.3, 1.8, 0.48),
            (0.2, 1.2, 0.38),
            (0.05, 0.6, 0.32),
            (-0.1, 0.0, 0.3),
            (-0.25, -0.6, 0.3),
            (-0.35, -1.2, 0.32),
            (-0.4, -1.8, 0.38),
            (-0.42, -2.3, 0.48),
            (-0.4, -2.8, 0.58),
            (-0.35, -3.2, 0.7),
            (-0.25, -3.5, 0.8),
            (-0.2, -3.6, 0.82),
            (-0.15, -3.5, 0.78),
            (-0.1, -3.2, 0.7),
            (-0.05, -2.8, 0.58),
            (0.05, -2.3, 0.48),
            (0.15, -1.8, 0.38),
            (0.25, -1.2, 0.32),
            (0.3, -0.6, 0.3),
        ]

        self.dynamic_waypoints = None
        self.detected_object = None
        self.use_dynamic_waypoints = False
        self.active_waypoints = None
        self.active_waypoint_index = 0
        self.active_waypoint_sent = False

        self.fake_detection_pub = self.create_publisher(Float32MultiArray, "/object_detections", 10)

        self.timer = self.create_timer(0.5, self.check_and_send_waypoints)

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
            self.ready_time = self.get_clock().now()

        self.velocity_settled = velocity_mag < self.VELOCITY_THRESHOLD

    def detection_callback(self, msg: Float32MultiArray):
        try:
            if self.active_waypoints is not None:
                return

            det = self._parse_detection(msg)
            if det is None:
                return

            self.detected_object = det

            if self.start_pos is not None and not self.waypoints_sent:
                now = self.get_clock().now()
                if self.object_detection_time is None:
                    self.generate_cinematographic_waypoints()
                    self.use_dynamic_waypoints = True
                    self.object_detection_time = now
                else:
                    elapsed = (now - self.object_detection_time).nanoseconds / 1e9
                    if elapsed >= self.DETECTION_DEBOUNCE:
                        self.generate_cinematographic_waypoints()
                        self.object_detection_time = now
        except Exception as e:
            self.get_logger().warn(f"Error processing: {e}")

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

            wp_z_m = max(self.min_altitude_m, min(self.max_altitude_m, self.orbit_altitude_m))
            abs_x = self.start_pos[0] + wp_x_m
            abs_y = self.start_pos[1] + wp_y_m
            abs_z = wp_z_m

            if circle_center_abs is None:
                circle_center_abs = (
                    self.start_pos[0] + obj_x_m + thirds_offset_x_m,
                    self.start_pos[1] + obj_y_m + thirds_offset_y_m,
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

        self.dynamic_waypoints = cinematographic_waypoints
        self.publish_simulated_detection()

    def _clamp_waypoint_to_bounds(self, wp):
        x, y, z = wp
        x = max(self.world_bounds_min[0], min(self.world_bounds_max[0], float(x)))
        y = max(self.world_bounds_min[1], min(self.world_bounds_max[1], float(y)))
        z = max(self.world_bounds_min[2], min(self.world_bounds_max[2], float(z)))
        z = max(self.min_altitude_m, min(self.max_altitude_m, z))
        return (x, y, z)

    def publish_simulated_detection(self):
        fake_detection = Float32MultiArray()
        fake_detection.data = [1.0, 320.0, 240.0, 1.5, 5000.0, 0.05, 0.9, 0.5, 0.5]
        self.fake_detection_pub.publish(fake_detection)

    def check_and_send_waypoints(self):
        if self.start_pos is not None and self.waypoints_sent:
            return

        if self.start_pos is not None and self.velocity_settled and self.ready_time is not None:
            elapsed = (self.get_clock().now() - self.ready_time).nanoseconds / 1e9
            if elapsed >= self.STARTUP_DELAY:
                if self.active_waypoints is None:
                    self._initialize_waypoint_sequence()
                self._step_waypoint_sequence()

    def _initialize_waypoint_sequence(self):
        if self.start_pos is not None and self.detected_object is None:
            fake_detection = Float32MultiArray()
            fake_detection.data = [1.0, 320.0, 240.0, 1.5, 5000.0, 0.05, 0.9, 0.5, 0.5]
            self.detected_object = self._parse_detection(fake_detection)
            self.generate_cinematographic_waypoints()
            self.use_dynamic_waypoints = True

        if self.use_dynamic_waypoints and self.dynamic_waypoints is not None:
            waypoints_to_send = self.dynamic_waypoints
        else:
            waypoints_to_send = [
                (self.start_pos[0] + dx, self.start_pos[1] + dy, self.start_pos[2] + dz) for dx, dy, dz in self.static_waypoints
            ]

        clamped = [self._clamp_waypoint_to_bounds(wp) for wp in waypoints_to_send]
        self.active_waypoints = self._limit_consecutive_waypoint_jumps(clamped)
        self.active_waypoint_index = 0
        self.active_waypoint_sent = False

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

    def _step_waypoint_sequence(self):
        if self.active_waypoints is None or self.current_pos is None:
            return

        if self.active_waypoint_index >= len(self.active_waypoints):
            self.waypoints_sent = True
            self.timer.cancel()
            return

        target = self.active_waypoints[self.active_waypoint_index]
        dx = self.current_pos[0] - target[0]
        dy = self.current_pos[1] - target[1]
        dz = self.current_pos[2] - target[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5

        if not self.active_waypoint_sent:
            x, y, z = target
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "world"
            goal.pose.position.x = float(x)
            goal.pose.position.y = float(y)
            goal.pose.position.z = float(z)
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.active_waypoint_sent = True
            return

        if dist <= self.DISTANCE_THRESHOLD_M:
            self.active_waypoint_index += 1
            self.active_waypoint_sent = False


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

