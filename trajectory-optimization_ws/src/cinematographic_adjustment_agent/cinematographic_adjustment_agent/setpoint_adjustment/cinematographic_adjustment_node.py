import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PolygonStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from cinematography_msgs.msg import CinematographyCosts

from ..config.planner_params import CinematographyParams
from ..cost_functions.cost_calculator import CostCalculator
from .state_tracker import StateTracker
from .trajectory_planner import (
    generate_orbit_waypoints_xy,
    generate_cost_optimized_orbit_waypoints,
    optimize_orbit_center_and_radius,
    pixel_to_camera_xy,
    clamp_depth_to_preference,
)
from ..setpoint_sender.waypoint_sender import WaypointPublisher
from ..config.constraints import CinematicConstraints

class CinematographicPlannerNode(Node):
    def __init__(self):
        super().__init__("cinematographic_planner_node")
        self.params = CinematographyParams.from_env()
        self.constraints = CinematicConstraints()
        self.bridge = CvBridge()
        self.depth_image = None
        self.center_uv = None
        self.bounding_box = None
        self.has_focus = False
        
        self.drone_position = np.array([0.0, 0.0, 0.0])
        self.drone_orientation = np.array([0.0, 0.0, 0.0, 0.0])
        self.has_odom = False
        
        self.cost_calculator = CostCalculator(self)

        self.declare_parameter("target_focus_distance_m", self.params.target_focus_distance_m)
        self.declare_parameter("min_focus_distance_m", self.params.min_focus_distance_m)
        self.declare_parameter("constant_altitude_m", self.params.constant_altitude_m)
        self.declare_parameter("orbit_radius_m", self.params.orbit_radius_m)
        self.declare_parameter("angular_velocity_rps", self.params.angular_velocity_rps)
        self.declare_parameter("look_duration_s", self.params.look_duration_s)
        self.declare_parameter("waypoint_rate_hz", self.params.waypoint_rate_hz)
        self.declare_parameter("num_initial_waypoints", self.params.num_initial_waypoints)
        self.declare_parameter("image_width", self.params.image_width)
        self.declare_parameter("image_height", self.params.image_height)
        self.declare_parameter("camera_fx", self.params.camera_fx)
        self.declare_parameter("camera_fy", self.params.camera_fy)
        self.declare_parameter("camera_cx", self.params.camera_cx)
        self.declare_parameter("camera_cy", self.params.camera_cy)
        self.declare_parameter("waypoints_topic", self.params.waypoints_topic)
        self.declare_parameter("odom_topic", self.params.odom_topic)
        self.declare_parameter("depth_topic", self.params.depth_topic)
        self.declare_parameter("center_topic", self.params.center_topic)
        self.declare_parameter("bbox_topic", self.params.bbox_topic)
        self.declare_parameter("dimensions_topic", self.params.dimensions_topic)
        self.declare_parameter("light_vector_topic", self.params.light_vector_topic)
        self.declare_parameter("thirds_strength", 0.6)
        self.declare_parameter("thirds_use_nearest_anchor", True)
        self.declare_parameter("candidate_angle_window_rad", 0.6)
        self.declare_parameter("candidate_angle_samples", 7)
        self.declare_parameter("candidate_radius_samples", 5)
        self.declare_parameter("candidate_radius_range_m", [0.8, 2.0])
        self.declare_parameter("max_waypoint_step_m", 0.5)

        self.params.target_focus_distance_m = float(self.get_parameter("target_focus_distance_m").value)
        self.params.min_focus_distance_m = float(self.get_parameter("min_focus_distance_m").value)
        self.params.constant_altitude_m = float(self.get_parameter("constant_altitude_m").value)
        self.params.orbit_radius_m = float(self.get_parameter("orbit_radius_m").value)
        self.params.angular_velocity_rps = float(self.get_parameter("angular_velocity_rps").value)
        self.params.look_duration_s = float(self.get_parameter("look_duration_s").value)
        self.params.waypoint_rate_hz = float(self.get_parameter("waypoint_rate_hz").value)
        self.params.num_initial_waypoints = int(self.get_parameter("num_initial_waypoints").value)
        self.params.image_width = int(self.get_parameter("image_width").value)
        self.params.image_height = int(self.get_parameter("image_height").value)
        self.params.camera_fx = float(self.get_parameter("camera_fx").value)
        self.params.camera_fy = float(self.get_parameter("camera_fy").value)
        self.params.camera_cx = float(self.get_parameter("camera_cx").value)
        self.params.camera_cy = float(self.get_parameter("camera_cy").value)
        self.params.waypoints_topic = str(self.get_parameter("waypoints_topic").value)
        self.params.odom_topic = str(self.get_parameter("odom_topic").value)
        self.params.depth_topic = str(self.get_parameter("depth_topic").value)
        self.params.center_topic = str(self.get_parameter("center_topic").value)
        self.params.bbox_topic = str(self.get_parameter("bbox_topic").value)
        self.params.dimensions_topic = str(self.get_parameter("dimensions_topic").value)
        self.params.light_vector_topic = str(self.get_parameter("light_vector_topic").value)
        self.thirds_strength = float(self.get_parameter("thirds_strength").value)
        self.thirds_use_nearest_anchor = bool(self.get_parameter("thirds_use_nearest_anchor").value)
        self.candidate_angle_window_rad = float(self.get_parameter("candidate_angle_window_rad").value)
        self.candidate_angle_samples = int(self.get_parameter("candidate_angle_samples").value)
        self.candidate_radius_samples = int(self.get_parameter("candidate_radius_samples").value)
        self.candidate_radius_range_m = [float(v) for v in self.get_parameter("candidate_radius_range_m").value]
        self.max_waypoint_step_m = float(self.get_parameter("max_waypoint_step_m").value)

        self.tracker = StateTracker(
            look_duration_s=self.params.look_duration_s,
            depth_window_size=15
        )

        self.center_sub = self.create_subscription(
            PolygonStamped,
            self.params.center_topic,
            self.center_callback,
            10,
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.params.depth_topic,
            self.depth_callback,
            10,
        )

        self.bounding_box_sub = self.create_subscription(
            PolygonStamped,
            self.params.bbox_topic,
            self.bounding_box_callback,
            10,
        )

        self.dimensions_sub = self.create_subscription(
            PolygonStamped,
            self.params.dimensions_topic,
            self.dimensions_callback,
            10,
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.params.odom_topic,
            self.odom_callback,
            10,
        )

        self.light_vector_sub = self.create_subscription(
            Vector3Stamped,
            self.params.light_vector_topic,
            self.light_vector_callback,
            10,
        )

        self.cost_pub = self.create_publisher(
            CinematographyCosts,
            "/cinematography/cinematography_costs",
            10,
        )

        self.waypoint_pub_node = WaypointPublisher(self, self.params.waypoints_topic)
        
        self.use_cost_optimization = True
        self.optimization_frequency = 2.0 
        self.last_optimization_time = 0.0
        self.optimized_center = None
        self.optimized_radius = None

        self.timer = self.create_timer(1.0 / max(1.0, self.params.waypoint_rate_hz), self.tick)
        self.phase_theta = 0.0
        self.last_tick_time_s = None
        self.smooth_pos = None
        self.smooth_vel = np.zeros(3, dtype=float)

    def center_callback(self, msg: PolygonStamped):
        if len(msg.polygon.points) == 0:
            return
        u = int(msg.polygon.points[0].x)
        v = int(msg.polygon.points[0].y)
        adjusted = self._apply_rule_of_thirds((u, v))
        self.center_uv = adjusted
        self.tracker.set_target_observation(adjusted)
        self.cost_calculator.update_center_point(msg)

    def _apply_rule_of_thirds(self, uv: Tuple[int, int]) -> Tuple[int, int]:
        strength = float(max(0.0, min(1.0, self.thirds_strength)))
        if strength <= 0.0:
            return uv

        u, v = int(uv[0]), int(uv[1])
        w = int(max(1, self.params.image_width))
        h = int(max(1, self.params.image_height))

        anchors = [
            (w / 3.0, h / 3.0),
            (2.0 * w / 3.0, h / 3.0),
            (w / 3.0, 2.0 * h / 3.0),
            (2.0 * w / 3.0, 2.0 * h / 3.0),
        ]

        if self.thirds_use_nearest_anchor:
            ax, ay = min(anchors, key=lambda a: (a[0] - u) ** 2 + (a[1] - v) ** 2)
        else:
            ax, ay = anchors[1]

        uu = int(round((1.0 - strength) * u + strength * ax))
        vv = int(round((1.0 - strength) * v + strength * ay))
        uu = max(0, min(uu, w - 1))
        vv = max(0, min(vv, h - 1))
        return (uu, vv)

    def bounding_box_callback(self, msg: PolygonStamped):
        self.bounding_box = msg
        self.has_focus = len(msg.polygon.points) > 0
        self.cost_calculator.update_bounding_box(msg)

    def dimensions_callback(self, msg: PolygonStamped):
        self.cost_calculator.update_dimensions(msg)

    def odom_callback(self, msg: Odometry):
        self.drone_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        self.drone_orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        self.has_odom = True
        self.cost_calculator.update_odometry(msg)

    def light_vector_callback(self, msg: Vector3Stamped):
        self.cost_calculator.update_light_vector(msg)
    
    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.depth_image = depth
            self.cost_calculator.update_depth_image(depth)

            if self.center_uv is not None:
                u, v = self.center_uv
                h, w = depth.shape[:2]
                uu = max(0, min(u, w - 1))
                vv = max(0, min(v, h - 1))
                z = float(depth[vv, uu])
                if z > 0 and not np.isnan(z):
                    self.tracker.record_depth(z)
        except Exception as e:
            self.get_logger().warn(f'Depth callback error: {e}')

    def _dt(self) -> float:
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.last_tick_time_s is None:
            self.last_tick_time_s = now_s
            return 1.0 / max(1.0, self.params.waypoint_rate_hz)
        dt = now_s - self.last_tick_time_s
        self.last_tick_time_s = now_s
        return float(max(1e-3, min(dt, 0.5)))

    def _next_kinematic_step(self, target_pos: np.ndarray, dt: float) -> np.ndarray:
        if self.smooth_pos is None:
            self.smooth_pos = self.drone_position.copy()
            self.smooth_vel = np.zeros(3, dtype=float)

        max_vel = self.constraints.FOCUS_MAX_VELOCITY_MPS if self.has_focus else self.constraints.DEFAULT_MAX_VELOCITY_MPS
        max_acc = self.constraints.FOCUS_MAX_ACCEL_MPS2 if self.has_focus else self.constraints.DEFAULT_MAX_ACCEL_MPS2

        disp = target_pos - self.smooth_pos
        dist = float(np.linalg.norm(disp))
        if dist < 1e-6:
            self.smooth_vel = np.zeros(3, dtype=float)
            return self.smooth_pos.copy()

        direction = disp / dist
        desired_speed = min(float(max_vel), math.sqrt(max(0.0, 2.0 * float(max_acc) * dist)))
        desired_vel = direction * desired_speed

        vel_error = desired_vel - self.smooth_vel
        vel_error_mag = float(np.linalg.norm(vel_error))
        if vel_error_mag > 1e-6:
            max_vel_change = float(max_acc) * dt
            if vel_error_mag <= max_vel_change:
                accel = vel_error / dt
            else:
                accel = (vel_error / vel_error_mag) * float(max_acc)
        else:
            accel = np.zeros(3, dtype=float)

        next_vel = self.smooth_vel + accel * dt
        vel_mag = float(np.linalg.norm(next_vel))
        if vel_mag > float(max_vel) and vel_mag > 0.0:
            next_vel = (next_vel / vel_mag) * float(max_vel)

        next_pos = self.smooth_pos + self.smooth_vel * dt + 0.5 * accel * dt * dt

        step = next_pos - self.smooth_pos
        step_mag = float(np.linalg.norm(step))
        if self.max_waypoint_step_m > 0.0 and step_mag > self.max_waypoint_step_m and step_mag > 0.0:
            next_pos = self.smooth_pos + (step / step_mag) * float(self.max_waypoint_step_m)
            next_vel = (next_pos - self.smooth_pos) / dt

        if float(np.linalg.norm(next_pos - target_pos)) < 0.05:
            next_pos = target_pos.copy()
            next_vel = np.zeros(3, dtype=float)

        self.smooth_pos = next_pos
        self.smooth_vel = next_vel
        return next_pos.copy()

    def _select_next_waypoint(self, center_xy: Tuple[float, float], radius_m: float, altitude_m: float, target_world: np.ndarray, dt: float) -> Optional[np.ndarray]:
        cx, cy = center_xy
        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])

        def cost_function(x: float, y: float, z: float) -> Optional[float]:
            return self.cost_calculator.estimate_cost_at_position(x, y, z, tx, ty, tz)

        self.phase_theta += 2.0 * math.pi * float(self.params.angular_velocity_rps) * dt

        angle_samples = max(1, self.candidate_angle_samples)
        window = float(max(0.0, self.candidate_angle_window_rad))
        angle_offsets = np.linspace(-window, window, angle_samples) if angle_samples > 1 else np.array([0.0])

        r_min, r_max = float(self.candidate_radius_range_m[0]), float(self.candidate_radius_range_m[1])
        r_min, r_max = (min(r_min, r_max), max(r_min, r_max))
        base_r = float(radius_m)
        r_low = max(0.1, min(base_r, base_r * 1.5, r_max))
        r_high = min(max(base_r, base_r * 0.7, r_min), r_max)
        if r_high < r_low:
            r_low, r_high = r_min, r_max

        radius_samples = max(1, self.candidate_radius_samples)
        radii = np.linspace(r_low, r_high, radius_samples) if radius_samples > 1 else np.array([base_r])

        best = None
        best_cost = float("inf")

        for da in angle_offsets:
            theta = self.phase_theta + float(da)
            ct, st = math.cos(theta), math.sin(theta)
            for rr in radii:
                x = float(cx + rr * ct)
                y = float(cy + rr * st)
                z = float(altitude_m)
                cost = cost_function(x, y, z)
                if cost is None or not math.isfinite(cost):
                    continue
                if cost < best_cost:
                    best_cost = float(cost)
                    best = np.array([x, y, z], dtype=float)

        return best

    def compute_target_xy(self) -> Optional[Tuple[float, float, float]]:
        if self.center_uv is None:
            return None
        
        median_depth = self.cost_calculator.get_median_depth_roi()

        if median_depth is not None:
            smoothed_depth = median_depth
        else:
            smoothed_depth = self.tracker.get_smoothed_depth()
        
        preferred_depth = clamp_depth_to_preference(
            smoothed_depth,
            self.params.target_focus_distance_m,
            self.params.min_focus_distance_m,
        )

        u, v = self.center_uv
        x_cam, y_cam = pixel_to_camera_xy(
            u,
            v,
            preferred_depth,
            self.params.camera_fx,
            self.params.camera_fy,
            self.params.camera_cx,
            self.params.camera_cy,
        )
        
        return x_cam, y_cam, preferred_depth
    
    def compute_target_world_position(self) -> Optional[np.ndarray]:
        target_cam = self.compute_target_xy()

        if target_cam is None or not self.has_odom:
            return None
        
        x_cam, y_cam, z_cam = target_cam
        
        target_world = self.drone_position + np.array([z_cam, -x_cam, -y_cam])
        
        return target_world

    def build_initial_waypoints(self) -> List[Tuple[float, float, float, float]]:
        target_world = self.compute_target_world_position()

        if target_world is None:
            return []
        
        tx, ty, tz = target_world[0], target_world[1], target_world[2]
        
        if self.optimized_center is not None and self.optimized_radius is not None:
            center_xy = self.optimized_center
            radius = self.optimized_radius
        else:
            center_xy = (tx, ty)
            radius = self.params.orbit_radius_m
        
        def cost_function(x: float, y: float, z: float) -> Optional[float]:
            return self.cost_calculator.estimate_cost_at_position(
                x, y, z, tx, ty, tz
            )
        
        z = self.params.constant_altitude_m
        
        if self.use_cost_optimization:
            try:
                orbit = generate_cost_optimized_orbit_waypoints(
                    center_xy_m=center_xy,
                    initial_radius_m=radius,
                    angular_velocity_rps=self.params.angular_velocity_rps,
                    duration_s=float(self.params.num_initial_waypoints) / self.params.waypoint_rate_hz,
                    waypoint_rate_hz=self.params.waypoint_rate_hz,
                    altitude_m=z,
                    cost_function=cost_function,
                    num_radius_samples=5,
                    radius_range=(0.8, 2.0),
                )
                return orbit
            except Exception as e:
                self.get_logger().warn(f'Optimization cost failed: {e}')
        
        orbit = generate_orbit_waypoints_xy(
            center_xy_m=center_xy,
            radius_m=radius,
            angular_velocity_rps=self.params.angular_velocity_rps,
            duration_s=float(self.params.num_initial_waypoints) / self.params.waypoint_rate_hz,
            waypoint_rate_hz=self.params.waypoint_rate_hz,
            altitude_m=z,
        )
        
        return orbit

    def tick(self):
        dt = self._dt()

        if self.tracker.should_switch_target():
            self.tracker.rotate_to_next_target()
        
        cost_result = self.cost_calculator.calculate_cost()
        
        if cost_result is not None and isinstance(cost_result, dict):
            total_cost = cost_result.get('total_cost', 0.0)
            msg = CinematographyCosts()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.light_cost = float(cost_result.get('lighting_cost', 0.0))
            msg.framing_cost = float(cost_result.get('framing_cost', 0.0))
            msg.smoothness_cost = float(cost_result.get('smoothness_cost', 0.0))
            msg.distance_cost = float(cost_result.get('distance_cost', 0.0))
            msg.total_cost = float(total_cost)
            self.cost_pub.publish(msg)
        
        now = self.get_clock().now()
        current_time = now.nanoseconds / 1e9

        if (self.use_cost_optimization and 
            (current_time - self.last_optimization_time) >= self.optimization_frequency):
            
            target_world = self.compute_target_world_position()

            if target_world is not None:
                tx, ty, tz = target_world[0], target_world[1], target_world[2]
                
                def cost_function(x: float, y: float, z: float) -> Optional[float]:
                    return self.cost_calculator.estimate_cost_at_position(
                        x, y, z, tx, ty, tz
                    )
                
                try:
                    initial_center = (tx, ty)
                    optimized_center, optimized_radius = optimize_orbit_center_and_radius(
                        initial_center_xy=initial_center,
                        initial_radius=self.params.orbit_radius_m,
                        altitude_m=self.params.constant_altitude_m,
                        cost_function=cost_function,
                        search_radius=0.5,
                        num_samples=9,
                    )

                    self.optimized_center = optimized_center
                    self.optimized_radius = optimized_radius
                    self.last_optimization_time = current_time
                    self.get_logger().info(
                        f'Optimized orbit: center=({optimized_center[0]:.2f}, {optimized_center[1]:.2f}), '
                        f'radius={optimized_radius:.2f}m'
                    )
                except Exception as e:
                    self.get_logger().warn(f'Orbit optimization failed: {e}')

        target_world = self.compute_target_world_position()
        if target_world is None:
            return

        tx, ty = float(target_world[0]), float(target_world[1])
        if self.optimized_center is not None and self.optimized_radius is not None:
            center_xy = self.optimized_center
            radius = float(self.optimized_radius)
        else:
            center_xy = (tx, ty)
            radius = float(self.params.orbit_radius_m)

        desired = self._select_next_waypoint(
            center_xy=center_xy,
            radius_m=radius,
            altitude_m=float(self.params.constant_altitude_m),
            target_world=target_world,
            dt=dt,
        )
        if desired is None:
            return

        next_pos = self._next_kinematic_step(desired, dt)
        self.waypoint_pub_node.publish_single_waypoint(float(next_pos[0]), float(next_pos[1]), float(next_pos[2]))


def main(args=None):
    rclpy.init(args=args)
    node = CinematographicPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
