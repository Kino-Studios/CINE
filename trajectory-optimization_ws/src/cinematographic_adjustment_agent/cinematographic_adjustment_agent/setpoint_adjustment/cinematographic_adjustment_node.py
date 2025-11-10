import sys
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped
from cv_bridge import CvBridge
from typing import Optional, Tuple, List

workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))))
config_path = os.path.join(workspace_root, 'config')
if config_path not in sys.path:
    sys.path.insert(0, config_path)

from ..config.planner_params import CinematographyParams
from ..cost_functions.cost_calculator import CostCalculator
from .state_tracker import StateTracker
from .trajectory_planner import (
    generate_orbit_waypoints_xy,
    pixel_to_camera_xy,
    clamp_depth_to_preference,
)
from .waypoint_publisher import WaypointPublisher


class CinematographicPlannerNode(Node):
    def __init__(self):
        super().__init__("cinematographic_planner_node")
        self.params = CinematographyParams()
        self.bridge = CvBridge()
        self.tracker = StateTracker(look_duration_s=self.params.look_duration_s)
        self.depth_image = None
        self.center_uv = None
        self.bounding_box = None
        
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))))
        constants_path = os.path.join(workspace_root, 'configs', 'user_params')
        self.cost_calculator = CostCalculator(self, constants_path=constants_path)

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

        self.center_sub = self.create_subscription(
            PolygonStamped,
            "/object_detection/center_point",
            self.center_callback,
            10,
        )
        self.depth_sub = self.create_subscription(
            Image,
            "/depth/image_raw",
            self.depth_callback,
            10,
        )
        self.bounding_box_sub = self.create_subscription(
            PolygonStamped,
            "/object_detection/bounding_box",
            self.bounding_box_callback,
            10,
        )
        self.dimensions_sub = self.create_subscription(
            PolygonStamped,
            "/object_detection/dimensions",
            self.dimensions_callback,
            10,
        )

        self.waypoint_pub_node = WaypointPublisher(self.params.waypoints_topic)

        self.timer = self.create_timer(1.0 / max(1.0, self.params.waypoint_rate_hz), self.tick)

    def center_callback(self, msg: PolygonStamped):
        if len(msg.polygon.points) == 0:
            return
        u = int(msg.polygon.points[0].x)
        v = int(msg.polygon.points[0].y)
        self.center_uv = (u, v)
        self.tracker.set_target_observation((u, v))
        self.cost_calculator.update_center_point(msg)

    def bounding_box_callback(self, msg: PolygonStamped):
        self.bounding_box = msg
        self.cost_calculator.update_bounding_box(msg)

    def dimensions_callback(self, msg: PolygonStamped):
        self.cost_calculator.update_dimensions(msg)

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
        except Exception:
            pass

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

    def build_initial_waypoints(self) -> List[Tuple[float, float, float, float]]:
        target = self.compute_target_xy()
        if target is None:
            return []
        tx, ty, _ = target
        orbit = generate_orbit_waypoints_xy(
            center_xy_m=(tx, ty),
            radius_m=self.params.orbit_radius_m,
            angular_velocity_rps=self.params.angular_velocity_rps,
            duration_s=float(self.params.num_initial_waypoints) / self.params.waypoint_rate_hz,
            waypoint_rate_hz=self.params.waypoint_rate_hz,
        )
        z = self.params.constant_altitude_m
        xyz_time = []
        for x, y, dt in orbit:
            xyz_time.append((x, y, z, dt))
        return xyz_time

    def tick(self):
        if self.tracker.should_switch_target():
            self.tracker.rotate_to_next_target()
        
        cost = self.cost_calculator.calculate_cost()
        if cost is not None:
            self.get_logger().debug(f'Current cost: {cost:.3f}')
        
        waypoints = self.build_initial_waypoints()
        if len(waypoints) == 0:
            return
        self.waypoint_pub_node.publish_waypoints_xyz(waypoints)


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

