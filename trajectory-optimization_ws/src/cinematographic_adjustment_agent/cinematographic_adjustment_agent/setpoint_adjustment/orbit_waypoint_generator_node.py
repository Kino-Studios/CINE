import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PolygonStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from ..cost_functions.cost_calculator import CostCalculator
from .trajectory_planner import (
    generate_cost_optimized_orbit_waypoints,
    generate_orbit_waypoints_xy,
    optimize_orbit_center_and_radius,
)


class OrbitWaypointGenerator(Node):
    def __init__(self):
        super().__init__("orbit_waypoint_generator")
        self.bridge = CvBridge()
        self.camera_info = None
        self.current_pose = None
        self.depth_image = None
        self.waypoints = []
        self.waypoint_index = 0
        self.publishing_active = False

        self.cost_calculator = CostCalculator(self)

        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/vio/pose", self.pose_callback, 10)
        self.depth_sub = self.create_subscription(Image, "/camera/depth/image_raw", self.depth_callback, 10)
        self.bbox_sub = self.create_subscription(PolygonStamped, "/tracker/bounding_box", self.bbox_callback, 10)
        self.center_sub = self.create_subscription(PolygonStamped, "/tracker/center_point", self.center_callback, 10)
        self.dims_sub = self.create_subscription(PolygonStamped, "/tracker/dimensions", self.dims_callback, 10)

        try:
            from mavros_msgs.msg import PositionTarget

            self._PositionTarget = PositionTarget
            self.setpoint_pub = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
        except Exception:
            self._PositionTarget = None
            self.setpoint_pub = None

        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("radius", 2.0)
        self.declare_parameter("angular_velocity", 0.1)
        self.declare_parameter("duration", 60.0)
        self.declare_parameter("waypoint_rate", 10.0)
        self.declare_parameter("optimize_cost", True)
        self.declare_parameter("altitude", 1.5)

        self.timer = self.create_timer(0.1, self.publish_waypoint)

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg
        if not self.publishing_active and self.camera_info is not None:
            self.generate_waypoints()
            self.publishing_active = True

    def depth_callback(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception:
            return
        self.cost_calculator.update_depth_image(self.depth_image)

    def bbox_callback(self, msg: PolygonStamped):
        self.cost_calculator.update_bounding_box(msg)

    def center_callback(self, msg: PolygonStamped):
        self.cost_calculator.update_center_point(msg)

    def dims_callback(self, msg: PolygonStamped):
        self.cost_calculator.update_dimensions(msg)

    def evaluate_cost_at_position(self, x: float, y: float, z: float) -> Optional[float]:
        if self.current_pose is None:
            return None
        if self.cost_calculator.center_point is None or len(self.cost_calculator.center_point.polygon.points) == 0:
            return None

        target = self.cost_calculator.center_point.polygon.points[0]
        target_world_x = float(getattr(target, "x", 0.0))
        target_world_y = float(getattr(target, "y", 0.0))
        target_world_z = float(getattr(target, "z", z))
        return self.cost_calculator.estimate_cost_at_position(x, y, z, target_world_x, target_world_y, target_world_z)

    def generate_waypoints(self):
        center_x = float(self.get_parameter("center_x").value)
        center_y = float(self.get_parameter("center_y").value)
        radius = float(self.get_parameter("radius").value)
        angular_velocity = float(self.get_parameter("angular_velocity").value)
        duration = float(self.get_parameter("duration").value)
        waypoint_rate = float(self.get_parameter("waypoint_rate").value)
        optimize = bool(self.get_parameter("optimize_cost").value)
        altitude = float(self.get_parameter("altitude").value)

        center = (center_x, center_y)
        if optimize:
            center, radius = optimize_orbit_center_and_radius(center, radius, altitude, self.evaluate_cost_at_position)
            self.waypoints = generate_cost_optimized_orbit_waypoints(
                center_xy_m=center,
                initial_radius_m=radius,
                angular_velocity_rps=angular_velocity,
                duration_s=duration,
                waypoint_rate_hz=waypoint_rate,
                altitude_m=altitude,
                cost_function=self.evaluate_cost_at_position,
            )
        else:
            self.waypoints = generate_orbit_waypoints_xy(center, radius, angular_velocity, duration, waypoint_rate, altitude)

        self.waypoint_index = 0

    def publish_waypoint(self):
        if not self.publishing_active or not self.waypoints:
            return

        if self.setpoint_pub is None or self._PositionTarget is None:
            return

        from std_msgs.msg import Header

        if self.waypoint_index >= len(self.waypoints):
            self.waypoint_index = 0

        x, y, z, dt = self.waypoints[self.waypoint_index]
        msg = self._PositionTarget()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = self._PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            self._PositionTarget.IGNORE_VX
            | self._PositionTarget.IGNORE_VY
            | self._PositionTarget.IGNORE_VZ
            | self._PositionTarget.IGNORE_AFX
            | self._PositionTarget.IGNORE_AFY
            | self._PositionTarget.IGNORE_AFZ
            | self._PositionTarget.IGNORE_YAW_RATE
        )
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.yaw = math.atan2(float(self.get_parameter("center_y").value) - float(y), float(self.get_parameter("center_x").value) - float(x))
        self.setpoint_pub.publish(msg)
        self.waypoint_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = OrbitWaypointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
