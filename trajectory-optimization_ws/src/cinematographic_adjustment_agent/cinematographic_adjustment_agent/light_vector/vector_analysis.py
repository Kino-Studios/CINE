import sys
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from typing import Optional, Tuple, List

workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))))
config_path = os.path.join(workspace_root, 'config')
if config_path not in sys.path:
    sys.path.insert(0, config_path)

from ...config.planner_params import CinematographyParams

class LightVectorAnalysisNode(Node):
    def __init__(self):
        super().__init__("cinematographic_planner_node")
        self.params = CinematographyParams()
        self.bridge = CvBridge()
        self.center_uv = None
        
        self.drone_position = np.array([0.0, 0.0, 0.0])
        self.drone_orientation = np.array([0.0, 0.0, 0.0, 0.0])
        self.has_odom = False
        
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))))
        constants_path = os.path.join(workspace_root, 'configs', 'user_params')

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
        self.params.fx = float(self.get_parameter("camera_fx").value)
        self.params.fy = float(self.get_parameter("camera_fy").value)
        self.params.cx = float(self.get_parameter("camera_cx").value)
        self.params.cy = float(self.get_parameter("camera_cy").value)
        self.params.waypoints_topic = str(self.get_parameter("waypoints_topic").value)

        self.image_sub = self.create_subscription(
            Image,
            "/camera_publisher/image_raw",
            self.image_callback,
            20
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odometry",
            self.odom_callback,
            10,
        )

        self.vector_publisher = self.create_publisher(
            Vector3Stamped,
            "/cinematography/light_vector",
            10
        )
        
        self.use_cost_optimization = True
        self.optimization_frequency = 2.0 
        self.last_optimization_time = 0.0
        self.optimized_center = None
        self.optimized_radius = None

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

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            return

        if cv_image.ndim == 3 and cv_image.shape[2] == 3:
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = cv_image.copy()

        blurred = cv2.GaussianBlur(gray, (10, 10), 0)
        flat_index = int(np.argmax(blurred))
        rows, cols = blurred.shape
        brightest_row, brightest_col = np.unravel_index(flat_index, (rows, cols))

        self.center_uv = ([brightest_col, brightest_row])
        self.publish_vectors()

    def publish_vectors(self):
        if self.center_uv is None or not self.has_odom:
            return
        
        camera_coords = np.array([
            (self.center_uv[0] - self.params.cx) / self.params.fx,
            (self.center_uv[1] - self.params.cy) / self.params.fy,
            1.0
        ])
        
        vector_normalized = camera_coords / np.linalg.norm(camera_coords)
        
        odom_rotation_matrix = Rotation.from_quat(self.drone_orientation).as_matrix()
        world_light_vector = odom_rotation_matrix @ vector_normalized
        
        vector_msg = Vector3Stamped()
        vector_msg.header.stamp = self.get_clock().now().to_msg()
        vector_msg.header.frame_id = "light_vector"
        vector_msg.vector.x = float(world_light_vector[0])
        vector_msg.vector.y = float(world_light_vector[1])
        vector_msg.vector.z = float(world_light_vector[2])
        
        self.vector_publisher.publish(vector_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LightVectorAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()