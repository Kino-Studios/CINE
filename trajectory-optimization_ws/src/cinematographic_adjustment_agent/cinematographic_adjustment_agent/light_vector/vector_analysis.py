import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

from ..config.planner_params import CinematographyParams


def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-12:
        return np.eye(3, dtype=float)
    x /= n
    y /= n
    z /= n
    w /= n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


class LightVectorAnalysisNode(Node):
    def __init__(self):
        super().__init__("light_vector_analysis")
        self.params = CinematographyParams.from_env()
        self.bridge = CvBridge()
        self.has_odom = False
        self.drone_orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

        self.declare_parameter("image_topic", self.params.image_topic)
        self.declare_parameter("odom_topic", self.params.odom_topic)
        self.declare_parameter("light_vector_topic", self.params.light_vector_topic)
        self.declare_parameter("camera_fx", self.params.camera_fx)
        self.declare_parameter("camera_fy", self.params.camera_fy)
        self.declare_parameter("camera_cx", self.params.camera_cx)
        self.declare_parameter("camera_cy", self.params.camera_cy)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.light_vector_topic = str(self.get_parameter("light_vector_topic").value)
        self.params.camera_fx = float(self.get_parameter("camera_fx").value)
        self.params.camera_fy = float(self.get_parameter("camera_fy").value)
        self.params.camera_cx = float(self.get_parameter("camera_cx").value)
        self.params.camera_cy = float(self.get_parameter("camera_cy").value)

        self._image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 20)
        self._odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self._pub = self.create_publisher(Vector3Stamped, self.light_vector_topic, 10)

    def odom_callback(self, msg: Odometry):
        self.drone_orientation = np.array(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            dtype=float,
        )
        self.has_odom = True

    def image_callback(self, msg: Image):
        if not self.has_odom:
            return

        try:
            import cv2
        except Exception:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        if cv_image.ndim == 3 and cv_image.shape[2] == 3:
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = cv_image.copy()

        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        flat_index = int(np.argmax(blurred))
        rows, cols = blurred.shape
        v, u = np.unravel_index(flat_index, (rows, cols))

        cam_vec = np.array(
            [
                (float(u) - self.params.camera_cx) / max(1e-6, self.params.camera_fx),
                (float(v) - self.params.camera_cy) / max(1e-6, self.params.camera_fy),
                1.0,
            ],
            dtype=float,
        )
        cam_vec /= max(1e-12, float(np.linalg.norm(cam_vec)))

        rot = _quat_to_rot(self.drone_orientation)
        world_vec = rot @ cam_vec
        world_vec /= max(1e-12, float(np.linalg.norm(world_vec)))

        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.vector.x = float(world_vec[0])
        out.vector.y = float(world_vec[1])
        out.vector.z = float(world_vec[2])
        self._pub.publish(out)


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

