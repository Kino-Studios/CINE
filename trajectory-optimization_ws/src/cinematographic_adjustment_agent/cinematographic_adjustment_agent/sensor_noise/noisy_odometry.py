# note: this file is purely for physical simulation. this should NOT be applied to real pipelines.

import copy
import math
import random

from cinematographic_adjustment_agent.utils.paths import add_config_path

add_config_path()

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from constants import SensorNoiseParams

class NoisyOdometryNode(Node):
    def __init__(self):
        super().__init__("noisy_odometry_node")

        self.enable_noise = bool(
            self.declare_parameter(
                "enable_sensor_noise",
                SensorNoiseParams.ENABLE_SENSOR_NOISE,
            ).value
        )
        self.linear_noise_std = float(
            self.declare_parameter(
                "linear_noise_std_m", SensorNoiseParams.LINEAR_NOISE_STD_M
            ).value
        )
        self.angular_noise_std = float(
            self.declare_parameter(
                "angular_noise_std_rad", SensorNoiseParams.ANGULAR_NOISE_STD_RAD
            ).value
        )
        self.raw_topic = str(
            self.declare_parameter(
                "raw_odom_topic", SensorNoiseParams.RAW_ODOM_TOPIC
            ).value
        )
        self.noisy_topic = str(
            self.declare_parameter(
                "noisy_odom_topic", SensorNoiseParams.NOISY_ODOM_TOPIC
            ).value
        )

        self._rng = random.Random()
        self._odom_pub = self.create_publisher(Odometry, self.noisy_topic, 30)
        self._odom_sub = self.create_subscription(
            Odometry, self.raw_topic, self._odom_callback, 30
        )

    def _add_noise(self, value: float, std: float) -> float:
        if not self.enable_noise or std <= 0.0:
            return value
        return value + self._rng.gauss(0.0, std)

    def _apply_orientation_noise(self, orientation):
        if not self.enable_noise or self.angular_noise_std <= 0.0:
            return
        x = self._add_noise(orientation.x, self.angular_noise_std)
        y = self._add_noise(orientation.y, self.angular_noise_std)
        z = self._add_noise(orientation.z, self.angular_noise_std)
        w = self._add_noise(orientation.w, self.angular_noise_std)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm > 1e-12:
            orientation.x = x / norm
            orientation.y = y / norm
            orientation.z = z / norm
            orientation.w = w / norm

    def _apply_noise(self, msg: Odometry) -> None:
        msg.pose.pose.position.x = self._add_noise(
            msg.pose.pose.position.x, self.linear_noise_std
        )
        msg.pose.pose.position.y = self._add_noise(
            msg.pose.pose.position.y, self.linear_noise_std
        )
        msg.pose.pose.position.z = self._add_noise(
            msg.pose.pose.position.z, self.linear_noise_std
        )

        self._apply_orientation_noise(msg.pose.pose.orientation)

        msg.twist.twist.linear.x = self._add_noise(
            msg.twist.twist.linear.x, self.linear_noise_std
        )
        msg.twist.twist.linear.y = self._add_noise(
            msg.twist.twist.linear.y, self.linear_noise_std
        )
        msg.twist.twist.linear.z = self._add_noise(
            msg.twist.twist.linear.z, self.linear_noise_std
        )

        msg.twist.twist.angular.x = self._add_noise(
            msg.twist.twist.angular.x, self.angular_noise_std
        )
        msg.twist.twist.angular.y = self._add_noise(
            msg.twist.twist.angular.y, self.angular_noise_std
        )
        msg.twist.twist.angular.z = self._add_noise(
            msg.twist.twist.angular.z, self.angular_noise_std
        )

    def _odom_callback(self, msg: Odometry):
        noisy_msg = copy.deepcopy(msg)
        self._apply_noise(noisy_msg)
        self._odom_pub.publish(noisy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NoisyOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
