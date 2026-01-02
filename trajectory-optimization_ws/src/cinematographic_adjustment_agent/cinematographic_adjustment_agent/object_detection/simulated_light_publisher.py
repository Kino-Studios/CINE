import math

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SimulatedLightPublisher(Node):
    def __init__(self):
        super().__init__("simulated_light_publisher")

        self.frame_id = str(self.declare_parameter("frame_id", "world").value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        self.light_vector_xyz = tuple(float(v) for v in self.declare_parameter("light_vector_xyz", [5.5, 1.7, 1.8]).value)
        self.light_distance_m = float(self.declare_parameter("light_distance_m", 4.8).value)
        self.topic = str(self.declare_parameter("topic", "/cinematography/light_vector").value)
        self.light_point_topic = str(self.declare_parameter("light_point_topic", "/cinematography/light_point").value)
        self.use_start_position = bool(self.declare_parameter("use_start_position", True).value)
        self.odom_topic = str(self.declare_parameter("odom_topic", "/drone_0_visual_slam/odom").value)

        self.start_position = None
        self.current_position = None
        if self.use_start_position:
            self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)

        self._pub = self.create_publisher(Vector3Stamped, self.topic, 10)
        self._point_pub = self.create_publisher(PointStamped, self.light_point_topic, 10)
        period = 1.0 / max(1.0, self.publish_rate_hz)
        self._timer = self.create_timer(period, self._tick)

    def _odom_callback(self, msg: Odometry):
        pose = msg.pose.pose.position
        if self.start_position is None:
            self.start_position = (pose.x, pose.y, pose.z)
        self.current_position = (pose.x, pose.y, pose.z)

    def _tick(self):
        now = self.get_clock().now().to_msg()

        msg = Vector3Stamped()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        point = PointStamped()
        point.header.stamp = now
        point.header.frame_id = self.frame_id

        if self.start_position is None:
            # fallback to static offset before receiving odom
            lx, ly, lz = self.light_vector_xyz
            ln = math.sqrt(lx * lx + ly * ly + lz * lz)
            scale = self.light_distance_m / ln if ln > 1e-6 and self.light_distance_m > 0.0 else 1.0
            msg.vector.x = float(lx * scale)
            msg.vector.y = float(ly * scale)
            msg.vector.z = float(lz * scale)
            px = msg.vector.x
            py = msg.vector.y
            pz = msg.vector.z
        else:
            base_x, base_y, base_z = self.start_position
            lx, ly, lz = self.light_vector_xyz
            scale = self.light_distance_m / math.sqrt(lx * lx + ly * ly + lz * lz) if self.light_distance_m > 0.0 else 1.0
            target_x = base_x + lx * scale
            target_y = base_y + ly * scale
            target_z = base_z + lz * scale
            px = target_x
            py = target_y
            pz = target_z

        point.point.x = px
        point.point.y = py
        point.point.z = pz
        self._point_pub.publish(point)

        if self.current_position is not None:
            dx = px - self.current_position[0]
            dy = py - self.current_position[1]
            dz = pz - self.current_position[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > 1e-6:
                msg.vector.x = dx
                msg.vector.y = dy
                msg.vector.z = dz
            else:
                msg.vector.x = 1.0
                msg.vector.y = 0.0
                msg.vector.z = 0.0
        else:
            msg.vector.x = 1.0
            msg.vector.y = 0.0
            msg.vector.z = 0.0

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedLightPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
