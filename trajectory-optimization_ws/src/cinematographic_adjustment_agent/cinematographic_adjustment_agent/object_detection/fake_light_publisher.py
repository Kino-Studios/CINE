import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node


class FakeLightPublisher(Node):
    def __init__(self):
        super().__init__("fake_light_publisher")

        self.frame_id = str(self.declare_parameter("frame_id", "map").value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        self.light_vector_xyz = tuple(float(v) for v in self.declare_parameter("light_vector_xyz", [1.0, 0.0, 0.0]).value)
        self.topic = str(self.declare_parameter("topic", "/cinematography/light_vector").value)

        self._pub = self.create_publisher(Vector3Stamped, self.topic, 10)
        period = 1.0 / max(1.0, self.publish_rate_hz)
        self._timer = self.create_timer(period, self._tick)

    def _tick(self):
        now = self.get_clock().now().to_msg()

        lx, ly, lz = self.light_vector_xyz
        ln = math.sqrt(lx * lx + ly * ly + lz * lz)

        msg = Vector3Stamped()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        if ln > 1e-6:
            msg.vector.x = float(lx / ln)
            msg.vector.y = float(ly / ln)
            msg.vector.z = float(lz / ln)
        else:
            msg.vector.x = 1.0
            msg.vector.y = 0.0
            msg.vector.z = 0.0

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLightPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

