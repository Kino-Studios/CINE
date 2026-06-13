from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.exceptions import (
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class WaypointPublisher:
    def __init__(self, node: Node, topic_name: str = "/move_base_simple/goal"):
        self.node = node
        self.topic_name = topic_name
        self.frame_id = "map"
        try:
            node.declare_parameter("frame_id", "map")
            self.frame_id = str(node.get_parameter("frame_id").value)
        except (ParameterAlreadyDeclaredException, ParameterNotDeclaredException):
            self.frame_id = "map"
        self.goal_pub = node.create_publisher(
            PoseStamped,
            topic_name,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

    def publish_single_waypoint(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def publish_waypoints_xyz(self, waypoints: List[Tuple[float, float, float, float]]):
        if len(waypoints) == 0:
            return

        first_waypoint = waypoints[0]
        x, y, z, w = first_waypoint

        self.publish_single_waypoint(float(x), float(y), float(z))


class WaypointSenderNode(Node):
    def __init__(self):
        super().__init__("waypoint_sender")
        self.declare_parameter("input_topic", "/cinematographic/goal")
        self.declare_parameter("output_topic", "/move_base_simple/goal")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self._pub = self.create_publisher(PoseStamped, output_topic, 10)
        self._sub = self.create_subscription(PoseStamped, input_topic, self._cb, 10)
        self._last: Optional[PoseStamped] = None

    def _cb(self, msg: PoseStamped):
        self._last = msg
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
