from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from typing import List, Tuple

class WaypointPublisher:
    def __init__(self, node: Node, topic_name: str = "/move_base_simple/goal"):
        self.node = node
        self.topic_name = topic_name
        self.goal_pub = node.create_publisher(
            PoseStamped,
            topic_name,
            QOS_PROFILE = QoSProfile(
                reliability = ReliabilityPolicy.RELIABLE,
                durability = DurabilityPolicy.VOLATILE,
                history = HistoryPolicy.KEEP_LAST,
                depth = 10
            )
        )
    
    def publish_waypoints_xyz(self, waypoints: List[Tuple[float, float, float, float]]):
        if len(waypoints) == 0:
            return

        first_waypoint = waypoints[0]
        x, y, z, w = first_waypoint

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)
    
    def publish_single_waypoint(self, x: float, y: float, z: float):
        msg = PoseStamped()

        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)
