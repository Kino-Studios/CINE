"""Publishes the walking subject's pose on /gazebo/subject_pose, replayed from its script.

Gazebo never publishes a scripted actor's pose, so this node reads the actor's script
from the rendered world and evaluates it at the current sim time (run with use_sim_time).
"""

from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from cine_gazebo.subject_trajectory import parse_actor_script, subject_pose_at

SUBJECT_POSE_TOPIC = '/gazebo/subject_pose'


class SubjectPosePublisher(Node):
    def __init__(self):
        super().__init__('subject_pose_publisher')
        world_file = self.declare_parameter('world_file', '').value
        publish_rate_hz = self.declare_parameter('publish_rate_hz', 30.0).value
        self._script = parse_actor_script(Path(world_file).read_text())
        self._publisher = self.create_publisher(PoseStamped, SUBJECT_POSE_TOPIC, 10)
        # Under sim time the timer only fires once /clock is running.
        self.create_timer(1.0 / publish_rate_hz, self._publish)

    def _publish(self):
        now = self.get_clock().now()
        pose = subject_pose_at(self._script, now.nanoseconds / 1e9)
        message = PoseStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self._script.world_name
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose.position
        orientation = message.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = pose.orientation_xyzw
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SubjectPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
