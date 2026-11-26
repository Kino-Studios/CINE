"""Republishes MAVROS's odometry reliably, with its twist in the world frame, for the sim.

Subscribes best effort to /mavros/local_position/odom, the only delivery MAVROS offers
there, and publishes each message on /cinematography/drone_odometry, where the reliable
subscribers chosen in cine_gazebo.odometry_routing can connect. MAVROS fills the twist in
base_link; the relay rotates it into the pose's frame, as VINS publishes it on hardware,
and names that frame in child_frame_id. Pose, header and covariances pass through.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from cine_gazebo.odometry_frames import body_twist_in_world, quaternion_xyzw
from cine_gazebo.odometry_routing import (
    MAVROS_ODOMETRY_TOPIC,
    RELAY_HISTORY_DEPTH,
    RELAY_RELIABILITY,
    RELAYED_ODOMETRY_TOPIC,
)


def _relay_qos(reliability_name):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=RELAY_HISTORY_DEPTH,
        reliability=ReliabilityPolicy[reliability_name],
    )


def _xyz(vector):
    return (vector.x, vector.y, vector.z)


def _assign_xyz(vector, values):
    vector.x, vector.y, vector.z = values


class OdometryRelay(Node):
    def __init__(self):
        super().__init__('odometry_relay')
        self._publisher = self.create_publisher(
            Odometry,
            RELAYED_ODOMETRY_TOPIC,
            _relay_qos(RELAY_RELIABILITY.publisher),
        )
        self.create_subscription(
            Odometry,
            MAVROS_ODOMETRY_TOPIC,
            self._relay,
            _relay_qos(RELAY_RELIABILITY.subscriber),
        )

    def _relay(self, odometry):
        twist = odometry.twist.twist

        try:
            world_twist = body_twist_in_world(
                quaternion_xyzw(odometry.pose.pose.orientation),
                _xyz(twist.linear),
                _xyz(twist.angular),
            )
        except ValueError as exc:
            # A twist in an unknown frame would mislead the planner more than a dropped message.
            self.get_logger().warning(f'Dropped odometry: {exc}', throttle_duration_sec=5.0)

            return

        _assign_xyz(twist.linear, world_twist.linear)
        _assign_xyz(twist.angular, world_twist.angular)
        # nav_msgs/Odometry gives the twist in child_frame_id, now the pose's own frame.
        odometry.child_frame_id = odometry.header.frame_id
        self._publisher.publish(odometry)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryRelay()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
