import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from traj_utils.msg import Bspline

from .mlp_loader import MLPLoader

try:
    from constants import MLPParams

    _BSPLINE_IN_TOPIC = MLPParams.BSPLINE_IN_TOPIC
    _CORRECTED_BSPLINE_TOPIC = MLPParams.CORRECTED_BSPLINE_TOPIC
    _CORRECTION_DELTA_TOPIC = MLPParams.CORRECTION_DELTA_TOPIC
except ImportError:
    _BSPLINE_IN_TOPIC = "/drone_0_planning/cinematic_bspline"
    _CORRECTED_BSPLINE_TOPIC = "/mlp/corrected_bspline"
    _CORRECTION_DELTA_TOPIC = "/mlp/correction_delta"


class MLPNode(Node):
    def __init__(self):
        super().__init__("mlp_correction_node")

        self.declare_parameter("mlp_weights_path", "")
        self.mlp_weights_path = self.get_parameter("mlp_weights_path").value

        self.loader = MLPLoader(self.mlp_weights_path)

        self.subscription = self.create_subscription(
            Bspline,
            _BSPLINE_IN_TOPIC,
            self.listener_callback,
            10,
        )

        self.publisher = self.create_publisher(Bspline, _CORRECTED_BSPLINE_TOPIC, 10)

        self.correction_publisher = self.create_publisher(
            Vector3Stamped, _CORRECTION_DELTA_TOPIC, 10
        )

    def listener_callback(self, msg: Bspline) -> None:
        pos_pts = np.array([[p.x, p.y, p.z] for p in msg.pos_pts], dtype=np.float64)

        knots = np.array(msg.knots, dtype=np.float64)
        diffs = np.diff(knots)
        positive_diffs = diffs[diffs > 0]
        dt = float(np.median(positive_diffs)) if len(positive_diffs) > 0 else 0.1

        corrected_pts = self.loader.correct(pos_pts, dt)

        msg.pos_pts = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in corrected_pts]
        self.publisher.publish(msg)

        mean_delta = np.mean(corrected_pts - pos_pts, axis=0)
        delta_msg = Vector3Stamped()
        delta_msg.header.stamp = self.get_clock().now().to_msg()
        delta_msg.header.frame_id = "odom_world"
        delta_msg.vector.x = float(mean_delta[0])
        delta_msg.vector.y = float(mean_delta[1])
        delta_msg.vector.z = float(mean_delta[2])
        self.correction_publisher.publish(delta_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MLPNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
