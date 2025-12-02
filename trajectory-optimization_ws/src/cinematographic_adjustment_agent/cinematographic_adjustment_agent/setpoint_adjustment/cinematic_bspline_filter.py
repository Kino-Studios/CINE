import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from cinematography_msgs.msg import CinematographyCosts
from traj_utils.msg import Bspline
from ..config.planner_params import CinematographyParams
from .bspline_utils import msg_to_arrays, arrays_to_msg


class CinematicBsplineFilter(Node):
    def __init__(self):
        super().__init__("cinematic_bspline_filter")
        self.params = CinematographyParams()
        self.light_vector = None
        self.latest_costs = {}
        self.sub = self.create_subscription(
            Bspline, 
            queue_size=10,
            callback=self.bspline_callback
        )
        self.light_sub = self.create_subscription(
            Vector3Stamped,
            self.params.light_vector_topic,
            queue_size=10,
            callback=self.light_callback
        )

        self.cost_sub = self.create_subscription(
            CinematographyCosts,
            "/cinematography/cinematography_costs",
            queue_size=10,
            callback=self.cost_callback
        )

        self.pub = self.create_publisher(
            Bspline,
            self.params.bspline_out_topic,
            queue_size=10
        )

    def light_callback(self, msg: Vector3Stamped):
        vec = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=float)
        norm = np.linalg.norm(vec)
        self.light_vector = vec / norm if norm > 0 else None

    def cost_callback(self, msg: CinematographyCosts):
        self.latest_costs = {
            "light_cost": float(msg.light_cost),
            "framing_cost": float(msg.framing_cost),
            "smoothness_cost": float(msg.smoothness_cost),
            "distance_cost": float(msg.distance_cost),
            "total_cost": float(msg.total_cost),
        }

    def bspline_callback(self, msg: Bspline):
        adjusted = self.adjust_bspline(msg)
        if adjusted is not None:
            self.pub.publish(adjusted)

    def adjust_bspline(self, msg: Bspline) -> Bspline:
        pos_pts, _ = msg_to_arrays(msg)

        framing_cost = self.latest_costs.get("framing_cost", 0.0)
        depth_cost = self.latest_costs.get("distance_cost", 0.0)
        smooth_cost = self.latest_costs.get("smoothness_cost", 0.0)
        light_cost = self.latest_costs.get("light_cost", 0.0)
        total_cost = self.latest_costs.get("total_cost", 0.0)

        offset = np.zeros(3, dtype=float)
        if self.light_vector is not None:
            light_scale = 1.0 + 0.1 * light_cost + 0.05 * total_cost
            offset += -self.light_vector * self.params.cinematic_offset_m * light_scale

        depth_scale = np.tanh(depth_cost)
        pos_pts[:, 2] += -0.05 * depth_scale

        framing_scale = min(1.5, 1.0 + 0.1 * framing_cost + 0.05 * total_cost)
        offset *= framing_scale

        pos_pts = pos_pts + offset
        pos_pts[:, 2] = np.maximum(pos_pts[:, 2], self.params.min_altitude_m)

        alpha = float(self.params.cinematic_smooth_alpha)
        alpha = min(
            0.9,
            alpha
            * (
                1.0
                + 0.1 * framing_cost
                + 0.05 * total_cost
                + 0.05 * smooth_cost
            ),
        )
        if pos_pts.shape[0] > 2 and alpha > 0.0:
            smoothed = pos_pts.copy()
            for i in range(1, pos_pts.shape[0] - 1):
                smoothed[i] = (1 - alpha) * pos_pts[i] + alpha * 0.5 * (pos_pts[i - 1] + pos_pts[i + 1])
            pos_pts = smoothed

        pos_pts = self.apply_mlp_adjustment(pos_pts)
        return arrays_to_msg(msg, pos_pts)

    # for later usage, not implemented yet
    def apply_mlp_adjustment(self, pos_pts: np.ndarray) -> np.ndarray:
        return pos_pts

def main(args=None):
    rclpy.init(args=args)
    node = CinematicBsplineFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
