import math

import numpy as np
import rclpy
from cinematography_msgs.msg import CinematographyCosts
from geometry_msgs.msg import PolygonStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from traj_utils.msg import Bspline

from ..config.planner_params import CinematographyParams
from ..config.constraints import CinematicConstraints
from .bspline_utils import msg_to_arrays, arrays_to_msg


class CinematicBsplineFilter(Node):
    def __init__(self):
        super().__init__("cinematic_bspline_filter")
        self.params = CinematographyParams.from_env()
        self.constraints = CinematicConstraints()
        self.light_vector = None
        self.latest_costs = {}
        self.has_focus = False

        self.declare_parameter("bspline_in_topic", self.params.bspline_in_topic)
        self.declare_parameter("bspline_out_topic", self.params.bspline_out_topic)
        self.declare_parameter("cinematic_offset_m", self.params.cinematic_offset_m)
        self.declare_parameter("cinematic_smooth_alpha", self.params.cinematic_smooth_alpha)
        self.declare_parameter("min_altitude_m", self.params.min_altitude_m)

        self.params.bspline_in_topic = str(self.get_parameter("bspline_in_topic").value)
        self.params.bspline_out_topic = str(self.get_parameter("bspline_out_topic").value)
        self.params.cinematic_offset_m = float(self.get_parameter("cinematic_offset_m").value)
        self.params.cinematic_smooth_alpha = float(self.get_parameter("cinematic_smooth_alpha").value)
        self.params.min_altitude_m = float(self.get_parameter("min_altitude_m").value)

        self.sub = self.create_subscription(
            Bspline,
            self.params.bspline_in_topic,
            self.bspline_callback,
            10,
        )

        self.light_sub = self.create_subscription(
            Vector3Stamped,
            self.params.light_vector_topic,
            self.light_callback,
            10,
        )

        self.cost_sub = self.create_subscription(
            CinematographyCosts,
            "/cinematography/cinematography_costs",
            self.cost_callback,
            10,
        )
        
        self.detections_sub = self.create_subscription(
            Float32MultiArray,
            'object_detections',
            self.detections_callback,
            10,
        )

        self.bbox_sub = self.create_subscription(
            PolygonStamped,
            self.params.bbox_topic,
            self.bbox_callback,
            10,
        )

        self.pub = self.create_publisher(
            Bspline,
            self.params.bspline_out_topic,
            10,
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

    def bbox_callback(self, msg: PolygonStamped):
        self.has_focus = len(msg.polygon.points) > 0
    
    def detections_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 1:
            self.has_focus = False
            return
        
        num_objects = int(msg.data[0])
        self.has_focus = num_objects > 0
        
        if self.has_focus and num_objects > 0:
            if not hasattr(self, '_detection_logged'):
                self._detection_logged = True

    def bspline_callback(self, msg: Bspline):
        adjusted = self.adjust_bspline(msg)
        if adjusted is not None:
            self.pub.publish(adjusted)

    def adjust_bspline(self, msg: Bspline) -> Bspline:
        pos_pts, knots = msg_to_arrays(msg)

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

        dt = self._estimate_dt(knots)
        max_vel = self.constraints.FOCUS_MAX_VELOCITY_MPS if self.has_focus else self.constraints.DEFAULT_MAX_VELOCITY_MPS
        max_acc = self.constraints.FOCUS_MAX_ACCEL_MPS2 if self.has_focus else self.constraints.DEFAULT_MAX_ACCEL_MPS2
        knots = self._time_scale_knots_for_limits(pos_pts, knots, dt, max_vel, max_acc)
        dt = self._estimate_dt(knots)
        pos_pts = self._enforce_velocity_acc_limits(pos_pts, dt, max_vel, max_acc)

        pos_pts = self.apply_mlp_adjustment(pos_pts)
        return arrays_to_msg(msg, pos_pts, knots=knots)

    def apply_mlp_adjustment(self, pos_pts: np.ndarray) -> np.ndarray:
        return pos_pts

    def _estimate_dt(self, knots: np.ndarray) -> float:
        if knots is None or len(knots) < 2:
            return self.constraints.MIN_DT

        diffs = np.diff(knots)
        positive = diffs[diffs > 0]

        if len(positive) == 0:
            return self.constraints.MIN_DT

        return float(max(self.constraints.MIN_DT, float(np.median(positive))))

    def _time_scale_knots_for_limits(
        self,
        pos_pts: np.ndarray,
        knots: np.ndarray,
        dt_est: float,
        max_vel: float,
        max_acc: float,
    ) -> np.ndarray:
        if knots is None or len(knots) < 2 or pos_pts is None or len(pos_pts) < 2:
            return knots

        dt_est = float(max(self.constraints.MIN_DT, dt_est))

        deltas = np.diff(pos_pts, axis=0)
        speeds = np.linalg.norm(deltas, axis=1) / dt_est
        peak_speed = float(np.max(speeds)) if speeds.size > 0 else 0.0

        scale_v = (peak_speed / max_vel) if (max_vel > 0 and peak_speed > max_vel) else 1.0

        if len(pos_pts) >= 3:
            acc = (pos_pts[2:] - 2 * pos_pts[1:-1] + pos_pts[:-2]) / (dt_est * dt_est)
            acc_mag = np.linalg.norm(acc, axis=1)
            peak_acc = float(np.max(acc_mag)) if acc_mag.size > 0 else 0.0
        else:
            peak_acc = 0.0

        scale_a = math.sqrt(peak_acc / max_acc) if (max_acc > 0 and peak_acc > max_acc) else 1.0
        scale = float(max(scale_v, scale_a, 1.0))

        if scale <= 1.0 + 1e-6:
            return knots

        diffs = np.diff(knots)
        diffs = np.maximum(diffs, self.constraints.MIN_DT) * scale
        scaled = np.empty_like(knots)
        scaled[0] = knots[0]
        scaled[1:] = knots[0] + np.cumsum(diffs)
        return scaled

    def _enforce_velocity_acc_limits(self, pos_pts: np.ndarray, dt: float, max_vel: float, max_acc: float) -> np.ndarray:
        pts = pos_pts.copy()

        if pts.shape[0] < 2:
            return pts

        for i in range(1, pts.shape[0]):
            delta = pts[i] - pts[i - 1]
            vel = delta / dt
            speed = np.linalg.norm(vel)

            if speed > max_vel and speed > 0:
                scale = max_vel / speed
                pts[i] = pts[i - 1] + delta * scale

        if pts.shape[0] >= 3:
            for i in range(1, pts.shape[0] - 1):
                acc = (pts[i + 1] - 2 * pts[i] + pts[i - 1]) / (math.pow(dt, 2))
                acc_mag = np.linalg.norm(acc)

                if acc_mag > max_acc and acc_mag > 0:
                    blend = max_acc / acc_mag
                    midpoint = 0.5 * (pts[i - 1] + pts[i + 1])
                    pts[i] = blend * pts[i] + (1.0 - blend) * midpoint

        return pts

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
