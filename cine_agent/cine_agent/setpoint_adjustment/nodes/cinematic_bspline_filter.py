import math

import numpy as np
import rclpy
from cine_config.constants import (
    BsplineFilterParams,
    CinematicConstraints,
    CinematographyParams,
    UserRequestedParams,
)
from geometry_msgs.msg import PointStamped, PolygonStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from traj_utils.msg import Bspline

from cine_msgs.msg import CinematographyCosts

from ..utils.bspline_utils import arrays_to_msg, msg_to_arrays


class CinematicBsplineFilter(Node):
    def __init__(self):
        super().__init__('cinematic_bspline_filter')
        self.params = CinematographyParams()
        self.constraints = CinematicConstraints()
        self.light_vector = None
        self.latest_costs = {}
        self.has_focus = False

        self.declare_parameter('bspline_in_topic', self.params.BSPLINE_IN_TOPIC)
        self.declare_parameter('bspline_out_topic', self.params.BSPLINE_OUT_TOPIC)
        self.declare_parameter('cinematic_offset_m', self.params.CINEMATIC_OFFSET_M)
        self.declare_parameter('cinematic_smooth_alpha', self.params.CINEMATIC_SMOOTH_ALPHA)
        self.declare_parameter('min_altitude_m', self.params.MIN_ALTITUDE_M)
        self.declare_parameter('focus_radius_m', BsplineFilterParams.FOCUS_RADIUS_M)
        self.declare_parameter(
            'focus_radius_tolerance_m', BsplineFilterParams.FOCUS_RADIUS_TOLERANCE_M
        )
        self.declare_parameter('focus_altitude_m', BsplineFilterParams.FOCUS_ALTITUDE_M)

        self.params.BSPLINE_IN_TOPIC = str(self.get_parameter('bspline_in_topic').value)
        self.params.BSPLINE_OUT_TOPIC = str(self.get_parameter('bspline_out_topic').value)
        self.params.CINEMATIC_OFFSET_M = float(self.get_parameter('cinematic_offset_m').value)
        self.params.CINEMATIC_SMOOTH_ALPHA = float(
            self.get_parameter('cinematic_smooth_alpha').value
        )
        self.params.MIN_ALTITUDE_M = float(self.get_parameter('min_altitude_m').value)
        self.focus_radius_m = float(self.get_parameter('focus_radius_m').value)
        self.focus_radius_tolerance_m = float(self.get_parameter('focus_radius_tolerance_m').value)
        self.focus_altitude_m = float(self.get_parameter('focus_altitude_m').value)

        self.sub = self.create_subscription(
            Bspline,
            self.params.BSPLINE_IN_TOPIC,
            self.bspline_callback,
            10,
        )

        self.light_sub = self.create_subscription(
            Vector3Stamped,
            self.params.LIGHT_VECTOR_TOPIC,
            self.light_callback,
            10,
        )

        self.cost_sub = self.create_subscription(
            CinematographyCosts,
            '/cinematography/cinematography_costs',
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
            self.params.BBOX_TOPIC,
            self.bbox_callback,
            10,
        )

        self.target_world = None

        self.target_sub = self.create_subscription(
            PointStamped,
            '/cinematography/target_world',
            self.target_world_callback,
            10,
        )

        self.pub = self.create_publisher(
            Bspline,
            self.params.BSPLINE_OUT_TOPIC,
            10,
        )

    def light_callback(self, msg: Vector3Stamped):
        vec = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=float)
        norm = np.linalg.norm(vec)
        self.light_vector = vec / norm if norm > 0 else None

    def cost_callback(self, msg: CinematographyCosts):
        self.latest_costs = {
            'light_cost': float(msg.light_cost),
            'framing_cost': float(msg.framing_cost),
            'smoothness_cost': float(msg.smoothness_cost),
            'distance_cost': float(msg.distance_cost),
            'total_cost': float(msg.total_cost),
        }

    def bbox_callback(self, msg: PolygonStamped):
        self.has_focus = len(msg.polygon.points) > 0

    def target_world_callback(self, msg: PointStamped):
        self.target_world = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)

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

        depth_cost = self.latest_costs.get('distance_cost', 0.0)
        smooth_cost = self.latest_costs.get('smoothness_cost', 0.0)

        depth_scale = np.tanh(depth_cost)

        alpha = float(self.params.CINEMATIC_SMOOTH_ALPHA)
        alpha = min(
            0.9,
            alpha * (1.0 + 0.05 * smooth_cost),
        )

        if pos_pts.shape[0] > 2 and alpha > 0.0:
            smoothed = pos_pts.copy()
            for i in range(1, pos_pts.shape[0] - 1):
                smoothed[i] = (1 - alpha) * pos_pts[i] + alpha * 0.5 * (
                    pos_pts[i - 1] + pos_pts[i + 1]
                )
            pos_pts = smoothed

        dt = self._estimate_dt(knots)

        focus_mask = self._compute_focus_mask(pos_pts)
        focus_ratio = (
            float(np.mean(focus_mask)) if focus_mask is not None and focus_mask.size > 0 else 0.0
        )
        focus_mask_count = int(np.count_nonzero(focus_mask)) if focus_mask is not None else 0
        use_focus_limits = focus_ratio >= 0.15

        max_vel = (
            self.constraints.FOCUS_MAX_VELOCITY_MPS
            if use_focus_limits
            else self.constraints.DEFAULT_MAX_VELOCITY_MPS
        )
        max_acc = (
            self.constraints.FOCUS_MAX_ACCEL_MPS2
            if use_focus_limits
            else self.constraints.DEFAULT_MAX_ACCEL_MPS2
        )
        focus_vel_damping = max(0.0, min(1.0, float(self.constraints.FOCUS_VELOCITY_DAMPING)))
        focus_acc_damping = max(0.0, min(1.0, float(self.constraints.FOCUS_ACCELERATION_DAMPING)))

        if use_focus_limits:
            max_vel = max_vel * focus_vel_damping
            max_acc = max_acc * focus_acc_damping

        knots = self._time_scale_knots_for_limits(
            pos_pts, knots, dt, max_vel, max_acc, focus_mask=focus_mask
        )
        dt = self._estimate_dt(knots)
        pos_pts = self._enforce_velocity_acc_limits(
            pos_pts, dt, max_vel, max_acc, focus_mask=focus_mask
        )

        pos_pts = self.apply_mlp_adjustment(pos_pts)
        pos_pts = self._enforce_focus_radius(pos_pts, focus_mask=focus_mask)

        if pos_pts.size:
            z_stats = (
                float(np.min(pos_pts[:, 2])),
                float(np.mean(pos_pts[:, 2])),
                float(np.max(pos_pts[:, 2])),
            )
        else:
            z_stats = (0.0, 0.0, 0.0)

        z_pre_stats = z_stats
        z_post_stats = (
            (
                float(np.min(pos_pts[:, 2])),
                float(np.mean(pos_pts[:, 2])),
                float(np.max(pos_pts[:, 2])),
            )
            if pos_pts.size
            else (0.0, 0.0, 0.0)
        )

        self.get_logger().debug(
            'CinematicBsplineFilter adjust: '
            f'depth_cost={depth_cost:.3f} '
            f'smooth_cost={smooth_cost:.3f} '
            f'depth_scale={depth_scale:.3f} '
            f'focus_ratio={focus_ratio:.3f} '
            f'mask_count={focus_mask_count} '
            f'use_focus_limits={use_focus_limits} '
            f'max_vel={max_vel:.3f} '
            f'max_acc={max_acc:.3f} '
            f'z_pre=({z_pre_stats[0]:.3f},{z_pre_stats[1]:.3f},{z_pre_stats[2]:.3f}) '
            f'z_post=({z_post_stats[0]:.3f},{z_post_stats[1]:.3f},{z_post_stats[2]:.3f})'
        )

        return arrays_to_msg(msg, pos_pts, knots=knots)

    def apply_mlp_adjustment(self, pos_pts: np.ndarray) -> np.ndarray:
        return pos_pts

    def _compute_focus_mask(self, pos_pts: np.ndarray) -> np.ndarray | None:
        if self.target_world is None or pos_pts is None:
            return None

        tol = max(0.01, float(self.focus_radius_tolerance_m))
        dists = np.linalg.norm(pos_pts - self.target_world, axis=1)
        desired = float(self.focus_radius_m)

        if desired <= 0.0:
            if dists.size == 0:
                return None
            k = max(3, int(math.ceil(0.2 * dists.size)))
            desired = float(np.median(np.partition(dists, k - 1)[:k]))

        return np.abs(dists - desired) <= tol

    def _enforce_focus_radius(
        self, pos_pts: np.ndarray, focus_mask: np.ndarray | None = None
    ) -> np.ndarray:
        if self.target_world is None or pos_pts is None:
            return pos_pts

        tol = max(0.01, float(self.focus_radius_tolerance_m))
        deltas = pos_pts - self.target_world
        dists = np.linalg.norm(deltas, axis=1)
        desired = float(self.focus_radius_m)

        if desired <= 0.0:
            desired = float(UserRequestedParams.DESIRED_FOCUS_DISTANCE_M)
        if desired <= 0.0:
            if dists.size == 0:
                return pos_pts

            k = max(3, int(math.ceil(0.2 * dists.size)))
            desired = float(np.median(np.partition(dists, k - 1)[:k]))

        out = pos_pts.copy()

        if focus_mask is None:
            focus_mask = np.abs(dists - desired) <= tol

        for i, (vec, dist, use_focus) in enumerate(zip(deltas, dists, focus_mask)):
            if not use_focus:
                continue
            if not np.isfinite(dist) or dist < 1e-6:
                continue
            if abs(dist - desired) <= tol * 2.0:
                dz = vec[2]
                horiz_vec = vec[:2]
                horiz_norm = math.hypot(horiz_vec[0], horiz_vec[1])

                target_horiz_sq = max(desired * desired - dz * dz, 0.0)

                if horiz_norm < 1e-6 or target_horiz_sq <= 0.0:
                    continue

                horiz_scale = math.sqrt(target_horiz_sq) / horiz_norm
                new_vec = vec.copy()
                new_vec[:2] = horiz_vec * horiz_scale
                out[i] = self.target_world + new_vec
                out[i, 2] = max(out[i, 2], self.params.MIN_ALTITUDE_M)

        return out

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
        focus_mask: np.ndarray | None = None,
    ) -> np.ndarray:
        if knots is None or len(knots) < 2 or pos_pts is None or len(pos_pts) < 2:
            return knots

        dt_est = float(max(self.constraints.MIN_DT, dt_est))

        deltas = np.diff(pos_pts, axis=0)
        speeds = np.linalg.norm(deltas, axis=1) / dt_est
        peak_speed = float(np.max(speeds)) if speeds.size > 0 else 0.0

        scale_v = (peak_speed / max_vel) if (max_vel > 0 and peak_speed > max_vel) else 1.0
        if focus_mask is not None and focus_mask.size == pos_pts.shape[0]:
            segment_focus = focus_mask[1:] & focus_mask[:-1]

            if segment_focus.any():
                focus_speeds = speeds[segment_focus]
                focus_peak = float(np.max(focus_speeds)) if focus_speeds.size > 0 else 0.0
                focus_max = self.constraints.FOCUS_MAX_VELOCITY_MPS

                if focus_max > 0 and focus_peak > focus_max:
                    scale_v = max(scale_v, focus_peak / focus_max)

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

    def _enforce_velocity_acc_limits(
        self,
        pos_pts: np.ndarray,
        dt: float,
        max_vel: float,
        max_acc: float,
        focus_mask: np.ndarray | None = None,
    ) -> np.ndarray:
        pts = pos_pts.copy()

        if pts.shape[0] < 2:
            return pts
        if focus_mask is None or focus_mask.size != pts.shape[0]:
            focus_mask = np.zeros(pts.shape[0], dtype=bool)

        for i in range(1, pts.shape[0]):
            delta = pts[i] - pts[i - 1]
            vel = delta / dt
            speed = np.linalg.norm(vel)
            seg_max_vel = (
                self.constraints.FOCUS_MAX_VELOCITY_MPS
                if (focus_mask[i] and focus_mask[i - 1])
                else max_vel
            )

            if speed > seg_max_vel and speed > 0:
                scale = seg_max_vel / speed
                pts[i] = pts[i - 1] + delta * scale

        if pts.shape[0] >= 3:
            for i in range(1, pts.shape[0] - 1):
                acc = (pts[i + 1] - 2 * pts[i] + pts[i - 1]) / (math.pow(dt, 2))
                acc_mag = np.linalg.norm(acc)
                seg_max_acc = (
                    self.constraints.FOCUS_MAX_ACCEL_MPS2
                    if (focus_mask[i] and focus_mask[i - 1] and focus_mask[i + 1])
                    else max_acc
                )

                if acc_mag > seg_max_acc and acc_mag > 0:
                    blend = seg_max_acc / acc_mag
                    midpoint = 0.5 * (pts[i - 1] + pts[i + 1])
                    pts[i] = blend * pts[i] + (1.0 - blend) * midpoint

        return pts


def main(args=None):
    rclpy.init(args=args)
    node = CinematicBsplineFilter()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
