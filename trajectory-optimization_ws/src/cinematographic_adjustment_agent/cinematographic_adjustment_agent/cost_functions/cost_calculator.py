"""Stateful cinematographic cost calculator for a single drone.

``CostCalculator`` accumulates sensor observations (depth image, bounding box,
odometry, light vector) and on each call to ``calculate_cost()`` returns a dict
of weighted scalar costs used to steer the cinematographic B-spline filter and
the cinematic waypoint generator.

Cost terms
----------
framing_cost
    Huber-penalised pixel offset of the detected subject centre from the
    rule-of-thirds anchor (pixels; image-plane frame).  Anchor and deadband
    come from ``UserRequestedParams``.
distance_cost
    Weighted sum of squared deviation from the desired focus depth
    (``desired_focus_distance_m``, metres), a proximity penalty below
    ``focus_distance_min_m``, and a temporal depth-rate stability term.
lighting_cost
    Forward-dot-light penalty active only while framing is engaged and
    ``light_avoidance_enabled`` is true.  Scaled by distance attenuation and
    image saturation fraction near the brightest pixel.
smoothness_cost
    L2 norm of angular-velocity delta between successive odometry callbacks
    (rad/s per callback interval), weighted by ``w_smooth``.

All position/depth quantities are in metres; angles in radians.  Odometry is
expected in the world (ENU) frame as ``nav_msgs/Odometry``; the orientation
quaternion is [x, y, z, w].  Camera intrinsics (fx, fy, cx, cy) are read from
``CinematographyParams`` and can be overridden via ROS 2 node parameters.

``estimate_position_cost_at_position()`` provides a geometry-only cost estimate
for candidate waypoint positions without requiring a live depth image.
"""

from cinematographic_adjustment_agent.utils.paths import add_config_path

add_config_path()

import logging
import math
from typing import Dict, Optional

import cv2
import numpy as np
from constants import (
    CinematographyParams,
    CostParams,
    ImageParams,
    LightAvoidanceParams,
    LightOrientationParams,
    ROSParams,
    SimulationParams,
    UserRequestedParams,
)
from geometry_msgs.msg import PolygonStamped, Vector3Stamped
from nav_msgs.msg import Odometry

COLLISION_DEPTH_MARGIN_M: float = 0.5
FOCUS_YAW_HOLD_DEADBAND_RAD: float = math.radians(2.0)

try:
    from rclpy.exceptions import (
        ParameterAlreadyDeclaredException,
        ParameterNotDeclaredException,
    )
except ImportError:  # rclpy unavailable outside a ROS environment (e.g. unit tests)

    class ParameterAlreadyDeclaredException(Exception):
        pass

    class ParameterNotDeclaredException(Exception):
        pass


PARAMETER_READ_ERRORS = (
    ParameterNotDeclaredException,
    TypeError,
    ValueError,
)

_logger = logging.getLogger(__name__)


class CostCalculator:
    def __init__(self, node):
        self.node = node
        self.bounding_box = None
        self.dimensions = None
        self.center_point = None
        self.depth_image = None
        self.bgr_image = None
        self.center_depth = None
        self.light_vector = None
        self.light_distance = None
        self._declare_parameter(
            "light_distance_attenuation_m", CostParams.LIGHT_DISTANCE_ATTENUATION_M
        )
        self.angular_velocity = None
        self.orientation = None
        self._prev_ang_vel = None
        self.ROSParams = ROSParams
        self.UserRequestedParams = UserRequestedParams
        self.ImageParams = ImageParams
        self.CinematographyParams = CinematographyParams()
        self.CostParams = CostParams
        self.LightOrientationParams = LightOrientationParams
        self.LightAvoidanceParams = LightAvoidanceParams
        self.SimulationParams = SimulationParams
        self._last_light_setpoint = np.array([0.0, 0.0, 0.0], dtype=float)
        self.position = None
        self.target_world: Optional[np.ndarray] = None
        self.framing_active = False
        self._declare_parameter("focus_object_id", UserRequestedParams.FOCUS_OBJECT_ID)
        self._declare_parameter("huber_delta_px", CostParams.HUBER_DELTA_PX)
        self._declare_parameter("deadband_px", CostParams.DEADBAND_PX)
        self._declare_parameter("w_framing", CostParams.W_FRAMING)
        self._declare_parameter("w_dist", CostParams.W_DIST)
        self._declare_parameter("w_near", CostParams.W_NEAR)
        self._declare_parameter("w_stab", CostParams.W_STAB)
        self._declare_parameter("w_light", CostParams.W_LIGHT)
        self._declare_parameter("w_smooth", CostParams.W_SMOOTH)
        self._declare_parameter("light_avoidance_enabled", LightAvoidanceParams.ENABLED)
        self._declare_parameter("light_yaw_deadband_rad", LightOrientationParams.YAW_DEADBAND_RAD)
        self._declare_parameter(
            "light_pitch_deadband_rad", LightOrientationParams.PITCH_DEADBAND_RAD
        )
        self._declare_parameter("light_roll_deadband_rad", LightOrientationParams.ROLL_DEADBAND_RAD)
        self._declare_parameter(
            "light_pitch_weight_factor", LightOrientationParams.PITCH_WEIGHT_FACTOR
        )
        self._declare_parameter(
            "light_roll_weight_factor", LightOrientationParams.ROLL_WEIGHT_FACTOR
        )
        self._declare_parameter("light_pitch_max_rad", LightOrientationParams.PITCH_MAX_RAD)
        self._declare_parameter(
            "light_pitch_damping_factor", LightOrientationParams.PITCH_DAMPING_FACTOR
        )
        self._declare_parameter(
            "light_setpoint_smooth_alpha", LightOrientationParams.SETPOINT_SMOOTH_ALPHA
        )
        self._declare_parameter(
            "light_planar_norm_threshold", LightOrientationParams.PLANAR_NORM_THRESHOLD
        )
        self._declare_parameter(
            "light_yaw_change_limit_rad", LightOrientationParams.YAW_CHANGE_LIMIT_RAD
        )
        self._declare_parameter(
            "light_pitch_change_limit_rad",
            LightOrientationParams.PITCH_CHANGE_LIMIT_RAD,
        )
        self._declare_parameter("light_roll_gain", LightOrientationParams.ROLL_GAIN)
        self._declare_parameter("light_pitch_gain", LightOrientationParams.PITCH_GAIN)
        self._declare_parameter("light_yaw_gain", LightOrientationParams.YAW_GAIN)
        self._declare_parameter("focus_yaw_hold_deadband_rad", FOCUS_YAW_HOLD_DEADBAND_RAD)
        self._declare_parameter("light_distance_bias_m", CostParams.LIGHT_DISTANCE_BIAS_M)
        self._declare_parameter("light_distance_gain", CostParams.LIGHT_DISTANCE_GAIN)
        self._declare_parameter(
            "light_cost_increase_when_close", CostParams.LIGHT_COST_INCREASE_WHEN_CLOSE
        )
        self._declare_parameter("use_simulated_light", SimulationParams.USE_SIMULATED_LIGHT)
        self._declare_parameter("light_tau", CostParams.LIGHT_TAU)
        self._declare_parameter("light_saturation_threshold", CostParams.LIGHT_SATURATION_THRESHOLD)
        self._declare_parameter("w_saturation", CostParams.W_SATURATION)
        self._declare_parameter(
            "include_lighting_in_position_cost",
            CostParams.INCLUDE_LIGHTING_IN_POSITION_COST,
        )
        self._declare_parameter(
            "include_framing_in_position_cost",
            CostParams.INCLUDE_FRAMING_IN_POSITION_COST,
        )
        self._declare_parameter(
            "include_lighting_in_orientation_cost",
            CostParams.INCLUDE_LIGHTING_IN_ORIENTATION_COST,
        )
        self._declare_parameter(
            "include_framing_in_orientation_cost",
            CostParams.INCLUDE_FRAMING_IN_ORIENTATION_COST,
        )
        self._declare_parameter("framing_vertical_weight", CostParams.FRAMING_VERTICAL_WEIGHT)

        self._declare_parameter("camera_fx", self.CinematographyParams.CAMERA_FX)
        self._declare_parameter("camera_fy", self.CinematographyParams.CAMERA_FY)
        self._declare_parameter("camera_cx", self.CinematographyParams.CAMERA_CX)
        self._declare_parameter("camera_cy", self.CinematographyParams.CAMERA_CY)
        self._declare_parameter("collision_depth_margin_m", COLLISION_DEPTH_MARGIN_M)

        self._last_framing_log_time = 0.0

    def _declare_parameter(self, name: str, default):
        try:
            self.node.declare_parameter(name, default)
        except ParameterAlreadyDeclaredException:
            return

    def _get_bool_parameter(self, name: str, default: bool) -> bool:
        try:
            return bool(self.node.get_parameter(name).value)
        except PARAMETER_READ_ERRORS:
            return bool(default)

    def _light_avoidance_enabled(self) -> bool:
        try:
            return bool(self.node.get_parameter("light_avoidance_enabled").value)
        except PARAMETER_READ_ERRORS:
            return bool(self.LightAvoidanceParams.ENABLED)

    def _image_size(self) -> tuple[float, float]:
        try:
            width = float(self.node.get_parameter("image_width").value)
            height = float(self.node.get_parameter("image_height").value)
            return max(1.0, width), max(1.0, height)
        except PARAMETER_READ_ERRORS:
            return float(self.ImageParams.IMAGE_WIDTH), float(self.ImageParams.IMAGE_HEIGHT)

    def update_light_vector(self, msg: Vector3Stamped):
        if not self._light_avoidance_enabled():
            self.light_vector = None
            self.light_distance = None
            return

        vector = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=float)
        norm = np.linalg.norm(vector)
        self.light_vector = vector / norm if norm > 0 else None
        self.light_distance = float(norm) if norm > 0 else None

    def _light_attenuation(self) -> float:
        try:
            if bool(self.node.get_parameter("use_simulated_light").value):
                return 1.0
        except PARAMETER_READ_ERRORS:
            pass

        if self.light_distance is None:
            return 1.0
        try:
            max_range = float(self.node.get_parameter("light_distance_attenuation_m").value)

        except PARAMETER_READ_ERRORS:
            max_range = 0.0
        if max_range <= 0.0:
            return 1.0

        normalized = min(self.light_distance, max_range) / max_range

        favor_close = bool(self.node.get_parameter("light_cost_increase_when_close").value)
        if favor_close:
            fac = 1.0 - normalized
        else:
            fac = normalized

        return max(0.0, min(1.0, fac))

    def update_odometry(self, msg: Odometry):
        self.angular_velocity = np.array(
            [
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ],
            dtype=float,
        )

        self.orientation = np.array(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            dtype=float,
        )
        self.position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )

    def update_bounding_box(self, msg: PolygonStamped):
        self.bounding_box = msg
        self._update_center_depth()

    def update_dimensions(self, msg: PolygonStamped):
        self.dimensions = msg

    def update_center_point(self, msg: PolygonStamped):
        self.center_point = msg
        self._update_center_depth()

    def update_depth_image(self, depth_image):
        self.depth_image = depth_image
        self._update_center_depth()

    def update_bgr_image(self, bgr_image):
        self.bgr_image = bgr_image

    def update_target_world(self, target_world: Optional[np.ndarray]):
        self.target_world = (
            np.array(target_world, dtype=float) if target_world is not None else None
        )

    def update_framing_active(self, active: bool):
        self.framing_active = bool(active)

    def _camera_forward_vector(self) -> Optional[np.ndarray]:
        if self.orientation is None:
            return None
        x, y, z, w = self.orientation

        norm = math.sqrt(x * x + y * y + z * z + w * w)

        if norm == 0:
            return None

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        r00 = 1 - 2 * (y * y + z * z)
        r01 = 2 * (x * y - z * w)
        r02 = 2 * (x * z + y * w)
        r10 = 2 * (x * y + z * w)
        r11 = 1 - 2 * (x * x + z * z)
        r12 = 2 * (y * z - x * w)
        r20 = 2 * (x * z - y * w)
        r21 = 2 * (y * z + x * w)
        r22 = 1 - 2 * (x * x + y * y)

        forward = np.array([r02, r12, r22], dtype=float)
        norm_f = np.linalg.norm(forward)
        return forward / norm_f if norm_f > 0 else None

    def _quaternion_to_euler(self, quat: np.ndarray) -> Optional[np.ndarray]:
        if quat is None or quat.shape[0] != 4:
            return None

        x, y, z, w = quat
        norm = math.sqrt(x * x + y * y + z * z + w * w)

        if norm == 0.0:
            return None

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return np.array([roll, pitch, yaw], dtype=float)

    def _angle_diff(self, target: float, current: float) -> float:
        delta = target - current
        return math.atan2(math.sin(delta), math.cos(delta))

    def _deadband_error(self, error: float, deadband: float) -> float:
        return max(0.0, abs(error) - deadband)

    def _update_center_depth(self):
        if self.depth_image is None:
            return

        center_x, center_y = None, None

        if self.center_point is not None and len(self.center_point.polygon.points) > 0:
            center_x = int(self.center_point.polygon.points[0].x)
            center_y = int(self.center_point.polygon.points[0].y)
        elif self.bounding_box is not None and len(self.bounding_box.polygon.points) > 0:
            points = self.bounding_box.polygon.points
            if len(points) >= 2:
                x_coords = [p.x for p in points]
                y_coords = [p.y for p in points]
                center_x = int(np.mean(x_coords))
                center_y = int(np.mean(y_coords))

        if center_x is not None and center_y is not None:
            h, w = self.depth_image.shape[:2]
            center_x = max(0, min(center_x, w - 1))
            center_y = max(0, min(center_y, h - 1))

            depth_value = self.depth_image[center_y, center_x]

            if depth_value > 0 and not np.isnan(depth_value):
                self.center_depth = float(depth_value)

    def _compute_yaw_to_target(self) -> Optional[float]:
        if self.position is None or self.target_world is None:
            return None
        dx = float(self.target_world[0]) - self.position[0]
        dy = float(self.target_world[1]) - self.position[1]
        if abs(dx) < self.SimulationParams.EPSILON and abs(dy) < self.SimulationParams.EPSILON:
            return None
        return math.atan2(dy, dx)

    def get_depth_at_point(self, x, y):
        if self.depth_image is None:
            return None

        h, w = self.depth_image.shape[:2]

        x = int(max(0, min(x, w - 1)))
        y = int(max(0, min(y, h - 1)))

        depth_value = self.depth_image[y, x]

        if depth_value > 0 and not np.isnan(depth_value):
            return float(depth_value)
        else:
            return None

    def get_median_depth_object(self):
        if self.depth_image is None or self.bounding_box is None:
            return None

        if len(self.bounding_box.polygon.points) < 2:
            return None

        h, w = self.depth_image.shape[:2]

        points = self.bounding_box.polygon.points
        x_coords = [p.x for p in points]
        y_coords = [p.y for p in points]

        x_min = int(max(0, min(x_coords)))
        x_max = int(min(w - 1, max(x_coords)))
        y_min = int(max(0, min(y_coords)))
        y_max = int(min(h - 1, max(y_coords)))

        if x_min >= x_max or y_min >= y_max:
            return None

        depth_region = self.depth_image[y_min : y_max + 1, x_min : x_max + 1]

        valid_depths = depth_region[(depth_region > 0) & ~np.isnan(depth_region)]

        if len(valid_depths) == 0:
            return None

        median_depth = float(np.median(valid_depths))

        return median_depth

    def get_median_depth_roi(self):
        if self.depth_image is None:
            return None

        center_x = None
        center_y = None

        if self.center_point is not None and len(self.center_point.polygon.points) > 0:
            center_x = int(self.center_point.polygon.points[0].x)
            center_y = int(self.center_point.polygon.points[0].y)
        elif self.bounding_box is not None and len(self.bounding_box.polygon.points) >= 2:
            points = self.bounding_box.polygon.points
            x_coords = [p.x for p in points]
            y_coords = [p.y for p in points]
            center_x = int(np.mean(x_coords))
            center_y = int(np.mean(y_coords))

        if center_x is None or center_y is None:
            return None

        h, w = self.depth_image.shape[:2]
        half_size = int(max(1, self.CostParams.ROI_SIZE // 2))

        x_min = max(0, center_x - half_size)
        x_max = min(w - 1, center_x + half_size)
        y_min = max(0, center_y - half_size)
        y_max = min(h - 1, center_y + half_size)

        if x_min > x_max or y_min > y_max:
            return None

        depth_region = self.depth_image[y_min : y_max + 1, x_min : x_max + 1]
        valid_depths = depth_region[(depth_region > 0) & ~np.isnan(depth_region)]

        if valid_depths.size == 0:
            return None

        return float(np.median(valid_depths))

    def huber_function(self, r, delta=None):
        if delta is None:
            delta = float(self.CostParams.HUBER_DELTA_PX)
        a = abs(r)
        return 0.5 * math.pow(a, 2) if a <= delta else delta * (a - 0.5 * delta)

    def _calculate_saturation_fraction(self) -> float:
        if self.bgr_image is None:
            return 0.0

        img = self.bgr_image
        h, w = img.shape[:2]

        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        _, _, _, max_loc = cv2.minMaxLoc(gray)
        bright_x, bright_y = max_loc

        roi_size = int(self.CostParams.ROI_SIZE)
        half_size = roi_size // 2

        x_min = max(0, bright_x - half_size)
        x_max = min(w - 1, bright_x + half_size)
        y_min = max(0, bright_y - half_size)
        y_max = min(h - 1, bright_y + half_size)

        roi = img[y_min : y_max + 1, x_min : x_max + 1]

        if roi.size == 0:
            return 0.0

        sat_thresh = float(self.node.get_parameter("light_saturation_threshold").value)

        if roi.dtype == np.uint8 and sat_thresh <= 1.0:
            adjusted_thresh = sat_thresh * 255.0
        else:
            adjusted_thresh = sat_thresh

        if len(roi.shape) == 3:
            intensity = np.max(roi, axis=2)
        else:
            intensity = roi

        saturated_pixels = np.sum(intensity >= adjusted_thresh)
        total_pixels = intensity.size

        if total_pixels == 0:
            return 0.0

        return float(saturated_pixels) / float(total_pixels)

    def _calculate_lighting_cost(
        self,
        forward: Optional[np.ndarray],
        light_dir: Optional[np.ndarray],
        attenuation: float,
        saturation_fraction: float = 0.0,
    ) -> float:
        w_light = float(self.node.get_parameter("w_light").value)
        tau = float(self.node.get_parameter("light_tau").value)

        w_saturation = float(self.node.get_parameter("w_saturation").value)
        strength = float(np.clip(saturation_fraction * w_saturation, 0.0, 1.0))

        if forward is None or light_dir is None:
            return 0.0

        dot = float(np.clip(np.dot(forward, light_dir), -1.0, 1.0))
        direction = max(0.0, dot - tau)

        return float(w_light * attenuation * strength * direction)

    def _resolve_desired_focus_distance(self) -> float:
        anchor_distance = getattr(self.node, "anchor_focus_distance", None)
        if anchor_distance is not None and anchor_distance > 0.0:
            return float(anchor_distance)
        return float(self.UserRequestedParams.DESIRED_FOCUS_DISTANCE_M)

    def calculate_cost(self) -> Optional[Dict[str, float]]:
        logger = self.node.get_logger()
        if self.depth_image is None:
            logger.debug("CostCalculator: waiting for depth image input")
            return None

        if self.bounding_box is None and self.center_point is None:
            logger.debug("CostCalculator: waiting for bounding box or center point")
            return None

        current_time = None
        try:
            current_time = self.node.get_clock().now().nanoseconds / 1e9
        except (AttributeError, TypeError):
            pass

        img_w, img_h = self._image_size()
        u_star = img_w * self.UserRequestedParams.THIRDS_ANCHOR[0]
        v_star = img_h * self.UserRequestedParams.THIRDS_ANCHOR[1]

        if self.center_point is not None and len(self.center_point.polygon.points) > 0:
            u_current = self.center_point.polygon.points[0].x
            v_current = self.center_point.polygon.points[0].y
        elif self.bounding_box is not None and len(self.bounding_box.polygon.points) > 0:
            points = self.bounding_box.polygon.points
            if len(points) >= 2:
                x_coords = [p.x for p in points]
                y_coords = [p.y for p in points]
                u_current = np.mean(x_coords)
                v_current = np.mean(y_coords)
            else:
                logger.debug("CostCalculator: insufficient bbox points (%d)", len(points))
                return None
        else:
            logger.debug("CostCalculator: unexpected detection fall-through")
            return None

        deadband = float(self.node.get_parameter("deadband_px").value)
        delta_px = float(self.node.get_parameter("huber_delta_px").value)
        vertical_weight = float(self.node.get_parameter("framing_vertical_weight").value)
        vertical_weight = max(0.0, vertical_weight)

        du = u_current - u_star
        du = 0.0 if abs(du) < deadband else du

        dv = (v_current - v_star) * vertical_weight
        dv = 0.0 if abs(dv) < deadband else dv

        w_framing = float(self.node.get_parameter("w_framing").value)
        framing_cost = w_framing * (
            self.huber_function(du, delta_px) + self.huber_function(dv, delta_px)
        )

        d_star = self._resolve_desired_focus_distance()
        d_min = self.UserRequestedParams.FOCUS_DISTANCE_MIN_M

        Z = self.get_median_depth_object()

        if Z is None:
            if self.center_depth is None:
                logger.debug("CostCalculator: depth unknown (no median or center depth)")
                return None

            Z = float(self.center_depth)
        Z = float(Z)

        J_dist = math.pow(Z - d_star, 2)
        J_near = math.pow(d_min - Z, 2) if Z < d_min else 0.0

        prev_z = getattr(self, "_prev_Z", None)
        prev_time = getattr(self, "_prev_depth_timestamp", None)
        J_stab = 0.0

        if prev_z is not None and prev_time is not None and current_time is not None:
            dt = current_time - prev_time
            if dt >= self.SimulationParams.EPSILON:
                dz = Z - prev_z
                J_stab = math.pow(dz / dt, 2)

        self._prev_Z = Z
        if current_time is not None:
            self._prev_depth_timestamp = current_time

        w_dist = float(self.node.get_parameter("w_dist").value)
        w_near = float(self.node.get_parameter("w_near").value)
        w_stab = float(self.node.get_parameter("w_stab").value)
        depth_cost = (w_dist * J_dist) + (w_near * J_near) + (w_stab * J_stab)

        current_euler = (
            self._quaternion_to_euler(self.orientation) if self.orientation is not None else None
        )

        lighting_cost = 0.0
        yaw_setpoint = 0.0
        pitch_setpoint = 0.0
        roll_setpoint = 0.0
        yaw_error = 0.0
        pitch_error = 0.0
        roll_error = 0.0
        actual_roll = 0.0
        actual_pitch = 0.0
        actual_yaw = 0.0

        light_override = False
        focus_override = False

        if self._light_avoidance_enabled() and self.framing_active:
            attenuation = self._light_attenuation()
            forward = self._camera_forward_vector()

            sat_frac = self._calculate_saturation_fraction()

            lighting_cost = self._calculate_lighting_cost(
                forward, self.light_vector, attenuation, sat_frac
            )
            w_light = float(self.node.get_parameter("w_light").value)

            yaw_deadband = float(self.node.get_parameter("light_yaw_deadband_rad").value)
            pitch_deadband = float(self.node.get_parameter("light_pitch_deadband_rad").value)
            roll_deadband = float(self.node.get_parameter("light_roll_deadband_rad").value)

            pitch_weight_factor = float(self.node.get_parameter("light_pitch_weight_factor").value)
            roll_weight_factor = float(self.node.get_parameter("light_roll_weight_factor").value)

            w_pitch = w_light * pitch_weight_factor
            w_roll = w_light * roll_weight_factor

            if self.light_vector is not None:
                target_dir = -self.light_vector.copy()
                target_norm = np.linalg.norm(target_dir)

                if target_norm > self.SimulationParams.EPSILON:
                    target_dir /= target_norm

                planar_norm = math.hypot(target_dir[0], target_dir[1])

                planar_deadzone = float(
                    self.node.get_parameter("light_planar_norm_threshold").value
                )
                planar_deadzone = max(0.0, planar_deadzone)

                if planar_norm >= planar_deadzone:
                    raw_yaw = math.atan2(target_dir[1], target_dir[0])
                else:
                    raw_yaw = self._last_light_setpoint[2]

                raw_pitch = math.atan2(-target_dir[2], planar_norm)
                raw_roll = 0.0

                alpha = float(self.node.get_parameter("light_setpoint_smooth_alpha").value)
                alpha = np.clip(alpha, 0.0, 1.0)

                roll_gain = float(self.node.get_parameter("light_roll_gain").value)
                pitch_gain = float(self.node.get_parameter("light_pitch_gain").value)
                yaw_gain = float(self.node.get_parameter("light_yaw_gain").value)

                roll_gain = max(0.0, roll_gain)
                pitch_gain = max(0.0, pitch_gain)
                yaw_gain = max(0.0, yaw_gain)

                raw_setpoints = np.array(
                    [raw_roll * roll_gain, raw_pitch * pitch_gain, raw_yaw * yaw_gain],
                    dtype=float,
                )

                smooth_setpoints = alpha * self._last_light_setpoint + (1.0 - alpha) * raw_setpoints

                yaw_limit = float(self.node.get_parameter("light_yaw_change_limit_rad").value)

                yaw_limit = max(0.0, yaw_limit)
                yaw_delta = self._angle_diff(smooth_setpoints[2], self._last_light_setpoint[2])

                if yaw_delta > yaw_limit:
                    yaw_delta = yaw_limit
                elif yaw_delta < -yaw_limit:
                    yaw_delta = -yaw_limit

                smooth_setpoints[2] = self._last_light_setpoint[2] + yaw_delta

                pitch_limit = float(self.node.get_parameter("light_pitch_change_limit_rad").value)
                pitch_limit = max(0.0, pitch_limit)

                pitch_delta = self._angle_diff(smooth_setpoints[1], self._last_light_setpoint[1])

                if pitch_delta > pitch_limit:
                    pitch_delta = pitch_limit
                elif pitch_delta < -pitch_limit:
                    pitch_delta = -pitch_limit

                smooth_setpoints[1] = self._last_light_setpoint[1] + pitch_delta
                roll_setpoint, pitch_setpoint, yaw_setpoint = smooth_setpoints
                self._last_light_setpoint = smooth_setpoints.copy()

                max_pitch = float(self.node.get_parameter("light_pitch_max_rad").value)

                pitch_setpoint = float(np.clip(pitch_setpoint, -abs(max_pitch), abs(max_pitch)))

                accel_factor = float(self.node.get_parameter("light_pitch_damping_factor").value)

                ang_vel_norm = (
                    float(np.linalg.norm(self.angular_velocity))
                    if self.angular_velocity is not None
                    else 0.0
                )

                damping = 1.0 / (1.0 + accel_factor * ang_vel_norm)
                pitch_setpoint *= damping
        else:
            self.light_vector = None
            self.light_distance = None
            self._last_light_setpoint = np.array([0.0, 0.0, 0.0], dtype=float)

        if self.framing_active and self.target_world is not None:
            target_yaw = self._compute_yaw_to_target()
            if target_yaw is not None:
                yaw_setpoint = target_yaw
                focus_override = True

        if current_euler is not None:
            roll_val, pitch_val, yaw_val = current_euler

            actual_roll = roll_val
            actual_pitch = pitch_val
            actual_yaw = yaw_val

            yaw_error = self._angle_diff(yaw_setpoint, yaw_val)
            pitch_error = self._angle_diff(pitch_setpoint, pitch_val)
            roll_error = self._angle_diff(roll_setpoint, roll_val)

        focus_deadband = FOCUS_YAW_HOLD_DEADBAND_RAD
        try:
            focus_deadband = max(
                0.0,
                float(self.node.get_parameter("focus_yaw_hold_deadband_rad").value),
            )
        except PARAMETER_READ_ERRORS:
            pass

        focus_cinematic_active = focus_override and (
            current_euler is None or abs(yaw_error) > focus_deadband
        )
        has_cinematic_yaw = light_override or focus_cinematic_active

        smoothness_cost = 0.0

        if self.angular_velocity is not None:
            if self._prev_ang_vel is None:
                self._prev_ang_vel = self.angular_velocity.copy()

            delta_ang = self.angular_velocity - self._prev_ang_vel
            w_smooth = float(self.node.get_parameter("w_smooth").value)
            smoothness_cost = w_smooth * float(np.linalg.norm(delta_ang))

            self._prev_ang_vel = self.angular_velocity.copy()

        total_cost = framing_cost + depth_cost + lighting_cost + smoothness_cost

        logger.debug(
            "CostCalculator: framing=%.3f distance=%.3f lighting=%.3f smoothness=%.3f total=%.3f",
            framing_cost,
            depth_cost,
            lighting_cost,
            smoothness_cost,
            total_cost,
        )

        if current_time is not None and current_time - self._last_framing_log_time >= 1.0:
            self._last_framing_log_time = current_time
            logger.info(
                "CostCalculator detail: framing=%.3f depth_cost=%.3f lighting=%.3f total=%.3f target=%.3f actual=%.3f err=%.3f d_min=%.3f",
                framing_cost,
                depth_cost,
                lighting_cost,
                total_cost,
                d_star,
                Z,
                Z - d_star,
                d_min,
            )

        return {
            "framing_cost": float(framing_cost),
            "distance_cost": float(depth_cost),
            "lighting_cost": float(lighting_cost),
            "smoothness_cost": float(smoothness_cost),
            "total_cost": float(total_cost),
            "yaw_setpoint": float(yaw_setpoint),
            "has_cinematic_yaw": has_cinematic_yaw,
            "pitch_setpoint": float(pitch_setpoint),
            "roll_setpoint": float(roll_setpoint),
            "yaw_error": float(yaw_error),
            "pitch_error": float(pitch_error),
            "roll_error": float(roll_error),
            "actual_yaw": float(actual_yaw),
            "actual_pitch": float(actual_pitch),
            "actual_roll": float(actual_roll),
        }

    def estimate_position_cost_at_position(
        self,
        drone_x: float,
        drone_y: float,
        drone_z: float,
        target_world_x: float,
        target_world_y: float,
        target_world_z: float,
        reference_drone_x: Optional[float] = None,
        reference_drone_y: Optional[float] = None,
        reference_drone_z: Optional[float] = None,
    ) -> Optional[float]:
        include_framing = self._get_bool_parameter(
            "include_framing_in_position_cost",
            self.CostParams.INCLUDE_FRAMING_IN_POSITION_COST,
        )
        include_lighting = self._get_bool_parameter(
            "include_lighting_in_position_cost",
            self.CostParams.INCLUDE_LIGHTING_IN_POSITION_COST,
        )

        depth_cost, lighting_cost, smoothness_cost, _ = self._compute_cost_components(
            drone_x,
            drone_y,
            drone_z,
            target_world_x,
            target_world_y,
            target_world_z,
            reference_drone_x,
            reference_drone_y,
            reference_drone_z,
            include_framing=include_framing,
            include_lighting=include_lighting,
        )
        return float(depth_cost + lighting_cost + smoothness_cost)

    def _is_waypoint_occluded(
        self,
        target_world_x: float,
        target_world_y: float,
        target_world_z: float,
    ) -> bool:

        if self.depth_image is None or self.orientation is None or self.position is None:
            return False

        try:
            fx = float(self.node.get_parameter("camera_fx").value)
            fy = float(self.node.get_parameter("camera_fy").value)
            cx = float(self.node.get_parameter("camera_cx").value)
            cy = float(self.node.get_parameter("camera_cy").value)
            margin = float(self.node.get_parameter("collision_depth_margin_m").value)
        except PARAMETER_READ_ERRORS:
            return False

        h, w = self.depth_image.shape[:2]

        tx, ty, tz, tw = self.orientation

        rx = target_world_x - self.position[0]
        ry = target_world_y - self.position[1]
        rz = target_world_z - self.position[2]

        norm = math.sqrt(tx * tx + ty * ty + tz * tz + tw * tw)
        if norm > 0:
            tx /= norm
            ty /= norm
            tz /= norm
            tw /= norm

        r00 = 1 - 2 * (ty * ty + tz * tz)
        r01 = 2 * (tx * ty - tz * tw)
        r02 = 2 * (tx * tz + ty * tw)

        r10 = 2 * (tx * ty + tz * tw)
        r11 = 1 - 2 * (tx * tx + tz * tz)
        r12 = 2 * (ty * tz - tx * tw)

        r20 = 2 * (tx * tz - ty * tw)
        r21 = 2 * (ty * tz + tx * tw)
        r22 = 1 - 2 * (tx * tx + ty * ty)

        cam_x = r00 * rx + r10 * ry + r20 * rz
        cam_y = r01 * rx + r11 * ry + r21 * rz
        cam_z = r02 * rx + r12 * ry + r22 * rz

        if cam_z <= 0.1:
            return False

        u = int(round((cam_x * fx) / cam_z + cx))
        v = int(round((cam_y * fy) / cam_z + cy))

        if u < 0 or u >= w or v < 0 or v >= h:
            return False

        depth_val = self.depth_image[v, u]
        if np.isnan(depth_val) or depth_val <= 0:
            return False

        if cam_z > (depth_val - margin):
            return True

        return False

    def _compute_cost_components(
        self,
        drone_x: float,
        drone_y: float,
        drone_z: float,
        target_world_x: float,
        target_world_y: float,
        target_world_z: float,
        reference_drone_x: Optional[float],
        reference_drone_y: Optional[float],
        reference_drone_z: Optional[float],
        include_framing: bool,
        include_lighting: bool,
    ) -> tuple[float, float, float, float]:

        if self._is_waypoint_occluded(target_world_x, target_world_y, target_world_z):
            return (float("inf"), 0.0, 0.0, 0.0)

        dx = target_world_x - drone_x
        dy = target_world_y - drone_y
        dz = target_world_z - drone_z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if not math.isfinite(distance) or distance <= self.SimulationParams.EPSILON:
            return (float("inf"), 0.0, 0.0, 0.0)

        d_star = self._resolve_desired_focus_distance()
        d_min = self.UserRequestedParams.FOCUS_DISTANCE_MIN_M

        j_dist = (distance - d_star) ** 2
        j_near = (d_min - distance) ** 2 if distance < d_min else 0.0

        w_dist = float(self.node.get_parameter("w_dist").value)
        w_near = float(self.node.get_parameter("w_near").value)
        w_stab = float(self.node.get_parameter("w_stab").value)
        j_stab = 0.0

        if (
            reference_drone_x is not None
            and reference_drone_y is not None
            and reference_drone_z is not None
        ):
            pdx = target_world_x - reference_drone_x
            pdy = target_world_y - reference_drone_y
            pdz = target_world_z - reference_drone_z
            prev_distance = math.sqrt(pdx * pdx + pdy * pdy + pdz * pdz)
            if math.isfinite(prev_distance) and prev_distance > self.SimulationParams.EPSILON:
                j_stab = (distance - prev_distance) ** 2

        depth_cost = (w_dist * j_dist) + (w_near * j_near) + (w_stab * j_stab)

        desired_forward = np.array([dx, dy, dz], dtype=float) / float(distance)

        lighting_cost = 0.0
        if include_lighting and self._light_avoidance_enabled():
            attenuation = self._light_attenuation()
            lighting_cost = self._calculate_lighting_cost(
                desired_forward, self.light_vector, attenuation
            )

        smoothness_cost = 0.0
        framing_cost = 0.0

        if (
            reference_drone_x is not None
            and reference_drone_y is not None
            and reference_drone_z is not None
        ):
            rdx = target_world_x - reference_drone_x
            rdy = target_world_y - reference_drone_y
            rdz = target_world_z - reference_drone_z
            ref_norm = math.sqrt(rdx * rdx + rdy * rdy + rdz * rdz)
            if math.isfinite(ref_norm) and ref_norm > self.SimulationParams.EPSILON:
                reference_forward = np.array([rdx, rdy, rdz], dtype=float) / float(ref_norm)
                dot_val = float(np.clip(np.dot(reference_forward, desired_forward), -1.0, 1.0))
                angle = math.acos(dot_val)

                w_smooth = float(self.node.get_parameter("w_smooth").value)
                smoothness_cost = w_smooth * float(angle)

                if include_framing:
                    try:
                        fx = float(self.node.get_parameter("camera_fx").value)
                    except PARAMETER_READ_ERRORS:
                        fx = None

                    if fx is not None and fx > 0.0:
                        err_px = (
                            fx * math.tan(angle)
                            if abs(angle) < (math.pi / 2.0 - self.SimulationParams.EPSILON)
                            else fx * self.CostParams.LARGE_PENALTY
                        )
                        deadband = float(self.node.get_parameter("deadband_px").value)
                        delta_px = float(self.node.get_parameter("huber_delta_px").value)
                        err_px = 0.0 if abs(err_px) < deadband else err_px
                        w_framing = float(self.node.get_parameter("w_framing").value)
                        framing_cost = w_framing * float(self.huber_function(err_px, delta_px))

        return depth_cost, lighting_cost, smoothness_cost, framing_cost
