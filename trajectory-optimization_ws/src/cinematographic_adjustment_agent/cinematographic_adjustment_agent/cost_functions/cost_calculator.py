import sys
import os
from pathlib import Path

# Add repo root to sys.path to find 'configs'
here = Path(__file__).resolve()
for parent in [here] + list(here.parents):
    if (parent / "configs").is_dir():
        python_constants_path = parent / "configs" / "python-constants"
        if str(python_constants_path) not in sys.path:
            sys.path.insert(0, str(python_constants_path))
        break

import math
from typing import Dict, Optional

import numpy as np
from geometry_msgs.msg import PolygonStamped, Vector3Stamped
from nav_msgs.msg import Odometry

from constants import ROSParams, UserRequestedParams, ImageParams, CostParams, LightOrientationParams


class CostCalculator:
    def __init__(self, node):
        self.node = node
        self.bounding_box = None
        self.dimensions = None
        self.center_point = None
        self.depth_image = None
        self.center_depth = None
        self.light_vector = None
        self.light_distance = None
        self._declare_parameter("light_distance_attenuation_m", 4.0)
        self.angular_velocity = None
        self.orientation = None
        self._prev_ang_vel = None
        self.ROSParams = ROSParams
        self.UserRequestedParams = UserRequestedParams
        self.ImageParams = ImageParams
        self.CostParams = CostParams
        self.LightOrientationParams = LightOrientationParams
        self._last_light_setpoint = np.array([0.0, 0.0, 0.0], dtype=float)
        self._declare_parameter("focus_object_id", UserRequestedParams.FOCUS_OBJECT_ID)
        self._declare_parameter("huber_delta_px", CostParams.HUBER_DELTA_PX)
        self._declare_parameter("deadband_px", CostParams.DEADBAND_PX)
        self._declare_parameter("w_framing", CostParams.W_FRAMING)
        self._declare_parameter("w_dist", CostParams.W_DIST)
        self._declare_parameter("w_near", CostParams.W_NEAR)
        self._declare_parameter("w_stab", CostParams.W_STAB)
        self._declare_parameter("w_light", CostParams.W_LIGHT)
        self._declare_parameter("w_smooth", CostParams.W_SMOOTH)
        self._declare_parameter("light_yaw_deadband_rad", LightOrientationParams.YAW_DEADBAND_RAD)
        self._declare_parameter("light_pitch_deadband_rad", LightOrientationParams.PITCH_DEADBAND_RAD)
        self._declare_parameter("light_roll_deadband_rad", LightOrientationParams.ROLL_DEADBAND_RAD)
        self._declare_parameter("light_pitch_weight_factor", LightOrientationParams.PITCH_WEIGHT_FACTOR)
        self._declare_parameter("light_roll_weight_factor", LightOrientationParams.ROLL_WEIGHT_FACTOR)
        self._declare_parameter("light_pitch_max_rad", LightOrientationParams.PITCH_MAX_RAD)
        self._declare_parameter("light_pitch_damping_factor", LightOrientationParams.PITCH_DAMPING_FACTOR)
        self._declare_parameter("light_setpoint_smooth_alpha", LightOrientationParams.SETPOINT_SMOOTH_ALPHA)
        self._declare_parameter("light_planar_norm_threshold", LightOrientationParams.PLANAR_NORM_THRESHOLD)
        self._declare_parameter("light_yaw_change_limit_rad", LightOrientationParams.YAW_CHANGE_LIMIT_RAD)
        self._declare_parameter("light_pitch_change_limit_rad", 0.15)
        self._declare_parameter("light_roll_gain", 1.0)
        self._declare_parameter("light_pitch_gain", 0.3)
        self._declare_parameter("light_yaw_gain", 2.5)
        self._declare_parameter("light_distance_bias_m", 0.1)
        self._declare_parameter("light_distance_gain", 1.5)
        self._declare_parameter("light_cost_increase_when_close", True)

    def _declare_parameter(self, name: str, default):
        try:
            self.node.declare_parameter(name, default)
        except Exception:
            return

    def _image_size(self) -> tuple[float, float]:
        try:
            w = float(self.node.get_parameter('image_width').value)
            h = float(self.node.get_parameter('image_height').value)
            return max(1.0, w), max(1.0, h)
        except Exception:
            return float(self.ImageParams.IMAGE_WIDTH), float(self.ImageParams.IMAGE_HEIGHT)

    def update_light_vector(self, msg: Vector3Stamped):
        vector = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=float)
        norm = np.linalg.norm(vector)
        self.light_vector = vector / norm if norm > 0 else None
        self.light_distance = float(norm) if norm > 0 else None

    def _light_attenuation(self) -> float:
        if self.light_distance is None:
            return 1.0
        try:
            max_range = float(self.node.get_parameter("light_distance_attenuation_m").value)
        except Exception:
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
        self.angular_velocity = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ], dtype=float)
        self.orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ], dtype=float)
    
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

        forward = np.array([r00, r10, r20], dtype=float)
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

        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
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
            else:
                self.center_depth = None
    
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
    
    def get_median_depth_roi(self):
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
        
        depth_region = self.depth_image[y_min:y_max+1, x_min:x_max+1]
        
        valid_depths = depth_region[(depth_region > 0) & ~np.isnan(depth_region)]
        
        if len(valid_depths) == 0:
            return None
        
        median_depth = float(np.median(valid_depths))
        
        return median_depth
    
    def huber_function(self, r, delta=6.0):
        a = abs(r)
        return 0.5 * math.pow(a, 2) if a <= delta else delta * (a - 0.5 * delta)
    
    def calculate_cost(self) -> Optional[Dict[str, float]]:
        if self.depth_image is None:
            return None

        if self.bounding_box is None and self.center_point is None:
            return None

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
                return None
        else:
            return None
        
        deadband = float(self.node.get_parameter('deadband_px').value)
        delta_px = float(self.node.get_parameter('huber_delta_px').value)
        
        du = u_current - u_star
        du = 0.0 if abs(du) < deadband else du
        
        dv = v_current - v_star
        dv = 0.0 if abs(dv) < deadband else dv
        
        w_framing = float(self.node.get_parameter('w_framing').value)
        framing_cost = w_framing * (self.huber_function(du, delta_px) + self.huber_function(dv, delta_px))

        d_star = self.UserRequestedParams.DESIRED_FOCUS_DISTANCE_M
        d_min = self.UserRequestedParams.FOCUS_DISTANCE_MIN_M

        Z = self.get_median_depth_roi()
        if Z is None:
            if self.center_depth is None:
                return None
            Z = float(self.center_depth)
        Z = float(Z)
        
        J_dist = math.pow(Z - d_star, 2)
        J_near = math.pow(d_min - Z, 2) if Z < d_min else 0.0
        if not hasattr(self, "_prev_Z"):
            self._prev_Z = Z
        J_stab = math.pow(Z - self._prev_Z, 2)
        self._prev_Z = Z

        w_dist = float(self.node.get_parameter('w_dist').value)
        w_near = float(self.node.get_parameter('w_near').value)
        w_stab = float(self.node.get_parameter('w_stab').value)
        depth_cost = (w_dist * J_dist) + (w_near * J_near) + (w_stab * J_stab)

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
        attenuation = self._light_attenuation()
        w_light = float(self.node.get_parameter('w_light').value)
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
            if target_norm > 1e-6:
                target_dir /= target_norm
            planar_norm = math.hypot(target_dir[0], target_dir[1])
            planar_deadzone = float(self.node.get_parameter("light_planar_norm_threshold").value)
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
            raw_setpoints = np.array([
                raw_roll * roll_gain,
                raw_pitch * pitch_gain,
                raw_yaw * yaw_gain
            ], dtype=float)
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
            self._last_light_setpoint = smooth_setpoints
            max_pitch = float(self.node.get_parameter("light_pitch_max_rad").value)
            pitch_setpoint = float(np.clip(pitch_setpoint, -abs(max_pitch), abs(max_pitch)))
            accel_factor = float(self.node.get_parameter("light_pitch_damping_factor").value)
            ang_vel_norm = float(np.linalg.norm(self.angular_velocity)) if self.angular_velocity is not None else 0.0
            damping = 1.0 / (1.0 + accel_factor * ang_vel_norm)
            pitch_setpoint *= damping
            current_euler = self._quaternion_to_euler(self.orientation) if self.orientation is not None else None
            if current_euler is not None:
                roll_val, pitch_val, yaw_val = current_euler
                actual_roll = roll_val
                actual_pitch = pitch_val
                actual_yaw = yaw_val
                yaw_error = self._angle_diff(yaw_setpoint, yaw_val)
                pitch_error = self._angle_diff(pitch_setpoint, pitch_val)
                roll_error = self._angle_diff(roll_setpoint, roll_val)
                yaw_error_cost = self._deadband_error(yaw_error, yaw_deadband)
                pitch_error_cost = self._deadband_error(pitch_error, pitch_deadband)
                roll_error_cost = self._deadband_error(roll_error, roll_deadband)
                lighting_cost = attenuation * (
                    w_light * yaw_error_cost
                    + w_pitch * pitch_error_cost
                    + w_roll * roll_error_cost
                )
            else:
                forward = self._camera_forward_vector()
                if forward is not None:
                    dot_val = float(np.clip(np.dot(forward, self.light_vector), -1.0, 1.0))
                    angle = math.acos(dot_val)
            distance_bias = float(self.node.get_parameter("light_distance_bias_m").value)
            distance_bias = max(1e-3, distance_bias)
            distance_gain = float(self.node.get_parameter("light_distance_gain").value)
            distance_gain = max(0.0, distance_gain)
            distance_factor = 1.0 / max(self.light_distance, distance_bias) if self.light_distance is not None else 1.0
            lighting_cost = w_light * (math.pi - angle) * attenuation * (1.0 + distance_gain * distance_factor)

        smoothness_cost = 0.0
        if self.angular_velocity is not None:
            if self._prev_ang_vel is None:
                self._prev_ang_vel = self.angular_velocity.copy()

            delta_ang = self.angular_velocity - self._prev_ang_vel
            w_smooth = float(self.node.get_parameter('w_smooth').value)
            smoothness_cost = w_smooth * float(np.linalg.norm(delta_ang))
            self._prev_ang_vel = self.angular_velocity.copy()

        total_cost = framing_cost + depth_cost + lighting_cost + smoothness_cost
        
        return {
            'framing_cost': float(framing_cost),
            'distance_cost': float(depth_cost),
            'lighting_cost': float(lighting_cost),
            'smoothness_cost': float(smoothness_cost),
            'total_cost': float(total_cost),
            'yaw_setpoint': float(yaw_setpoint),
            'pitch_setpoint': float(pitch_setpoint),
            'roll_setpoint': float(roll_setpoint),
            'yaw_error': float(yaw_error),
            'pitch_error': float(pitch_error),
            'roll_error': float(roll_error),
            'actual_yaw': float(actual_yaw),
            'actual_pitch': float(actual_pitch),
            'actual_roll': float(actual_roll),
        }
    
    def estimate_cost_at_position(self, drone_x: float, drone_y: float, drone_z: float, 
                                  target_world_x: float, target_world_y: float, target_world_z: float) -> Optional[float]:

        dx = target_world_x - drone_x
        dy = target_world_y - drone_y
        dz = target_world_z - drone_z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        d_star = self.UserRequestedParams.DESIRED_FOCUS_DISTANCE_M
        d_min = self.UserRequestedParams.FOCUS_DISTANCE_MIN_M
        
        J_dist = math.pow(distance - d_star, 2)
        J_near = math.pow(d_min - distance, 2) if distance < d_min else 0.0
        
        if hasattr(self, "_prev_Z") and self._prev_Z is not None:
            J_stab = 0.0
        else:
            prev_Z = getattr(self, "_prev_Z", distance)
            J_stab = math.pow(distance - prev_Z, 2)
        
        w_dist = float(self.node.get_parameter('w_dist').value)
        w_near = float(self.node.get_parameter('w_near').value)
        w_stab = float(self.node.get_parameter('w_stab').value)
        depth_cost = (w_dist * J_dist) + (w_near * J_near) + (w_stab * J_stab)
        
        img_w, img_h = self._image_size()

        if self.center_point is not None and len(self.center_point.polygon.points) > 0:
            u_current = float(self.center_point.polygon.points[0].x)
            v_current = float(self.center_point.polygon.points[0].y)
        elif self.bounding_box is not None and len(self.bounding_box.polygon.points) >= 2:
            points = self.bounding_box.polygon.points
            u_current = float(np.mean([p.x for p in points]))
            v_current = float(np.mean([p.y for p in points]))
        else:
            u_current = img_w / 2.0
            v_current = img_h / 2.0

        u_star = img_w * self.UserRequestedParams.THIRDS_ANCHOR[0]
        v_star = img_h * self.UserRequestedParams.THIRDS_ANCHOR[1]
        
        du_est = u_current - u_star
        dv_est = v_current - v_star
        
        deadband = float(self.node.get_parameter('deadband_px').value)
        delta_px = float(self.node.get_parameter('huber_delta_px').value)
        
        du_est = 0.0 if abs(du_est) < deadband else du_est
        dv_est = 0.0 if abs(dv_est) < deadband else dv_est
        
        w_framing = float(self.node.get_parameter('w_framing').value)
        framing_cost = w_framing * (self.huber_function(du_est, delta_px) + self.huber_function(dv_est, delta_px))
        
        total_cost = depth_cost + framing_cost
        return total_cost

    def estimate_full_cost_at_position(
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
        """
        Model-based cost estimate for candidate waypoint selection.

        Unlike `calculate_cost()` (which scores the *current* observed image),
        this estimates how the cost terms would change if the drone were to move
        to (drone_x, drone_y, drone_z), using only geometric information.
        """

        dx = target_world_x - drone_x
        dy = target_world_y - drone_y
        dz = target_world_z - drone_z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if not math.isfinite(distance) or distance <= 1e-6:
            return None

        d_star = self.UserRequestedParams.DESIRED_FOCUS_DISTANCE_M
        d_min = self.UserRequestedParams.FOCUS_DISTANCE_MIN_M

        j_dist = (distance - d_star) ** 2
        j_near = (d_min - distance) ** 2 if distance < d_min else 0.0

        w_dist = float(self.node.get_parameter("w_dist").value)
        w_near = float(self.node.get_parameter("w_near").value)
        w_stab = float(self.node.get_parameter("w_stab").value)
        j_stab = 0.0
        if reference_drone_x is not None and reference_drone_y is not None and reference_drone_z is not None:
            pdx = target_world_x - reference_drone_x
            pdy = target_world_y - reference_drone_y
            pdz = target_world_z - reference_drone_z
            prev_distance = math.sqrt(pdx * pdx + pdy * pdy + pdz * pdz)
            if math.isfinite(prev_distance) and prev_distance > 1e-6:
                j_stab = (distance - prev_distance) ** 2

        depth_cost = (w_dist * j_dist) + (w_near * j_near) + (w_stab * j_stab)
        attenuation = self._light_attenuation()

        desired_forward = np.array([dx, dy, dz], dtype=float) / float(distance)

        lighting_cost = 0.0
        if self.light_vector is not None:
            dot_val = float(np.clip(np.dot(desired_forward, self.light_vector), -1.0, 1.0))
            angle = math.acos(dot_val)
            w_light = float(self.node.get_parameter("w_light").value)
            lighting_cost = w_light * float(math.pi - angle) * attenuation

        smoothness_cost = 0.0
        framing_cost = 0.0
        if reference_drone_x is not None and reference_drone_y is not None and reference_drone_z is not None:
            rdx = target_world_x - reference_drone_x
            rdy = target_world_y - reference_drone_y
            rdz = target_world_z - reference_drone_z
            ref_norm = math.sqrt(rdx * rdx + rdy * rdy + rdz * rdz)
            if math.isfinite(ref_norm) and ref_norm > 1e-6:
                reference_forward = np.array([rdx, rdy, rdz], dtype=float) / float(ref_norm)
                dot_val = float(np.clip(np.dot(reference_forward, desired_forward), -1.0, 1.0))
                angle = math.acos(dot_val)

                w_smooth = float(self.node.get_parameter("w_smooth").value)
                smoothness_cost = w_smooth * float(angle)

                # "Framing" proxy: penalize large changes in viewing direction required
                # to keep the subject centered/stable (approx. pixel displacement).
                try:
                    fx = float(self.node.get_parameter("camera_fx").value)
                except Exception:
                    fx = None

                if fx is not None and fx > 0.0:
                    err_px = fx * math.tan(angle) if abs(angle) < (math.pi / 2.0 - 1e-3) else fx * 1e3
                    deadband = float(self.node.get_parameter("deadband_px").value)
                    delta_px = float(self.node.get_parameter("huber_delta_px").value)
                    err_px = 0.0 if abs(err_px) < deadband else err_px
                    w_framing = float(self.node.get_parameter("w_framing").value)
                    framing_cost = w_framing * float(self.huber_function(err_px, delta_px))

        return float(depth_cost + framing_cost + lighting_cost + smoothness_cost)
