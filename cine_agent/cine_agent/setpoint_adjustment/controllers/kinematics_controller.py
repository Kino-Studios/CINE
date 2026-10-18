import math

import numpy as np
from constants import CinematicConstraints

from ..utils.config_loader import ConfigLoader


class KinematicsController:
    def __init__(self, config: ConfigLoader):
        self.config = config
        self.constraints = CinematicConstraints()
        self.smooth_pos = None
        self.smooth_vel = np.zeros(3, dtype=float)

    def next_kinematic_step(
        self, target_pos: np.ndarray, current_pos: np.ndarray, dt: float, has_focus: bool
    ) -> np.ndarray:
        if self.smooth_pos is None:
            self.smooth_pos = current_pos.copy()
            self.smooth_vel = np.zeros(3, dtype=float)

        max_vel = (
            self.constraints.FOCUS_MAX_VELOCITY_MPS
            if has_focus
            else self.constraints.DEFAULT_MAX_VELOCITY_MPS
        )
        max_acc = (
            self.constraints.FOCUS_MAX_ACCEL_MPS2
            if has_focus
            else self.constraints.DEFAULT_MAX_ACCEL_MPS2
        )

        disp = target_pos - self.smooth_pos
        dist = float(np.linalg.norm(disp))
        if dist < 1e-6:
            self.smooth_vel = np.zeros(3, dtype=float)

            return self.smooth_pos.copy()

        direction = disp / dist
        desired_speed = min(float(max_vel), math.sqrt(max(0.0, 2.0 * float(max_acc) * dist)))
        desired_vel = direction * desired_speed

        vel_error = desired_vel - self.smooth_vel
        vel_error_mag = float(np.linalg.norm(vel_error))
        if vel_error_mag > 1e-6:
            max_vel_change = float(max_acc) * dt
            if vel_error_mag <= max_vel_change:
                accel = vel_error / dt
            else:
                accel = (vel_error / vel_error_mag) * float(max_acc)
        else:
            accel = np.zeros(3, dtype=float)

        next_vel = self.smooth_vel + accel * dt
        vel_mag = float(np.linalg.norm(next_vel))
        if vel_mag > float(max_vel) and vel_mag > 0.0:
            next_vel = (next_vel / vel_mag) * float(max_vel)

        next_pos = self.smooth_pos + self.smooth_vel * dt + 0.5 * accel * dt * dt

        step = next_pos - self.smooth_pos
        step_mag = float(np.linalg.norm(step))
        if (
            self.config.max_waypoint_step_m > 0.0
            and step_mag > self.config.max_waypoint_step_m
            and step_mag > 0.0
        ):
            next_pos = self.smooth_pos + (step / step_mag) * float(self.config.max_waypoint_step_m)
            next_vel = (next_pos - self.smooth_pos) / dt

        if float(np.linalg.norm(next_pos - target_pos)) < 0.05:
            next_pos = target_pos.copy()
            next_vel = np.zeros(3, dtype=float)

        self.smooth_pos = next_pos
        self.smooth_vel = next_vel

        return next_pos.copy()
