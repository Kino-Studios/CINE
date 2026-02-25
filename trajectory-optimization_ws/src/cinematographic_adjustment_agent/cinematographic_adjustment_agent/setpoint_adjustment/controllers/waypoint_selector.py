import math
from typing import Optional, List, Tuple, Callable
import numpy as np
from rclpy.node import Node

from ..utils.config_loader import ConfigLoader
from ..utils.trajectory_planner import (
    generate_orbit_waypoints_xy,
    generate_cost_optimized_orbit_waypoints,
    optimize_orbit_center_and_radius,
)

class WaypointSelector:
    def __init__(self, node: Node, config: ConfigLoader):
        self.node = node
        self.config = config
        self.phase_theta = 0.0

        self.use_cost_optimization = True
        self.optimization_frequency = 2.0
        self.last_optimization_time = 0.0
        self.optimized_center: Optional[Tuple[float, float]] = None
        self.optimized_radius: Optional[float] = None

    def select_next_waypoint(
        self,
        center_xy: Tuple[float, float],
        radius_m: float,
        altitude_m: float,
        target_world: np.ndarray,
        drone_position: np.ndarray,
        cost_estimation_func: Callable[[float, float, float, float, float, float, float, float, float], Optional[float]],
        dt: float
    ) -> Optional[np.ndarray]:

        cx, cy = center_xy
        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])

        def cost_function(x: float, y: float, z: float) -> Optional[float]:
            return cost_estimation_func(
                x,
                y,
                z,
                tx,
                ty,
                tz,
                float(drone_position[0]),
                float(drone_position[1]),
                float(drone_position[2]),
            )

        self.phase_theta += 2.0 * math.pi * float(self.config.params.ANGULAR_VELOCITY_RPS) * dt

        angle_samples = max(1, self.config.candidate_angle_samples)
        window = float(max(0.0, self.config.candidate_angle_window_rad))
        angle_offsets = np.linspace(-window, window, angle_samples) if angle_samples > 1 else np.array([0.0])

        r_min, r_max = float(self.config.candidate_radius_range_m[0]), float(self.config.candidate_radius_range_m[1])
        r_min, r_max = (min(r_min, r_max), max(r_min, r_max))
        base_r = float(radius_m)
        r_low = max(0.1, min(base_r, base_r * 1.5, r_max))
        r_high = min(max(base_r, base_r * 0.7, r_min), r_max)

        if r_high < r_low:
            r_low, r_high = r_min, r_max

        radius_samples = max(1, self.config.candidate_radius_samples)
        radii = np.linspace(r_low, r_high, radius_samples) if radius_samples > 1 else np.array([base_r])

        best = None
        best_cost = float("inf")

        for da in angle_offsets:
            theta = self.phase_theta + float(da)
            ct, st = math.cos(theta), math.sin(theta)

            for rr in radii:
                x = float(cx + rr * ct)
                y = float(cy + rr * st)
                z = float(altitude_m)

                cost = cost_function(x, y, z)
                if cost is None or not math.isfinite(cost):
                    continue

                if cost < best_cost:
                    best_cost = float(cost)
                    best = np.array([x, y, z], dtype=float)

        return best

    def build_initial_waypoints(
        self,
        target_world: np.ndarray,
        drone_position: np.ndarray,
        cost_estimation_func: Callable[[float, float, float, float, float, float, float, float, float], Optional[float]]
    ) -> List[Tuple[float, float, float, float]]:

        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])

        if self.optimized_center is not None and self.optimized_radius is not None:
            center_xy = self.optimized_center
            radius = self.optimized_radius
        else:
            center_xy = (tx, ty)
            radius = self.config.params.ORBIT_RADIUS_M

        def cost_function(x: float, y: float, z: float) -> Optional[float]:
            return cost_estimation_func(
                x,
                y,
                z,
                tx,
                ty,
                tz,
                float(drone_position[0]),
                float(drone_position[1]),
                float(drone_position[2]),
            )

        z = self.config.params.CONSTANT_ALTITUDE_M

        if self.use_cost_optimization:
            try:
                orbit = generate_cost_optimized_orbit_waypoints(
                    center_xy_m=center_xy,
                    initial_radius_m=radius,
                    angular_velocity_rps=self.config.params.ANGULAR_VELOCITY_RPS,
                    duration_s=float(self.config.params.NUM_INITIAL_WAYPOINTS) / self.config.params.WAYPOINT_RATE_HZ,
                    waypoint_rate_hz=self.config.params.WAYPOINT_RATE_HZ,
                    altitude_m=z,
                    cost_function=cost_function,
                    num_radius_samples=5,
                    radius_range=(0.8, 2.0),
                )

                return orbit
            except Exception as e:
                self.node.get_logger().warn(f'Optimization cost failed: {e}')

        orbit = generate_orbit_waypoints_xy(
            center_xy_m=center_xy,
            radius_m=radius,
            angular_velocity_rps=self.config.params.ANGULAR_VELOCITY_RPS,
            duration_s=float(self.config.params.NUM_INITIAL_WAYPOINTS) / self.config.params.WAYPOINT_RATE_HZ,
            waypoint_rate_hz=self.config.params.WAYPOINT_RATE_HZ,
            altitude_m=z,
        )

        return orbit

    def optimize_orbit_parameters_if_needed(
        self,
        current_time: float,
        target_world: np.ndarray,
        drone_position: np.ndarray,
        cost_estimation_func: Callable[[float, float, float, float, float, float, float, float, float], Optional[float]]
    ) -> None:
        if not self.use_cost_optimization or (current_time - self.last_optimization_time) < self.optimization_frequency:
            return

        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])

        def cost_function(x: float, y: float, z: float) -> Optional[float]:
            return cost_estimation_func(
                x,
                y,
                z,
                tx,
                ty,
                tz,
                float(drone_position[0]),
                float(drone_position[1]),
                float(drone_position[2]),
            )

        try:
            initial_center = (tx, ty)
            opt_center, opt_radius = optimize_orbit_center_and_radius(
                initial_center_xy=initial_center,
                initial_radius=self.config.params.ORBIT_RADIUS_M,
                altitude_m=self.config.params.CONSTANT_ALTITUDE_M,
                cost_function=cost_function,
                search_radius=0.5,
                num_samples=9,
            )

            self.optimized_center = opt_center
            self.optimized_radius = opt_radius
            self.last_optimization_time = current_time
            self.node.get_logger().info(
                f'Optimized orbit: center=({opt_center[0]:.2f}, {opt_center[1]:.2f}), '
                f'radius={opt_radius:.2f}m'
            )
        except Exception as e:
            self.node.get_logger().warn(f'Orbit optimization failed: {e}')
