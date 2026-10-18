import math
from typing import Callable, List, Optional, Tuple

import numpy as np


def generate_orbit_waypoints_xy(
    center_xy_m: Tuple[float, float],
    radius_m: float,
    angular_velocity_rps: float,
    duration_s: float,
    waypoint_rate_hz: float,
    altitude_m: float,
) -> List[Tuple[float, float, float, float]]:
    cx, cy = center_xy_m
    num_points = max(1, int(duration_s * waypoint_rate_hz))
    dt = 1.0 / waypoint_rate_hz
    waypoints = []
    theta = 0.0

    for _ in range(num_points):
        x = cx + radius_m * math.cos(theta)
        y = cy + radius_m * math.sin(theta)
        waypoints.append((x, y, altitude_m, dt))
        theta += 2.0 * math.pi * angular_velocity_rps * dt

    return waypoints


def generate_cost_optimized_orbit_waypoints(
    center_xy_m: Tuple[float, float],
    initial_radius_m: float,
    angular_velocity_rps: float,
    duration_s: float,
    waypoint_rate_hz: float,
    altitude_m: float,
    cost_function: Callable[[float, float, float], Optional[float]],
    num_radius_samples: int = 5,
    radius_range: Tuple[float, float] = (0.8, 2.0),
) -> List[Tuple[float, float, float, float]]:

    cx, cy = center_xy_m
    num_points = max(1, int(duration_s * waypoint_rate_hz))
    dt = 1.0 / waypoint_rate_hz
    waypoints = []

    theta = 0.0
    current_radius = initial_radius_m

    for i in range(num_points):
        radius_candidates = np.linspace(
            max(radius_range[0], current_radius * 0.7),
            min(radius_range[1], current_radius * 1.3),
            num_radius_samples,
        )

        best_radius = current_radius
        best_cost = float("inf")

        for test_radius in radius_candidates:
            test_x = cx + test_radius * math.cos(theta)
            test_y = cy + test_radius * math.sin(theta)

            cost = cost_function(test_x, test_y, altitude_m)
            if cost is not None and cost < best_cost:
                best_cost = cost
                best_radius = test_radius

        current_radius = best_radius
        x = cx + current_radius * math.cos(theta)
        y = cy + current_radius * math.sin(theta)
        waypoints.append((x, y, altitude_m, dt))

        theta += 2.0 * math.pi * angular_velocity_rps * dt

    return waypoints


def optimize_orbit_center_and_radius(
    initial_center_xy: Tuple[float, float],
    initial_radius: float,
    altitude_m: float,
    cost_function: Callable[[float, float, float], Optional[float]],
    search_radius: float = 0.5,
    num_samples: int = 9,
) -> Tuple[Tuple[float, float], float]:
    best_center = initial_center_xy
    best_radius = initial_radius
    best_cost = float("inf")

    cx, cy = initial_center_xy
    x_samples = np.linspace(cx - search_radius, cx + search_radius, int(np.sqrt(num_samples)))
    y_samples = np.linspace(cy - search_radius, cy + search_radius, int(np.sqrt(num_samples)))

    radius_candidates = np.linspace(max(0.5, initial_radius * 0.7), initial_radius * 1.3, 5)

    for test_x in x_samples:
        for test_y in y_samples:
            for test_radius in radius_candidates:
                cost = cost_function(test_x, test_y, altitude_m)

                if cost is not None and cost < best_cost:
                    best_cost = cost
                    best_center = (float(test_x), float(test_y))
                    best_radius = float(test_radius)

    return best_center, best_radius


def pixel_to_camera_xy(
    u: float,
    v: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Tuple[float, float]:
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy

    return float(x), float(y)


def clamp_depth_to_preference(depth_m: float | None, preferred_m: float, min_m: float) -> float:
    if depth_m is None:
        return max(preferred_m, min_m)

    return max(depth_m, min_m)
