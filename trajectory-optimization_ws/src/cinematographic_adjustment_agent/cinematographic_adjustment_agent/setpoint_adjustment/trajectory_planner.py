import math
import numpy as np
from typing import List, Tuple


def generate_orbit_waypoints_xy(
    center_xy_m: Tuple[float, float],
    radius_m: float,
    angular_velocity_rps: float,
    duration_s: float,
    waypoint_rate_hz: float,
) -> List[Tuple[float, float, float]]:
    cx, cy = center_xy_m
    num_points = max(1, int(duration_s * waypoint_rate_hz))
    dt = 1.0 / waypoint_rate_hz
    waypoints = []
    theta = 0.0
    for _ in range(num_points):
        x = cx + radius_m * math.cos(theta)
        y = cy + radius_m * math.sin(theta)
        waypoints.append((x, y, dt))
        theta += 2.0 * math.pi * angular_velocity_rps * dt
    return waypoints

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

