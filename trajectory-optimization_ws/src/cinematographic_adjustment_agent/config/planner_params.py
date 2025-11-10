from dataclasses import dataclass

@dataclass
class CinematographyParams:
    target_focus_distance_m: float = 1.5
    min_focus_distance_m: float = 1.0
    constant_altitude_m: float = 1.5
    orbit_radius_m: float = 1.5
    angular_velocity_rps: float = 0.2
    look_duration_s: float = 3.0
    waypoint_rate_hz: float = 10.0
    num_initial_waypoints: int = 40
    image_width: int = 1152
    image_height: int = 648
    camera_fx: float = 525.0
    camera_fy: float = 525.0
    camera_cx: float = 576.0
    camera_cy: float = 324.0
    waypoints_topic: str = "cinematography/waypoints"

