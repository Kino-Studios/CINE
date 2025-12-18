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
    waypoints_topic: str = "/move_base_simple/goal"
    odom_topic: str = "/odom_world"
    depth_topic: str = "/depth/image_raw"
    center_topic: str = "/object_detection/center_point"
    bbox_topic: str = "/object_detection/bounding_box"
    dimensions_topic: str = "/object_detection/dimensions"
    light_vector_topic: str = "/cinematography/light_vector"
    image_topic: str = "/camera_publisher/image_raw"
    bspline_in_topic: str = "drone_0_planning/bspline"
    bspline_out_topic: str = "drone_0_planning/cinematic_bspline"
    cinematic_offset_m: float = 0.5
    cinematic_smooth_alpha: float = 0.2
    min_altitude_m: float = 0.2
