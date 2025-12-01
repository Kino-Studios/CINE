from dataclasses import dataclass

@dataclass
class CinematographyParams:
    TARGET_FOCUS_DISTANCE_M: float = 1.5
    MIN_FOCUS_DISTANCE_M: float = 1.0
    CONSTANT_ALTITUDE_M: float = 1.5
    ORBIT_RADIUS_M: float = 1.5
    ANGULAR_VELOCITY_RPS: float = 0.2
    LOOK_DURATION_S: float = 3.0
    WAYPOINT_RATE_HZ: float = 10.0
    NUM_INITIAL_WAYPOINTS: int = 40
    IMAGE_WIDTH: int = 1152
    IMAGE_HEIGHT: int = 648
    CAMERA_FX: float = 525.0
    CAMERA_FY: float = 525.0
    CAMERA_CX: float = 576.0
    CAMERA_CY: float = 324.0
    WAYPOINTS_TOPIC: str = "cinematography/waypoints"

