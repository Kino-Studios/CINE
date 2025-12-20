import os
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
    bspline_in_topic: str = "/drone_0_planning/bspline"
    bspline_out_topic: str = "/drone_0_planning/cinematic_bspline"
    cinematic_offset_m: float = 0.5
    cinematic_smooth_alpha: float = 0.2
    min_altitude_m: float = 0.2

    @staticmethod
    def _env_float(name: str, default: float) -> float:
        value = os.getenv(name)
        if value is None or value == "":
            return default
        return float(value)

    @staticmethod
    def _env_int(name: str, default: int) -> int:
        value = os.getenv(name)
        if value is None or value == "":
            return default
        return int(float(value))

    @staticmethod
    def _env_str(name: str, default: str) -> str:
        value = os.getenv(name)
        if value is None or value == "":
            return default
        return str(value)

    @classmethod
    def from_env(cls) -> "CinematographyParams":
        base = cls()
        
        return cls(
            target_focus_distance_m=cls._env_float("CINEMATOGRAPHY_TARGET_FOCUS_DISTANCE_M", base.target_focus_distance_m),
            min_focus_distance_m=cls._env_float("CINEMATOGRAPHY_MIN_FOCUS_DISTANCE_M", base.min_focus_distance_m),
            constant_altitude_m=cls._env_float("CINEMATOGRAPHY_CONSTANT_ALTITUDE_M", base.constant_altitude_m),
            orbit_radius_m=cls._env_float("CINEMATOGRAPHY_ORBIT_RADIUS_M", base.orbit_radius_m),
            angular_velocity_rps=cls._env_float("CINEMATOGRAPHY_ANGULAR_VELOCITY_RPS", base.angular_velocity_rps),
            look_duration_s=cls._env_float("CINEMATOGRAPHY_LOOK_DURATION_S", base.look_duration_s),
            waypoint_rate_hz=cls._env_float("CINEMATOGRAPHY_WAYPOINT_RATE_HZ", base.waypoint_rate_hz),
            num_initial_waypoints=cls._env_int("CINEMATOGRAPHY_NUM_INITIAL_WAYPOINTS", base.num_initial_waypoints),
            image_width=cls._env_int("CINEMATOGRAPHY_IMAGE_WIDTH", base.image_width),
            image_height=cls._env_int("CINEMATOGRAPHY_IMAGE_HEIGHT", base.image_height),
            camera_fx=cls._env_float("CINEMATOGRAPHY_CAMERA_FX", base.camera_fx),
            camera_fy=cls._env_float("CINEMATOGRAPHY_CAMERA_FY", base.camera_fy),
            camera_cx=cls._env_float("CINEMATOGRAPHY_CAMERA_CX", base.camera_cx),
            camera_cy=cls._env_float("CINEMATOGRAPHY_CAMERA_CY", base.camera_cy),
            waypoints_topic=cls._env_str("CINEMATOGRAPHY_WAYPOINTS_TOPIC", base.waypoints_topic),
            odom_topic=cls._env_str("CINEMATOGRAPHY_ODOM_TOPIC", base.odom_topic),
            depth_topic=cls._env_str("CINEMATOGRAPHY_DEPTH_TOPIC", base.depth_topic),
            center_topic=cls._env_str("CINEMATOGRAPHY_CENTER_TOPIC", base.center_topic),
            bbox_topic=cls._env_str("CINEMATOGRAPHY_BBOX_TOPIC", base.bbox_topic),
            dimensions_topic=cls._env_str("CINEMATOGRAPHY_DIMENSIONS_TOPIC", base.dimensions_topic),
            light_vector_topic=cls._env_str("CINEMATOGRAPHY_LIGHT_VECTOR_TOPIC", base.light_vector_topic),
            image_topic=cls._env_str("CINEMATOGRAPHY_IMAGE_TOPIC", base.image_topic),
            bspline_in_topic=cls._env_str("CINEMATOGRAPHY_BSPLINE_IN_TOPIC", base.bspline_in_topic),
            bspline_out_topic=cls._env_str("CINEMATOGRAPHY_BSPLINE_OUT_TOPIC", base.bspline_out_topic),
            cinematic_offset_m=cls._env_float("CINEMATOGRAPHY_CINEMATIC_OFFSET_M", base.cinematic_offset_m),
            cinematic_smooth_alpha=cls._env_float("CINEMATOGRAPHY_CINEMATIC_SMOOTH_ALPHA", base.cinematic_smooth_alpha),
            min_altitude_m=cls._env_float("CINEMATOGRAPHY_MIN_ALTITUDE_M", base.min_altitude_m),
        )
