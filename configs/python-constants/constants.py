import os
from dataclasses import dataclass, field
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

try:
    from .config_loader import ConfigLoader
except (ImportError, ValueError):
    from config_loader import ConfigLoader

config = ConfigLoader()

@dataclass
class CinematicConstraints:
    # kinematic constraints
    DEFAULT_MAX_VELOCITY_MPS: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "default_max_velocity_mps", 1.55))
    DEFAULT_MAX_ACCEL_MPS2: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "default_max_accel_mps2", 3.7))
    DEFAULT_SPEED_SCALE: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "default_speed_scale", 0.8))
    VEL_DAMPING: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "vel_damping", 0.08))
    ACC_DAMPING: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "acc_damping", 0.05))
    MIN_DT: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "min_dt", 0.05))
    USE_CINEMATIC: bool = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "use_cinematic", True))
    FRAMING_ACTIVE: bool = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "framing_active", False))

    # focus constraints
    FOCUS_MAX_VELOCITY_MPS: float = field(default_factory=lambda: config.get("focus", "focus_velocity_constraints", "focus_max_velocity_mps", 0.7))
    FOCUS_MAX_ACCEL_MPS2: float = field(default_factory=lambda: config.get("focus", "focus_velocity_constraints", "focus_max_accel_mps2", 2.5))
    FOCUS_SPEED_SCALE: float = field(default_factory=lambda: config.get("focus", "focus_velocity_constraints", "focus_speed_scale", 0.7))
    YAWDOT_MAX_FRAMING: float = field(default_factory=lambda: config.get("focus", "focus_yaw_constraints", "yawdot_max_framing", 0.6))
    YAW_LPF_ALPHA_FRAMING: float = field(default_factory=lambda: config.get("focus", "focus_yaw_constraints", "yaw_lpf_alpha_framing", 0.9))

    # yaw and target constraints
    YAWDOT_MAX: float = field(default_factory=lambda: config.get("defaults", "yaw_constraints", "yawdot_max", 1.25))
    YAW_LPF_ALPHA: float = field(default_factory=lambda: config.get("defaults", "yaw_constraints", "yaw_lpf_alpha", 0.85))
    USE_TARGET_YAW: bool = field(default_factory=lambda: config.get("defaults", "yaw_constraints", "use_target_yaw", True))
    ALWAYS_FACE_TARGET: bool = field(default_factory=lambda: config.get("defaults", "yaw_constraints", "always_face_target", True))

    # target distances
    TARGET_DISTANCE_M: float = field(default_factory=lambda: config.get("defaults", "target_distance", "target_distance_m", 1.0))
    TARGET_DISTANCE_TOL_M: float = field(default_factory=lambda: config.get("defaults", "target_distance", "target_distance_tol_m", 0.05))
    TARGET_DISTANCE_GATE_M: float = field(default_factory=lambda: config.get("defaults", "target_distance", "target_distance_gate_m", 1.0))

    # light avoidance parameters
    LIGHT_AVOID_GAIN: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_avoid_gain", 2.5))
    LIGHT_INFLUENCE_DISTANCE_M: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_influence_distance_m", 4.0))
    LIGHT_AVOID_SPEED_THRESHOLD_MPS: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_avoid_speed_threshold_mps", 3.0))
    LIGHT_AVOID_BLEND_BASE: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_avoid_blend_base", 0.05))
    LIGHT_AVOID_MAX_BLEND: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_avoid_max_blend", 0.25))
    LIGHT_AVOID_TRANSITION_S: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_avoid_transition_s", 6.0))
    LIGHT_COOLDOWN_AFTER_FRAMING_S: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_cooldown_after_framing_s", 0.7))
    LIGHT_DISABLE_NEAR_FINISH_S: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_disable_near_finish_s", 0.1))
    LIGHT_DISTANCE_GAIN: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_distance_gain", 1.5))
    LIGHT_DISTANCE_BLEND_GAIN: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_distance_blend_gain", 1.2))
    LIGHT_ROTATION_DAMPING: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_rotation_damping", 0.85))
    LIGHT_YAW_SMOOTH_ALPHA: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_yaw_smooth_alpha", 0.7))

class Constraints:
    MAX_VELOCITY = config.get("defaults", "kinematic_constraints", "max_velocity", 1.5)
    MAX_ACCELERATION = config.get("defaults", "kinematic_constraints", "max_acceleration", 10.0)

class ROSParams:
    QOS_PROFILE = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )

class UserRequestedParams:
    FOCUS_OBJECT_ID = config.get("focus", "focus_parameters", "focus_object_id", 2)
    THIRDS_ANCHOR = config.get("focus", "focus_parameters", "thirds_anchor", [2 / 3.0, 1 / 3.0])
    THIRDS_STRENGTH = config.get("focus", "focus_parameters", "thirds_strength", 0.6)
    THIRDS_USE_NEAREST_ANCHOR = config.get("focus", "focus_parameters", "thirds_use_nearest_anchor", True)

    DESIRED_FOCUS_DISTANCE_M = config.get("focus", "focus_distance", "desired_focus_distance_m", 1.0)
    FOCUS_DISTANCE_MIN_M = config.get("focus", "focus_distance", "focus_distance_min_m", 1.0)

class ImageParams:
    IMAGE_WIDTH = config.get("camera", "camera_params", "image_width")
    IMAGE_HEIGHT = config.get("camera", "camera_params", "image_height")

class WaypointParams:
    # mission control and frame settings
    FRAME_ID = config.get("waypoints", "waypoint_generator_params", "frame_id", "world")
    ENABLE_COST_OPTIMIZATION = config.get("waypoints", "waypoint_generator_params", "enable_cost_optimization", True)
    USE_DYNAMIC_WAYPOINTS_ON_DETECTION = config.get("waypoints", "waypoint_generator_params", "use_dynamic_waypoints_on_detection", True)
    
    # waypoint generation settings
    WAYPOINT_LOOKAHEAD = config.get("waypoints", "waypoint_generator_params", "waypoint_lookahead", 2)
    DYNAMIC_WAYPOINT_LOOKAHEAD = config.get("waypoints", "waypoint_generator_params", "dynamic_waypoint_lookahead", 3)
    DISTANCE_THRESHOLD_M = config.get("waypoints", "waypoint_generator_params", "distance_threshold_m", 0.5)
    PROGRESS_TIMEOUT_S = config.get("waypoints", "waypoint_generator_params", "progress_timeout_s", 4.0)
    MIN_GOAL_DISTANCE_M = config.get("waypoints", "waypoint_generator_params", "min_goal_distance_m", 0.5)
    MIN_FIRST_WAYPOINT_DISTANCE_M = config.get("waypoints", "waypoint_generator_params", "min_first_waypoint_distance_m", 0.5)
    ENABLE_PROGRESS_SKIP = config.get("waypoints", "waypoint_generator_params", "enable_progress_skip", False)
    MIN_ORBIT_RADIUS_M = config.get("waypoints", "waypoint_generator_params", "min_orbit_radius_m", 0.2)
    WAYPOINT_RADIUS_M = config.get("waypoints", "waypoint_generator_params", "waypoint_radius_m", 0.19)
    MAX_WAYPOINT_JUMP_M = config.get("waypoints", "waypoint_generator_params", "max_waypoint_jump_m", 3.0)
    MIN_WAYPOINT_SEPARATION_M = config.get("waypoints", "waypoint_generator_params", "min_waypoint_separation_m", 0.5)
    
    # bezier smoothing
    BEZIER_BLEND_LENGTH_M = config.get("waypoints", "waypoint_generator_params", "bezier_blend_length_m", 0.1)
    BEZIER_TRANSITION_POINTS = config.get("waypoints", "waypoint_generator_params", "bezier_transition_points", 6)
    BEZIER_MIN_SPEED_MPS = config.get("waypoints", "waypoint_generator_params", "bezier_min_speed_mps", 0.05)
    
    # anchors or arcs
    ANCHOR_START_INDEX = config.get("waypoints", "waypoint_generator_params", "anchor_start_index", -1)
    ANCHOR_END_INDEX = config.get("waypoints", "waypoint_generator_params", "anchor_end_index", -1)
    ANCHOR_INDEX_WINDOW = config.get("waypoints", "waypoint_generator_params", "anchor_index_window", 2)
    ANCHOR_MIN_GAP = config.get("waypoints", "waypoint_generator_params", "anchor_min_gap", 2)
    ARC_WAYPOINT_SPACING_M = config.get("waypoints", "waypoint_generator_params", "arc_waypoint_spacing_m", 1.0)
    
    # visualization
    ANCHOR_MARKER_SCALE_M = config.get("waypoints", "waypoint_generator_params", "anchor_marker_scale_m", 0.28)
    TARGET_MARKER_SCALE_M = config.get("waypoints", "waypoint_generator_params", "target_marker_scale_m", 0.32)
    
    # waypoint scaling
    WAYPOINT_XY_SCALE = config.get("waypoints", "waypoint_generator_params", "waypoint_xy_scale", 1.0)
    WAYPOINT_Y_EXTRA_SCALE = config.get("waypoints", "waypoint_generator_params", "waypoint_y_extra_scale", 1.0)
    WAYPOINT_Z_SCALE = config.get("waypoints", "waypoint_generator_params", "waypoint_z_scale", 1.0)
    
    # zed constraints
    ENABLE_Z_CLAMP = config.get("waypoints", "waypoint_generator_params", "enable_z_clamp", False)
    Z_CEILING_OVERRIDE_M = config.get("waypoints", "waypoint_generator_params", "z_ceiling_override_m", -1.0)

    # candidate generation parameters
    CANDIDATE_ANGLE_WINDOW_RAD = config.get("waypoints", "waypoint_generator_params", "candidate_angle_window_rad", 0.6)
    CANDIDATE_ANGLE_SAMPLES = config.get("waypoints", "waypoint_generator_params", "candidate_angle_samples", 7)
    CANDIDATE_RADIUS_SAMPLES = config.get("waypoints", "waypoint_generator_params", "candidate_radius_samples", 5)
    CANDIDATE_RADIUS_RANGE_M = config.get("waypoints", "waypoint_generator_params", "candidate_radius_range_m", [0.8, 2.0])
    MAX_WAYPOINT_STEP_M = config.get("waypoints", "waypoint_generator_params", "max_waypoint_step_m", 0.5)

class BsplineFilterParams:
    # focus parameters
    FOCUS_RADIUS_M = config.get("defaults", "bspline_filter", "focus_radius_m", -1.0)
    FOCUS_RADIUS_TOLERANCE_M = config.get("defaults", "bspline_filter", "focus_radius_tolerance_m", 0.05)
    FOCUS_ALTITUDE_M = config.get("defaults", "bspline_filter", "focus_altitude_m", -1.0)

    # cinematic filtering
    CINEMATIC_OFFSET_M = config.get("defaults", "bspline_filter", "cinematic_offset_m", 0.5)
    CINEMATIC_SMOOTH_ALPHA = config.get("defaults", "bspline_filter", "cinematic_smooth_alpha", 0.2)

    #altitude bounds
    MIN_ALTITUDE_M = config.get("defaults", "bspline_filter", "min_altitude_m", 0.2)
    MAX_ALTITUDE_M = config.get("defaults", "bspline_filter", "max_altitude_m", 3.0)
    VIRTUAL_CEILING_M = config.get("defaults", "bspline_filter", "virtual_ceiling_m", 2.8)

    #worlds bounds
    WORLD_BOUNDS_MIN = config.get("defaults", "bspline_filter", "world_bounds_min", [-10.0, -10.0, 0.0])
    WORLD_BOUNDS_MAX = config.get("defaults", "bspline_filter", "world_bounds_max", [10.0, 10.0, 2.8])

class SimulationParams:
    SIMULATE_OBJECT = config.get("params", "simulation_params", "simulate_object", True)
    SIMULATED_OBJECT_WORLD_OFFSET = config.get("params", "simulation_params", "simulated_object_world_offset", [0.3, 0.6, 0.0])
    SIMULATED_OBJECT_SIDE_OFFSET_M = config.get("params", "simulation_params", "simulated_object_side_offset_m", 0.6)
    SIMULATED_OBJECT_RADIUS_M = config.get("params", "simulation_params", "simulated_object_radius_m", 0.8)
    SIMULATED_OBJECT_OUTWARD_OFFSET_M = config.get("params", "simulation_params", "simulated_object_outward_offset_m", 1.0)
    DRONE_ID = config.get("params", "simulation_params", "drone_id", "0")
    TRAJECTORY_DATA_DIR = config.get("params", "simulation_params", "trajectory_data_dir", "trajectory_data")
    USE_SIMULATED_FOCUS = config.get("params", "simulation_params", "use_simulated_focus", True)
    USE_SIMULATED_LIGHT = config.get("params", "simulation_params", "use_simulated_light", True)
    TARGET_XYZ = config.get("params", "simulation_params", "target_xyz", [2.34, 4.35, 1.35])
    FOCUS_DEPTH_M = config.get("params", "simulation_params", "focus_depth_m", 1.5)

class DepthAnythingParams:
    DEPTH_ANYTHING_PATH = config.get("params", "depth_anything_params", "depth_anything_path", "")
    VENV_SITE_PATH = config.get("params", "depth_anything_params", "venv_site_path", "")

class CostParams:
    # cost weights
    W_FRAMING = config.get("costs", "cost_function_weights", "w_framing", 0.056)
    W_DIST = config.get("costs", "cost_function_weights", "w_dist", 4.0)
    W_NEAR = config.get("costs", "cost_function_weights", "w_near", 25.0)
    W_STAB = config.get("costs", "cost_function_weights", "w_stab", 0.2)
    W_LIGHT = config.get("costs", "cost_function_weights", "w_light", 1.0)
    W_SMOOTH = config.get("costs", "cost_function_weights", "w_smooth", 1.0)

    # cost function parameters
    HUBER_DELTA_PX = config.get("costs", "cost_function_parameters", "huber_delta_px", 6.0)
    DEADBAND_PX = config.get("costs", "cost_function_parameters", "deadband_px", 4.0)

class LightOrientationParams:
    # deadbands
    YAW_DEADBAND_RAD = config.get("light", "light_orientation_deadbands", "light_yaw_deadband_rad", 0.05)
    PITCH_DEADBAND_RAD = config.get("light", "light_orientation_deadbands", "light_pitch_deadband_rad", 0.04)
    ROLL_DEADBAND_RAD = config.get("light", "light_orientation_deadbands", "light_roll_deadband_rad", 0.01)

    # cost weights
    PITCH_WEIGHT_FACTOR = config.get("light", "light_orientation_weights", "light_pitch_weight_factor", 0.2)
    ROLL_WEIGHT_FACTOR = config.get("light", "light_orientation_weights", "light_roll_weight_factor", 0.0)

    # rotational limits
    PITCH_MAX_RAD = config.get("light", "light_orientation_limits", "light_pitch_max_rad", 0.35)
    PITCH_DAMPING_FACTOR = config.get("light", "light_orientation_limits", "light_pitch_damping_factor", 2.0)
    YAW_CHANGE_LIMIT_RAD = config.get("light", "light_orientation_limits", "light_yaw_change_limit_rad", 0.5)

    # setpoint smoothing
    SETPOINT_SMOOTH_ALPHA = config.get("light", "light_smoothing", "light_setpoint_smooth_alpha", 0.3)
    PLANAR_NORM_THRESHOLD = config.get("light", "light_smoothing", "light_planar_norm_threshold", 0.01)

@dataclass
class CinematographyParams:
    # focus params
    TARGET_FOCUS_DISTANCE_M: float = field(default_factory=lambda: config.get("focus", "focus_distance", "target_focus_distance_m", 1.0))
    MIN_FOCUS_DISTANCE_M: float = field(default_factory=lambda: config.get("focus", "focus_distance", "min_focus_distance_m", 1.0))
    
    # waypoint generation
    CONSTANT_ALTITUDE_M: float = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "orbit_altitude_m", 1.5))
    ORBIT_RADIUS_M: float = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "orbit_radius_m", 1.5))
    ANGULAR_VELOCITY_RPS: float = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "angular_velocity_rps", 0.2))
    LOOK_DURATION_S: float = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "look_duration_s", 3.0))
    WAYPOINT_RATE_HZ: float = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "waypoint_rate_hz", 10.0))
    NUM_INITIAL_WAYPOINTS: int = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "num_initial_waypoints", 40))
    
    # camera params
    IMAGE_WIDTH: int = field(default_factory=lambda: config.get("camera", "camera_params", "image_width", 1152))
    IMAGE_HEIGHT: int = field(default_factory=lambda: config.get("camera", "camera_params", "image_height", 648))
    CAMERA_FX: float = field(default_factory=lambda: config.get("camera", "camera_params", "camera_fx", 525.0))
    CAMERA_FY: float = field(default_factory=lambda: config.get("camera", "camera_params", "camera_fy", 525.0))
    CAMERA_CX: float = field(default_factory=lambda: config.get("camera", "camera_params", "camera_cx", 576.0))
    CAMERA_CY: float = field(default_factory=lambda: config.get("camera", "camera_params", "camera_cy", 324.0))
    
    # topic strings
    WAYPOINTS_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "waypoints_topic", "/move_base_simple/goal"))
    ODOM_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "odom_topic", "/odom_world"))
    DEPTH_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "depth_topic", "/depth/image_raw"))
    CENTER_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "center_topic", "/object_detection/center_point"))
    BBOX_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "bbox_topic", "/object_detection/bounding_box"))
    DIMENSIONS_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "dimensions_topic", "/object_detection/dimensions"))
    LIGHT_VECTOR_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "light_vector_topic", "/cinematography/light_vector"))
    IMAGE_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "image_topic", "/camera_publisher/image_raw"))
    BSPLINE_IN_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "bspline_in_topic", "/drone_0_planning/bspline"))
    BSPLINE_OUT_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "bspline_out_topic", "/drone_0_planning/cinematic_bspline"))
    
    # b-spline filter params
    CINEMATIC_OFFSET_M: float = field(default_factory=lambda: config.get("defaults", "bspline_filter", "cinematic_offset_m", 0.5))
    CINEMATIC_SMOOTH_ALPHA: float = field(default_factory=lambda: config.get("defaults", "bspline_filter", "cinematic_smooth_alpha", 0.2))
    MIN_ALTITUDE_M: float = field(default_factory=lambda: config.get("defaults", "bspline_filter", "min_altitude_m", 0.2))
