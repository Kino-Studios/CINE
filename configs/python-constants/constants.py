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
    DEFAULT_MAX_VELOCITY_MPS: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "default_max_velocity_mps"))
    DEFAULT_MAX_ACCEL_MPS2: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "default_max_accel_mps2"))
    DEFAULT_SPEED_SCALE: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "default_speed_scale"))
    VEL_DAMPING: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "vel_damping"))
    ACC_DAMPING: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "acc_damping"))
    MIN_DT: float = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "min_dt"))
    USE_CINEMATIC: bool = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "use_cinematic"))
    FRAMING_ACTIVE: bool = field(default_factory=lambda: config.get("defaults", "kinematic_constraints", "framing_active"))

    # focus constraints
    FOCUS_MAX_VELOCITY_MPS: float = field(default_factory=lambda: config.get("focus", "focus_velocity_constraints", "focus_max_velocity_mps"))
    FOCUS_MAX_ACCEL_MPS2: float = field(default_factory=lambda: config.get("focus", "focus_velocity_constraints", "focus_max_accel_mps2"))
    FOCUS_SPEED_SCALE: float = field(default_factory=lambda: config.get("focus", "focus_velocity_constraints", "focus_speed_scale"))
    YAWDOT_MAX_FRAMING: float = field(default_factory=lambda: config.get("focus", "focus_yaw_constraints", "yawdot_max_framing"))
    YAW_LPF_ALPHA_FRAMING: float = field(default_factory=lambda: config.get("focus", "focus_yaw_constraints", "yaw_lpf_alpha_framing"))
    FOCUS_VELOCITY_DAMPING: float = field(default_factory=lambda: config.get("focus", "focus_damping", "focus_velocity_damping"))
    FOCUS_ACCELERATION_DAMPING: float = field(default_factory=lambda: config.get("focus", "focus_damping", "focus_acceleration_damping"))
    FOCUS_YAW_DAMPING: float = field(default_factory=lambda: config.get("focus", "focus_damping", "focus_yaw_damping"))

    # yaw and target constraints
    YAWDOT_MAX: float = field(default_factory=lambda: config.get("defaults", "yaw_constraints", "yawdot_max"))
    YAW_LPF_ALPHA: float = field(default_factory=lambda: config.get("defaults", "yaw_constraints", "yaw_lpf_alpha"))
    USE_TARGET_YAW: bool = field(default_factory=lambda: config.get("defaults", "yaw_constraints", "use_target_yaw"))
    ALWAYS_FACE_TARGET: bool = field(default_factory=lambda: config.get("defaults", "yaw_constraints", "always_face_target"))

    # target distances
    TARGET_DISTANCE_M: float = field(default_factory=lambda: config.get("defaults", "target_distance", "target_distance_m"))
    TARGET_DISTANCE_TOL_M: float = field(default_factory=lambda: config.get("defaults", "target_distance", "target_distance_tol_m"))
    TARGET_DISTANCE_GATE_M: float = field(default_factory=lambda: config.get("defaults", "target_distance", "target_distance_gate_m"))

    # light avoidance parameters
    LIGHT_AVOID_GAIN: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_avoid_gain"))
    LIGHT_INFLUENCE_DISTANCE_M: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_influence_distance_m"))
    LIGHT_AVOID_SPEED_THRESHOLD_MPS: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_avoid_speed_threshold_mps"))
    LIGHT_AVOID_BLEND_BASE: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_avoid_blend_base"))
    LIGHT_AVOID_MAX_BLEND: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_avoid_max_blend"))
    LIGHT_AVOID_TRANSITION_S: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_avoid_transition_s"))
    LIGHT_COOLDOWN_AFTER_FRAMING_S: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_cooldown_after_framing_s"))
    LIGHT_DISABLE_NEAR_FINISH_S: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_disable_near_finish_s"))
    LIGHT_DISTANCE_GAIN: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_distance_gain"))
    LIGHT_DISTANCE_BLEND_GAIN: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_distance_blend_gain"))
    LIGHT_ROTATION_DAMPING: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_rotation_damping"))
    LIGHT_YAW_SMOOTH_ALPHA: float = field(default_factory=lambda: config.get("light", "light_avoidance", "light_yaw_smooth_alpha"))

class Constraints:
    MAX_VELOCITY = config.get("defaults", "kinematic_constraints", "max_velocity")
    MAX_ACCELERATION = config.get("defaults", "kinematic_constraints", "max_acceleration")

class ROSParams:
    QOS_PROFILE = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=config.get("topics", "topics", "qos_depth")
    )

class UserRequestedParams:
    FOCUS_OBJECT_ID = config.get("focus", "focus_parameters", "focus_object_id")
    THIRDS_ANCHOR = config.get("focus", "focus_parameters", "thirds_anchor")
    THIRDS_STRENGTH = config.get("focus", "focus_parameters", "thirds_strength")
    THIRDS_USE_NEAREST_ANCHOR = config.get("focus", "focus_parameters", "thirds_use_nearest_anchor")

    DESIRED_FOCUS_DISTANCE_M = config.get("focus", "focus_distance", "desired_focus_distance_m")
    FOCUS_DISTANCE_MIN_M = config.get("focus", "focus_distance", "focus_distance_min_m")

class ImageParams:
    IMAGE_WIDTH = config.get("camera", "camera_params", "image_width")
    IMAGE_HEIGHT = config.get("camera", "camera_params", "image_height")

class WaypointParams:
    # mission control and frame settings
    FRAME_ID = config.get("waypoints", "waypoint_generator_params", "frame_id")
    ENABLE_COST_OPTIMIZATION = config.get("waypoints", "waypoint_generator_params", "enable_cost_optimization")
    USE_DYNAMIC_WAYPOINTS_ON_DETECTION = config.get("waypoints", "waypoint_generator_params", "use_dynamic_waypoints_on_detection")
    
    # waypoint generation settings
    WAYPOINT_LOOKAHEAD = config.get("waypoints", "waypoint_generator_params", "waypoint_lookahead")
    DYNAMIC_WAYPOINT_LOOKAHEAD = config.get("waypoints", "waypoint_generator_params", "dynamic_waypoint_lookahead")
    DISTANCE_THRESHOLD_M = config.get("waypoints", "waypoint_generator_params", "waypoint_radius_m")
    PROGRESS_TIMEOUT_S = config.get("waypoints", "waypoint_generator_params", "progress_timeout_s")
    MIN_GOAL_DISTANCE_M = config.get("waypoints", "waypoint_generator_params", "min_goal_distance_m")
    MIN_FIRST_WAYPOINT_DISTANCE_M = config.get("waypoints", "waypoint_generator_params", "min_first_waypoint_distance_m")
    ENABLE_PROGRESS_SKIP = config.get("waypoints", "waypoint_generator_params", "enable_progress_skip")
    MIN_ORBIT_RADIUS_M = config.get("waypoints", "waypoint_generator_params", "min_orbit_radius_m")
    WAYPOINT_RADIUS_M = config.get("waypoints", "waypoint_generator_params", "waypoint_radius_m")
    MAX_WAYPOINT_JUMP_M = config.get("waypoints", "waypoint_generator_params", "max_waypoint_jump_m")
    MIN_WAYPOINT_SEPARATION_M = config.get("waypoints", "waypoint_generator_params", "min_waypoint_separation_m")
    
    # bezier smoothing
    BEZIER_BLEND_LENGTH_M = config.get("waypoints", "waypoint_generator_params", "bezier_blend_length_m")
    BEZIER_TRANSITION_POINTS = config.get("waypoints", "waypoint_generator_params", "bezier_transition_points")
    BEZIER_MIN_SPEED_MPS = config.get("waypoints", "waypoint_generator_params", "bezier_min_speed_mps")
    
    # anchors or arcs
    ANCHOR_START_INDEX = config.get("waypoints", "waypoint_generator_params", "anchor_start_index")
    ANCHOR_END_INDEX = config.get("waypoints", "waypoint_generator_params", "anchor_end_index")
    ANCHOR_INDEX_WINDOW = config.get("waypoints", "waypoint_generator_params", "anchor_index_window")
    ANCHOR_MIN_GAP = config.get("waypoints", "waypoint_generator_params", "anchor_min_gap")
    ARC_WAYPOINT_SPACING_M = config.get("waypoints", "waypoint_generator_params", "arc_waypoint_spacing_m")
    
    # visualization
    ANCHOR_MARKER_SCALE_M = config.get("waypoints", "waypoint_generator_params", "anchor_marker_scale_m")
    TARGET_MARKER_SCALE_M = config.get("waypoints", "waypoint_generator_params", "target_marker_scale_m")
    
    # waypoint scaling
    WAYPOINT_XY_SCALE = config.get("waypoints", "waypoint_generator_params", "waypoint_xy_scale")
    WAYPOINT_Y_EXTRA_SCALE = config.get("waypoints", "waypoint_generator_params", "waypoint_y_extra_scale")
    WAYPOINT_Z_SCALE = config.get("waypoints", "waypoint_generator_params", "waypoint_z_scale")
    FOCUS_ENTRY_DECEL_WAYPOINTS = config.get("waypoints", "waypoint_generator_params", "focus_entry_decel_waypoints")
    
    # zed constraints
    ENABLE_Z_CLAMP = config.get("waypoints", "waypoint_generator_params", "enable_z_clamp")
    Z_CEILING_OVERRIDE_M = config.get("waypoints", "waypoint_generator_params", "z_ceiling_override_m")

    # candidate generation parameters
    CANDIDATE_ANGLE_WINDOW_RAD = config.get("waypoints", "waypoint_generator_params", "candidate_angle_window_rad")
    CANDIDATE_ANGLE_SAMPLES = config.get("waypoints", "waypoint_generator_params", "candidate_angle_samples")
    CANDIDATE_RADIUS_SAMPLES = config.get("waypoints", "waypoint_generator_params", "candidate_radius_samples")
    CANDIDATE_RADIUS_RANGE_M = config.get("waypoints", "waypoint_generator_params", "candidate_radius_range_m")
    MAX_WAYPOINT_STEP_M = config.get("waypoints", "waypoint_generator_params", "max_waypoint_step_m")

class BsplineFilterParams:
    # focus parameters
    FOCUS_RADIUS_M = config.get("defaults", "bspline_filter", "focus_radius_m")
    FOCUS_RADIUS_TOLERANCE_M = config.get("defaults", "bspline_filter", "focus_radius_tolerance_m")
    FOCUS_ALTITUDE_M = config.get("defaults", "bspline_filter", "focus_altitude_m")

    # cinematic filtering
    CINEMATIC_OFFSET_M = config.get("defaults", "bspline_filter", "cinematic_offset_m")
    CINEMATIC_SMOOTH_ALPHA = config.get("defaults", "bspline_filter", "cinematic_smooth_alpha")

    #altitude bounds
    MIN_ALTITUDE_M = config.get("defaults", "bspline_filter", "min_altitude_m")
    MAX_ALTITUDE_M = config.get("defaults", "bspline_filter", "max_altitude_m")
    VIRTUAL_CEILING_M = config.get("defaults", "bspline_filter", "virtual_ceiling_m")

    #worlds bounds
    WORLD_BOUNDS_MIN = config.get("defaults", "bspline_filter", "world_bounds_min")
    WORLD_BOUNDS_MAX = config.get("defaults", "bspline_filter", "world_bounds_max")

class SimulationParams:
    SIMULATE_OBJECT = config.get("params", "simulation_params", "simulate_object")
    SIMULATED_OBJECT_WORLD_OFFSET = config.get("params", "simulation_params", "simulated_object_world_offset")
    SIMULATED_OBJECT_SIDE_OFFSET_M = config.get("params", "simulation_params", "simulated_object_side_offset_m")
    SIMULATED_OBJECT_RADIUS_M = config.get("params", "simulation_params", "simulated_object_radius_m")
    SIMULATED_OBJECT_OUTWARD_OFFSET_M = config.get("params", "simulation_params", "simulated_object_outward_offset_m")
    DRONE_ID = config.get("params", "simulation_params", "drone_id")
    TRAJECTORY_DATA_DIR = config.get("params", "simulation_params", "trajectory_data_dir")
    USE_CINEMATIC_BSPLINE = config.get("params", "simulation_params", "use_cinematic_bspline")
    USE_SIMULATED_FOCUS = config.get("params", "simulation_params", "use_simulated_focus")
    USE_SIMULATED_LIGHT = config.get("params", "simulation_params", "use_simulated_light")
    TARGET_XYZ = config.get("params", "simulation_params", "target_xyz")
    FOCUS_DEPTH_M = config.get("params", "simulation_params", "focus_depth_m")
    EPSILON = config.get("params", "simulation_params", "epsilon")
    LOGGING_SEVERITY = config.get("params", "simulation_params", "logging_severity", "DEBUG")
    USE_SIMULATION = config.get("params", "simulation_params", "use_simulation", True)

class DepthAnythingParams:
    DEPTH_ANYTHING_PATH = config.get("params", "depth_anything_params", "depth_anything_path")
    VENV_SITE_PATH = config.get("params", "depth_anything_params", "venv_site_path")

class CostParams:
    # cost weights
    W_FRAMING = config.get("costs", "cost_function_weights", "w_framing")
    W_DIST = config.get("costs", "cost_function_weights", "w_dist")
    W_NEAR = config.get("costs", "cost_function_weights", "w_near")
    W_STAB = config.get("costs", "cost_function_weights", "w_stab")
    W_LIGHT = config.get("costs", "cost_function_weights", "w_light")
    W_SMOOTH = config.get("costs", "cost_function_weights", "w_smooth")

    # cost function parameters
    HUBER_DELTA_PX = config.get("costs", "cost_function_parameters", "huber_delta_px")
    DEADBAND_PX = config.get("costs", "cost_function_parameters", "deadband_px")
    LIGHT_TAU = config.get("costs", "cost_function_parameters", "light_tau")
    LIGHT_SATURATION_THRESHOLD = config.get("costs", "cost_function_parameters", "light_saturation_threshold")
    W_SATURATION = config.get("costs", "cost_function_weights", "w_saturation")
    LIGHT_DISTANCE_ATTENUATION_M = config.get("costs", "cost_function_parameters", "light_distance_attenuation_m")
    LIGHT_DISTANCE_BIAS_M = config.get("costs", "cost_function_parameters", "light_distance_bias_m")
    LIGHT_DISTANCE_GAIN = config.get("costs", "cost_function_parameters", "light_distance_gain")
    LIGHT_COST_INCREASE_WHEN_CLOSE = config.get("costs", "cost_function_parameters", "light_cost_increase_when_close")
    ROI_SIZE = config.get("costs", "cost_function_parameters", "roi_size")
    LARGE_PENALTY = config.get("costs", "cost_function_parameters", "large_penalty")
    INCLUDE_LIGHTING_IN_POSITION_COST = config.get("costs", "cost_function_parameters", "include_lighting_in_position_cost")
    INCLUDE_FRAMING_IN_POSITION_COST = config.get("costs", "cost_function_parameters", "include_framing_in_position_cost")
    INCLUDE_LIGHTING_IN_ORIENTATION_COST = config.get("costs", "cost_function_parameters", "include_lighting_in_orientation_cost")
    INCLUDE_FRAMING_IN_ORIENTATION_COST = config.get("costs", "cost_function_parameters", "include_framing_in_orientation_cost")
    FRAMING_VERTICAL_WEIGHT = config.get("costs", "cost_function_parameters", "framing_vertical_weight")

class LightOrientationParams:
    # deadbands
    YAW_DEADBAND_RAD = config.get("light", "light_orientation_deadbands", "light_yaw_deadband_rad")
    PITCH_DEADBAND_RAD = config.get("light", "light_orientation_deadbands", "light_pitch_deadband_rad")
    ROLL_DEADBAND_RAD = config.get("light", "light_orientation_deadbands", "light_roll_deadband_rad")

    # cost weights
    PITCH_WEIGHT_FACTOR = config.get("light", "light_orientation_weights", "light_pitch_weight_factor")
    ROLL_WEIGHT_FACTOR = config.get("light", "light_orientation_weights", "light_roll_weight_factor")

    # rotational limits
    PITCH_MAX_RAD = config.get("light", "light_orientation_limits", "light_pitch_max_rad")
    PITCH_DAMPING_FACTOR = config.get("light", "light_orientation_limits", "light_pitch_damping_factor")
    YAW_CHANGE_LIMIT_RAD = config.get("light", "light_orientation_limits", "light_yaw_change_limit_rad")
    PITCH_CHANGE_LIMIT_RAD = config.get("light", "light_orientation_limits", "light_pitch_change_limit_rad")
    ROLL_GAIN = config.get("light", "light_orientation_gains", "light_roll_gain")
    PITCH_GAIN = config.get("light", "light_orientation_gains", "light_pitch_gain")
    YAW_GAIN = config.get("light", "light_orientation_gains", "light_yaw_gain")

    # setpoint smoothing
    SETPOINT_SMOOTH_ALPHA = config.get("light", "light_smoothing", "light_setpoint_smooth_alpha")
    PLANAR_NORM_THRESHOLD = config.get("light", "light_smoothing", "light_planar_norm_threshold")


class MLPParams:
    USE_MLP_CORRECTION: bool = config.get("mlp", "mlp", "use_mlp_correction", False)
    MLP_WEIGHTS_PATH: str = config.get("mlp", "mlp", "mlp_weights_path", "")
    BSPLINE_IN_TOPIC: str = config.get("mlp", "mlp", "bspline_in_topic", "/drone_0_planning/cinematic_bspline")
    CORRECTED_BSPLINE_TOPIC: str = config.get("mlp", "mlp", "corrected_bspline_topic", "/mlp/corrected_bspline")
    CORRECTION_DELTA_TOPIC: str = config.get("mlp", "mlp", "correction_delta_topic", "/mlp/correction_delta")


class LightAvoidanceParams:
    ENABLED = config.get("light", "light_avoidance", "light_avoidance_enabled")


class SensorNoiseParams:
    ENABLE_SENSOR_NOISE = config.get("sensing", "sensor_noise", "enable_sensor_noise", False)
    NOISY_ODOM_TOPIC = config.get("sensing", "sensor_noise", "noisy_odom_topic", "/odometry/noisy")
    LINEAR_NOISE_STD_M = config.get("sensing", "sensor_noise", "linear_noise_std_m", 0.0005)
    ANGULAR_NOISE_STD_RAD = config.get("sensing", "sensor_noise", "angular_noise_std_rad", 0.0003)
    RAW_ODOM_TOPIC = config.get("topics", "topics", "odom_topic", "/odom_world")

    @classmethod
    def effective_odom_topic(cls) -> str:
        return cls.NOISY_ODOM_TOPIC if cls.ENABLE_SENSOR_NOISE else cls.RAW_ODOM_TOPIC


@dataclass
class CinematographyParams:
    # focus params
    TARGET_FOCUS_DISTANCE_M: float = field(default_factory=lambda: config.get("focus", "focus_distance", "target_focus_distance_m"))
    MIN_FOCUS_DISTANCE_M: float = field(default_factory=lambda: config.get("focus", "focus_distance", "min_focus_distance_m"))
    
    # waypoint generation
    CONSTANT_ALTITUDE_M: float = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "orbit_altitude_m"))
    ORBIT_RADIUS_M: float = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "orbit_radius_m"))
    ANGULAR_VELOCITY_RPS: float = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "angular_velocity_rps"))
    LOOK_DURATION_S: float = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "look_duration_s"))
    WAYPOINT_RATE_HZ: float = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "waypoint_rate_hz"))
    NUM_INITIAL_WAYPOINTS: int = field(default_factory=lambda: config.get("waypoints", "waypoint_generator_params", "num_initial_waypoints"))
    
    # camera params
    IMAGE_WIDTH: int = field(default_factory=lambda: config.get("camera", "camera_params", "image_width"))
    IMAGE_HEIGHT: int = field(default_factory=lambda: config.get("camera", "camera_params", "image_height"))
    CAMERA_FX: float = field(default_factory=lambda: config.get("camera", "camera_params", "camera_fx"))
    CAMERA_FY: float = field(default_factory=lambda: config.get("camera", "camera_params", "camera_fy"))
    CAMERA_CX: float = field(default_factory=lambda: config.get("camera", "camera_params", "camera_cx"))
    CAMERA_CY: float = field(default_factory=lambda: config.get("camera", "camera_params", "camera_cy"))
    
    # topic strings
    WAYPOINTS_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "waypoints_topic"))
    ODOM_TOPIC: str = field(default_factory=lambda: SensorNoiseParams.effective_odom_topic())
    DEPTH_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "depth_topic"))
    CENTER_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "center_topic"))
    BBOX_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "bbox_topic"))
    DIMENSIONS_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "dimensions_topic"))
    LIGHT_VECTOR_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "light_vector_topic"))
    IMAGE_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "image_topic"))
    BSPLINE_IN_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "bspline_in_topic"))
    BSPLINE_OUT_TOPIC: str = field(default_factory=lambda: config.get("topics", "topics", "bspline_out_topic"))
    
    # b-spline filter params
    CINEMATIC_OFFSET_M: float = field(default_factory=lambda: config.get("defaults", "bspline_filter", "cinematic_offset_m"))
    CINEMATIC_SMOOTH_ALPHA: float = field(default_factory=lambda: config.get("defaults", "bspline_filter", "cinematic_smooth_alpha"))
    MIN_ALTITUDE_M: float = field(default_factory=lambda: config.get("defaults", "bspline_filter", "min_altitude_m"))
