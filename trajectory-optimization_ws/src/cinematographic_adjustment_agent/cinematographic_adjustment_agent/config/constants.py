from dataclasses import dataclass
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

@dataclass
class CinematicConstraints:
    FOCUS_MAX_VELOCITY_MPS: float = 0.7
    FOCUS_MAX_ACCEL_MPS2: float = 2.5
    DEFAULT_MAX_VELOCITY_MPS: float = 1.65
    DEFAULT_MAX_ACCEL_MPS2: float = 4.7
    DEFAULT_SPEED_SCALE: float = 0.8
    FOCUS_SPEED_SCALE: float = 0.7
    TARGET_DISTANCE_M: float = 1.0
    TARGET_DISTANCE_TOL_M: float = 0.05
    TARGET_DISTANCE_GATE_M: float = 1.0
    YAWDOT_MAX: float = 1.25
    YAWDOT_MAX_FRAMING: float = 0.6
    YAW_LPF_ALPHA: float = 0.85
    YAW_LPF_ALPHA_FRAMING: float = 0.9
    USE_TARGET_YAW: bool = True
    ALWAYS_FACE_TARGET: bool = True
    USE_CINEMATIC: bool = True
    FRAMING_ACTIVE: bool = False
    VEL_DAMPING: float = 0.08
    ACC_DAMPING: float = 0.05
    MIN_DT: float = 0.05
    LIGHT_AVOID_GAIN: float = 2.5
    LIGHT_INFLUENCE_DISTANCE_M: float = 4.0
    LIGHT_AVOID_SPEED_THRESHOLD_MPS: float = 3.0
    LIGHT_AVOID_BLEND_BASE: float = 0.05
    LIGHT_AVOID_MAX_BLEND: float = 0.25
    LIGHT_AVOID_TRANSITION_S: float = 6.0
    LIGHT_COOLDOWN_AFTER_FRAMING_S: float = 0.7
    LIGHT_DISABLE_NEAR_FINISH_S: float = 0.1
    LIGHT_DISTANCE_GAIN: float = 1.5
    LIGHT_DISTANCE_BLEND_GAIN: float = 1.2
    LIGHT_ROTATION_DAMPING: float = 0.85
    LIGHT_YAW_SMOOTH_ALPHA: float = 0.7

class Constraints:
    MAX_VELOCITY = 1.5
    MAX_ACCELERATION = 10

class ROSParams:
    QOS_PROFILE = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )

class UserRequestedParams:
    FOCUS_OBJECT_ID = 2
    THIRDS_ANCHOR = [2 / 3.0, 1 / 3.0]

    DESIRED_FOCUS_DISTANCE_M = 1.0
    FOCUS_DISTANCE_MIN_M = 1.0

class ImageParams:
    IMAGE_WIDTH = 1152
    IMAGE_HEIGHT = 648
