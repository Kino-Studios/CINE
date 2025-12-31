from dataclasses import dataclass
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

@dataclass
class CinematicConstraints:
    FOCUS_MAX_VELOCITY_MPS: float = 0.8
    FOCUS_MAX_ACCEL_MPS2: float = 3.1
    DEFAULT_MAX_VELOCITY_MPS: float = 1.1
    DEFAULT_MAX_ACCEL_MPS2: float = 4.8
    DEFAULT_SPEED_SCALE: float = 0.8
    FOCUS_SPEED_SCALE: float = 0.7
    TARGET_DISTANCE_M: float = 1.0
    TARGET_DISTANCE_TOL_M: float = 0.05
    TARGET_DISTANCE_GATE_M: float = 1.0
    YAWDOT_MAX: float = 1.25
    YAWDOT_MAX_FRAMING: float = 0.6
    YAW_LPF_ALPHA: float = 0.5
    YAW_LPF_ALPHA_FRAMING: float = 0.8
    USE_TARGET_YAW: bool = True
    ALWAYS_FACE_TARGET: bool = True
    USE_CINEMATIC: bool = True
    FRAMING_ACTIVE: bool = False
    VEL_DAMPING: float = 0.08
    ACC_DAMPING: float = 0.05
    MIN_DT: float = 0.05

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
