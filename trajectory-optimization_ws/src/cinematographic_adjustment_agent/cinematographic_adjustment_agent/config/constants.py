from dataclasses import dataclass
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy 

@dataclass
class CinematicConstraints:
    FOCUS_MAX_VELOCITY_MPS: float = 0.05
    FOCUS_MAX_ACCEL_MPS2: float = 0.2
    DEFAULT_MAX_VELOCITY_MPS: float = 2.0
    DEFAULT_MAX_ACCEL_MPS2: float = 3.0
    MIN_DT: float = 0.05

class Constraints:
    MAX_VELOCITY = 1.5
    MAX_ACCELERATION = 10

class ROSParams:
    QOS_PROFILE = QoSProfile(
        reliability = ReliabilityPolicy.RELIABLE,
        durability = DurabilityPolicy.VOLATILE,
        history = HistoryPolicy.KEEP_LAST,
        depth = 10
    )

class UserRequestedParams:
    FOCUS_OBJECT_ID = 2
    THIRDS_ANCHOR = [2 / 3.0, 1 / 3.0]

    DESIRED_FOCUS_DISTANCE_M = 1.0
    FOCUS_DISTANCE_MIN_M = 1.0

class ImageParams:
    IMAGE_WIDTH = 1152
    IMAGE_HEIGHT = 648
