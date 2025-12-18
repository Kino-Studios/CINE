from dataclasses import dataclass

@dataclass
class CinematicConstraints:
    FOCUS_MAX_VELOCITY_MPS: float = 1.0
    FOCUS_MAX_ACCEL_MPS2: float = 1.5
    DEFAULT_MAX_VELOCITY_MPS: float = 2.0
    DEFAULT_MAX_ACCEL_MPS2: float = 3.0
    MIN_DT: float = 0.05
