from dataclasses import dataclass

@dataclass
class CinematicConstraints:
    focus_max_velocity_mps: float = 1.0
    focus_max_accel_mps2: float = 1.5
    default_max_velocity_mps: float = 2.0
    default_max_accel_mps2: float = 3.0
    min_dt: float = 0.05
