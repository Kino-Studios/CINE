import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PoseSetpoint:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def position_command_to_setpoint(x: float, y: float, z: float, yaw: float) -> PoseSetpoint | None:
    if not all(math.isfinite(value) for value in (x, y, z, yaw)):
        return None

    return PoseSetpoint(x, y, z, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
