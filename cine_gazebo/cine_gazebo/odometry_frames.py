"""Rotates MAVROS's base_link odometry twist into the world frame the planner reads."""

import math
from dataclasses import dataclass

from cine_gazebo.sitl_observation import rotate_by_quaternion

# Far below the rounding error of any unit quaternion, so only a missing orientation
# (an all-zero message) lands under it.
MINIMUM_QUATERNION_NORM = 1e-6


@dataclass(frozen=True)
class WorldTwist:
    linear: tuple[float, float, float]
    angular: tuple[float, float, float]


def quaternion_xyzw(orientation) -> tuple[float, float, float, float]:
    return (orientation.x, orientation.y, orientation.z, orientation.w)


def body_twist_in_world(orientation, linear, angular) -> WorldTwist:
    norm = math.sqrt(sum(component * component for component in orientation))
    if not (math.isfinite(norm) and norm >= MINIMUM_QUATERNION_NORM):
        raise ValueError(f'odometry orientation {orientation} has no rotation to apply')

    # A scaled quaternion names the same rotation; the rotation formula needs a unit one.
    unit_orientation = tuple(component / norm for component in orientation)

    return WorldTwist(
        linear=rotate_by_quaternion(unit_orientation, linear),
        angular=rotate_by_quaternion(unit_orientation, angular),
    )
