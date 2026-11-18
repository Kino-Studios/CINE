"""Replays a Gazebo actor's scripted walk, so ROS can know where the subject is.

Gazebo moves a `<script>` actor only in the renderer and publishes its pose nowhere, so
this module evaluates the same script from the rendered world file. Position follows
Gazebo Harmonic's own interpolation (gz-common PoseAnimation over a gz-math Catmull-Rom
spline); orientation is a shortest-path slerp, where Gazebo uses a squad rotation spline.
Kept free of ROS imports so CI can test it without a ROS install.
"""

import bisect
import math
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass


@dataclass(frozen=True)
class Waypoint:
    time: float
    position: tuple[float, float, float]
    rpy: tuple[float, float, float]


@dataclass(frozen=True)
class ActorScript:
    """One actor's script; `world_name` is also the frame Gazebo reports poses in."""

    world_name: str
    waypoints: tuple[Waypoint, ...]
    loop: bool
    delay_start: float
    tension: float


@dataclass(frozen=True)
class SubjectPose:
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


def _parse_waypoint(element) -> Waypoint:
    pose_element = element.find('pose')
    x, y, z, roll, pitch, yaw = (float(value) for value in pose_element.text.split())
    if pose_element.get('degrees', 'false').lower() in ('true', '1'):
        roll, pitch, yaw = (math.radians(angle) for angle in (roll, pitch, yaw))

    return Waypoint(
        time=float(element.find('time').text), position=(x, y, z), rpy=(roll, pitch, yaw)
    )


def _bool_text(element, default: bool) -> bool:
    if element is None:
        return default

    return element.text.strip().lower() in ('true', '1')


def parse_actor_script(world_sdf_text: str, actor_name: str = 'subject') -> ActorScript:
    world = ElementTree.fromstring(world_sdf_text).find('world')
    actor = world.find(f"actor[@name='{actor_name}']")
    if actor is None:
        raise ValueError(f"World '{world.get('name')}' has no actor named '{actor_name}'")

    script = actor.find('script')
    trajectories = script.findall('trajectory')
    # Gazebo plays several trajectories back to back; the CINE world needs only one.
    if len(trajectories) != 1:
        raise ValueError(
            f"Actor '{actor_name}' must have exactly one trajectory, has {len(trajectories)}"
        )

    trajectory = trajectories[0]
    waypoints = sorted(
        (_parse_waypoint(element) for element in trajectory.findall('waypoint')),
        key=lambda waypoint: waypoint.time,
    )
    delay_element = script.find('delay_start')

    return ActorScript(
        world_name=world.get('name'),
        waypoints=tuple(waypoints),
        loop=_bool_text(script.find('loop'), default=True),
        delay_start=0.0 if delay_element is None else float(delay_element.text),
        tension=min(max(float(trajectory.get('tension', '0')), 0.0), 1.0),
    )


def _trajectory_time(script: ActorScript, sim_time_seconds: float) -> float:
    """Map sim time onto the trajectory the way gz-sim's SceneManager does."""
    duration = script.waypoints[-1].time - script.waypoints[0].time
    elapsed = max(sim_time_seconds - script.delay_start, 0.0)
    if not script.loop or duration <= 0.0:
        return min(elapsed, duration)

    if elapsed <= duration:
        return elapsed

    # Gazebo subtracts whole loops while the time is strictly past the end, so a time on a
    # loop boundary stays on the last waypoint.
    wrapped = math.fmod(elapsed, duration)

    return duration if wrapped == 0.0 else wrapped


def _tangents(positions: list[tuple[float, float, float]], tension: float) -> list[tuple]:
    """Catmull-Rom tangents as gz-math's Spline::RecalcTangents computes them."""
    scale = 0.5 * (1.0 - tension)
    last = len(positions) - 1
    closed = positions[0] == positions[last]

    def difference(after, before):
        return tuple(scale * (a - b) for a, b in zip(after, before))

    tangents = []
    for index in range(len(positions)):
        if index == 0:
            before = positions[last - 1] if closed else positions[0]
            tangents.append(difference(positions[1], before))
        elif index == last:
            tangents.append(
                tangents[0] if closed else difference(positions[last], positions[last - 1])
            )
        else:
            tangents.append(difference(positions[index + 1], positions[index - 1]))

    return tangents


def _hermite(start, end, start_tangent, end_tangent, fraction: float) -> tuple[float, float, float]:
    squared = fraction * fraction
    cubed = squared * fraction
    start_weight = 2 * cubed - 3 * squared + 1
    start_tangent_weight = cubed - 2 * squared + fraction
    end_weight = -2 * cubed + 3 * squared
    end_tangent_weight = cubed - squared

    return tuple(
        start_weight * p0 + start_tangent_weight * m0 + end_weight * p1 + end_tangent_weight * m1
        for p0, m0, p1, m1 in zip(start, start_tangent, end, end_tangent)
    )


def _quaternion_xyzw(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    half_roll, half_pitch, half_yaw = roll / 2, pitch / 2, yaw / 2
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _slerp(start, end, fraction: float) -> tuple[float, float, float, float]:
    dot = sum(a * b for a, b in zip(start, end))
    if dot < 0.0:
        end, dot = tuple(-component for component in end), -dot

    if dot > 0.9995:
        blended = tuple(a + fraction * (b - a) for a, b in zip(start, end))
    else:
        angle = math.acos(dot)
        start_weight = math.sin((1.0 - fraction) * angle) / math.sin(angle)
        end_weight = math.sin(fraction * angle) / math.sin(angle)
        blended = tuple(start_weight * a + end_weight * b for a, b in zip(start, end))
    norm = math.sqrt(sum(component * component for component in blended))

    return tuple(component / norm for component in blended)


def subject_pose_at(script: ActorScript, sim_time_seconds: float) -> SubjectPose:
    waypoints = script.waypoints
    first_time = waypoints[0].time
    local_time = _trajectory_time(script, sim_time_seconds)
    times = [waypoint.time - first_time for waypoint in waypoints]

    after_index = bisect.bisect_left(times, local_time)
    if after_index == len(times) or times[after_index] == local_time:
        on_index = min(after_index, len(times) - 1)
        waypoint = waypoints[on_index]

        return SubjectPose(
            position=waypoint.position, orientation_xyzw=_quaternion_xyzw(*waypoint.rpy)
        )

    before_index = after_index - 1
    fraction = (local_time - times[before_index]) / (times[after_index] - times[before_index])
    positions = [waypoint.position for waypoint in waypoints]
    tangents = _tangents(positions, script.tension)
    position = _hermite(
        positions[before_index],
        positions[after_index],
        tangents[before_index],
        tangents[after_index],
        fraction,
    )
    orientation = _slerp(
        _quaternion_xyzw(*waypoints[before_index].rpy),
        _quaternion_xyzw(*waypoints[after_index].rpy),
        fraction,
    )

    return SubjectPose(position=position, orientation_xyzw=orientation)
