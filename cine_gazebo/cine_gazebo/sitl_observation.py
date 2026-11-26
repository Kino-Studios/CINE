"""ROS-free half of sitl_flight_starter; quaternions are (x, y, z, w), vectors local ENU."""

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from cine_gazebo.flight_starter_logic import GPS_SOURCE_SET, Observation

# About how long the autopilot takes to show a command's effect; one call per kind per interval.
COMMAND_RETRY_INTERVAL_S = 1.0
# A call MAVROS never answers stops blocking retries after this long.
COMMAND_RESPONSE_TIMEOUT_S = 5.0
LOCAL_POSITION_MAX_AGE_S = 1.0
# MAV_SYS_STATUS_PREARM_CHECK in MAVLink's SYS_STATUS sensor bitmasks.
PREARM_CHECK_BIT = 0x10000000
# Land anyway after this long without the GPS source switch, rather than hover until timeout.
LAND_SWITCH_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class LinkState:
    """What /mavros/state reports."""

    connected: bool
    mode: str
    armed: bool


@dataclass(frozen=True)
class SensorStatus:
    sensors_enabled: int
    sensors_health: int


@dataclass(frozen=True)
class OdometrySample:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    # As the source reports it: body frame for MAVROS, VINS's own world frame for VIO.
    linear_velocity: tuple[float, float, float]
    received_s: float


def load_excitation_pattern(path):
    raw_points = yaml.safe_load(Path(path).read_text())['excitation_pattern']
    points = tuple(tuple(float(value) for value in point) for point in raw_points)
    if any(len(point) != 4 for point in points):
        raise ValueError(f'{path}: every excitation point must be [x, y, z, yaw]')

    if len(set(points)) != len(points):
        raise ValueError(
            f'{path}: excitation points must be distinct, because the node finds the '
            'index of a setpoint from its value',
        )

    return points


def rotate_by_quaternion(orientation, vector):
    axis_x, axis_y, axis_z, scalar = orientation
    axis = (axis_x, axis_y, axis_z)

    def cross(first, second):
        return (
            first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0],
        )

    doubled_cross = tuple(2.0 * component for component in cross(axis, vector))
    second_cross = cross(axis, doubled_cross)

    return tuple(
        vector[index] + scalar * doubled_cross[index] + second_cross[index] for index in range(3)
    )


def yaw_of(orientation):
    """Heading in radians from x, counter-clockwise about z."""

    x, y, z, w = orientation

    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rotate_about_vertical(vector, angle):
    cosine, sine = math.cos(angle), math.sin(angle)

    return (
        cosine * vector[0] - sine * vector[1],
        sine * vector[0] + cosine * vector[1],
        vector[2],
    )


def autopilot_velocity_in_local(autopilot):
    """MAVROS gives the odometry twist in the body frame; turn it into the local frame."""

    return rotate_by_quaternion(autopilot.orientation, autopilot.linear_velocity)


def vio_velocity_in_local(autopilot, vio):
    """Rotate VINS's velocity by the yaw gap; its world frame is level but its heading arbitrary."""

    heading_offset = yaw_of(autopilot.orientation) - yaw_of(vio.orientation)

    return rotate_about_vertical(vio.linear_velocity, heading_offset)


def position_error(position, excitation_pattern, target_index, hold_setpoint):
    """Distance from the excitation point at `target_index`, else from the hold point."""

    if target_index is not None:
        return math.dist(position, excitation_pattern[target_index][:3])

    if hold_setpoint is not None:
        return math.dist(position, hold_setpoint[:3])

    return math.inf


def build_observation(
    now_s,
    link,
    autopilot,
    vio,
    ekf_source_set,
    excitation_pattern,
    target_index,
    hold_setpoint,
    sensor_status,
):
    """Build the state machine's Observation; `link`, `autopilot`, `vio`, `sensor_status` are None
    until their first message, and `target_index` is the last excitation setpoint sent."""

    if ekf_source_set is None:
        raise ValueError('the active EKF source set must be known; the autopilot boots on 1')

    has_odometry = autopilot is not None
    has_vio = has_odometry and vio is not None
    position = autopilot.position if has_odometry else (0.0, 0.0, 0.0)

    return Observation(
        time_s=now_s,
        connected=link is not None and link.connected,
        mode=link.mode if link is not None else '',
        armed=link is not None and link.armed,
        has_local_position=has_odometry
        and now_s - autopilot.received_s <= LOCAL_POSITION_MAX_AGE_S,
        prearm_checks_passed=prearm_checks_passed(sensor_status),
        altitude_m=position[2],
        autopilot_velocity=(
            autopilot_velocity_in_local(autopilot) if has_odometry else (0.0, 0.0, 0.0)
        ),
        vio_velocity=vio_velocity_in_local(autopilot, vio) if has_vio else None,
        vio_age_s=now_s - vio.received_s if has_vio else None,
        position_error_m=(
            position_error(position, excitation_pattern, target_index, hold_setpoint)
            if has_odometry
            else math.inf
        ),
        ekf_source_set=ekf_source_set,
        target_index=target_index,
    )


def prearm_checks_passed(sensor_status):
    """Whether pre-arm checks pass; ArduPilot leaves the bit out of the enabled mask when
    ARMING_CHECK disables every check, and then nothing blocks arming."""

    if sensor_status is None:
        return False

    if not sensor_status.sensors_enabled & PREARM_CHECK_BIT:
        return True

    return bool(sensor_status.sensors_health & PREARM_CHECK_BIT)


def land_request(active_source_set, now_s, switch_requested_s):
    """Switch to the GPS source set first, then LAND once accepted or `LAND_SWITCH_TIMEOUT_S`
    after `switch_requested_s` (None before the first request); repeated each step until LAND."""

    if active_source_set == GPS_SOURCE_SET:
        return 'land_mode'

    if switch_requested_s is None or now_s - switch_requested_s < LAND_SWITCH_TIMEOUT_S:
        return 'switch_source'

    return 'land_mode'


class CommandGate:
    """Lets one call per command kind through at a time, at most once per retry interval."""

    def __init__(self):
        self._pending = set()
        self._last_sent_s = {}

    def may_send(self, kind, now_s):
        last_sent_s = self._last_sent_s.get(kind)
        if last_sent_s is None:
            return True

        waited_s = now_s - last_sent_s

        if kind in self._pending:
            return waited_s >= COMMAND_RESPONSE_TIMEOUT_S

        return waited_s >= COMMAND_RETRY_INTERVAL_S

    def sent(self, kind, now_s):
        self._pending.add(kind)
        self._last_sent_s[kind] = now_s

    def answered(self, kind):
        self._pending.discard(kind)
