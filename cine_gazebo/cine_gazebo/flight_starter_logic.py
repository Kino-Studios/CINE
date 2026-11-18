"""Pure state machine that sequences a simulated drone from link-up to the mission trigger.

It reads one observation per call and returns the single action the caller should
perform. It never talks to ROS, so every transition can be tested on its own.

Node contract: every command action (`set_mode`, `arm`, `takeoff`, `switch_source`,
`land`) is re-emitted on every step until its effect shows up in the observation,
so the node must rate-limit them. `setpoint` is also streamed every step while
exciting. A `land` action also means "select the GPS EKF source set first" (its
`source_set` is 1). After `FAILED` the vehicle may still be armed: the machine
only lands it when it still has a commandable vehicle.
"""

import enum
import math
from dataclasses import dataclass

GUIDED_MODE = 'GUIDED'
VALID_NAV_SOURCES = ('sim', 'vins')
EXTERNAL_VISION_SOURCE_SET = 2
GPS_SOURCE_SET = 1
# ArduPilot's takeoff command stops climbing near the target, so "arrived" is slightly below it.
TAKEOFF_REACHED_FRACTION = 0.95
# Below this the vehicle is still on the ground, so the takeoff command was probably not accepted.
AIRBORNE_ALTITUDE_M = 0.3
LAND_MODE = 'LAND'
POST_ARM_STATES = (
    'TAKING_OFF',
    'EXCITING',
    'SETTLING',
    'SWITCHING_SOURCE',
    'CONFIRMING_HOLD',
    'TRIGGERING',
)

DEFAULT_EXCITATION_PATTERN = (
    (1.5, 1.5, 3.5, 1.047),
    (-1.5, 1.5, 1.5, -1.047),
    (-1.5, -1.5, 3.5, 1.047),
    (1.5, -1.5, 1.5, -1.047),
    (0.0, 0.0, 2.5, 0.0),
)


class StarterState(enum.Enum):
    WAITING_FOR_LINK = 'waiting_for_link'
    WAITING_FOR_POSITION = 'waiting_for_position'
    SETTING_GUIDED = 'setting_guided'
    ARMING = 'arming'
    TAKING_OFF = 'taking_off'
    EXCITING = 'exciting'
    SETTLING = 'settling'
    SWITCHING_SOURCE = 'switching_source'
    CONFIRMING_HOLD = 'confirming_hold'
    TRIGGERING = 'triggering'
    DONE = 'done'
    LANDING_ON_GPS = 'landing_on_gps'
    FAILED = 'failed'


@dataclass(frozen=True)
class Observation:
    time_s: float
    connected: bool
    mode: str
    armed: bool
    has_local_position: bool
    altitude_m: float
    autopilot_velocity: tuple[float, float, float]
    vio_velocity: tuple[float, float, float] | None
    vio_age_s: float | None
    # Distance from the current excitation target, or from the hold point once the source switched.
    position_error_m: float
    # Which EKF source set is active; None when unknown.
    ekf_source_set: int | None
    # Excitation-pattern index that position_error_m was measured against; None for the hold point.
    target_index: int | None


@dataclass(frozen=True)
class Action:
    # One of: none, set_mode, arm, takeoff, setpoint, hold_position, switch_source, trigger,
    # land. `hold_position` asks the caller to fix the hold point at the vehicle's current
    # position and stream it as the setpoint; it is sent once.
    kind: str
    mode: str | None = None
    altitude_m: float | None = None
    # (x, y, z, yaw)
    setpoint: tuple[float, float, float, float] | None = None
    source_set: int | None = None
    message: str = ''


@dataclass(frozen=True)
class StarterConfig:
    # 'sim' trusts the simulator's position source and skips the VIO handover;
    # 'vins' excites the estimator, then hands the autopilot over to it.
    nav_source: str
    takeoff_altitude_m: float = 2.5
    link_timeout_s: float = 120.0
    takeoff_timeout_s: float = 30.0
    excitation_pattern: tuple[tuple[float, float, float, float], ...] = DEFAULT_EXCITATION_PATTERN
    max_excitation_passes: int = 3
    vio_velocity_tolerance_mps: float = 0.3
    vio_ready_hold_s: float = 5.0
    vio_max_age_s: float = 0.2
    hold_confirm_s: float = 5.0
    hold_tolerance_m: float = 0.5
    # The EKF source switch waits until the vehicle moves slower than this and sits within
    # hold_tolerance_m of the hold point, so the switch never lands mid-leg.
    switch_max_speed_mps: float = 0.2
    settle_timeout_s: float = 15.0
    command_timeout_s: float = 10.0
    landing_timeout_s: float = 60.0


class FlightStarter:
    """Call `step` once per observation; `none` means do nothing this cycle.

    `set_mode` and `arm` repeat on every step until the autopilot confirms, so a
    dropped command is retried; the caller may rate-limit them. `setpoint` repeats
    every step while exciting, because GUIDED mode needs a streamed target.
    """

    def __init__(self, config: StarterConfig):
        if config.nav_source not in VALID_NAV_SOURCES:
            raise ValueError(
                f'nav_source must be one of {VALID_NAV_SOURCES}, got {config.nav_source!r}',
            )

        if config.nav_source == 'vins' and not config.excitation_pattern:
            raise ValueError('excitation_pattern must not be empty when nav_source is vins')

        self._config = config
        self._state = StarterState.WAITING_FOR_LINK
        self._failure_message = ''
        self._state_entered_s = None
        self._agreement_started_s = None
        self._pattern_index = 0
        self._completed_passes = 0

    @property
    def state(self) -> StarterState:
        return self._state

    @property
    def failure_message(self) -> str:
        return self._failure_message

    def step(self, observation: Observation) -> Action:
        if self._state_entered_s is None:
            self._state_entered_s = observation.time_s

        if self._state.name in POST_ARM_STATES:
            guard_failure = self._post_arm_failure(observation)
            if guard_failure is not None:
                return guard_failure

        handler = {
            StarterState.WAITING_FOR_LINK: self._wait_for_link,
            StarterState.WAITING_FOR_POSITION: self._wait_for_position,
            StarterState.SETTING_GUIDED: self._set_guided,
            StarterState.ARMING: self._arm,
            StarterState.TAKING_OFF: self._take_off,
            StarterState.EXCITING: self._excite,
            StarterState.SETTLING: self._settle,
            StarterState.SWITCHING_SOURCE: self._begin_hold,
            StarterState.CONFIRMING_HOLD: self._confirm_hold,
            StarterState.LANDING_ON_GPS: self._finish_landing,
        }.get(self._state)
        if handler is None:
            return Action('none', message=self._failure_message)

        return handler(observation)

    def _post_arm_failure(self, observation):
        if not observation.connected:
            return self._fail(observation, 'MAVROS link lost')

        if not observation.armed:
            return self._fail(observation, 'disarmed unexpectedly')

        if observation.mode != GUIDED_MODE:
            return self._fail(observation, f'mode changed to {observation.mode} by the autopilot')

        return None

    def _enter(self, state, observation):
        self._state = state
        self._state_entered_s = observation.time_s

    def _fail(self, observation, message):
        self._failure_message = message
        self._enter(StarterState.FAILED, observation)

        return Action('none', message=message)

    def _land(self, observation, message):
        self._failure_message = message
        self._enter(StarterState.LANDING_ON_GPS, observation)

        return Action('land', source_set=GPS_SOURCE_SET, message=message)

    def _command_unconfirmed_too_long(self, observation):
        return observation.time_s - self._state_entered_s > self._config.command_timeout_s

    def _waited_too_long(self, observation):
        return observation.time_s - self._state_entered_s > self._config.link_timeout_s

    def _wait_for_link(self, observation):
        if observation.connected:
            self._enter(StarterState.WAITING_FOR_POSITION, observation)

            return self._wait_for_position(observation)

        if self._waited_too_long(observation):
            return self._fail(
                observation,
                f'MAVROS did not connect within {self._config.link_timeout_s:g} s',
            )

        return Action('none', message='waiting for MAVROS')

    def _wait_for_position(self, observation):
        if observation.has_local_position:
            self._enter(StarterState.SETTING_GUIDED, observation)

            return self._set_guided(observation)

        if self._waited_too_long(observation):
            return self._fail(
                observation,
                f'EKF local position did not appear within {self._config.link_timeout_s:g} s',
            )

        return Action('none', message='waiting for EKF local position')

    def _set_guided(self, observation):
        if observation.mode == GUIDED_MODE:
            self._enter(StarterState.ARMING, observation)

            return self._arm(observation)

        if self._command_unconfirmed_too_long(observation):
            return self._fail(
                observation,
                f'GUIDED mode refused within {self._config.command_timeout_s:g} s',
            )

        return Action('set_mode', mode=GUIDED_MODE, message='requesting GUIDED mode')

    def _arm(self, observation):
        if observation.armed:
            self._enter(StarterState.TAKING_OFF, observation)

            return Action(
                'takeoff',
                altitude_m=self._config.takeoff_altitude_m,
                message='armed, taking off',
            )

        if self._command_unconfirmed_too_long(observation):
            return self._fail(
                observation,
                f'arming refused within {self._config.command_timeout_s:g} s',
            )

        return Action('arm', message='requesting arm')

    def _take_off(self, observation):
        config = self._config
        if observation.altitude_m >= TAKEOFF_REACHED_FRACTION * config.takeoff_altitude_m:
            if config.nav_source == 'sim':
                return self._trigger(observation)

            self._enter(StarterState.EXCITING, observation)

            return self._current_setpoint('takeoff altitude reached, exciting VINS')

        if observation.time_s - self._state_entered_s > config.takeoff_timeout_s:
            return self._land(
                observation,
                f'takeoff did not reach {config.takeoff_altitude_m:g} m '
                f'within {config.takeoff_timeout_s:g} s',
            )

        if observation.altitude_m < AIRBORNE_ALTITUDE_M:
            return Action(
                'takeoff',
                altitude_m=config.takeoff_altitude_m,
                message='vehicle still on the ground, repeating takeoff',
            )

        return Action('none', message='climbing')

    def _current_setpoint(self, message):
        return Action(
            'setpoint',
            setpoint=self._config.excitation_pattern[self._pattern_index],
            message=message,
        )

    def _vio_agrees(self, observation):
        config = self._config

        if observation.vio_velocity is None or observation.vio_age_s is None:
            return False

        if observation.vio_age_s > config.vio_max_age_s:
            return False

        return all(
            abs(vio - autopilot) <= config.vio_velocity_tolerance_mps
            for vio, autopilot in zip(
                observation.vio_velocity,
                observation.autopilot_velocity,
                strict=True,
            )
        )

    def _excite(self, observation):
        config = self._config

        if self._vio_agrees(observation):
            if self._agreement_started_s is None:
                self._agreement_started_s = observation.time_s

            if observation.time_s - self._agreement_started_s >= config.vio_ready_hold_s:
                if self._speed_mps(observation) > config.switch_max_speed_mps:
                    self._enter(StarterState.SETTLING, observation)

                    return Action(
                        'hold_position',
                        message='VIO agrees with the autopilot, stopping before the source switch',
                    )

                return self._switch_to_external_vision(observation)
        else:
            self._agreement_started_s = None

        reached_target = (
            observation.target_index == self._pattern_index
            and observation.position_error_m <= config.hold_tolerance_m
        )
        if not reached_target:
            return self._current_setpoint('flying to excitation point')

        self._pattern_index += 1
        if self._pattern_index == len(config.excitation_pattern):
            self._pattern_index = 0
            self._completed_passes += 1
            if self._completed_passes >= config.max_excitation_passes:
                return self._land(
                    observation,
                    f'VINS did not initialise after {config.max_excitation_passes} '
                    'excitation passes',
                )

        return self._current_setpoint('flying to next excitation point')

    @staticmethod
    def _speed_mps(observation):
        return math.hypot(*observation.autopilot_velocity)

    def _switch_to_external_vision(self, observation):
        self._enter(StarterState.SWITCHING_SOURCE, observation)

        return Action(
            'switch_source',
            source_set=EXTERNAL_VISION_SOURCE_SET,
            message='VIO agrees with the autopilot, switching to external vision',
        )

    def _settle(self, observation):
        config = self._config
        # target_index is None once the caller has fixed the hold point, so the error below is
        # measured from it.
        at_rest_on_hold_point = (
            observation.target_index is None
            and observation.position_error_m <= config.hold_tolerance_m
            and self._speed_mps(observation) <= config.switch_max_speed_mps
        )
        if at_rest_on_hold_point:
            return self._switch_to_external_vision(observation)

        if observation.time_s - self._state_entered_s > config.settle_timeout_s:
            return self._land(
                observation,
                f'vehicle did not settle at the hold point within {config.settle_timeout_s:g} s '
                'before the switch to VINS',
            )

        return Action('none', message='stopping at the hold point')

    def _begin_hold(self, observation):
        if observation.ekf_source_set == EXTERNAL_VISION_SOURCE_SET:
            self._enter(StarterState.CONFIRMING_HOLD, observation)

            return self._confirm_hold(observation)

        if self._command_unconfirmed_too_long(observation):
            return self._land(observation, 'EKF source switch refused')

        return Action(
            'switch_source',
            source_set=EXTERNAL_VISION_SOURCE_SET,
            message='repeating switch to external vision',
        )

    def _confirm_hold(self, observation):
        config = self._config

        if observation.ekf_source_set != EXTERNAL_VISION_SOURCE_SET:
            return self._land(observation, 'EKF source fell back from external vision')

        if observation.position_error_m > config.hold_tolerance_m:
            return self._land(
                observation,
                f'position drifted {observation.position_error_m:.2f} m after switching to VINS',
            )

        if observation.time_s - self._state_entered_s >= config.hold_confirm_s:
            return self._trigger(observation)

        return Action('none', message='holding on VINS')

    def _trigger(self, observation):
        self._enter(StarterState.DONE, observation)

        return Action('trigger', message='starting mission')

    def _finish_landing(self, observation):
        if not observation.armed:
            self._enter(StarterState.FAILED, observation)

            return Action('none', message=self._failure_message)

        if observation.time_s - self._state_entered_s > self._config.landing_timeout_s:
            return self._fail(
                observation,
                f'landing did not complete within {self._config.landing_timeout_s:g} s '
                f'({self._failure_message})',
            )

        if observation.mode != LAND_MODE:
            return Action('land', source_set=GPS_SOURCE_SET, message='repeating land')

        return Action('none', message='landing')
