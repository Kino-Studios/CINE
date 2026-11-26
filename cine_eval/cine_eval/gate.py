"""Pass/review verdict for one scored run, against the gate thresholds."""

from dataclasses import dataclass

from cine_config.constants import CinematicConstraints

from cine_eval.channels.result import OK
from cine_eval.score import RunScore, validate_statuses
from cine_eval.settings import GateThresholds

CONTACT_CLASSES = (
    ('static', 'min_static_clearance_m'),
    ('subject', 'min_subject_clearance_m'),
    ('ground', 'min_ground_clearance_m'),
)


@dataclass(frozen=True)
class GateVerdict:
    failures: tuple[str, ...]
    unchecked: tuple[str, ...]
    provisional: bool

    @property
    def label(self) -> str:
        # A run whose collision check did not run (wholly or for one class) is never a PASS.
        collision_check_skipped = any(
            rule == 'no_contact' or rule.startswith('no_contact:') for rule in self.unchecked
        )

        return 'REVIEW' if self.failures or collision_check_skipped else 'PASS'


def judge(score: RunScore, run_metadata: dict, thresholds: GateThresholds) -> GateVerdict:
    validate_statuses(score.channels)
    failures: list[str] = []
    unchecked: list[str] = []

    def summary_of(channel: str) -> dict | None:
        result = score.channels.get(channel)

        return result.summary if result is not None and result.status == OK else None

    def record(rule: str, failure: str | None, checked: bool) -> None:
        if not checked:
            unchecked.append(rule)
        elif failure is not None:
            failures.append(failure)

    clearance = summary_of('clearance')
    record(
        'no_contact',
        _no_contact(clearance),
        clearance is not None and 'collision_free' in clearance,
    )

    for class_name, key in CONTACT_CLASSES:
        if clearance is not None and key not in clearance:
            unchecked.append(f'no_contact:{class_name}')
            unchecked.append(f'min_clearance:{class_name}')

    static_and_ground = _clearance_values(clearance, ('static', 'ground'))
    record(
        'min_clearance',
        _below_minimum('min_clearance', static_and_ground, thresholds.min_clearance_m),
        clearance is not None,
    )

    subject = _clearance_values(clearance, ('subject',))
    record(
        'min_subject_clearance',
        _below_minimum('min_subject_clearance', subject, thresholds.min_subject_clearance_m),
        bool(subject),
    )

    mission = summary_of('mission')
    completed = None if mission is None else mission.get('completed')
    record(
        'mission_completed',
        f'mission_completed: completed {completed:.2f}, below 1.00'
        if completed is not None and completed < 1.0
        else None,
        completed is not None,
    )

    motion = summary_of('motion')
    speed_cap_mps = thresholds.speed_cap_margin * CinematicConstraints().FOCUS_MAX_VELOCITY_MPS
    peak_speed = None if motion is None else motion.get('peak_speed_mps')
    record(
        'speed_cap_while_filming',
        f'speed_cap_while_filming: peak speed {peak_speed:.2f} m/s, above cap {speed_cap_mps:.2f} m/s'
        if peak_speed is not None and peak_speed > speed_cap_mps
        else None,
        peak_speed is not None,
    )

    focus = summary_of('focus')
    held_share = None if focus is None else focus.get('held_share')

    if focus is not None and focus.get('filming_started') is False:
        focus_failure = 'focus_held: filming never started (start waypoint not reached)'
    elif held_share is not None and held_share < thresholds.min_focus_held_share:
        focus_failure = (
            f'focus_held: held {held_share:.2f} of filming time, '
            f'below {thresholds.min_focus_held_share:.2f}'
        )
    else:
        focus_failure = None

    record('focus_held', focus_failure, held_share is not None)

    following = summary_of('following')
    jump = None if following is None else following.get('max_setpoint_jump_m')
    record(
        'setpoint_jump',
        f'setpoint_jump: largest step {jump:.2f} m, above {thresholds.max_setpoint_jump_m:.2f} m'
        if jump is not None and jump > thresholds.max_setpoint_jump_m
        else None,
        jump is not None,
    )

    freshness = summary_of('freshness')
    age_s = None if freshness is None else freshness.get('max_age_s')
    record(
        'data_age',
        f'data_age: oldest data {age_s:.3f} s, above {thresholds.max_data_age_s:.3f} s'
        if age_s is not None and age_s > thresholds.max_data_age_s
        else None,
        age_s is not None,
    )

    real_time_factor = run_metadata.get('real_time_factor')
    record(
        'real_time_factor',
        f'real_time_factor: {real_time_factor:.2f}, below {thresholds.min_real_time_factor:.2f}'
        if real_time_factor is not None and real_time_factor < thresholds.min_real_time_factor
        else None,
        real_time_factor is not None,
    )

    return GateVerdict(
        failures=tuple(failures),
        unchecked=tuple(unchecked),
        provisional=thresholds.provisional,
    )


def _clearance_values(clearance: dict | None, classes: tuple[str, ...]) -> dict[str, float]:
    """Minimum clearance per obstacle class that the clearance channel reported."""

    if clearance is None:
        return {}

    return {
        name: clearance[key]
        for name, key in CONTACT_CLASSES
        if name in classes and key in clearance
    }


def _below_minimum(rule: str, values: dict[str, float], minimum_m: float) -> str | None:
    if not values:
        return None

    worst_class = min(values, key=values.get)
    if values[worst_class] >= minimum_m:
        return None

    return f'{rule}: {worst_class} clearance {values[worst_class]:.2f} m, below {minimum_m:.2f} m'


def _no_contact(clearance: dict | None) -> str | None:
    if clearance is None or clearance.get('collision_free', 1.0) == 1.0:
        return None

    values = _clearance_values(clearance, tuple(name for name, _ in CONTACT_CLASSES))
    contacting = [
        f'{name} (min {value:.2f} m < 0 m)' for name, value in values.items() if value < 0
    ]
    when = (
        f'; first contact at {clearance["first_contact_s"]:.1f} s'
        if 'first_contact_s' in clearance
        else ''
    )

    if not contacting:
        return f'no_contact: contact reported{when}'

    return f'no_contact: contact with {", ".join(contacting)}{when}'
