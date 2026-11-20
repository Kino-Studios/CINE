"""Focus channel: did the drone hold the 3-D focus distance to the subject while filming."""

import numpy as np

from cine_eval.channels.result import OK, ChannelResult, unchecked
from cine_eval.record import DRONE_TRUE, FOCUS_TARGET, FRAMING_ACTIVE, SUBJECT_TRUE, FlightRecord
from cine_eval.settings import ScoringSettings
from cine_eval.timeline import filming_mask, fixed_clock, sample, sample_hold

NAME = 'focus'
BOUNDARY_SLACK_M = 1e-9
REQUIRED_STREAMS = (DRONE_TRUE, SUBJECT_TRUE, FRAMING_ACTIVE)
# The field extraction gives every std_msgs/Float32 sample, the anchor distance included.
ANCHOR_DISTANCE_FIELD = 'value'


def score_focus(record: FlightRecord, settings: ScoringSettings) -> ChannelResult:
    missing = [name for name in REQUIRED_STREAMS if not record.has(name)]
    if missing:
        return unchecked(NAME, f'missing streams: {", ".join(missing)}')

    drone = record.streams[DRONE_TRUE]
    subject = record.streams[SUBJECT_TRUE]
    start = max(drone.time[0], subject.time[0])
    end = min(drone.time[-1], subject.time[-1])
    if end < start:
        return unchecked(NAME, 'drone_true and subject_true do not overlap in time')

    clock = fixed_clock(start, end, settings.clock_hz)
    target_m, target_source = _target_series(record, clock)
    if target_m is None:
        return unchecked(NAME, 'no focus target: neither anchor stream nor run.json')

    distance_m = np.linalg.norm(
        sample(drone, 'position', clock) - sample(subject, 'position', clock),
        axis=1,
    )
    error_m = distance_m - target_m
    filming = filming_mask(record, clock) & np.isfinite(error_m)
    if not filming.any():
        return unchecked(NAME, 'no filming time')

    filming_error = np.abs(error_m[filming])
    held = filming_error <= settings.focus_tolerance_m + BOUNDARY_SLACK_M
    summary = {
        'held_share': float(held.mean()),
        'worst_deviation_m': float(filming_error.max()),
        'filming_s': float(filming.sum() / settings.clock_hz),
        'target_m': float(np.median(target_m[filming])),
    }
    series = {
        'time': clock,
        'distance_m': distance_m,
        'error_m': error_m,
        'filming': filming,
    }

    return ChannelResult(
        name=NAME,
        status=OK,
        reason=target_source,
        summary=summary,
        series=series,
        clock_hz=settings.clock_hz,
    )


def _target_series(record: FlightRecord, clock: np.ndarray) -> tuple[np.ndarray | None, str]:
    """The target at each instant: the anchor distance, else the run.json config value.

    CINE aims for the config value until it publishes an anchor, so run.json also covers the
    time before the first anchor message.
    """

    run_target = record.metadata.get('run', {}).get('focus_target_m')

    if record.has(FOCUS_TARGET):
        target_m = sample_hold(record.streams[FOCUS_TARGET], ANCHOR_DISTANCE_FIELD, clock)
        before_first = np.isnan(target_m)
        if run_target is None or not before_first.any():
            return target_m, 'target from anchor stream'

        target_m[before_first] = float(run_target)

        return target_m, 'target from anchor stream, and from run.json before its first message'

    if run_target is not None:
        return np.full(len(clock), float(run_target)), 'target from run.json'

    return None, ''
