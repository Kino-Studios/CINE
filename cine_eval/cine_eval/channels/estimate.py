"""Estimate channel: how far the subject estimate sits from the subject's true position."""

from dataclasses import replace

import numpy as np

from cine_eval.channels.result import OK, ChannelResult, unchecked
from cine_eval.frames import FrameError, positions_in_frame
from cine_eval.record import (
    DETECTIONS,
    SUBJECT_ESTIMATE,
    SUBJECT_TRUE,
    FlightRecord,
)
from cine_eval.settings import ScoringSettings
from cine_eval.timeline import filming_mask, fixed_clock, sample

NAME = 'estimate'
REQUIRED_STREAMS = (SUBJECT_ESTIMATE, SUBJECT_TRUE)


def score_estimate(record: FlightRecord, settings: ScoringSettings) -> ChannelResult:
    missing = [name for name in REQUIRED_STREAMS if not record.has(name)]
    if missing:
        return unchecked(NAME, f'missing streams: {", ".join(missing)}')

    estimate = record.streams[SUBJECT_ESTIMATE]
    truth = record.streams[SUBJECT_TRUE]
    raw_position = np.asarray(estimate.fields['position'], dtype=float)
    finite = np.all(np.isfinite(raw_position), axis=1)
    if not finite.any():
        return unchecked(NAME, f'{SUBJECT_ESTIMATE} has no finite position')

    non_finite_share = float(1.0 - finite.mean())

    try:
        estimate_position = positions_in_frame(
            record.metadata,
            raw_position[finite],
            estimate.frame_id,
            truth.frame_id,
            SUBJECT_ESTIMATE,
            SUBJECT_TRUE,
        )
    except FrameError as exception:
        return unchecked(NAME, str(exception))

    estimate = replace(
        estimate,
        time=estimate.time[finite],
        receive_time=estimate.receive_time[finite],
        fields={'position': estimate_position},
    )

    start = max(estimate.time[0], truth.time[0])
    end = min(estimate.time[-1], truth.time[-1])
    if end < start:
        return unchecked(NAME, 'subject_estimate and subject_true do not overlap in time')

    clock = fixed_clock(start, end, settings.clock_hz)
    error_m = np.linalg.norm(
        sample(estimate, 'position', clock) - sample(truth, 'position', clock),
        axis=1,
    )
    summary = {
        'median_error_m': float(np.median(error_m)),
        'p95_error_m': float(np.percentile(error_m, 95)),
        'max_error_m': float(error_m.max()),
        'non_finite_estimate_share': non_finite_share,
    }
    notes = []
    stale_share = _stale_detection_share(record, clock, settings)
    if stale_share is None:
        notes.append('stale_detection_share needs detections and the framing cue')
    else:
        summary['stale_detection_share'] = stale_share

    return ChannelResult(
        name=NAME,
        status=OK,
        reason='; '.join(notes),
        summary=summary,
        series={'time': clock, 'error_m': error_m},
        clock_hz=settings.clock_hz,
    )


def _stale_detection_share(
    record: FlightRecord,
    clock: np.ndarray,
    settings: ScoringSettings,
) -> float | None:
    """Share of filming time with no detection newer than ``detection_stale_s``."""

    if not record.has(DETECTIONS):
        return None

    filming = filming_mask(record, clock)
    if filming is None or not filming.any():
        return None

    detection_time = record.streams[DETECTIONS].time
    last_index = np.searchsorted(detection_time, clock[filming], side='right') - 1
    age_s = clock[filming] - detection_time[np.maximum(last_index, 0)]
    stale = (last_index < 0) | (age_s > settings.detection_stale_s)

    return float(stale.mean())
