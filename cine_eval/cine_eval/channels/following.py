"""Following channel: how closely the drone tracked the setpoints it was commanded."""

import numpy as np

from cine_eval.channels.result import OK, ChannelResult, unchecked
from cine_eval.frames import FrameError, positions_in_frame
from cine_eval.record import DRONE_TRUE, SETPOINTS, FlightRecord
from cine_eval.settings import ScoringSettings
from cine_eval.timeline import sample

NAME = 'following'
REQUIRED_STREAMS = (DRONE_TRUE, SETPOINTS)


def score_following(record: FlightRecord, settings: ScoringSettings) -> ChannelResult:
    missing = [name for name in REQUIRED_STREAMS if not record.has(name)]
    if missing:
        return unchecked(NAME, f'missing streams: {", ".join(missing)}')

    setpoints = record.streams[SETPOINTS]

    try:
        setpoint_position = positions_in_frame(
            record.metadata,
            setpoints.fields['position'],
            setpoints.frame_id,
            record.streams[DRONE_TRUE].frame_id,
            SETPOINTS,
            DRONE_TRUE,
        )
    except FrameError as exception:
        return unchecked(NAME, str(exception))

    drone_position = sample(record.streams[DRONE_TRUE], 'position', setpoints.time)
    error_m = np.linalg.norm(drone_position - setpoint_position, axis=1)
    covered = np.isfinite(error_m)
    if not covered.any():
        return unchecked(NAME, 'drone_true does not overlap the setpoints in time')

    summary = {
        'median_tracking_error_m': float(np.median(error_m[covered])),
        'max_tracking_error_m': float(error_m[covered].max()),
    }
    reason = ''

    if len(setpoint_position) >= 2:
        steps_m = np.linalg.norm(np.diff(setpoint_position, axis=0), axis=1)
        summary['max_setpoint_jump_m'] = float(steps_m.max())
    else:
        reason = 'a single setpoint: setpoint jump omitted'

    return ChannelResult(
        name=NAME,
        status=OK,
        reason=reason,
        summary=summary,
        series={'time': setpoints.time, 'tracking_error_m': error_m},
        clock_hz=0.0,
    )
