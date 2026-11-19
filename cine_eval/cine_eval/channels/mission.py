"""Mission channel: did the drone reach the final waypoint, and did the run abort."""

import numpy as np

from cine_eval.channels.result import OK, ChannelResult, unchecked
from cine_eval.record import DRONE_TRUE, FlightRecord
from cine_eval.settings import ScoringSettings

NAME = 'mission'
ARRIVAL_RADIUS_M = 0.5


def score_mission(record: FlightRecord, settings: ScoringSettings) -> ChannelResult:
    if not record.has(DRONE_TRUE):
        return unchecked(NAME, f'missing streams: {DRONE_TRUE}')

    run = record.metadata.get('run', {})
    waypoints = run.get('waypoints')
    if not waypoints:
        return unchecked(NAME, 'no waypoints: run.json absent or lists none')

    waypoints_frame = run.get('waypoints_frame')
    if waypoints_frame is None:
        return unchecked(NAME, 'run.json has no waypoints_frame')

    drone = record.streams[DRONE_TRUE]
    if waypoints_frame != drone.frame_id:
        transform_key = f'{waypoints_frame}->{drone.frame_id}'
        if transform_key not in record.metadata.get('frame_transforms', {}):
            return unchecked(
                NAME,
                f'waypoints are in frame {waypoints_frame!r} but {DRONE_TRUE} is in '
                f'frame {drone.frame_id!r}, and the record has no transform {transform_key!r}',
            )

        return unchecked(
            NAME,
            f'frame transform not implemented: {transform_key!r} is recorded but not yet applied',
        )

    final_waypoint = np.asarray(waypoints[-1], dtype=float)
    drone_position = np.asarray(drone.fields['position'], dtype=float)
    closest_m = float(np.linalg.norm(drone_position - final_waypoint, axis=1).min())
    summary = {
        'completed': 1.0 if closest_m <= ARRIVAL_RADIUS_M else 0.0,
        'duration_s': float(drone.time[-1] - drone.time[0]),
        'aborted': 1.0 if run.get('outcome') == 'aborted' else 0.0,
    }

    return ChannelResult(
        name=NAME,
        status=OK,
        reason=f'closest approach to the final waypoint {closest_m:.2f} m',
        summary=summary,
        series={},
        clock_hz=0.0,
    )
