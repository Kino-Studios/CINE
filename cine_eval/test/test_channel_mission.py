import numpy as np
import pytest
from cine_eval.channels.mission import score_mission
from cine_eval.record import DRONE_TRUE, FlightRecord, Stream
from cine_eval.settings import load_scoring_settings

SETTINGS = load_scoring_settings()
TIME = np.arange(0.0, 10.0 + 1e-9, 0.1)


def _record(run=None, final_x=10.0):
    position = np.stack([np.linspace(0.0, final_x, len(TIME)), 0 * TIME, 0 * TIME + 1.0], axis=1)
    stream = Stream(
        name=DRONE_TRUE,
        frame_id='map',
        time=TIME,
        receive_time=TIME.copy(),
        fields={'position': position},
    )
    metadata = {} if run is None else {'run': run}

    return FlightRecord(streams={DRONE_TRUE: stream}, metadata=metadata)


def test_mission_completed_when_final_waypoint_reached():
    record = _record({'waypoints': [[0, 0, 1], [10.3, 0, 1]], 'waypoints_frame': 'map'})

    result = score_mission(record, SETTINGS)

    assert result.summary['completed'] == 1.0
    assert result.summary['duration_s'] == pytest.approx(10.0)
    assert result.summary['aborted'] == 0.0


def test_mission_not_completed_when_final_waypoint_missed():
    record = _record({'waypoints': [[0, 0, 1], [20, 0, 1]], 'waypoints_frame': 'map'})

    result = score_mission(record, SETTINGS)

    assert result.summary['completed'] == 0.0


def test_aborted_outcome_is_reported():
    record = _record(
        {'waypoints': [[0, 0, 1], [20, 0, 1]], 'waypoints_frame': 'map', 'outcome': 'aborted'},
    )

    result = score_mission(record, SETTINGS)

    assert result.summary['aborted'] == 1.0


def test_mission_without_waypoints_is_unchecked():
    result = score_mission(_record(), SETTINGS)

    assert result.status == 'unchecked'
    assert 'waypoints' in result.reason


def test_mission_is_unchecked_without_waypoints_frame():
    result = score_mission(_record({'waypoints': [[0, 0, 1], [10, 0, 1]]}), SETTINGS)

    assert result.status == 'unchecked'
    assert result.reason == 'run.json has no waypoints_frame'


def test_mission_is_unchecked_when_waypoints_frame_differs_from_truth():
    run = {'waypoints': [[0, 0, 1], [10, 0, 1]], 'waypoints_frame': 'odom'}

    result = score_mission(_record(run), SETTINGS)

    assert result.status == 'unchecked'
    assert result.reason == (
        "waypoints are in frame 'odom' but drone_true is in frame 'map', "
        "and the record has no transform 'odom->map'"
    )
