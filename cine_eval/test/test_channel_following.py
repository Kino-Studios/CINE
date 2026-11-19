import numpy as np
import pytest
from cine_eval.channels.following import score_following
from cine_eval.record import DRONE_TRUE, SETPOINTS, FlightRecord, Stream
from cine_eval.settings import load_scoring_settings

SETTINGS = load_scoring_settings()


def _stream(name, time, position, frame_id='map'):
    time = np.asarray(time, dtype=float)

    return Stream(
        name=name,
        frame_id=frame_id,
        time=time,
        receive_time=time.copy(),
        fields={'position': np.asarray(position, dtype=float)},
    )


def _record(setpoint_position, setpoint_frame='map'):
    drone_time = np.arange(0.0, 4.0 + 1e-9, 0.1)
    drone_position = np.stack(
        [drone_time, np.zeros_like(drone_time), np.zeros_like(drone_time)],
        axis=1,
    )
    setpoint_time = np.arange(0.0, 4.0 + 1e-9, 1.0)

    return FlightRecord(
        streams={
            DRONE_TRUE: _stream(DRONE_TRUE, drone_time, drone_position),
            SETPOINTS: _stream(SETPOINTS, setpoint_time, setpoint_position, setpoint_frame),
        },
    )


def test_setpoint_jump_is_the_largest_consecutive_step():
    record = _record([[0, 0, 0], [1, 0, 0], [2, 0, 0], [2, 3, 4], [4, 0, 0]])

    result = score_following(record, SETTINGS)

    assert result.summary['max_setpoint_jump_m'] == pytest.approx(np.sqrt(4 + 9 + 16))


def test_tracking_error_is_distance_from_the_drone_to_each_setpoint():
    record = _record([[0, 0, 0], [1, 0.5, 0], [2, 0, 1.0], [3, 0, 0], [4, 0, 0]])

    result = score_following(record, SETTINGS)

    assert result.summary['max_tracking_error_m'] == pytest.approx(1.0)
    assert result.summary['median_tracking_error_m'] == pytest.approx(0.0)


def test_following_without_setpoints_is_unchecked():
    record = _record([[0, 0, 0]] * 5)
    del record.streams[SETPOINTS]

    result = score_following(record, SETTINGS)

    assert result.status == 'unchecked'
    assert 'setpoints' in result.reason


def test_following_is_unchecked_when_setpoint_frame_differs_from_truth():
    record = _record([[0, 0, 0]] * 5, setpoint_frame='odom')

    result = score_following(record, SETTINGS)

    assert result.status == 'unchecked'
    assert result.reason == (
        "setpoints is in frame 'odom' but drone_true is in frame 'map', "
        "and the record has no transform 'odom->map'"
    )


def test_following_reports_a_recorded_but_unapplied_transform():
    record = _record([[0, 0, 0]] * 5, setpoint_frame='odom')
    record.metadata['frame_transforms'] = {'odom->map': [0, 0, 0]}

    result = score_following(record, SETTINGS)

    assert result.reason == (
        "frame transform not implemented: 'odom->map' is recorded but not yet applied"
    )
