import numpy as np
import pytest
from cine_eval.record import (
    DRONE_TRUE,
    FRAMING_ACTIVE,
    SUBJECT_ESTIMATE,
    FlightRecord,
    RecordError,
    Stream,
    load,
    save,
    validate,
)


def _drone_stream(time, position):
    return Stream(
        name=DRONE_TRUE,
        frame_id='map',
        time=np.asarray(time, dtype=float),
        receive_time=np.asarray(time, dtype=float) + 0.01,
        fields={'position': np.asarray(position, dtype=float)},
    )


def test_save_then_load_returns_identical_streams(tmp_path):
    drone = Stream(
        name=DRONE_TRUE,
        frame_id='map',
        time=np.array([0.0, 0.1, 0.2]),
        receive_time=np.array([0.01, 0.11, 0.21]),
        fields={
            'position': np.array([[0.0, 0.0, 1.0], [0.1, 0.0, 1.0], [0.2, 0.0, 1.0]]),
            'orientation': np.array([[0.0, 0.0, 0.0, 1.0]] * 3),
        },
        stamp_source='receive',
    )
    framing = Stream(
        name=FRAMING_ACTIVE,
        frame_id='',
        time=np.array([0.0, 1.0]),
        receive_time=np.array([0.0, 1.0]),
        fields={'active': np.array([False, True])},
    )
    record = FlightRecord(
        streams={DRONE_TRUE: drone, FRAMING_ACTIVE: framing},
        metadata={'bag': 'run-1'},
    )

    save(record, tmp_path)
    loaded = load(tmp_path)

    assert np.array_equal(loaded.streams[DRONE_TRUE].fields['position'], drone.fields['position'])
    assert np.array_equal(
        loaded.streams[DRONE_TRUE].fields['orientation'],
        drone.fields['orientation'],
    )
    assert np.array_equal(loaded.streams[DRONE_TRUE].time, drone.time)
    assert np.array_equal(loaded.streams[DRONE_TRUE].receive_time, drone.receive_time)
    assert np.array_equal(loaded.streams[FRAMING_ACTIVE].fields['active'], [False, True])
    assert loaded.streams[FRAMING_ACTIVE].fields['active'].dtype == bool
    assert loaded.streams[DRONE_TRUE].frame_id == 'map'
    assert loaded.streams[DRONE_TRUE].stamp_source == 'receive'
    assert loaded.streams[FRAMING_ACTIVE].stamp_source == 'header'
    assert loaded.metadata == {'bag': 'run-1'}


def test_validate_rejects_decreasing_time():
    stream = _drone_stream([0.0, 0.2, 0.1], [[0, 0, 0]] * 3)

    with pytest.raises(RecordError, match='time decreases'):
        validate(FlightRecord(streams={DRONE_TRUE: stream}, metadata={}))


def test_validate_rejects_field_length_mismatch():
    stream = _drone_stream([0.0, 0.1, 0.2], [[0, 0, 0]] * 2)

    with pytest.raises(RecordError, match='length'):
        validate(FlightRecord(streams={DRONE_TRUE: stream}, metadata={}))


def test_validate_rejects_a_stream_with_no_samples():
    stream = _drone_stream([], np.empty((0, 3)))

    with pytest.raises(RecordError, match='no samples'):
        validate(FlightRecord(streams={DRONE_TRUE: stream}, metadata={}))


def test_absent_stream_is_not_in_record(tmp_path):
    stream = _drone_stream([0.0, 0.1], [[0, 0, 0]] * 2)
    save(FlightRecord(streams={DRONE_TRUE: stream}, metadata={}), tmp_path)

    loaded = load(tmp_path)

    assert not loaded.has(SUBJECT_ESTIMATE)
    assert loaded.has(DRONE_TRUE)
