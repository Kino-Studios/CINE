import numpy as np
import pytest
from cine_eval.channels.estimate import score_estimate
from cine_eval.record import (
    DETECTIONS,
    FRAMING_ACTIVE,
    SUBJECT_ESTIMATE,
    SUBJECT_TRUE,
    FlightRecord,
    Stream,
)
from cine_eval.settings import load_scoring_settings

TIME = np.arange(0.0, 10.0 + 1e-9, 0.02)
SETTINGS = load_scoring_settings()


def _stream(name, time, frame_id='map', **fields):
    time = np.asarray(time, dtype=float)

    return Stream(
        name=name,
        frame_id=frame_id,
        time=time,
        receive_time=time.copy(),
        fields=fields,
    )


def _truth_position():
    return np.stack([TIME * 0.5, np.zeros_like(TIME), np.ones_like(TIME)], axis=1)


def _record(estimate_offset=(0.3, 0.4, 0.0), estimate_frame='map', extra=(), metadata=None):
    truth = _truth_position()
    streams = [
        _stream(SUBJECT_TRUE, TIME, position=truth),
        _stream(
            SUBJECT_ESTIMATE,
            TIME,
            frame_id=estimate_frame,
            position=truth + np.asarray(estimate_offset),
        ),
        *extra,
    ]

    return FlightRecord(
        streams={stream.name: stream for stream in streams},
        metadata=metadata or {},
    )


def test_estimate_offset_by_constant_vector_reports_that_error():
    result = score_estimate(_record(), SETTINGS)

    assert result.summary['median_error_m'] == pytest.approx(0.5)
    assert result.summary['p95_error_m'] == pytest.approx(0.5)
    assert result.summary['max_error_m'] == pytest.approx(0.5)


def test_estimate_in_a_different_frame_without_transform_is_unchecked():
    result = score_estimate(_record(estimate_frame='camera_link'), SETTINGS)

    assert result.status == 'unchecked'
    assert 'camera_link' in result.reason
    assert 'map' in result.reason


def test_estimate_in_another_frame_is_scored_through_the_recorded_transform():
    # The estimate frame sits 2 m east of the truth frame, so the recorded transform adds
    # 2 m in x; an estimate 2 m short in its own frame is then exactly on the truth.
    metadata = {'frame_transforms': {'odom->map': [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]}}

    result = score_estimate(
        _record(estimate_offset=(-2.0, 0.0, 0.0), estimate_frame='odom', metadata=metadata),
        SETTINGS,
    )

    assert result.status == 'ok'
    assert result.summary['median_error_m'] == pytest.approx(0.0)


def test_estimate_in_another_frame_is_rotated_by_the_recorded_transform():
    # A quarter turn about z maps the estimate's (0, -t/2, 1) onto the truth's (t/2, 0, 1).
    quarter_turn_about_z = [0.0, 0.0, 0.0, 0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)]
    metadata = {'frame_transforms': {'odom->map': quarter_turn_about_z}}
    record = _record(estimate_frame='odom', metadata=metadata)
    record.streams[SUBJECT_ESTIMATE].fields['position'] = np.stack(
        [np.zeros_like(TIME), -TIME * 0.5, np.ones_like(TIME)],
        axis=1,
    )

    result = score_estimate(record, SETTINGS)

    assert result.summary['max_error_m'] == pytest.approx(0.0, abs=1e-9)


def test_estimate_with_a_malformed_transform_is_unchecked():
    metadata = {'frame_transforms': {'camera_link->map': [0.0] * 7}}

    result = score_estimate(_record(estimate_frame='camera_link', metadata=metadata), SETTINGS)

    assert result.status == 'unchecked'
    assert 'camera_link->map' in result.reason


def test_non_finite_estimates_are_left_out_of_the_error_and_counted():
    record = _record()
    position = record.streams[SUBJECT_ESTIMATE].fields['position']
    position[::4] = np.nan

    result = score_estimate(record, SETTINGS)

    assert result.summary['max_error_m'] == pytest.approx(0.5)
    assert result.summary['non_finite_estimate_share'] == pytest.approx(0.25, abs=0.01)


def test_an_estimate_that_is_never_finite_is_unchecked():
    record = _record()
    record.streams[SUBJECT_ESTIMATE].fields['position'][:] = np.inf

    result = score_estimate(record, SETTINGS)

    assert result.status == 'unchecked'
    assert 'finite' in result.reason


def test_missing_estimate_stream_is_unchecked():
    record = FlightRecord(
        streams={SUBJECT_TRUE: _stream(SUBJECT_TRUE, TIME, position=_truth_position())},
    )

    result = score_estimate(record, SETTINGS)

    assert result.status == 'unchecked'
    assert SUBJECT_ESTIMATE in result.reason


def test_stale_detection_share_counts_gaps_longer_than_the_bound():
    detection_time = np.concatenate([np.arange(0.0, 4.0, 0.1), np.arange(6.0, 10.0 + 1e-9, 0.1)])
    extra = (
        _stream(DETECTIONS, detection_time),
        _stream(FRAMING_ACTIVE, TIME, active=np.ones(len(TIME), dtype=bool)),
    )

    result = score_estimate(_record(extra=extra), SETTINGS)

    assert result.summary['stale_detection_share'] == pytest.approx(0.15, abs=0.02)


def test_stale_detection_share_is_omitted_without_the_framing_cue():
    extra = (_stream(DETECTIONS, np.arange(0.0, 10.0, 0.1)),)

    result = score_estimate(_record(extra=extra), SETTINGS)

    assert 'stale_detection_share' not in result.summary
    assert result.status == 'ok'
