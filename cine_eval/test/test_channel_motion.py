import numpy as np
import pytest
from cine_eval.channels.motion import score_motion
from cine_eval.record import DRONE_TRUE, FRAMING_ACTIVE, FlightRecord, Stream
from cine_eval.settings import load_scoring_settings

CLOCK_HZ = 50.0


def _stream(name, time, **fields):
    time = np.asarray(time, dtype=float)

    return Stream(name=name, frame_id='map', time=time, receive_time=time.copy(), fields=fields)


def _record(time, x_position, cue_on=True):
    position = np.stack([x_position, np.zeros_like(time), np.ones_like(time)], axis=1)

    return FlightRecord(
        streams={
            DRONE_TRUE: _stream(DRONE_TRUE, time, position=position),
            FRAMING_ACTIVE: _stream(FRAMING_ACTIVE, time, active=np.full(len(time), cue_on)),
        },
    )


def _minimum_jerk_record(duration_s, wobble_m=0.0):
    time = np.arange(0.0, duration_s + 1e-9, 1.0 / CLOCK_HZ)
    fraction = time / duration_s
    x_position = 2.0 * (10 * fraction**3 - 15 * fraction**4 + 6 * fraction**5)
    x_position = x_position + wobble_m * np.sin(2 * np.pi * 3.0 * time)

    return _record(time, x_position)


def test_constant_speed_line_has_zero_acceleration_and_jerk():
    time = np.arange(0.0, 10.0 + 1e-9, 1.0 / CLOCK_HZ)

    result = score_motion(_record(time, 1.0 * time), load_scoring_settings())

    assert result.summary['peak_acceleration_mps2'] < 1e-6
    assert result.summary['peak_jerk_mps3'] < 1e-6
    assert result.summary['mean_speed_mps'] == pytest.approx(1.0)


def test_speed_invariant_measures_do_not_change_when_motion_is_time_scaled():
    settings = load_scoring_settings()

    fast = score_motion(_minimum_jerk_record(4.0), settings).summary
    slow = score_motion(_minimum_jerk_record(8.0), settings).summary

    assert fast['log_dimensionless_jerk'] == pytest.approx(slow['log_dimensionless_jerk'], abs=0.05)
    assert fast['sparc'] == pytest.approx(slow['sparc'], abs=0.05)
    assert fast['peak_jerk_mps3'] >= 6 * slow['peak_jerk_mps3']


def test_minimum_jerk_profile_is_smoother_than_a_jerky_one():
    settings = load_scoring_settings()

    clean = score_motion(_minimum_jerk_record(4.0), settings).summary
    jerky = score_motion(_minimum_jerk_record(4.0, wobble_m=0.05), settings).summary

    assert jerky['log_dimensionless_jerk'] < clean['log_dimensionless_jerk']
    assert jerky['sparc'] < clean['sparc']


def test_motion_with_cue_never_on_is_a_missed_shot():
    time = np.arange(0.0, 5.0 + 1e-9, 1.0 / CLOCK_HZ)

    result = score_motion(_record(time, 1.0 * time, cue_on=False), load_scoring_settings())

    assert result.status == 'ok'
    assert result.summary['filming_started'] is False
    assert result.summary['filming_s'] == 0.0


def test_missed_shot_has_no_motion_measurements():
    time = np.arange(0.0, 5.0 + 1e-9, 1.0 / CLOCK_HZ)

    result = score_motion(_record(time, 1.0 * time, cue_on=False), load_scoring_settings())

    assert result.summary['peak_speed_mps'] is None
    assert result.summary['rms_jerk_mps3'] is None
    assert result.summary['log_dimensionless_jerk'] is None
    assert result.summary['sparc'] is None


def test_cue_on_only_after_the_drone_stream_ends_is_a_missed_shot():
    drone_time = np.arange(0.0, 5.0 + 1e-9, 1.0 / CLOCK_HZ)
    cue_time = np.arange(0.0, 10.0 + 1e-9, 1.0 / CLOCK_HZ)
    position = np.stack([drone_time, np.zeros_like(drone_time), np.ones_like(drone_time)], axis=1)
    record = FlightRecord(
        streams={
            DRONE_TRUE: _stream(DRONE_TRUE, drone_time, position=position),
            FRAMING_ACTIVE: _stream(FRAMING_ACTIVE, cue_time, active=cue_time >= 6.0),
        },
    )

    result = score_motion(record, load_scoring_settings())

    assert result.status == 'ok'
    assert result.summary['filming_started'] is False


def test_cue_stream_present_but_empty_is_a_missed_shot():
    time = np.arange(0.0, 5.0 + 1e-9, 1.0 / CLOCK_HZ)
    record = _record(time, 1.0 * time)
    record.streams[FRAMING_ACTIVE] = _stream(FRAMING_ACTIVE, [], active=np.array([], dtype=bool))

    result = score_motion(record, load_scoring_settings())

    assert result.status == 'ok'
    assert result.summary['filming_started'] is False


def test_filmed_run_says_filming_started():
    result = score_motion(_minimum_jerk_record(4.0), load_scoring_settings())

    assert result.summary['filming_started'] is True


def test_motion_without_cue_is_unchecked_naming_the_stream():
    record = _minimum_jerk_record(4.0)
    del record.streams[FRAMING_ACTIVE]

    result = score_motion(record, load_scoring_settings())

    assert result.status == 'unchecked'
    assert 'framing_active' in result.reason


def test_filming_segments_shorter_than_the_filter_are_unchecked():
    time = np.arange(0.0, 5.0 + 1e-9, 1.0 / CLOCK_HZ)
    record = _record(time, 1.0 * time)
    record.streams[FRAMING_ACTIVE].fields['active'][:] = False
    record.streams[FRAMING_ACTIVE].fields['active'][10:14] = True

    result = score_motion(record, load_scoring_settings())

    assert result.status == 'unchecked'
    assert result.reason == 'filming segments too short for smoothing'
