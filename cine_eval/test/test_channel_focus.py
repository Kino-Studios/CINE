import numpy as np
import pytest
from cine_eval.channels.focus import score_focus
from cine_eval.record import (
    DRONE_TRUE,
    FOCUS_TARGET,
    FRAMING_ACTIVE,
    SUBJECT_TRUE,
    FlightRecord,
    Stream,
)
from cine_eval.settings import load_scoring_settings

TIME = np.arange(0.0, 10.0 + 1e-9, 0.02)


def _stream(name, time, **fields):
    time = np.asarray(time, dtype=float)

    return Stream(name=name, frame_id='map', time=time, receive_time=time.copy(), fields=fields)


def _cue(on_until):
    return _stream(FRAMING_ACTIVE, TIME, active=TIME < on_until)


def _subject():
    return _stream(SUBJECT_TRUE, TIME, position=np.tile([0.0, 0.0, 1.0], (len(TIME), 1)))


def _drone_at(distance_m):
    position = np.stack([distance_m, np.zeros_like(TIME), np.ones_like(TIME)], axis=1)

    return _stream(DRONE_TRUE, TIME, position=position)


def _record(*streams, metadata=None):
    return FlightRecord(
        streams={stream.name: stream for stream in streams},
        metadata={'run': {'focus_target_m': 1.5}} if metadata is None else metadata,
    )


def test_drone_circling_at_target_holds_focus_all_filming_time():
    angle = TIME * 0.5
    position = np.stack([1.5 * np.cos(angle), 1.5 * np.sin(angle), np.ones_like(TIME)], axis=1)
    record = _record(_stream(DRONE_TRUE, TIME, position=position), _subject(), _cue(100.0))

    result = score_focus(record, load_scoring_settings())

    assert result.summary['held_share'] == 1.0
    assert result.summary['worst_deviation_m'] < 1e-9


def test_drone_at_twice_the_target_never_holds_focus():
    record = _record(_drone_at(np.full_like(TIME, 3.0)), _subject(), _cue(100.0))

    result = score_focus(record, load_scoring_settings())

    assert result.summary['held_share'] == 0.0
    assert result.summary['worst_deviation_m'] == pytest.approx(1.5)


def test_only_filming_time_counts():
    distance = np.where(TIME < 5.0, 1.5, 3.0)
    record = _record(_drone_at(distance), _subject(), _cue(5.0))

    result = score_focus(record, load_scoring_settings())

    assert result.summary['held_share'] == 1.0
    assert result.summary['filming_s'] == pytest.approx(5.0, abs=0.02)


def test_target_comes_from_anchor_stream_before_run_metadata():
    anchor = _stream(FOCUS_TARGET, [0.0], value=np.array([3.0]))
    record = _record(_drone_at(np.full_like(TIME, 3.0)), _subject(), _cue(100.0), anchor)

    result = score_focus(record, load_scoring_settings())

    assert result.summary['held_share'] == 1.0
    assert result.reason == 'target from anchor stream'


def test_run_metadata_target_covers_filming_before_the_first_anchor():
    distance = np.where(TIME < 5.0, 1.5, 3.0)
    anchor = _stream(FOCUS_TARGET, [5.0], value=np.array([3.0]))
    record = _record(_drone_at(distance), _subject(), _cue(100.0), anchor)

    result = score_focus(record, load_scoring_settings())

    assert result.summary['held_share'] == 1.0
    assert result.summary['filming_s'] == pytest.approx(10.0, abs=0.02)
    assert result.reason == 'target from anchor stream, and from run.json before its first message'


def test_target_from_run_metadata_is_named_in_reason():
    record = _record(_drone_at(np.full_like(TIME, 1.5)), _subject(), _cue(100.0))

    result = score_focus(record, load_scoring_settings())

    assert result.reason == 'target from run.json'


def test_distance_exactly_at_tolerance_counts_as_held():
    settings = load_scoring_settings()
    record = _record(
        _drone_at(np.full_like(TIME, 1.5 + settings.focus_tolerance_m)),
        _subject(),
        _cue(100.0),
    )

    result = score_focus(record, settings)

    assert result.summary['held_share'] == 1.0


def test_focus_without_cue_is_unchecked():
    record = _record(_drone_at(np.full_like(TIME, 1.5)), _subject())

    result = score_focus(record, load_scoring_settings())

    assert result.status == 'unchecked'
    assert 'framing_active' in result.reason


def test_focus_with_cue_never_on_is_a_missed_shot_held_for_no_time():
    record = _record(_drone_at(np.full_like(TIME, 1.5)), _subject(), _cue(-1.0))

    result = score_focus(record, load_scoring_settings())

    assert result.status == 'ok'
    assert result.summary['held_share'] == 0.0
    assert result.summary['filming_s'] == 0.0
    assert result.summary['filming_started'] is False


def test_missed_shot_reports_no_deviation_and_no_target():
    record = _record(_drone_at(np.full_like(TIME, 1.5)), _subject(), _cue(-1.0))

    result = score_focus(record, load_scoring_settings())

    assert result.summary['worst_deviation_m'] is None
    assert result.summary['target_m'] is None


def test_cue_on_only_after_the_drone_stream_ends_is_a_missed_shot():
    drone_time = TIME[TIME <= 5.0]
    drone = _stream(
        DRONE_TRUE,
        drone_time,
        position=np.tile([1.5, 0.0, 1.0], (len(drone_time), 1)),
    )
    cue = _stream(FRAMING_ACTIVE, TIME, active=TIME >= 6.0)
    record = _record(drone, _subject(), cue)

    result = score_focus(record, load_scoring_settings())

    assert result.status == 'ok'
    assert result.summary['filming_started'] is False


def test_cue_stream_present_but_empty_is_a_missed_shot():
    empty_cue = _stream(FRAMING_ACTIVE, [], active=np.array([], dtype=bool))
    record = _record(_drone_at(np.full_like(TIME, 1.5)), _subject(), empty_cue)

    result = score_focus(record, load_scoring_settings())

    assert result.status == 'ok'
    assert result.summary['filming_started'] is False


def test_filmed_run_says_filming_started():
    record = _record(_drone_at(np.full_like(TIME, 1.5)), _subject(), _cue(100.0))

    result = score_focus(record, load_scoring_settings())

    assert result.summary['filming_started'] is True


def test_cue_on_only_while_no_target_is_known_is_unchecked_not_a_missed_shot():
    anchor = _stream(FOCUS_TARGET, [5.0], value=np.array([1.5]))
    record = _record(
        _drone_at(np.full_like(TIME, 1.5)),
        _subject(),
        _cue(5.0),
        anchor,
        metadata={},
    )

    result = score_focus(record, load_scoring_settings())

    assert result.status == 'unchecked'
    assert result.reason == 'no filming time'


def test_focus_without_any_target_is_unchecked():
    record = _record(_drone_at(np.full_like(TIME, 1.5)), _subject(), _cue(100.0), metadata={})

    result = score_focus(record, load_scoring_settings())

    assert result.status == 'unchecked'
    assert result.reason == 'no focus target: neither anchor stream nor run.json'
