import numpy as np
import pytest
from cine_eval.channels.clearance import score_clearance
from cine_eval.record import DRONE_TRUE, SUBJECT_TRUE, FlightRecord, Stream
from cine_eval.scene import Box
from cine_eval.settings import load_scoring_settings

TIME = np.arange(0.0, 10.0 + 1e-9, 0.02)
SETTINGS = load_scoring_settings()
WORLD_FRAME = 'cine_field'
WALL = Box('wall', np.array([5.0, 0.0, 1.0]), np.array([2.0, 2.0, 2.0]), 0.0)
FAR_POST = Box('far_post', np.array([50.0, 50.0, 1.0]), np.array([1.0, 1.0, 2.0]), 0.0)
TIME_WITH_GAP = TIME[(TIME <= 4.0 + 1e-9) | (TIME >= 6.0 - 1e-9)]


def _stream(name, time, position, frame_id=WORLD_FRAME):
    time = np.asarray(time, dtype=float)

    return Stream(
        name=name,
        frame_id=frame_id,
        time=time,
        receive_time=time.copy(),
        fields={'position': np.asarray(position, dtype=float)},
    )


def _record(drone_position, subject_position=None, time=TIME):
    streams = {DRONE_TRUE: _stream(DRONE_TRUE, time, drone_position)}

    if subject_position is not None:
        streams[SUBJECT_TRUE] = _stream(SUBJECT_TRUE, time, subject_position)

    return FlightRecord(streams=streams)


def _flight(*waypoints, time=TIME):
    """Piecewise-linear positions through (time, x, y, z) waypoints, sampled on ``time``."""

    waypoint_array = np.array(waypoints, dtype=float)

    return np.stack(
        [np.interp(time, waypoint_array[:, 0], waypoint_array[:, column]) for column in (1, 2, 3)],
        axis=1,
    )


def _hover(x, y, z, time=TIME):
    return _flight((0.0, x, y, z), (10.0, x, y, z), time=time)


def test_drone_passing_through_a_box_scores_contact():
    record = _record(_flight((0.0, 0.0, 0.0, 1.0), (10.0, 10.0, 0.0, 1.0)))

    result = score_clearance(record, SETTINGS, [WALL], WORLD_FRAME)

    assert result.summary['contact_frames'] > 0
    assert result.summary['min_clearance_m'] < 0
    assert result.summary['collision_free'] == 0.0
    assert 3.5 < result.summary['first_contact_s'] < 4.0


def test_clearance_subtracts_the_airframe_radius():
    record = _record(_hover(2.0, 0.0, 1.0))

    result = score_clearance(record, SETTINGS, [WALL], WORLD_FRAME)

    assert result.summary['min_static_clearance_m'] == pytest.approx(2.0 - 0.25)


def test_clean_flight_with_every_contact_class_checked_is_collision_free():
    record = _record(_hover(2.0, 0.0, 1.0), subject_position=_hover(-5.0, 0.0, 1.0))

    result = score_clearance(record, SETTINGS, [WALL], WORLD_FRAME)

    assert result.summary['collision_free'] == 1.0
    assert 'first_contact_s' not in result.summary


def test_clean_flight_without_the_subject_stream_leaves_collision_freedom_unknown():
    record = _record(_hover(2.0, 0.0, 1.0))

    result = score_clearance(record, SETTINGS, [WALL], WORLD_FRAME)

    assert 'collision_free' not in result.summary
    assert 'collision_free unknown: subject not checked' in result.reason


def test_clean_flight_in_a_world_without_static_obstacles_leaves_collision_freedom_unknown():
    record = _record(_hover(2.0, 0.0, 1.0), subject_position=_hover(-5.0, 0.0, 1.0))

    result = score_clearance(record, SETTINGS, [], WORLD_FRAME)

    assert 'collision_free' not in result.summary
    assert 'collision_free unknown: static obstacles not checked' in result.reason


def test_flight_that_never_leaves_the_pad_leaves_collision_freedom_unknown():
    record = _record(_hover(2.0, 0.0, 0.0), subject_position=_hover(-5.0, 0.0, 1.0))

    result = score_clearance(record, SETTINGS, [WALL], WORLD_FRAME)

    assert 'collision_free' not in result.summary
    assert 'collision_free unknown: ground not checked' in result.reason


def test_drone_flying_through_the_walking_subject_is_a_collision():
    subject = _flight((0.0, 0.0, 0.0, 1.0), (10.0, 12.0, 0.0, 1.0))
    drone = _flight((0.0, 6.0, -5.0, 1.0), (10.0, 6.0, 5.0, 1.0))

    result = score_clearance(_record(drone, subject), SETTINGS, [], WORLD_FRAME)

    assert result.summary['min_subject_clearance_m'] < 0
    assert result.summary['collision_free'] == 0.0


def test_drone_crossing_the_subject_path_after_it_has_moved_on_is_clear():
    subject = _flight((0.0, 0.0, 0.0, 1.0), (10.0, 12.0, 0.0, 1.0))
    drone = _flight((0.0, 6.0, -7.0, 1.0), (10.0, 6.0, 3.0, 1.0))

    result = score_clearance(_record(drone, subject), SETTINGS, [FAR_POST], WORLD_FRAME)

    assert result.summary['collision_free'] == 1.0


def test_subject_cylinder_spans_the_ground_to_head_height_whatever_the_pose_height():
    subject = _hover(0.0, 0.0, 1.0)
    drone = _hover(0.0, 0.0, 2.5)

    result = score_clearance(_record(drone, subject), SETTINGS, [], WORLD_FRAME)

    assert result.summary['min_subject_clearance_m'] == pytest.approx(2.5 - 1.8 - 0.25)


def test_standing_on_the_pad_is_not_ground_contact():
    drone = _flight(
        (0.0, 0.0, 0.0, 0.0),
        (2.0, 0.0, 0.0, 0.0),
        (5.0, 0.0, 0.0, 3.0),
        (7.0, 0.0, 0.0, 3.0),
        (10.0, 0.0, 0.0, 0.0),
    )

    result = score_clearance(_record(drone), SETTINGS, [], WORLD_FRAME)

    assert result.summary['min_ground_clearance_m'] > 0
    assert result.summary['contact_frames'] == 0.0


def test_dipping_below_the_airborne_height_mid_flight_into_the_ground_is_contact():
    drone = _flight(
        (0.0, 0.0, 0.0, 3.0),
        (2.0, 0.0, 0.0, 3.0),
        (2.5, 0.0, 0.0, 0.05),
        (3.0, 0.0, 0.0, 3.0),
        (10.0, 0.0, 0.0, 3.0),
    )

    result = score_clearance(_record(drone), SETTINGS, [], WORLD_FRAME)

    assert result.summary['min_ground_clearance_m'] < 0
    assert result.summary['collision_free'] == 0.0


def test_flight_that_never_leaves_the_pad_omits_ground_clearance_and_says_why():
    result = score_clearance(_record(_hover(0.0, 0.0, 0.0)), SETTINGS, [WALL], WORLD_FRAME)

    assert 'min_ground_clearance_m' not in result.summary
    assert 'never airborne' in result.reason


def test_clearance_without_subject_stream_still_scores_static_and_ground():
    result = score_clearance(_record(_hover(3.0, 0.0, 1.0)), SETTINGS, [WALL], WORLD_FRAME)

    assert result.status == 'ok'
    assert 'min_subject_clearance_m' not in result.summary
    assert 'subject_true absent' in result.reason
    assert 'min_ground_clearance_m' in result.summary


def test_subject_clearance_ignores_instants_outside_the_subject_span():
    short_time = TIME[TIME <= 5.0]
    streams = {
        DRONE_TRUE: _stream(DRONE_TRUE, TIME, _hover(0.0, 0.0, 1.0)),
        SUBJECT_TRUE: _stream(
            SUBJECT_TRUE,
            short_time,
            np.tile([4.0, 0.0, 1.0], (len(short_time), 1)),
        ),
    }

    result = score_clearance(FlightRecord(streams=streams), SETTINGS, [], WORLD_FRAME)

    assert result.summary['min_subject_clearance_m'] == pytest.approx(4.0 - 0.3 - 0.25)


def test_subject_stream_covering_half_the_flight_leaves_collision_freedom_unknown():
    short_time = TIME[TIME <= 5.0]
    streams = {
        DRONE_TRUE: _stream(DRONE_TRUE, TIME, _hover(0.0, 0.0, 1.0)),
        SUBJECT_TRUE: _stream(SUBJECT_TRUE, short_time, _hover(4.0, 0.0, 1.0, time=short_time)),
    }

    result = score_clearance(FlightRecord(streams=streams), SETTINGS, [FAR_POST], WORLD_FRAME)

    assert 'collision_free' not in result.summary
    assert f'largest {SUBJECT_TRUE} gap 5.00 s from 5.00 s' in result.reason


def test_subject_stream_starting_later_than_the_drone_by_less_than_the_largest_gap_is_checked():
    late_time = TIME[TIME >= 0.1 - 1e-9]
    streams = {
        DRONE_TRUE: _stream(DRONE_TRUE, TIME, _hover(0.0, 0.0, 1.0)),
        SUBJECT_TRUE: _stream(SUBJECT_TRUE, late_time, _hover(4.0, 0.0, 1.0, time=late_time)),
    }

    result = score_clearance(FlightRecord(streams=streams), SETTINGS, [FAR_POST], WORLD_FRAME)

    assert result.summary['collision_free'] == 1.0


def test_reason_names_the_nearest_obstacle_and_the_world_label():
    result = score_clearance(
        _record(_hover(3.0, 0.0, 1.0)),
        SETTINGS,
        [WALL],
        WORLD_FRAME,
        world_label='cine_field.sdf.template',
    )

    assert 'wall' in result.reason
    assert 'cine_field.sdf.template' in result.reason


def test_missing_drone_stream_is_unchecked():
    result = score_clearance(FlightRecord(streams={}), SETTINGS, [WALL], WORLD_FRAME)

    assert result.status == 'unchecked'
    assert DRONE_TRUE in result.reason


def test_summary_holds_only_finite_floats():
    subject = _hover(3.0, 3.0, 1.0)

    result = score_clearance(_record(_hover(0.0, 0.0, 1.0), subject), SETTINGS, [WALL], WORLD_FRAME)

    assert all(np.isfinite(value) for value in result.summary.values())


def test_drone_pose_in_another_frame_than_the_world_leaves_clearance_unchecked():
    streams = {DRONE_TRUE: _stream(DRONE_TRUE, TIME, _hover(2.0, 0.0, 1.0), frame_id='map')}

    result = score_clearance(FlightRecord(streams=streams), SETTINGS, [WALL], WORLD_FRAME)

    assert result.status == 'unchecked'
    assert "frame 'map'" in result.reason
    assert "frame 'cine_field'" in result.reason


def test_subject_pose_in_another_frame_than_the_world_leaves_collision_freedom_unknown():
    streams = {
        DRONE_TRUE: _stream(DRONE_TRUE, TIME, _hover(2.0, 0.0, 1.0)),
        SUBJECT_TRUE: _stream(SUBJECT_TRUE, TIME, _hover(-5.0, 0.0, 1.0), frame_id='map'),
    }

    result = score_clearance(FlightRecord(streams=streams), SETTINGS, [WALL], WORLD_FRAME)

    assert 'min_subject_clearance_m' not in result.summary
    assert 'collision_free unknown: subject not checked' in result.reason


def test_drone_pose_gap_hiding_an_excursion_leaves_collision_freedom_unknown():
    record = _record(
        _hover(2.0, 0.0, 1.0, time=TIME_WITH_GAP),
        subject_position=_hover(-5.0, 0.0, 1.0, time=TIME_WITH_GAP),
        time=TIME_WITH_GAP,
    )

    result = score_clearance(record, SETTINGS, [WALL], WORLD_FRAME)

    assert 'collision_free' not in result.summary
    assert f'largest {DRONE_TRUE} gap 2.00 s from 4.00 s' in result.reason


def test_subject_pose_gap_leaves_collision_freedom_unknown():
    streams = {
        DRONE_TRUE: _stream(DRONE_TRUE, TIME, _hover(2.0, 0.0, 1.0)),
        SUBJECT_TRUE: _stream(
            SUBJECT_TRUE,
            TIME_WITH_GAP,
            _hover(-5.0, 0.0, 1.0, time=TIME_WITH_GAP),
        ),
    }

    result = score_clearance(FlightRecord(streams=streams), SETTINGS, [WALL], WORLD_FRAME)

    assert 'collision_free' not in result.summary
    assert f'largest {SUBJECT_TRUE} gap 2.00 s from 4.00 s' in result.reason


def test_subject_coverage_does_not_count_an_interpolated_gap_as_covered():
    streams = {
        DRONE_TRUE: _stream(DRONE_TRUE, TIME, _hover(2.0, 0.0, 1.0)),
        SUBJECT_TRUE: _stream(
            SUBJECT_TRUE,
            TIME_WITH_GAP,
            _hover(-5.0, 0.0, 1.0, time=TIME_WITH_GAP),
        ),
    }

    result = score_clearance(FlightRecord(streams=streams), SETTINGS, [WALL], WORLD_FRAME)

    assert result.summary['subject_covered_share'] == pytest.approx(0.8, abs=0.01)
