import numpy as np
from cine_eval.record import DRONE_TRUE, FRAMING_ACTIVE, FlightRecord, Stream
from cine_eval.timeline import (
    filming_mask,
    fixed_clock,
    largest_gap,
    sample,
    sample_hold,
    smoothed_derivatives,
)


def _stream(name, time, **fields):
    time = np.asarray(time, dtype=float)

    return Stream(
        name=name,
        frame_id='map',
        time=time,
        receive_time=time.copy(),
        fields=fields,
    )


def test_sample_hold_keeps_last_cue_value():
    cue = _stream(FRAMING_ACTIVE, [1.0, 3.0], active=np.array([True, False]))
    clock = fixed_clock(0.0, 4.0, 1.0)

    held = sample_hold(cue, 'active', clock)

    assert held.tolist() == [False, True, True, False, False]


def test_sample_hold_of_floats_is_nan_before_first_sample():
    stream = _stream(DRONE_TRUE, [1.0, 2.0], value=np.array([5.0, 6.0]))

    held = sample_hold(stream, 'value', np.array([0.0, 1.5, 2.5]))

    assert np.isnan(held[0])
    assert held[1:].tolist() == [5.0, 6.0]


def test_sample_interpolates_each_column_linearly():
    stream = _stream(
        DRONE_TRUE,
        [0.0, 2.0],
        position=np.array([[0.0, 10.0, 0.0], [2.0, 20.0, 4.0]]),
    )

    values = sample(stream, 'position', np.array([1.0]))

    assert values.tolist() == [[1.0, 15.0, 2.0]]


def test_sample_outside_span_is_nan():
    stream = _stream(DRONE_TRUE, [1.0, 2.0], value=np.array([1.0, 2.0]))

    values = sample(stream, 'value', np.array([0.0, 1.5, 3.0]))

    assert np.isnan(values[0])
    assert values[1] == 1.5
    assert np.isnan(values[2])


def test_fixed_clock_includes_both_ends():
    assert fixed_clock(0.0, 1.0, 4.0).tolist() == [0.0, 0.25, 0.5, 0.75, 1.0]


def test_smoothed_derivatives_recover_constant_acceleration():
    clock = fixed_clock(0.0, 4.0, 50.0)
    position = (0.5 * 2.0 * clock**2)[:, None]

    _, acceleration, jerk = smoothed_derivatives(position, 50.0, 0.3, 3)

    assert np.allclose(acceleration[20:-20], 2.0, atol=1e-6)
    assert np.allclose(jerk[20:-20], 0.0, atol=1e-6)


def test_filming_mask_is_none_without_cue():
    record = FlightRecord(
        streams={DRONE_TRUE: _stream(DRONE_TRUE, [0.0, 1.0], position=np.zeros((2, 3)))},
        metadata={},
    )

    assert filming_mask(record, np.array([0.0, 0.5])) is None


def test_filming_mask_follows_the_framing_cue():
    cue = _stream(FRAMING_ACTIVE, [1.0, 3.0], active=np.array([True, False]))
    record = FlightRecord(streams={FRAMING_ACTIVE: cue}, metadata={})

    mask = filming_mask(record, fixed_clock(0.0, 4.0, 1.0))

    assert mask.tolist() == [False, True, True, False, False]


def test_smoothed_derivatives_of_segment_too_short_for_the_fit_are_nan():
    position = np.arange(4.0)[:, None] * np.ones((1, 3))

    velocity, acceleration, jerk = smoothed_derivatives(position, 50.0, 0.3, 3)

    assert velocity.shape == position.shape
    assert np.isnan(velocity).all()
    assert np.isnan(acceleration).all()
    assert np.isnan(jerk).all()


def test_smoothed_derivatives_shrink_the_window_to_fit_a_short_segment():
    clock = fixed_clock(0.0, 7.0 / 50.0, 50.0)
    position = (0.5 * 2.0 * clock**2)[:, None]

    _, acceleration, _ = smoothed_derivatives(position, 50.0, 0.3, 3)

    assert np.allclose(acceleration, 2.0, atol=1e-6)


def test_sample_is_nan_between_source_samples_further_apart_than_the_largest_gap():
    stream = _stream(DRONE_TRUE, [0.0, 1.0, 3.0, 4.0], value=np.array([0.0, 1.0, 3.0, 4.0]))

    values = sample(stream, 'value', np.array([0.5, 2.0, 3.5]), max_gap_s=1.5)

    assert values[0] == 0.5
    assert np.isnan(values[1])
    assert values[2] == 3.5


def test_sample_keeps_the_source_sample_that_opens_a_gap():
    stream = _stream(DRONE_TRUE, [0.0, 1.0, 3.0], value=np.array([0.0, 1.0, 3.0]))

    values = sample(stream, 'value', np.array([1.0]), max_gap_s=1.5)

    assert values.tolist() == [1.0]


def test_sample_of_vectors_is_nan_in_every_column_inside_a_gap():
    stream = _stream(DRONE_TRUE, [0.0, 3.0], position=np.array([[0.0, 0.0, 0.0], [3.0, 3.0, 3.0]]))

    values = sample(stream, 'position', np.array([1.5]), max_gap_s=1.0)

    assert np.isnan(values).all()


def test_largest_gap_names_its_length_and_start():
    gap = largest_gap(np.array([0.0, 1.0, 3.5, 4.0]), 0.0, 4.0)

    assert gap.length_s == 2.5
    assert gap.start_s == 1.0


def test_largest_gap_counts_the_wait_before_the_first_sample_of_the_window():
    gap = largest_gap(np.array([2.0, 2.5, 3.0]), 0.0, 3.0)

    assert gap.length_s == 2.0
    assert gap.start_s == 0.0


def test_largest_gap_counts_the_wait_after_the_last_sample_of_the_window():
    gap = largest_gap(np.array([0.0, 0.5, 1.0]), 0.0, 4.0)

    assert gap.length_s == 3.0
    assert gap.start_s == 1.0
