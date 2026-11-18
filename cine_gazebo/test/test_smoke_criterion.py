import math

from smoke_flight import (
    AUTOPILOT_SOURCE,
    GROUND_TRUTH_SOURCE,
    PositionSample,
    closest_distances,
    reached_setpoint,
)

TARGET = (0.0, 2.0, 2.5)


def test_both_sources_reaching_inside_the_deadline_passes():
    samples = [
        PositionSample(time_s=1.0, source=AUTOPILOT_SOURCE, position=(0.0, 0.5, 2.5)),
        PositionSample(time_s=12.0, source=AUTOPILOT_SOURCE, position=(0.1, 1.8, 2.4)),
        PositionSample(time_s=14.0, source=GROUND_TRUTH_SOURCE, position=(0.0, 2.2, 2.6)),
    ]

    assert reached_setpoint(samples, TARGET, tolerance_m=0.5, deadline_s=30.0)


def test_reaching_after_the_deadline_fails():
    samples = [
        PositionSample(time_s=12.0, source=AUTOPILOT_SOURCE, position=(0.0, 2.0, 2.5)),
        PositionSample(time_s=31.0, source=GROUND_TRUTH_SOURCE, position=(0.0, 2.0, 2.5)),
    ]

    assert not reached_setpoint(samples, TARGET, tolerance_m=0.5, deadline_s=30.0)


def test_only_one_source_reaching_fails():
    samples = [
        PositionSample(time_s=12.0, source=AUTOPILOT_SOURCE, position=(0.0, 2.0, 2.5)),
        PositionSample(time_s=12.0, source=GROUND_TRUTH_SOURCE, position=(0.0, 0.9, 2.5)),
    ]

    assert not reached_setpoint(samples, TARGET, tolerance_m=0.5, deadline_s=30.0)


def test_closest_distance_is_reported_per_source():
    samples = [
        PositionSample(time_s=5.0, source=GROUND_TRUTH_SOURCE, position=(0.0, 0.0, 2.5)),
        PositionSample(time_s=9.0, source=GROUND_TRUTH_SOURCE, position=(0.0, 1.0, 2.5)),
    ]

    assert closest_distances(samples, TARGET) == {
        AUTOPILOT_SOURCE: math.inf,
        GROUND_TRUTH_SOURCE: 1.0,
    }
