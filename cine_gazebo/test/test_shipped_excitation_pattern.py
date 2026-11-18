import math
from pathlib import Path

from cine_gazebo.sitl_observation import load_excitation_pattern

PATTERN_FILE = Path(__file__).parents[1] / 'config' / 'excitation_pattern.yaml'
# The world's nearest obstacle is a hedge whose face is 8.25 m east of the takeoff point; the
# pattern stays inside a 4 m box centred on the takeoff point, 1.5 m or more above the ground.
BOX_HALF_WIDTH_M = 2.0
MINIMUM_ALTITUDE_M = 1.5
MAXIMUM_ALTITUDE_M = 3.5


def _axis_values(axis_index):
    return [point[axis_index] for point in load_excitation_pattern(PATTERN_FILE)]


def test_pattern_moves_at_least_one_and_a_half_metres_along_east():
    values = _axis_values(0)

    assert max(values) - min(values) >= 1.5


def test_pattern_moves_at_least_one_and_a_half_metres_along_north():
    values = _axis_values(1)

    assert max(values) - min(values) >= 1.5


def test_pattern_moves_at_least_one_and_a_half_metres_in_height():
    values = _axis_values(2)

    assert max(values) - min(values) >= 1.5


def test_pattern_swings_yaw_sixty_degrees_to_both_sides():
    values = _axis_values(3)

    assert max(values) >= math.radians(60) - 1e-3
    assert min(values) <= -math.radians(60) + 1e-3


def test_pattern_stays_within_sixty_degrees_of_the_takeoff_heading():
    values = _axis_values(3)

    assert max(abs(value) for value in values) <= math.radians(60) + 1e-3


def test_pattern_stays_inside_the_four_metre_box_around_the_takeoff_point():
    east_values = _axis_values(0)
    north_values = _axis_values(1)

    assert max(abs(value) for value in east_values + north_values) <= BOX_HALF_WIDTH_M


def test_pattern_keeps_to_safe_altitudes():
    values = _axis_values(2)

    assert min(values) >= MINIMUM_ALTITUDE_M
    assert max(values) <= MAXIMUM_ALTITUDE_M
