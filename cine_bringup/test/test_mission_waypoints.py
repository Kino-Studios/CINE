import pytest
from cine_bringup.mission_waypoints import load_mission_waypoints


def write_waypoints(tmp_path, text):
    path = tmp_path / 'waypoints.yaml'
    path.write_text(text)

    return str(path)


def test_three_point_file_returns_three_tuples(tmp_path):
    path = write_waypoints(tmp_path, '- [0, 0, 1]\n- [1.5, -2, 1]\n- [3, 4.25, 2]\n')

    assert load_mission_waypoints(path) == [(0.0, 0.0, 1.0), (1.5, -2.0, 1.0), (3.0, 4.25, 2.0)]


def test_five_point_file_is_accepted(tmp_path):
    path = write_waypoints(tmp_path, '[[0, 0, 1], [1, 0, 1], [2, 0, 1], [3, 0, 1], [4, 0, 1]]')

    assert len(load_mission_waypoints(path)) == 5


def test_six_point_file_raises_naming_the_limit(tmp_path):
    path = write_waypoints(
        tmp_path,
        '[[0, 0, 1], [1, 0, 1], [2, 0, 1], [3, 0, 1], [4, 0, 1], [5, 0, 1]]',
    )

    with pytest.raises(ValueError, match='at most 5 waypoints'):
        load_mission_waypoints(path)


def test_point_with_two_numbers_raises(tmp_path):
    path = write_waypoints(tmp_path, '- [0, 0, 1]\n- [1, 2]\n')

    with pytest.raises(ValueError, match='3 numbers'):
        load_mission_waypoints(path)


def test_empty_file_raises(tmp_path):
    path = write_waypoints(tmp_path, '')

    with pytest.raises(ValueError, match='empty'):
        load_mission_waypoints(path)


def test_top_level_mapping_raises(tmp_path):
    path = write_waypoints(tmp_path, 'points: [[0, 0, 1]]\n')

    with pytest.raises(ValueError, match='list'):
        load_mission_waypoints(path)


def test_non_numeric_coordinate_raises(tmp_path):
    path = write_waypoints(tmp_path, '- [0, high, 1]\n')

    with pytest.raises(ValueError, match='3 numbers'):
        load_mission_waypoints(path)
