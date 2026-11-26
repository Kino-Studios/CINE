import ast
from pathlib import Path

import pytest
from cine_gazebo.odometry_routing import RELAY_RELIABILITY, mission_odometry

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
GAZEBO_LAUNCH_FILE = REPOSITORY_ROOT / 'cine_gazebo' / 'launch' / 'gazebo_sim.launch.py'
CINEMATIC_RUN_LAUNCH_FILE = REPOSITORY_ROOT / 'cine_bringup' / 'launch' / 'cinematic_run.launch.py'


def _declared_argument_names(launch_file):
    return {
        node.args[0].value
        for node in ast.walk(ast.parse(launch_file.read_text()))
        if isinstance(node, ast.Call)
        and getattr(node.func, 'id', None) == 'DeclareLaunchArgument'
        and node.args
        and isinstance(node.args[0], ast.Constant)
    }


def _cinematic_run_argument_names_passed_by_gazebo_launch():
    return {
        key.value
        for node in ast.walk(ast.parse(GAZEBO_LAUNCH_FILE.read_text()))
        if isinstance(node, ast.keyword) and node.arg == 'launch_arguments'
        for dictionary in ast.walk(node.value)
        if isinstance(dictionary, ast.Dict)
        for key in dictionary.keys
        if isinstance(key, ast.Constant)
    }


def test_sim_mission_reads_the_relayed_odometry():
    assert mission_odometry('sim').odometry_topic == '/cinematography/drone_odometry'


def test_sim_mission_points_the_cost_calculator_at_the_relayed_odometry():
    assert mission_odometry('sim').cost_calculator_odometry_topic == (
        '/cinematography/drone_odometry'
    )


def test_sim_mission_starts_the_odometry_relay():
    assert mission_odometry('sim').starts_relay


def test_vins_mission_reads_vio_odometry_as_before():
    assert mission_odometry('vins').odometry_topic == '/vio/odometry'


def test_vins_mission_keeps_the_cost_calculator_on_imu_propagate():
    assert mission_odometry('vins').cost_calculator_odometry_topic == '/imu_propagate'


def test_vins_mission_starts_no_relay():
    assert not mission_odometry('vins').starts_relay


def test_unknown_nav_source_is_rejected():
    with pytest.raises(ValueError, match='gps'):
        mission_odometry('gps')


def test_sim_mission_passes_the_cost_calculator_topic_as_a_declared_argument():
    # A misspelled launch argument would be accepted silently and leave the default in force.
    passed_arguments = _cinematic_run_argument_names_passed_by_gazebo_launch()

    assert 'cost_calculator_odometry_topic' in passed_arguments
    assert 'cost_calculator_odometry_topic' in _declared_argument_names(CINEMATIC_RUN_LAUNCH_FILE)


def test_relay_subscribes_best_effort_so_it_connects_to_mavros():
    assert RELAY_RELIABILITY.subscriber == 'BEST_EFFORT'


def test_relay_publishes_reliably_so_the_mission_readers_connect():
    assert RELAY_RELIABILITY.publisher == 'RELIABLE'
