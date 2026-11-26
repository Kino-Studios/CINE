import pytest
from cine_gazebo.depth_source import depth_routing


def test_gazebo_depth_renders_the_simulator_depth_camera():
    assert depth_routing('gazebo', mission=True).renders_simulator_depth is True


def test_gazebo_depth_leaves_the_depth_network_off():
    assert depth_routing('gazebo', mission=True).starts_depth_network is False


def test_network_depth_starts_the_depth_network():
    assert depth_routing('network', mission=True).starts_depth_network is True


def test_network_depth_renders_no_simulator_depth_camera():
    assert depth_routing('network', mission=True).renders_simulator_depth is False


def test_smoke_flight_renders_no_simulator_depth_camera():
    assert depth_routing('gazebo', mission=False).renders_simulator_depth is False


def test_smoke_flight_starts_no_depth_network():
    assert depth_routing('network', mission=False).starts_depth_network is False


def test_unknown_depth_source_names_the_choices():
    with pytest.raises(
        ValueError,
        match=r"Unknown depth_source 'lidar'; choose one of \['gazebo', 'network'\]",
    ):
        depth_routing('lidar', mission=True)
