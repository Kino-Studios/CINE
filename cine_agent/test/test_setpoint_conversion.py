import math

import pytest
from cine_agent.setpoint_sender.setpoint_conversion import position_command_to_setpoint


def test_yaw_zero_gives_identity_orientation():
    setpoint = position_command_to_setpoint(1.0, 2.0, 3.0, 0.0)

    assert (setpoint.x, setpoint.y, setpoint.z) == (1.0, 2.0, 3.0)
    assert (setpoint.qx, setpoint.qy, setpoint.qz, setpoint.qw) == (0.0, 0.0, 0.0, 1.0)


def test_yaw_quarter_turn_rotates_about_z():
    setpoint = position_command_to_setpoint(0.0, 0.0, 0.0, math.pi / 2)

    assert setpoint.qz == pytest.approx(math.sqrt(0.5))
    assert setpoint.qw == pytest.approx(math.sqrt(0.5))


def test_non_finite_command_is_dropped():
    assert position_command_to_setpoint(float('nan'), 0.0, 1.0, 0.0) is None


def test_non_finite_yaw_is_dropped():
    assert position_command_to_setpoint(0.0, 0.0, 1.0, float('inf')) is None
