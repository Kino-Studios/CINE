from types import SimpleNamespace

import pytest
from cine_gazebo.odometry_frames import body_twist_in_world, quaternion_xyzw

IDENTITY = (0.0, 0.0, 0.0, 1.0)
YAW_90_DEGREES = (0.0, 0.0, 0.7071067811865476, 0.7071067811865476)
# Roll 30 degrees, then yaw 60 degrees: every component differs, so a swapped order shows.
ROLL_30_THEN_YAW_60_DEGREES = (
    0.2241438680420134,
    0.12940952255126034,
    0.4829629131445341,
    0.8365163037378079,
)


def test_identity_orientation_leaves_the_linear_velocity_unchanged():
    twist = body_twist_in_world(IDENTITY, (1.0, 2.0, 3.0), (0.1, 0.2, 0.3))

    assert twist.linear == (1.0, 2.0, 3.0)


def test_identity_orientation_leaves_the_angular_velocity_unchanged():
    twist = body_twist_in_world(IDENTITY, (1.0, 2.0, 3.0), (0.1, 0.2, 0.3))

    assert twist.angular == (0.1, 0.2, 0.3)


def test_yaw_of_90_degrees_maps_body_forward_velocity_to_world_y():
    twist = body_twist_in_world(YAW_90_DEGREES, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    assert twist.linear == pytest.approx((0.0, 1.0, 0.0))


def test_yaw_of_90_degrees_maps_body_roll_rate_to_world_y():
    twist = body_twist_in_world(YAW_90_DEGREES, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))

    assert twist.angular == pytest.approx((0.0, 1.0, 0.0))


def test_roll_then_yaw_rotates_body_velocity_into_world():
    twist = body_twist_in_world(ROLL_30_THEN_YAW_60_DEGREES, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0))

    assert twist.linear == pytest.approx((0.299038106, 0.982050808, 3.598076211))


def test_quaternion_message_converts_in_x_y_z_w_order():
    message = SimpleNamespace(x=0.1, y=0.2, z=0.3, w=0.9)

    assert quaternion_xyzw(message) == (0.1, 0.2, 0.3, 0.9)


def test_non_unit_quaternion_is_normalized_before_rotating():
    twist = body_twist_in_world((0.0, 0.0, 2.0, 2.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    assert twist.linear == pytest.approx((0.0, 1.0, 0.0))


def test_zero_quaternion_is_rejected_as_no_orientation():
    with pytest.raises(ValueError, match='orientation'):
        body_twist_in_world((0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_infinite_quaternion_is_rejected_as_no_orientation():
    with pytest.raises(ValueError, match='orientation'):
        body_twist_in_world((float('inf'), 0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
