import math

import pytest
from cine_agent.cost_functions.subject_projection import (
    CameraIntrinsics,
    camera_pixel_to_world,
    scale_pixel_to_image,
)

INTRINSICS = CameraIntrinsics(focal_x=500.0, focal_y=400.0, center_x=320.0, center_y=180.0)
LEVEL_FACING_EAST = (0.0, 0.0, 0.0, 1.0)


def test_subject_on_optical_axis_lands_ahead_when_level_facing_east():
    world = camera_pixel_to_world(320.0, 180.0, 5.0, INTRINSICS, (0.0, 0.0, 0.0), LEVEL_FACING_EAST)

    assert world == pytest.approx((5.05, 0.0, 0.0), abs=1e-9)


def test_subject_on_optical_axis_lands_north_when_drone_yawed_ninety_degrees():
    yaw_ninety = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))

    world = camera_pixel_to_world(320.0, 180.0, 5.0, INTRINSICS, (1.0, 2.0, 3.0), yaw_ninety)

    assert world == pytest.approx((1.0, 7.05, 3.0), abs=1e-9)


def test_pixel_right_of_centre_lands_to_the_drone_right():
    world = camera_pixel_to_world(
        INTRINSICS.center_x + INTRINSICS.focal_x,
        180.0,
        2.0,
        INTRINSICS,
        (0.0, 0.0, 0.0),
        LEVEL_FACING_EAST,
    )

    assert world[1] == pytest.approx(-2.0, abs=1e-9)


def test_pixel_below_centre_lands_below_the_drone():
    world = camera_pixel_to_world(
        320.0,
        INTRINSICS.center_y + INTRINSICS.focal_y,
        2.0,
        INTRINSICS,
        (0.0, 0.0, 7.0),
        LEVEL_FACING_EAST,
    )

    assert world[2] == pytest.approx(5.0, abs=1e-9)


def test_detection_pixel_scales_to_depth_image():
    scaled_u, scaled_v = scale_pixel_to_image(1151, 647, (1152, 648), (640, 360))

    assert scaled_u == 1151 * 640 / 1152
    assert scaled_v == 647 * 360 / 648
