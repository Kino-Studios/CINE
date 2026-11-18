import math

import pytest
from cine_agent.camera_pose.camera_pose import Pose, compose_camera_pose

LEVEL_AT_ORIGIN = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
BODY_TO_CAMERA = Pose(0.05, 0.0, 0.0, -0.5, 0.5, -0.5, 0.5)


def rotate(quaternion, vector):
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    tx = 2 * (qy * vz - qz * vy)
    ty = 2 * (qz * vx - qx * vz)
    tz = 2 * (qx * vy - qy * vx)

    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def test_level_drone_camera_looks_along_body_x():
    camera = compose_camera_pose(LEVEL_AT_ORIGIN, BODY_TO_CAMERA)

    optical_axis = rotate((camera.qx, camera.qy, camera.qz, camera.qw), (0, 0, 1))
    assert optical_axis == pytest.approx((1, 0, 0), abs=1e-9)
    assert (camera.x, camera.y, camera.z) == pytest.approx((0.05, 0.0, 0.0))


def test_yawed_drone_moves_camera_offset_with_it():
    yawed_left = Pose(1.0, 2.0, 3.0, 0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))

    camera = compose_camera_pose(yawed_left, BODY_TO_CAMERA)

    assert (camera.x, camera.y, camera.z) == pytest.approx((1.0, 2.05, 3.0))
