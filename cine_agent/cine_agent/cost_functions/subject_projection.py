"""Pinhole back-projection of a detected subject into the world frame.

Optical frame: z forward, x right, y down (Hartley and Zisserman, Multiple View
Geometry, 2nd ed., section 6.1). Body frame follows REP-103: x forward, y left,
z up. World is ENU.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraIntrinsics:
    focal_x: float
    focal_y: float
    center_x: float
    center_y: float


def scale_pixel_to_image(
    pixel_u: float,
    pixel_v: float,
    source_size: tuple[int, int],
    target_size: tuple[int, int],
) -> tuple[float, float]:
    return (
        pixel_u * target_size[0] / source_size[0],
        pixel_v * target_size[1] / source_size[1],
    )


def _rotate_by_quaternion(
    vector: tuple[float, float, float],
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = quaternion_xyzw
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if norm > 0.0:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    vx, vy, vz = vector

    return (
        (1 - 2 * (qy * qy + qz * qz)) * vx
        + 2 * (qx * qy - qz * qw) * vy
        + 2 * (qx * qz + qy * qw) * vz,
        2 * (qx * qy + qz * qw) * vx
        + (1 - 2 * (qx * qx + qz * qz)) * vy
        + 2 * (qy * qz - qx * qw) * vz,
        2 * (qx * qz - qy * qw) * vx
        + 2 * (qy * qz + qx * qw) * vy
        + (1 - 2 * (qx * qx + qy * qy)) * vz,
    )


def camera_pixel_to_world(
    pixel_u: float,
    pixel_v: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
    drone_position: tuple[float, float, float],
    drone_orientation_xyzw: tuple[float, float, float, float],
    mount_offset_m: tuple[float, float, float] = (0.05, 0.0, 0.0),
) -> tuple[float, float, float]:
    optical_x = (pixel_u - intrinsics.center_x) * depth_m / intrinsics.focal_x
    optical_y = (pixel_v - intrinsics.center_y) * depth_m / intrinsics.focal_y

    body_vector = (
        depth_m + mount_offset_m[0],
        -optical_x + mount_offset_m[1],
        -optical_y + mount_offset_m[2],
    )
    rotated = _rotate_by_quaternion(body_vector, drone_orientation_xyzw)

    return (
        drone_position[0] + rotated[0],
        drone_position[1] + rotated[1],
        drone_position[2] + rotated[2],
    )
