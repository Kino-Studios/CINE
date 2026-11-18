from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def _rotate(pose: Pose, x: float, y: float, z: float) -> tuple[float, float, float]:
    tx = 2.0 * (pose.qy * z - pose.qz * y)
    ty = 2.0 * (pose.qz * x - pose.qx * z)
    tz = 2.0 * (pose.qx * y - pose.qy * x)

    return (
        x + pose.qw * tx + (pose.qy * tz - pose.qz * ty),
        y + pose.qw * ty + (pose.qz * tx - pose.qx * tz),
        z + pose.qw * tz + (pose.qx * ty - pose.qy * tx),
    )


def compose_camera_pose(body: Pose, body_to_camera: Pose) -> Pose:
    offset_x, offset_y, offset_z = _rotate(
        body, body_to_camera.x, body_to_camera.y, body_to_camera.z
    )

    return Pose(
        body.x + offset_x,
        body.y + offset_y,
        body.z + offset_z,
        body.qw * body_to_camera.qx
        + body.qx * body_to_camera.qw
        + body.qy * body_to_camera.qz
        - body.qz * body_to_camera.qy,
        body.qw * body_to_camera.qy
        - body.qx * body_to_camera.qz
        + body.qy * body_to_camera.qw
        + body.qz * body_to_camera.qx,
        body.qw * body_to_camera.qz
        + body.qx * body_to_camera.qy
        - body.qy * body_to_camera.qx
        + body.qz * body_to_camera.qw,
        body.qw * body_to_camera.qw
        - body.qx * body_to_camera.qx
        - body.qy * body_to_camera.qy
        - body.qz * body_to_camera.qz,
    )
