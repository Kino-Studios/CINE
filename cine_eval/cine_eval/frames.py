"""Rigid frame transforms recorded with a run, applied to positions before they are compared.

A transform is stored in record.metadata['frame_transforms'] under '<source>-><target>' as
[x, y, z, qx, qy, qz, qw]: a source-frame point is rotated by the unit quaternion, then
translated by (x, y, z), to give the same point in the target frame.
"""

import numpy as np

TRANSFORMS_KEY = 'frame_transforms'


class FrameError(ValueError):
    """The positions cannot be put into the requested frame; the message says why."""


def positions_in_frame(
    metadata: dict,
    positions,
    source_frame: str,
    target_frame: str,
    stream_name: str,
    target_name: str,
) -> np.ndarray:
    """Returns the (N, 3) positions expressed in target_frame, or raises FrameError."""

    positions = np.asarray(positions, dtype=float)

    if source_frame == target_frame:
        return positions

    transform_key = f'{source_frame}->{target_frame}'
    transform = metadata.get(TRANSFORMS_KEY, {}).get(transform_key)
    if transform is None:
        raise FrameError(
            f'{stream_name} is in frame {source_frame!r} but {target_name} is in '
            f'frame {target_frame!r}, and the record has no transform {transform_key!r}',
        )

    values = np.asarray(transform, dtype=float)
    if values.shape != (7,) or not np.all(np.isfinite(values)):
        raise FrameError(f'transform {transform_key!r} is not seven finite numbers')

    quaternion_norm = np.linalg.norm(values[3:])
    if quaternion_norm < 1e-9:
        raise FrameError(f'transform {transform_key!r} has a zero rotation quaternion')

    rotation = _rotation_matrix(values[3:] / quaternion_norm)

    return positions @ rotation.T + values[:3]


def _rotation_matrix(unit_quaternion: np.ndarray) -> np.ndarray:
    qx, qy, qz, qw = unit_quaternion

    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
    )
