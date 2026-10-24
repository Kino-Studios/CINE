"""Turns Depth Anything 3 network output into depth in metres."""

import numpy as np

# From the Depth Anything 3 README: metric depth = focal_px * network_output / 300.
_METRIC_FOCAL_DIVISOR = 300.0


def metric_depth_metres(
    network_depth: np.ndarray, camera_focal_px: float, calibration_image_width: int
) -> np.ndarray:
    """Scales a metric model's output, which is at the model's processing resolution.

    The focal length must be the one at that resolution, so it is rescaled from the width the
    camera was calibrated at to the width of ``network_depth``.
    """
    processed_width = network_depth.shape[1]
    processed_focal_px = camera_focal_px * processed_width / calibration_image_width

    return (processed_focal_px * network_depth / _METRIC_FOCAL_DIVISOR).astype(np.float32)


def relative_depth_metres(network_depth: np.ndarray, max_depth_m: float) -> np.ndarray:
    """Stretches relative depth over 0 to ``max_depth_m``; only the ordering is meaningful."""
    depth_min = float(network_depth.min())
    depth_max = float(network_depth.max())
    if depth_max <= depth_min:
        return np.zeros_like(network_depth, dtype=np.float32)

    return ((network_depth - depth_min) / (depth_max - depth_min) * max_depth_m).astype(np.float32)
