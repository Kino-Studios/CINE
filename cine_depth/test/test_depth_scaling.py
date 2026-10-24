import numpy as np
import pytest
from cine_depth.depth_scaling import metric_depth_metres, relative_depth_metres


def test_metric_depth_uses_focal_length_over_300():
    network_depth = np.full((4, 6), 3.0, dtype=np.float32)
    depth = metric_depth_metres(network_depth, camera_focal_px=600.0, calibration_image_width=6)
    assert depth[0, 0] == pytest.approx(6.0)


def test_metric_depth_rescales_focal_length_to_processed_width():
    network_depth = np.full((4, 6), 3.0, dtype=np.float32)
    depth = metric_depth_metres(network_depth, camera_focal_px=600.0, calibration_image_width=12)
    assert depth[0, 0] == pytest.approx(3.0)


def test_relative_depth_spans_zero_to_max_depth():
    network_depth = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    depth = relative_depth_metres(network_depth, max_depth_m=10.0)
    np.testing.assert_allclose(depth, [[0.0, 5.0, 10.0]])


def test_relative_depth_of_flat_image_is_zero():
    network_depth = np.full((2, 2), 4.0, dtype=np.float32)
    depth = relative_depth_metres(network_depth, max_depth_m=10.0)
    assert not depth.any()
