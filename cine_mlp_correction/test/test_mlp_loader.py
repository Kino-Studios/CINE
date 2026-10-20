import numpy as np
import pytest

pytest.importorskip('onnxruntime')

from cine_mlp_correction.mlp_loader import MLPLoader  # noqa: E402


def test_missing_model_passes_control_points_through_unchanged(tmp_path):
    control_points = np.array([[0.0, 0.0, 1.0], [0.5, 0.0, 1.0], [1.0, 0.0, 1.0]])

    corrected = MLPLoader(str(tmp_path)).correct(control_points, dt=0.1)

    assert corrected is control_points
