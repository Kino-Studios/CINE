import pytest
from cine_depth.output_size import output_size, scaled_intrinsics


def test_zero_request_keeps_input_size():
    assert output_size(1152, 648, 0, 0) == (1152, 648)


def test_width_only_keeps_aspect():
    assert output_size(1152, 648, 640, 0) == (640, 360)


def test_explicit_size_is_used():
    assert output_size(1152, 648, 640, 360) == (640, 360)


def test_height_only_keeps_aspect():
    assert output_size(1152, 648, 0, 360) == (640, 360)


def test_intrinsics_scale_with_width():
    focal_x, focal_y, centre_x, centre_y = scaled_intrinsics(
        497.5985575, 497.5985575, 576, 324, 1152, 640
    )

    assert focal_x == pytest.approx(276.444, abs=1e-3)
    assert focal_y == pytest.approx(276.444, abs=1e-3)
    assert centre_x == pytest.approx(320.0, abs=1e-3)
    assert centre_y == pytest.approx(180.0, abs=1e-3)
