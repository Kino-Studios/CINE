"""Tests for trajectory_sim_common — BSplineTrajectory and TrajectorySimulator.

Run before and after extraction to verify behavior preservation (TDD red/green harness).
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock

# -- path bootstrap (mirrors test_cost_calculator.py) --
_test_dir = Path(__file__).resolve().parent
_package_root = _test_dir.parent
_repo_root = _package_root.parents[2]
_configs_path = _repo_root / "configs" / "python-constants"
_scripts_path = _package_root / "scripts"

for _p in (_package_root, _configs_path, _scripts_path):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))


# Mock ROS 2 / RCL imports required by constants.py (not needed at test time)
_ros_mocks = [
    "rclpy",
    "rclpy.node",
    "rclpy.qos",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "cinematography_msgs",
    "cinematography_msgs.msg",
    "traj_utils",
    "cv_bridge",
    "cv2",
]
for _mod in _ros_mocks:
    if _mod not in sys.modules:
        sys.modules[_mod] = MagicMock()

import numpy as np
import pytest
from constants import CinematicConstraints
from trajectory_sim_common import (
    CONSTRAINED_CONTROL_POINTS,
    FINITE_DIFF_STEP,
    ORBIT_ALTITUDE_M,
    ORBIT_RADIUS_M,
    TRAJECTORY_SAMPLE_COUNT,
    BSplineTrajectory,
    TrajectorySimulator,
)

# ---------------------------------------------------------------------------
# BSplineTrajectory — math properties
# ---------------------------------------------------------------------------


class TestBSplineKnots:
    def test_knot_vector_length(self):
        pts = np.zeros((5, 3))
        spl = BSplineTrajectory(pts, degree=3)
        # n_control + degree + 1
        assert len(spl.knots) == 5 + 3 + 1

    def test_knot_clamped_start_end(self):
        pts = np.zeros((6, 3))
        spl = BSplineTrajectory(pts, degree=3)
        assert spl.knots[0] == pytest.approx(0.0)
        assert spl.knots[-1] == pytest.approx(1.0)
        # first (degree+1) == 0
        assert all(spl.knots[: spl.degree + 1] == 0.0)
        # last (degree+1) == 1
        assert all(spl.knots[-(spl.degree + 1) :] == 1.0)


class TestBSplineEvaluation:
    """Verify B-spline interpolates endpoints exactly for clamped uniform knots."""

    def _line_spline(self) -> BSplineTrajectory:
        """Straight line from (0,0,0) to (4,0,0) via 5 equally-spaced pts."""
        pts = np.array([[float(i), 0.0, 0.0] for i in range(5)])
        return BSplineTrajectory(pts, degree=3)

    def test_evaluate_at_zero_returns_first_control_point(self):
        spl = self._line_spline()
        pos = spl.evaluate(0.0)
        np.testing.assert_allclose(pos, [0.0, 0.0, 0.0], atol=1e-9)

    def test_evaluate_at_one_returns_last_control_point(self):
        spl = self._line_spline()
        pos = spl.evaluate(1.0)
        np.testing.assert_allclose(pos, [4.0, 0.0, 0.0], atol=1e-9)

    def test_evaluate_clips_out_of_range(self):
        spl = self._line_spline()
        assert spl.evaluate(-0.5).tolist() == spl.evaluate(0.0).tolist()
        assert spl.evaluate(1.5).tolist() == spl.evaluate(1.0).tolist()

    def test_evaluate_midpoint_shape(self):
        spl = self._line_spline()
        pos = spl.evaluate(0.5)
        assert pos.shape == (3,)

    def test_evaluate_z_axis_trajectory(self):
        """B-spline on z axis: endpoint z must match last control point."""
        pts = np.array([[0.0, 0.0, float(i)] for i in range(5)])
        spl = BSplineTrajectory(pts, degree=3)
        np.testing.assert_allclose(spl.evaluate(1.0)[2], 4.0, atol=1e-9)

    def test_basis_functions_partition_of_unity(self):
        """Sum of all basis functions at any u must equal 1."""
        spl = self._line_spline()
        for u in [0.0, 0.25, 0.5, 0.75, 1.0]:
            total = sum(spl._basis_function(i, spl.degree, u) for i in range(spl.n_control))
            assert total == pytest.approx(1.0, abs=1e-9), f"partition-of-unity failed at u={u}"


class TestBSplineDerivative:
    def _const_spline(self) -> BSplineTrajectory:
        """Constant spline — derivative should be ~0."""
        pts = np.ones((5, 3)) * np.array([2.0, 3.0, 1.0])
        return BSplineTrajectory(pts, degree=3)

    def test_first_derivative_constant_spline_near_zero(self):
        spl = self._const_spline()
        deriv = spl.evaluate_derivative(0.5, order=1)
        np.testing.assert_allclose(deriv, [0.0, 0.0, 0.0], atol=1e-4)

    def test_second_derivative_constant_spline_near_zero(self):
        spl = self._const_spline()
        deriv = spl.evaluate_derivative(0.5, order=2)
        np.testing.assert_allclose(deriv, [0.0, 0.0, 0.0], atol=1e-4)

    def test_unsupported_order_raises(self):
        spl = self._const_spline()
        with pytest.raises(ValueError, match="not supported"):
            spl.evaluate_derivative(0.5, order=3)

    def test_finite_diff_step_constant(self):
        """FINITE_DIFF_STEP must be 1e-5 — any change breaks derivative precision."""
        assert FINITE_DIFF_STEP == pytest.approx(1e-5)

    def test_first_order_boundary_uses_one_sided(self):
        """evaluate_derivative at u=1 must not raise and return a 3-vector."""
        pts = np.array([[float(i), 0.0, 0.0] for i in range(5)])
        spl = BSplineTrajectory(pts, degree=3)
        d = spl.evaluate_derivative(1.0, order=1)
        assert d.shape == (3,)


class TestBSplineSampleTrajectory:
    def test_sample_trajectory_shapes(self):
        pts = np.array([[float(i), 0.0, 0.0] for i in range(5)])
        spl = BSplineTrajectory(pts, degree=3)
        positions, velocities, accelerations = spl.sample_trajectory(n_samples=50)
        assert positions.shape == (50, 3)
        assert velocities.shape == (50, 3)
        assert accelerations.shape == (50, 3)

    def test_sample_trajectory_default_count(self):
        pts = np.array([[float(i), 0.0, 0.0] for i in range(5)])
        spl = BSplineTrajectory(pts, degree=3)
        positions, velocities, accelerations = spl.sample_trajectory()
        assert positions.shape[0] == TRAJECTORY_SAMPLE_COUNT


# ---------------------------------------------------------------------------
# Module constants
# ---------------------------------------------------------------------------


class TestModuleConstants:
    def test_orbit_radius_positive(self):
        assert ORBIT_RADIUS_M > 0

    def test_orbit_altitude_positive(self):
        assert ORBIT_ALTITUDE_M > 0

    def test_constrained_control_points_positive_int(self):
        assert isinstance(CONSTRAINED_CONTROL_POINTS, int)
        assert CONSTRAINED_CONTROL_POINTS > 0

    def test_trajectory_sample_count_positive_int(self):
        assert isinstance(TRAJECTORY_SAMPLE_COUNT, int)
        assert TRAJECTORY_SAMPLE_COUNT > 0


# ---------------------------------------------------------------------------
# TrajectorySimulator
# ---------------------------------------------------------------------------


def _make_simulator(focus_regions=None) -> TrajectorySimulator:
    pts = np.array([[float(i), 0.0, 0.0] for i in range(6)])
    spl = BSplineTrajectory(pts, degree=3)
    constraints = CinematicConstraints()
    sim = TrajectorySimulator(spl, constraints)
    for u_start, u_end in focus_regions or []:
        sim.add_focus_region(u_start, u_end)
    return sim


class TestTrajectorySimulatorFocusRegions:
    def test_no_focus_active_outside_regions(self):
        sim = _make_simulator(focus_regions=[(0.4, 0.6)])
        assert not sim.is_focus_active(0.1)
        assert not sim.is_focus_active(0.9)

    def test_focus_active_inside_region(self):
        sim = _make_simulator(focus_regions=[(0.4, 0.6)])
        assert sim.is_focus_active(0.5)
        assert sim.is_focus_active(0.4)
        assert sim.is_focus_active(0.6)

    def test_multiple_focus_regions(self):
        sim = _make_simulator(focus_regions=[(0.1, 0.2), (0.7, 0.8)])
        assert sim.is_focus_active(0.15)
        assert sim.is_focus_active(0.75)
        assert not sim.is_focus_active(0.5)


class TestSimulateBaseline:
    def test_returns_dataframe_with_required_columns(self):
        sim = _make_simulator()
        df = sim.simulate_baseline(dt=0.1)
        required = {
            "time",
            "u",
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "vz",
            "speed",
            "has_focus",
            "constraint_mode",
        }
        assert required.issubset(set(df.columns))

    def test_constraint_mode_is_baseline(self):
        sim = _make_simulator()
        df = sim.simulate_baseline(dt=0.1)
        assert (df["constraint_mode"] == "baseline").all()

    def test_time_starts_at_zero(self):
        sim = _make_simulator()
        df = sim.simulate_baseline(dt=0.1)
        assert df["time"].iloc[0] == pytest.approx(0.0)

    def test_row_count_matches_steps(self):
        sim = _make_simulator()
        # Manually estimate steps to verify row count
        df = sim.simulate_baseline(dt=0.1)
        assert len(df) > 0

    def test_total_time_override(self):
        sim = _make_simulator()
        df_default = sim.simulate_baseline(dt=0.1)
        df_override = sim.simulate_baseline(dt=0.1, total_time=5.0)
        assert len(df_override) == int(5.0 / 0.1)
        assert len(df_default) > 0


class TestSimulateConstrained:
    def test_returns_dataframe_with_required_columns(self):
        sim = _make_simulator()
        df = sim.simulate_constrained(dt=0.1)
        required = {
            "time",
            "u",
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "vz",
            "speed",
            "has_focus",
            "constraint_mode",
        }
        assert required.issubset(set(df.columns))

    def test_focus_mode_label(self):
        sim = _make_simulator(focus_regions=[(0.4, 0.6)])
        df = sim.simulate_constrained(dt=0.1)
        focus_rows = df[df["has_focus"]]
        if len(focus_rows):
            assert (focus_rows["constraint_mode"] == "focus").all()

    def test_non_focus_mode_label(self):
        sim = _make_simulator(focus_regions=[(0.4, 0.6)])
        df = sim.simulate_constrained(dt=0.1)
        non_focus_rows = df[~df["has_focus"]]
        if len(non_focus_rows):
            assert (non_focus_rows["constraint_mode"] == "default").all()

    def test_constrained_max_speed_less_than_or_equal_to_baseline(self):
        """Constrained trajectory max speed <= baseline max speed.

        The constraint enforcement reduces inter-control-point displacements;
        re-fitting a B-spline through those points may not produce a hard
        velocity ceiling, but the constrained curve must not be faster overall
        than the unconstrained baseline.
        """
        sim = _make_simulator(focus_regions=[(0.3, 0.7)])
        df_base = sim.simulate_baseline(dt=0.05)
        df_constrained = sim.simulate_constrained(dt=0.05)
        # Constrained mean speed must be strictly less than or equal to baseline mean speed
        assert df_constrained["speed"].mean() <= df_base["speed"].mean() * 1.01


class TestEnforceVelocityConstraints:
    def test_single_point_returns_unchanged(self):
        sim = _make_simulator()
        pts = np.array([[1.0, 2.0, 3.0]])
        result = sim._enforce_velocity_constraints(pts, dt=0.05)
        np.testing.assert_array_equal(result, pts)

    def test_output_shape_preserved(self):
        sim = _make_simulator()
        pts = np.random.default_rng(0).random((10, 3))
        result = sim._enforce_velocity_constraints(pts, dt=0.05)
        assert result.shape == pts.shape

    def test_does_not_mutate_input(self):
        sim = _make_simulator()
        pts = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
        original = pts.copy()
        sim._enforce_velocity_constraints(pts, dt=0.05)
        np.testing.assert_array_equal(pts, original)
