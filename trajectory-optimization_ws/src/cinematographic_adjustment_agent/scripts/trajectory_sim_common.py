"""Shared B-spline trajectory primitives for data-collection scripts.

Both trajectory_data_collector.py and simple_trajectory_collector.py use
these classes. Extracted to eliminate duplication while preserving all behavior.
"""

import logging
from typing import List, Tuple

import numpy as np
import pandas as pd

logger = logging.getLogger(__name__)

# -- Module constants (previously magic numbers scattered across scripts) --

# Orbit scenario geometry
ORBIT_RADIUS_M: float = 3.0
ORBIT_ALTITUDE_M: float = 1.5

# Finite-difference step for numerical derivatives.
# 1e-5 balances truncation vs. floating-point cancellation at float64.
FINITE_DIFF_STEP: float = 1e-5

# Number of u-samples for trajectory length estimation and sampling
TRAJECTORY_SAMPLE_COUNT: int = 100

# Control-point count when re-sampling trajectory for constraint enforcement
CONSTRAINED_CONTROL_POINTS: int = 50


class BSplineTrajectory:
    """Uniform clamped B-spline curve in R^3.

    Parameterised on u in [0, 1]. Basis functions use the standard
    Cox–de Boor recursion; knot vector is clamped (first and last
    `degree + 1` knots coincide at 0 and 1).
    """

    def __init__(self, control_points: np.ndarray, degree: int = 3) -> None:
        self.control_points = control_points
        self.degree = degree
        self.n_control = len(control_points)

        n_knots = self.n_control + self.degree + 1
        self.knots = self._generate_uniform_knots(n_knots)

    def _generate_uniform_knots(self, n_knots: int) -> np.ndarray:
        knots = np.zeros(n_knots)
        n_internal = n_knots - 2 * (self.degree + 1)

        knots[: self.degree + 1] = 0
        if n_internal > 0:
            knots[self.degree + 1 : self.degree + 1 + n_internal] = np.linspace(
                0, 1, n_internal + 2
            )[1:-1]
        knots[-(self.degree + 1) :] = 1

        return knots

    def _basis_function(self, i: int, k: int, u: float) -> float:
        """Cox–de Boor recursion: N_{i,k}(u)."""
        if k == 0:
            return 1.0 if self.knots[i] <= u < self.knots[i + 1] else 0.0

        # Edge case: clamp basis to 1 at u == 1 for the last active segment
        if u == 1.0 and i == len(self.knots) - self.degree - 2:
            return 1.0

        denom1 = self.knots[i + k] - self.knots[i]
        denom2 = self.knots[i + k + 1] - self.knots[i + 1]

        term1 = 0.0
        if denom1 > 1e-10:
            term1 = (u - self.knots[i]) / denom1 * self._basis_function(i, k - 1, u)

        term2 = 0.0
        if denom2 > 1e-10:
            term2 = (self.knots[i + k + 1] - u) / denom2 * self._basis_function(i + 1, k - 1, u)

        return term1 + term2

    def evaluate(self, u: float) -> np.ndarray:
        """Evaluate curve position at parameter u in [0, 1]."""
        u = np.clip(u, 0.0, 1.0)
        position = np.zeros(3)

        for i in range(self.n_control):
            basis = self._basis_function(i, self.degree, u)
            position += basis * self.control_points[i]

        return position

    def evaluate_derivative(self, u: float, order: int = 1) -> np.ndarray:
        """Numerical derivative via central finite differences.

        Uses one-sided differences near the boundary to avoid out-of-range u.
        Supports order 1 (velocity) and order 2 (acceleration).
        """
        h = FINITE_DIFF_STEP

        if order == 1:
            if u + h > 1.0:
                return (self.evaluate(u) - self.evaluate(u - h)) / h
            elif u - h < 0.0:
                return (self.evaluate(u + h) - self.evaluate(u)) / h
            else:
                return (self.evaluate(u + h) - self.evaluate(u - h)) / (2 * h)

        elif order == 2:
            if u + h > 1.0 or u - h < 0.0:
                return np.zeros(3)
            else:
                return (self.evaluate(u + h) - 2 * self.evaluate(u) + self.evaluate(u - h)) / (
                    h * h
                )

        else:
            raise ValueError(f"Derivative order {order} not supported")

    def sample_trajectory(
        self, n_samples: int = TRAJECTORY_SAMPLE_COUNT
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Sample positions, first-order and second-order derivatives across u in [0,1].

        Returns:
            positions:     (n_samples, 3)
            velocities:    (n_samples, 3)  — d/du, not d/dt
            accelerations: (n_samples, 3)  — d²/du², not d²/dt²
        """
        u_values = np.linspace(0, 1, n_samples)
        positions = np.array([self.evaluate(u) for u in u_values])
        velocities = np.array([self.evaluate_derivative(u, order=1) for u in u_values])
        accelerations = np.array([self.evaluate_derivative(u, order=2) for u in u_values])

        return positions, velocities, accelerations


class TrajectorySimulator:
    """Simulate a B-spline trajectory with optional cinematographic velocity constraints.

    Focus regions mark u-parameter intervals where `FOCUS_MAX_VELOCITY_MPS` applies;
    elsewhere `DEFAULT_MAX_VELOCITY_MPS` is used.
    """

    def __init__(self, bspline: BSplineTrajectory, constraints: "CinematicConstraints") -> None:  # noqa: F821  # ros-injected
        self.bspline = bspline
        self.constraints = constraints
        self.focus_regions: List[Tuple[float, float]] = []

    def add_focus_region(self, u_start: float, u_end: float) -> None:
        self.focus_regions.append((u_start, u_end))

    def is_focus_active(self, u: float) -> bool:
        for u_start, u_end in self.focus_regions:
            if u_start <= u <= u_end:
                return True
        return False

    def _estimate_total_time(self) -> float:
        positions, _, _ = self.bspline.sample_trajectory(n_samples=TRAJECTORY_SAMPLE_COUNT)
        total_length = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
        return total_length / self.constraints.DEFAULT_MAX_VELOCITY_MPS

    def simulate_baseline(self, dt: float = 0.05, total_time: float = None) -> pd.DataFrame:
        """Simulate the raw B-spline trajectory without velocity constraints.

        Args:
            dt:         Time step in seconds.
            total_time: Override total duration; estimated from arc-length if None.

        Returns:
            DataFrame with columns:
            time, u, x, y, z, vx, vy, vz, speed, has_focus, constraint_mode
        """
        if total_time is None:
            total_time = self._estimate_total_time()

        n_steps = int(total_time / dt)
        u_values = np.linspace(0, 1, n_steps)

        data = []
        for i, u in enumerate(u_values):
            t = i * dt
            pos = self.bspline.evaluate(u)
            vel_derivative = self.bspline.evaluate_derivative(u, order=1)
            du_dt = 1.0 / total_time
            vel = vel_derivative * du_dt
            speed = np.linalg.norm(vel)
            has_focus = self.is_focus_active(u)

            data.append(
                {
                    "time": t,
                    "u": u,
                    "x": pos[0],
                    "y": pos[1],
                    "z": pos[2],
                    "vx": vel[0],
                    "vy": vel[1],
                    "vz": vel[2],
                    "speed": speed,
                    "has_focus": has_focus,
                    "constraint_mode": "baseline",
                }
            )

        return pd.DataFrame(data)

    def simulate_constrained(self, dt: float = 0.05, total_time: float = None) -> pd.DataFrame:
        """Simulate trajectory with cinematographic velocity constraints applied.

        Re-samples `CONSTRAINED_CONTROL_POINTS` control points from the original
        B-spline, enforces per-segment velocity limits, fits a new B-spline, then
        simulates that constrained curve.

        Args:
            dt:         Time step in seconds.
            total_time: Override total duration; estimated from arc-length if None.

        Returns:
            DataFrame with columns:
            time, u, x, y, z, vx, vy, vz, speed, has_focus, constraint_mode
        """
        if total_time is None:
            total_time = self._estimate_total_time()

        u_control = np.linspace(0, 1, CONSTRAINED_CONTROL_POINTS)
        pos_pts = np.array([self.bspline.evaluate(u) for u in u_control])
        constrained_pts = self._enforce_velocity_constraints(pos_pts, dt)
        constrained_bspline = BSplineTrajectory(constrained_pts, degree=self.bspline.degree)

        n_steps = int(total_time / dt)
        u_values = np.linspace(0, 1, n_steps)

        data = []
        for i, u in enumerate(u_values):
            t = i * dt
            pos = constrained_bspline.evaluate(u)
            vel_derivative = constrained_bspline.evaluate_derivative(u, order=1)
            du_dt = 1.0 / total_time
            vel = vel_derivative * du_dt
            speed = np.linalg.norm(vel)
            has_focus = self.is_focus_active(u)

            data.append(
                {
                    "time": t,
                    "u": u,
                    "x": pos[0],
                    "y": pos[1],
                    "z": pos[2],
                    "vx": vel[0],
                    "vy": vel[1],
                    "vz": vel[2],
                    "speed": speed,
                    "has_focus": has_focus,
                    "constraint_mode": "focus" if has_focus else "default",
                }
            )

        return pd.DataFrame(data)

    def _enforce_velocity_constraints(self, pos_pts: np.ndarray, dt: float) -> np.ndarray:
        """Clip inter-point velocity to the applicable per-region maximum.

        Iterates control points in order; if the step from point i-1 to point i
        exceeds the velocity limit it scales point i back along the same direction.

        Args:
            pos_pts: (N, 3) array of position control points.
            dt:      Time step used to convert displacement to velocity.

        Returns:
            (N, 3) array with velocity-constrained positions (copy, input unchanged).
        """
        pts = pos_pts.copy()
        if pts.shape[0] < 2:
            return pts

        n_pts = pts.shape[0]
        u_values = np.linspace(0, 1, n_pts)

        for i in range(1, n_pts):
            u = u_values[i]
            has_focus = self.is_focus_active(u)
            max_vel = (
                self.constraints.FOCUS_MAX_VELOCITY_MPS
                if has_focus
                else self.constraints.DEFAULT_MAX_VELOCITY_MPS
            )

            delta = pts[i] - pts[i - 1]
            vel = delta / dt
            speed = np.linalg.norm(vel)

            if speed > max_vel and speed > 0:
                scale = max_vel / speed
                pts[i] = pts[i - 1] + delta * scale

        return pts
