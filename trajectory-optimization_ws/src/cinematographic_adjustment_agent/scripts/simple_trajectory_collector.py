#!/usr/bin/env python3

import numpy as np
import pandas as pd
from typing import List, Tuple
from pathlib import Path
from datetime import datetime

import sys

# Add repo root to sys.path to find 'configs'
here = Path(__file__).resolve()
for parent in [here] + list(here.parents):
    if (parent / "configs").is_dir():
        python_constants_path = parent / "configs" / "python-constants"
        if str(python_constants_path) not in sys.path:
            sys.path.insert(0, str(python_constants_path))
        break

from constants import CinematicConstraints
class BSplineTrajectory:
    
    def __init__(self, control_points: np.ndarray, degree: int = 3):
        self.control_points = control_points
        self.degree = degree
        self.n_control = len(control_points)
        n_knots = self.n_control + self.degree + 1
        self.knots = self._generate_uniform_knots(n_knots)
    
    def _generate_uniform_knots(self, n_knots: int) -> np.ndarray:
        knots = np.zeros(n_knots)
        n_internal = n_knots - 2 * (self.degree + 1)
        knots[:self.degree + 1] = 0
        if n_internal > 0:
            knots[self.degree + 1:self.degree + 1 + n_internal] = np.linspace(0, 1, n_internal + 2)[1:-1]
        knots[-(self.degree + 1):] = 1
        return knots
    
    def _basis_function(self, i: int, k: int, u: float) -> float:
        if k == 0:
            return 1.0 if self.knots[i] <= u < self.knots[i + 1] else 0.0
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
        u = np.clip(u, 0.0, 1.0)
        position = np.zeros(3)
        for i in range(self.n_control):
            basis = self._basis_function(i, self.degree, u)
            position += basis * self.control_points[i]
        return position
    
    def evaluate_derivative(self, u: float, order: int = 1) -> np.ndarray:
        h = 1e-5
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
                return (self.evaluate(u + h) - 2 * self.evaluate(u) + self.evaluate(u - h)) / (h * h)
        else:
            raise ValueError(f"Derivative order {order} not supported")


class TrajectorySimulator:
    
    def __init__(self, bspline: BSplineTrajectory, constraints: CinematicConstraints):
        self.bspline = bspline
        self.constraints = constraints
        self.focus_regions = []
    
    def add_focus_region(self, u_start: float, u_end: float):
        self.focus_regions.append((u_start, u_end))
    
    def is_focus_active(self, u: float) -> bool:
        for u_start, u_end in self.focus_regions:
            if u_start <= u <= u_end:
                return True
        return False
    
    def simulate_baseline(self, dt: float = 0.05, total_time: float = None) -> pd.DataFrame:
        if total_time is None:
            positions = np.array([self.bspline.evaluate(u) for u in np.linspace(0, 1, 100)])
            total_length = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
            total_time = total_length / self.constraints.DEFAULT_MAX_VELOCITY_MPS
        
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
            
            data.append({
                'time': t,
                'u': u,
                'x': pos[0],
                'y': pos[1],
                'z': pos[2],
                'vx': vel[0],
                'vy': vel[1],
                'vz': vel[2],
                'speed': speed,
                'has_focus': has_focus,
                'constraint_mode': 'baseline'
            })
        
        return pd.DataFrame(data)
    
    def simulate_constrained(self, dt: float = 0.05, total_time: float = None) -> pd.DataFrame:
        if total_time is None:
            positions = np.array([self.bspline.evaluate(u) for u in np.linspace(0, 1, 100)])
            total_length = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
            total_time = total_length / self.constraints.DEFAULT_MAX_VELOCITY_MPS
        
        n_control = 50
        u_control = np.linspace(0, 1, n_control)
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
            
            data.append({
                'time': t,
                'u': u,
                'x': pos[0],
                'y': pos[1],
                'z': pos[2],
                'vx': vel[0],
                'vy': vel[1],
                'vz': vel[2],
                'speed': speed,
                'has_focus': has_focus,
                'constraint_mode': 'focus' if has_focus else 'default'
            })
        
        return pd.DataFrame(data)
    
    def _enforce_velocity_constraints(self, pos_pts: np.ndarray, dt: float) -> np.ndarray:
        pts = pos_pts.copy()
        if pts.shape[0] < 2:
            return pts
        
        n_pts = pts.shape[0]
        u_values = np.linspace(0, 1, n_pts)
        
        for i in range(1, n_pts):
            u = u_values[i]
            has_focus = self.is_focus_active(u)
            max_vel = (self.constraints.FOCUS_MAX_VELOCITY_MPS if has_focus 
                      else self.constraints.DEFAULT_MAX_VELOCITY_MPS)
            delta = pts[i] - pts[i - 1]
            vel = delta / dt
            speed = np.linalg.norm(vel)
            if speed > max_vel and speed > 0:
                scale = max_vel / speed
                pts[i] = pts[i - 1] + delta * scale
        
        return pts


def generate_room_orbit_trajectory() -> Tuple[BSplineTrajectory, List[Tuple[float, float]]]:
    room_center = np.array([0.0, 0.0, 1.5])
    orbit_radius = 3.0
    altitude = 1.5
    n_points = 16
    angles = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
    
    waypoints = np.array([
        [room_center[0] + orbit_radius * np.cos(a), 
         room_center[1] + orbit_radius * np.sin(a), 
         altitude]
        for a in angles
    ])
    
    focus_regions = [(0.0, 1.0)]
    
    bspline = BSplineTrajectory(waypoints)
    return bspline, focus_regions


def main():
    print("Generating room orbit trajectory data...")
    
    output_dir = Path("./trajectory_data")
    output_dir.mkdir(exist_ok=True, parents=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    bspline, focus_regions = generate_room_orbit_trajectory()
    constraints = CinematicConstraints()
    simulator = TrajectorySimulator(bspline, constraints)
    
    for u_start, u_end in focus_regions:
        simulator.add_focus_region(u_start, u_end)
    
    print("Simulating baseline trajectory...")
    baseline_df = simulator.simulate_baseline(dt=0.05)
    
    print("Simulating constrained trajectory...")
    constrained_df = simulator.simulate_constrained(dt=0.05)
    
    baseline_path = output_dir / f"baseline_room_orbit_{timestamp}.csv"
    constrained_path = output_dir / f"constrained_room_orbit_{timestamp}.csv"
    
    baseline_df.to_csv(baseline_path, index=False)
    constrained_df.to_csv(constrained_path, index=False)
    
    print(f"\nBaseline data saved to: {baseline_path}")
    print(f"Constrained data saved to: {constrained_path}")
    
    print(f"\nBaseline - Mean speed: {baseline_df['speed'].mean():.3f} m/s, Max: {baseline_df['speed'].max():.3f} m/s")
    print(f"Constrained - Mean speed: {constrained_df['speed'].mean():.3f} m/s, Max: {constrained_df['speed'].max():.3f} m/s")
    print(f"Speed reduction: {(baseline_df['speed'].mean() - constrained_df['speed'].mean()):.3f} m/s")
    print("\nData collection complete!")


if __name__ == "__main__":
    main()
