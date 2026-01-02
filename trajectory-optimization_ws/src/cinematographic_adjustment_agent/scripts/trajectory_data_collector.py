import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional
import math
from pathlib import Path
from datetime import datetime

import sys
import os
from pathlib import Path

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
        
        # First (degree+1) knots are 0
        knots[:self.degree + 1] = 0
        
        # Internal knots are evenly spaced
        if n_internal > 0:
            knots[self.degree + 1:self.degree + 1 + n_internal] = np.linspace(0, 1, n_internal + 2)[1:-1]
        
        # Last (degree+1) knots are 1
        knots[-(self.degree + 1):] = 1
        
        return knots
    
    def _basis_function(self, i: int, k: int, u: float) -> float:
        """
        Recursive B-spline basis function N_i,k(u)
        
        Args:
            i: Basis function index
            k: Degree
            u: Parameter value [0, 1]
        """
        if k == 0:
            return 1.0 if self.knots[i] <= u < self.knots[i + 1] else 0.0
        
        # Handle edge case at u = 1.0
        if u == 1.0 and i == len(self.knots) - self.degree - 2:
            return 1.0
        
        # Recursive definition
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
    
    def sample_trajectory(self, n_samples: int = 100) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        u_values = np.linspace(0, 1, n_samples)
        positions = np.array([self.evaluate(u) for u in u_values])
        velocities = np.array([self.evaluate_derivative(u, order=1) for u in u_values])
        accelerations = np.array([self.evaluate_derivative(u, order=2) for u in u_values])
        
        return positions, velocities, accelerations


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
            positions, _, _ = self.bspline.sample_trajectory(n_samples=100)
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
        """
        Simulate trajectory with cinematographic velocity constraints
        
        Args:
            dt: Time step (seconds)
            total_time: Total simulation time (auto-computed if None)
        
        Returns:
            DataFrame with columns: time, u, x, y, z, vx, vy, vz, speed, has_focus, constraint_mode
        """
        if total_time is None:
            # Estimate time based on trajectory length and max velocity
            positions, _, _ = self.bspline.sample_trajectory(n_samples=100)
            total_length = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
            total_time = total_length / self.constraints.DEFAULT_MAX_VELOCITY_MPS
        
        # Sample trajectory to get control points for constraint enforcement
        n_control = 50
        u_control = np.linspace(0, 1, n_control)
        pos_pts = np.array([self.bspline.evaluate(u) for u in u_control])
        
        # Apply velocity constraints to control points
        constrained_pts = self._enforce_velocity_constraints(pos_pts, dt)
        
        # Create new B-spline from constrained control points
        constrained_bspline = BSplineTrajectory(constrained_pts, degree=self.bspline.degree)
        
        # Simulate with constrained trajectory
        n_steps = int(total_time / dt)
        u_values = np.linspace(0, 1, n_steps)
        
        data = []
        for i, u in enumerate(u_values):
            t = i * dt
            pos = constrained_bspline.evaluate(u)
            vel_derivative = constrained_bspline.evaluate_derivative(u, order=1)
            
            # Scale derivative by parameter velocity
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
        """
        Apply velocity constraints to position control points
        Based on cinematic_bspline_filter.py algorithm
        """
        pts = pos_pts.copy()
        
        if pts.shape[0] < 2:
            return pts
        
        # Determine which control points are in focus regions
        n_pts = pts.shape[0]
        u_values = np.linspace(0, 1, n_pts)
        
        for i in range(1, n_pts):
            u = u_values[i]
            has_focus = self.is_focus_active(u)
            
            # Select max velocity based on focus state
            max_vel = (self.constraints.FOCUS_MAX_VELOCITY_MPS if has_focus 
                      else self.constraints.DEFAULT_MAX_VELOCITY_MPS)
            
            # Compute velocity between consecutive points
            delta = pts[i] - pts[i - 1]
            vel = delta / dt
            speed = np.linalg.norm(vel)
            
            # Scale down if velocity exceeds limit
            if speed > max_vel and speed > 0:
                scale = max_vel / speed
                pts[i] = pts[i - 1] + delta * scale
        
        return pts


class DataCollector:
    """Collects and analyzes trajectory comparison data"""
    
    def __init__(self, output_dir: str = "./trajectory_data"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True, parents=True)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    def generate_test_trajectory(self, scenario: str = "simple") -> Tuple[BSplineTrajectory, List[Tuple[float, float]]]:
        if scenario == "simple":
            waypoints = np.array([
                [0.0, 0.0, 1.5],
                [2.0, 0.0, 1.5],
                [4.0, 0.0, 1.5],
                [6.0, 0.0, 1.5],
                [8.0, 0.0, 1.5],
            ])
            focus_regions = [(0.4, 0.6)]
        
        elif scenario == "orbit":
            n_points = 8
            angles = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
            radius = 3.0
            altitude = 1.5
            
            waypoints = np.array([
                [radius * np.cos(a), radius * np.sin(a), altitude]
                for a in angles
            ])
            
            focus_regions = [(0.2, 0.3), (0.7, 0.8)]
        
        elif scenario == "zigzag":
            waypoints = np.array([
                [0.0, 0.0, 1.5],
                [1.0, 1.0, 1.5],
                [2.0, 0.0, 1.5],
                [3.0, 1.0, 1.5],
                [4.0, 0.0, 1.5],
                [5.0, 1.0, 1.5],
                [6.0, 0.0, 1.5],
            ])
            focus_regions = [(0.3, 0.4), (0.6, 0.7)]
        
        else:
            raise ValueError(f"Unknown scenario: {scenario}")
        
        bspline = BSplineTrajectory(waypoints)
        return bspline, focus_regions
    
    def collect_comparison_data(self, scenario: str = "simple") -> Tuple[pd.DataFrame, pd.DataFrame]:
        print(f"Generating {scenario} scenario...")
        bspline, focus_regions = self.generate_test_trajectory(scenario)
        
        constraints = CinematicConstraints()
        simulator = TrajectorySimulator(bspline, constraints)
        
        # Add focus regions
        for u_start, u_end in focus_regions:
            simulator.add_focus_region(u_start, u_end)
        
        print("Simulating baseline trajectory...")
        baseline_df = simulator.simulate_baseline(dt=0.05)
        
        print("Simulating constrained trajectory...")
        constrained_df = simulator.simulate_constrained(dt=0.05)
        
        return baseline_df, constrained_df
    
    def save_data(self, baseline_df: pd.DataFrame, constrained_df: pd.DataFrame, scenario: str):
        baseline_path = self.output_dir / f"baseline_{scenario}_{self.timestamp}.csv"
        constrained_path = self.output_dir / f"constrained_{scenario}_{self.timestamp}.csv"
        
        baseline_df.to_csv(baseline_path, index=False)
        constrained_df.to_csv(constrained_path, index=False)
        
        print(f"Saved baseline data to: {baseline_path}")
        print(f"Saved constrained data to: {constrained_path}")
    
    def plot_comparison(self, baseline_df: pd.DataFrame, constrained_df: pd.DataFrame, scenario: str):
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle(f'Trajectory Comparison: {scenario.capitalize()} Scenario', fontsize=16)
        
        ax = axes[0, 0]
        ax.plot(baseline_df['x'], baseline_df['y'], baseline_df['z'], 
                'b-', label='Baseline', alpha=0.7, linewidth=2)
        ax.plot(constrained_df['x'], constrained_df['y'], constrained_df['z'], 
                'r--', label='Constrained', alpha=0.7, linewidth=2)
        
        focus_mask_baseline = baseline_df['has_focus']
        if focus_mask_baseline.any():
            ax.scatter(baseline_df.loc[focus_mask_baseline, 'x'],
                      baseline_df.loc[focus_mask_baseline, 'y'],
                      baseline_df.loc[focus_mask_baseline, 'z'],
                      c='orange', s=20, alpha=0.3, label='Focus Region')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('3D Trajectory')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        ax = axes[0, 1]
        ax.plot(baseline_df['x'], baseline_df['y'], 'b-', label='Baseline', alpha=0.7, linewidth=2)
        ax.plot(constrained_df['x'], constrained_df['y'], 'r--', label='Constrained', alpha=0.7, linewidth=2)
        
        if focus_mask_baseline.any():
            ax.scatter(baseline_df.loc[focus_mask_baseline, 'x'],
                      baseline_df.loc[focus_mask_baseline, 'y'],
                      c='orange', s=30, alpha=0.3, label='Focus Region')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('XY Trajectory (Top View)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        ax = axes[1, 0]
        ax.plot(baseline_df['time'], baseline_df['speed'], 'b-', label='Baseline', alpha=0.7, linewidth=2)
        ax.plot(constrained_df['time'], constrained_df['speed'], 'r--', label='Constrained', alpha=0.7, linewidth=2)
        
        constraints = CinematicConstraints()
        ax.axhline(y=constraints.DEFAULT_MAX_VELOCITY_MPS, color='g', linestyle=':', 
                  label=f'Default Max ({constraints.DEFAULT_MAX_VELOCITY_MPS} m/s)')
        ax.axhline(y=constraints.FOCUS_MAX_VELOCITY_MPS, color='orange', linestyle=':', 
                  label=f'Focus Max ({constraints.FOCUS_MAX_VELOCITY_MPS} m/s)')
        
        for i in range(len(baseline_df)):
            if baseline_df.loc[i, 'has_focus']:
                t_start = baseline_df.loc[i, 'time']
                t_end = baseline_df.loc[min(i+1, len(baseline_df)-1), 'time']
                ax.axvspan(t_start, t_end, alpha=0.2, color='orange')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        ax.set_title('Speed Comparison')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        ax = axes[1, 1]
        ax.plot(baseline_df['time'], baseline_df['vx'], 'b-', label='Vx Baseline', alpha=0.5)
        ax.plot(baseline_df['time'], baseline_df['vy'], 'g-', label='Vy Baseline', alpha=0.5)
        ax.plot(baseline_df['time'], baseline_df['vz'], 'r-', label='Vz Baseline', alpha=0.5)
        ax.plot(constrained_df['time'], constrained_df['vx'], 'b--', label='Vx Constrained', alpha=0.5)
        ax.plot(constrained_df['time'], constrained_df['vy'], 'g--', label='Vy Constrained', alpha=0.5)
        ax.plot(constrained_df['time'], constrained_df['vz'], 'r--', label='Vz Constrained', alpha=0.5)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Velocity Components')
        ax.legend(ncol=2)
        ax.grid(True, alpha=0.3)
        
        ax = axes[2, 0]
        pos_diff = np.sqrt((baseline_df['x'] - constrained_df['x'])**2 +
                          (baseline_df['y'] - constrained_df['y'])**2 +
                          (baseline_df['z'] - constrained_df['z'])**2)
        ax.plot(baseline_df['time'], pos_diff, 'purple', linewidth=2)
        
        for i in range(len(baseline_df)):
            if baseline_df.loc[i, 'has_focus']:
                t_start = baseline_df.loc[i, 'time']
                t_end = baseline_df.loc[min(i+1, len(baseline_df)-1), 'time']
                ax.axvspan(t_start, t_end, alpha=0.2, color='orange')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position Difference (m)')
        ax.set_title('Position Deviation (Baseline vs Constrained)')
        ax.grid(True, alpha=0.3)
        
        ax = axes[2, 1]
        speed_diff = baseline_df['speed'] - constrained_df['speed']
        ax.plot(baseline_df['time'], speed_diff, 'purple', linewidth=2)
        ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        
        for i in range(len(baseline_df)):
            if baseline_df.loc[i, 'has_focus']:
                t_start = baseline_df.loc[i, 'time']
                t_end = baseline_df.loc[min(i+1, len(baseline_df)-1), 'time']
                ax.axvspan(t_start, t_end, alpha=0.2, color='orange')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed Difference (m/s)')
        ax.set_title('Speed Reduction (Baseline - Constrained)')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        plot_path = self.output_dir / f"comparison_{scenario}_{self.timestamp}.png"
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"Saved comparison plot to: {plot_path}")
        
        plt.show()
    
    def print_statistics(self, baseline_df: pd.DataFrame, constrained_df: pd.DataFrame, scenario: str):
        print(f"\n{'='*60}")
        print(f"Statistics for {scenario.upper()} scenario")
        print(f"{'='*60}")
        
        print("\nOVERALL STATISTICS:")
        print(f"  Baseline - Mean speed: {baseline_df['speed'].mean():.3f} m/s, "
              f"Max speed: {baseline_df['speed'].max():.3f} m/s")
        print(f"  Constrained - Mean speed: {constrained_df['speed'].mean():.3f} m/s, "
              f"Max speed: {constrained_df['speed'].max():.3f} m/s")
        
        baseline_focus = baseline_df[baseline_df['has_focus']]
        constrained_focus = constrained_df[constrained_df['has_focus']]
        
        if len(baseline_focus) > 0:
            print("\nFOCUS REGION STATISTICS:")
            print(f"  Baseline - Mean speed: {baseline_focus['speed'].mean():.3f} m/s, "
                  f"Max speed: {baseline_focus['speed'].max():.3f} m/s")
            print(f"  Constrained - Mean speed: {constrained_focus['speed'].mean():.3f} m/s, "
                  f"Max speed: {constrained_focus['speed'].max():.3f} m/s")
            print(f"  Speed reduction: {(baseline_focus['speed'].mean() - constrained_focus['speed'].mean()):.3f} m/s "
                  f"({100 * (baseline_focus['speed'].mean() - constrained_focus['speed'].mean()) / baseline_focus['speed'].mean():.1f}%)")
        
        baseline_nofocus = baseline_df[~baseline_df['has_focus']]
        constrained_nofocus = constrained_df[~constrained_df['has_focus']]
        
        if len(baseline_nofocus) > 0:
            print("\nNON-FOCUS REGION STATISTICS:")
            print(f"  Baseline - Mean speed: {baseline_nofocus['speed'].mean():.3f} m/s")
            print(f"  Constrained - Mean speed: {constrained_nofocus['speed'].mean():.3f} m/s")
        
        pos_diff = np.sqrt((baseline_df['x'] - constrained_df['x'])**2 +
                          (baseline_df['y'] - constrained_df['y'])**2 +
                          (baseline_df['z'] - constrained_df['z'])**2)
        print(f"\nPOSITION DEVIATION:")
        print(f"  Mean: {pos_diff.mean():.3f} m, Max: {pos_diff.max():.3f} m")
        
        print(f"{'='*60}\n")


def main():
    print("Cinematographic Trajectory Data Collection Tool")
    print("=" * 60)
    
    collector = DataCollector(output_dir="./trajectory_data")
    
    scenarios = ["simple", "orbit", "zigzag"]
    
    for scenario in scenarios:
        print(f"\n{'='*60}")
        print(f"Processing scenario: {scenario.upper()}")
        print(f"{'='*60}")
        
        baseline_df, constrained_df = collector.collect_comparison_data(scenario)
        
        collector.save_data(baseline_df, constrained_df, scenario)
        
        collector.print_statistics(baseline_df, constrained_df, scenario)
        
        collector.plot_comparison(baseline_df, constrained_df, scenario)
    
    print("\n" + "="*60)
    print("Data collection complete!")
    print(f"Results saved to: {collector.output_dir}")
    print("="*60)


if __name__ == "__main__":
    main()
