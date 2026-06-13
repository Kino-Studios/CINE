"""Cinematographic Trajectory Data Collection Tool.

Generates and compares baseline vs. velocity-constrained B-spline trajectories
for simple, orbit, and zigzag scenarios. Saves CSV data and comparison plots.
"""

import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# -- repo-root path bootstrap to find 'configs' --
_here = Path(__file__).resolve()
for _parent in [_here] + list(_here.parents):
    if (_parent / "configs").is_dir():
        _constants_path = _parent / "configs" / "python-constants"
        if str(_constants_path) not in sys.path:
            sys.path.insert(0, str(_constants_path))
        break

from constants import CinematicConstraints
from trajectory_sim_common import (
    ORBIT_ALTITUDE_M,
    ORBIT_RADIUS_M,
    BSplineTrajectory,
    TrajectorySimulator,
)

logger = logging.getLogger(__name__)

# Altitude used for non-orbit scenarios (straight-line, zigzag)
DEFAULT_ALTITUDE_M: float = ORBIT_ALTITUDE_M


class DataCollector:
    """Collect and analyse trajectory comparison data across scenarios."""

    def __init__(self, output_dir: str = "./trajectory_data") -> None:
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True, parents=True)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    def generate_test_trajectory(
        self, scenario: str = "simple"
    ) -> Tuple[BSplineTrajectory, List[Tuple[float, float]]]:
        """Build a BSplineTrajectory and focus-region list for a named scenario."""
        if scenario == "simple":
            waypoints = np.array(
                [
                    [0.0, 0.0, DEFAULT_ALTITUDE_M],
                    [2.0, 0.0, DEFAULT_ALTITUDE_M],
                    [4.0, 0.0, DEFAULT_ALTITUDE_M],
                    [6.0, 0.0, DEFAULT_ALTITUDE_M],
                    [8.0, 0.0, DEFAULT_ALTITUDE_M],
                ]
            )
            focus_regions = [(0.4, 0.6)]

        elif scenario == "orbit":
            n_points = 8
            angles = np.linspace(0, 2 * np.pi, n_points, endpoint=False)

            waypoints = np.array(
                [
                    [ORBIT_RADIUS_M * np.cos(a), ORBIT_RADIUS_M * np.sin(a), ORBIT_ALTITUDE_M]
                    for a in angles
                ]
            )
            focus_regions = [(0.2, 0.3), (0.7, 0.8)]

        elif scenario == "zigzag":
            waypoints = np.array(
                [
                    [0.0, 0.0, DEFAULT_ALTITUDE_M],
                    [1.0, 1.0, DEFAULT_ALTITUDE_M],
                    [2.0, 0.0, DEFAULT_ALTITUDE_M],
                    [3.0, 1.0, DEFAULT_ALTITUDE_M],
                    [4.0, 0.0, DEFAULT_ALTITUDE_M],
                    [5.0, 1.0, DEFAULT_ALTITUDE_M],
                    [6.0, 0.0, DEFAULT_ALTITUDE_M],
                ]
            )
            focus_regions = [(0.3, 0.4), (0.6, 0.7)]

        else:
            raise ValueError(f"Unknown scenario: {scenario}")

        bspline = BSplineTrajectory(waypoints)
        return bspline, focus_regions

    def collect_comparison_data(
        self, scenario: str = "simple"
    ) -> Tuple[pd.DataFrame, pd.DataFrame]:
        logger.info("Generating %s scenario...", scenario)
        bspline, focus_regions = self.generate_test_trajectory(scenario)

        constraints = CinematicConstraints()
        simulator = TrajectorySimulator(bspline, constraints)

        for u_start, u_end in focus_regions:
            simulator.add_focus_region(u_start, u_end)

        logger.info("Simulating baseline trajectory...")
        baseline_df = simulator.simulate_baseline(dt=0.05)

        logger.info("Simulating constrained trajectory...")
        constrained_df = simulator.simulate_constrained(dt=0.05)

        return baseline_df, constrained_df

    def save_data(
        self,
        baseline_df: pd.DataFrame,
        constrained_df: pd.DataFrame,
        scenario: str,
    ) -> None:
        baseline_path = self.output_dir / f"baseline_{scenario}_{self.timestamp}.csv"
        constrained_path = self.output_dir / f"constrained_{scenario}_{self.timestamp}.csv"

        baseline_df.to_csv(baseline_path, index=False)
        constrained_df.to_csv(constrained_path, index=False)

        logger.info("Saved baseline data to: %s", baseline_path)
        logger.info("Saved constrained data to: %s", constrained_path)

    def plot_comparison(
        self,
        baseline_df: pd.DataFrame,
        constrained_df: pd.DataFrame,
        scenario: str,
    ) -> None:
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle(f"Trajectory Comparison: {scenario.capitalize()} Scenario", fontsize=16)

        focus_mask_baseline = baseline_df["has_focus"]

        ax = axes[0, 0]
        ax.plot(
            baseline_df["x"],
            baseline_df["y"],
            baseline_df["z"],
            "b-",
            label="Baseline",
            alpha=0.7,
            linewidth=2,
        )
        ax.plot(
            constrained_df["x"],
            constrained_df["y"],
            constrained_df["z"],
            "r--",
            label="Constrained",
            alpha=0.7,
            linewidth=2,
        )
        if focus_mask_baseline.any():
            ax.scatter(
                baseline_df.loc[focus_mask_baseline, "x"],
                baseline_df.loc[focus_mask_baseline, "y"],
                baseline_df.loc[focus_mask_baseline, "z"],
                c="orange",
                s=20,
                alpha=0.3,
                label="Focus Region",
            )
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("3D Trajectory")
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[0, 1]
        ax.plot(baseline_df["x"], baseline_df["y"], "b-", label="Baseline", alpha=0.7, linewidth=2)
        ax.plot(
            constrained_df["x"],
            constrained_df["y"],
            "r--",
            label="Constrained",
            alpha=0.7,
            linewidth=2,
        )
        if focus_mask_baseline.any():
            ax.scatter(
                baseline_df.loc[focus_mask_baseline, "x"],
                baseline_df.loc[focus_mask_baseline, "y"],
                c="orange",
                s=30,
                alpha=0.3,
                label="Focus Region",
            )
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("XY Trajectory (Top View)")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis("equal")

        constraints = CinematicConstraints()
        ax = axes[1, 0]
        ax.plot(
            baseline_df["time"],
            baseline_df["speed"],
            "b-",
            label="Baseline",
            alpha=0.7,
            linewidth=2,
        )
        ax.plot(
            constrained_df["time"],
            constrained_df["speed"],
            "r--",
            label="Constrained",
            alpha=0.7,
            linewidth=2,
        )
        ax.axhline(
            y=constraints.DEFAULT_MAX_VELOCITY_MPS,
            color="g",
            linestyle=":",
            label=f"Default Max ({constraints.DEFAULT_MAX_VELOCITY_MPS} m/s)",
        )
        ax.axhline(
            y=constraints.FOCUS_MAX_VELOCITY_MPS,
            color="orange",
            linestyle=":",
            label=f"Focus Max ({constraints.FOCUS_MAX_VELOCITY_MPS} m/s)",
        )
        for i in range(len(baseline_df)):
            if baseline_df.loc[i, "has_focus"]:
                t_start = baseline_df.loc[i, "time"]
                t_end = baseline_df.loc[min(i + 1, len(baseline_df) - 1), "time"]
                ax.axvspan(t_start, t_end, alpha=0.2, color="orange")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Speed (m/s)")
        ax.set_title("Speed Comparison")
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[1, 1]
        for comp, style in [("vx", "b"), ("vy", "g"), ("vz", "r")]:
            ax.plot(
                baseline_df["time"],
                baseline_df[comp],
                f"{style}-",
                label=f"{comp.upper()} Baseline",
                alpha=0.5,
            )
            ax.plot(
                constrained_df["time"],
                constrained_df[comp],
                f"{style}--",
                label=f"{comp.upper()} Constrained",
                alpha=0.5,
            )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.set_title("Velocity Components")
        ax.legend(ncol=2)
        ax.grid(True, alpha=0.3)

        pos_diff = np.sqrt(
            (baseline_df["x"] - constrained_df["x"]) ** 2
            + (baseline_df["y"] - constrained_df["y"]) ** 2
            + (baseline_df["z"] - constrained_df["z"]) ** 2
        )
        ax = axes[2, 0]
        ax.plot(baseline_df["time"], pos_diff, "purple", linewidth=2)
        for i in range(len(baseline_df)):
            if baseline_df.loc[i, "has_focus"]:
                t_start = baseline_df.loc[i, "time"]
                t_end = baseline_df.loc[min(i + 1, len(baseline_df) - 1), "time"]
                ax.axvspan(t_start, t_end, alpha=0.2, color="orange")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position Difference (m)")
        ax.set_title("Position Deviation (Baseline vs Constrained)")
        ax.grid(True, alpha=0.3)

        ax = axes[2, 1]
        speed_diff = baseline_df["speed"] - constrained_df["speed"]
        ax.plot(baseline_df["time"], speed_diff, "purple", linewidth=2)
        ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
        for i in range(len(baseline_df)):
            if baseline_df.loc[i, "has_focus"]:
                t_start = baseline_df.loc[i, "time"]
                t_end = baseline_df.loc[min(i + 1, len(baseline_df) - 1), "time"]
                ax.axvspan(t_start, t_end, alpha=0.2, color="orange")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Speed Difference (m/s)")
        ax.set_title("Speed Reduction (Baseline - Constrained)")
        ax.grid(True, alpha=0.3)

        plt.tight_layout()

        plot_path = self.output_dir / f"comparison_{scenario}_{self.timestamp}.png"
        plt.savefig(plot_path, dpi=300, bbox_inches="tight")
        logger.info("Saved comparison plot to: %s", plot_path)

        plt.show()

    def print_statistics(
        self,
        baseline_df: pd.DataFrame,
        constrained_df: pd.DataFrame,
        scenario: str,
    ) -> None:
        sep = "=" * 60
        logger.info("%s\nStatistics for %s scenario\n%s", sep, scenario.upper(), sep)

        logger.info(
            "OVERALL: Baseline mean=%.3f m/s max=%.3f m/s | Constrained mean=%.3f m/s max=%.3f m/s",
            baseline_df["speed"].mean(),
            baseline_df["speed"].max(),
            constrained_df["speed"].mean(),
            constrained_df["speed"].max(),
        )

        baseline_focus = baseline_df[baseline_df["has_focus"]]
        constrained_focus = constrained_df[constrained_df["has_focus"]]
        if len(baseline_focus) > 0:
            reduction = baseline_focus["speed"].mean() - constrained_focus["speed"].mean()
            pct = 100.0 * reduction / baseline_focus["speed"].mean()
            logger.info(
                "FOCUS: Baseline mean=%.3f max=%.3f | Constrained mean=%.3f max=%.3f | "
                "Reduction=%.3f m/s (%.1f%%)",
                baseline_focus["speed"].mean(),
                baseline_focus["speed"].max(),
                constrained_focus["speed"].mean(),
                constrained_focus["speed"].max(),
                reduction,
                pct,
            )

        baseline_nofocus = baseline_df[~baseline_df["has_focus"]]
        constrained_nofocus = constrained_df[~constrained_df["has_focus"]]
        if len(baseline_nofocus) > 0:
            logger.info(
                "NON-FOCUS: Baseline mean=%.3f m/s | Constrained mean=%.3f m/s",
                baseline_nofocus["speed"].mean(),
                constrained_nofocus["speed"].mean(),
            )

        pos_diff = np.sqrt(
            (baseline_df["x"] - constrained_df["x"]) ** 2
            + (baseline_df["y"] - constrained_df["y"]) ** 2
            + (baseline_df["z"] - constrained_df["z"]) ** 2
        )
        logger.info("POSITION DEVIATION: mean=%.3f m max=%.3f m", pos_diff.mean(), pos_diff.max())


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    logger.info("Cinematographic Trajectory Data Collection Tool")

    collector = DataCollector(output_dir="./trajectory_data")
    scenarios = ["simple", "orbit", "zigzag"]

    for scenario in scenarios:
        logger.info("Processing scenario: %s", scenario.upper())
        baseline_df, constrained_df = collector.collect_comparison_data(scenario)
        collector.save_data(baseline_df, constrained_df, scenario)
        collector.print_statistics(baseline_df, constrained_df, scenario)
        collector.plot_comparison(baseline_df, constrained_df, scenario)

    logger.info("Data collection complete. Results saved to: %s", collector.output_dir)


if __name__ == "__main__":
    main()
