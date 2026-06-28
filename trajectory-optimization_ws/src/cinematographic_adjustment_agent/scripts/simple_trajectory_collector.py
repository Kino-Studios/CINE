#!/usr/bin/env python3
"""Room-orbit trajectory data collector.

Generates a full-circle orbital trajectory around a room centre, simulates
baseline and velocity-constrained variants, then saves CSV files for analysis.
"""

import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Tuple

import numpy as np

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

ROOM_CENTER: np.ndarray = np.array([0.0, 0.0, ORBIT_ALTITUDE_M])
ORBIT_N_POINTS: int = 16


def generate_room_orbit_trajectory() -> Tuple[BSplineTrajectory, List[Tuple[float, float]]]:
    """Build a full-circle orbital B-spline and a single full-range focus region."""
    angles = np.linspace(0, 2 * np.pi, ORBIT_N_POINTS, endpoint=False)

    waypoints = np.array(
        [
            [
                ROOM_CENTER[0] + ORBIT_RADIUS_M * np.cos(a),
                ROOM_CENTER[1] + ORBIT_RADIUS_M * np.sin(a),
                ORBIT_ALTITUDE_M,
            ]
            for a in angles
        ]
    )

    focus_regions = [(0.0, 1.0)]
    bspline = BSplineTrajectory(waypoints)

    return bspline, focus_regions


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    logger.info("Generating room orbit trajectory data...")

    output_dir = Path("./trajectory_data")
    output_dir.mkdir(exist_ok=True, parents=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    bspline, focus_regions = generate_room_orbit_trajectory()
    constraints = CinematicConstraints()
    simulator = TrajectorySimulator(bspline, constraints)

    for u_start, u_end in focus_regions:
        simulator.add_focus_region(u_start, u_end)

    logger.info("Simulating baseline trajectory...")
    baseline_df = simulator.simulate_baseline(dt=0.05)

    logger.info("Simulating constrained trajectory...")
    constrained_df = simulator.simulate_constrained(dt=0.05)

    baseline_path = output_dir / f"baseline_room_orbit_{timestamp}.csv"
    constrained_path = output_dir / f"constrained_room_orbit_{timestamp}.csv"

    baseline_df.to_csv(baseline_path, index=False)
    constrained_df.to_csv(constrained_path, index=False)

    logger.info("Baseline data saved to: %s", baseline_path)
    logger.info("Constrained data saved to: %s", constrained_path)

    speed_reduction = baseline_df["speed"].mean() - constrained_df["speed"].mean()
    logger.info(
        "Baseline mean=%.3f m/s max=%.3f m/s | Constrained mean=%.3f m/s max=%.3f m/s | "
        "Reduction=%.3f m/s",
        baseline_df["speed"].mean(),
        baseline_df["speed"].max(),
        constrained_df["speed"].mean(),
        constrained_df["speed"].max(),
        speed_reduction,
    )
    logger.info("Data collection complete.")


if __name__ == "__main__":
    main()
