from typing import Optional, Tuple

import numpy as np

from ..cost_functions.cost_calculator import CostCalculator
from ..utils.config_loader import ConfigLoader
from ..utils.state_tracker import StateTracker
from ..utils.trajectory_planner import clamp_depth_to_preference, pixel_to_camera_xy


class TargetComputer:
    def __init__(
        self, config: ConfigLoader, tracker: StateTracker, cost_calculator: CostCalculator
    ):
        self.config = config
        self.tracker = tracker
        self.cost_calculator = cost_calculator
        self.center_uv: Optional[Tuple[int, int]] = None

    def update_center(self, u: int, v: int) -> None:
        """Update the raw target center point from bounding box or other sources, applying rule of thirds."""
        adjusted = self._apply_rule_of_thirds((u, v))
        self.center_uv = adjusted
        self.tracker.set_target_observation(adjusted)

    def _apply_rule_of_thirds(self, uv: Tuple[int, int]) -> Tuple[int, int]:
        strength = float(max(0.0, min(1.0, self.config.thirds_strength)))
        if strength <= 0.0:
            return uv

        u, v = int(uv[0]), int(uv[1])
        w = int(max(1, self.config.params.IMAGE_WIDTH))
        h = int(max(1, self.config.params.IMAGE_HEIGHT))

        anchors = [
            (w / 3.0, h / 3.0),
            (2.0 * w / 3.0, h / 3.0),
            (w / 3.0, 2.0 * h / 3.0),
            (2.0 * w / 3.0, 2.0 * h / 3.0),
        ]

        if self.config.thirds_use_nearest_anchor:
            ax, ay = min(anchors, key=lambda a: (a[0] - u) ** 2 + (a[1] - v) ** 2)
        else:
            ax, ay = anchors[1]

        uu = int(round((1.0 - strength) * u + strength * ax))
        vv = int(round((1.0 - strength) * v + strength * ay))
        uu = max(0, min(uu, w - 1))
        vv = max(0, min(vv, h - 1))
        return (uu, vv)

    def compute_target_xy(self) -> Optional[Tuple[float, float, float]]:
        if self.center_uv is None:
            return None

        median_depth = self.cost_calculator.get_median_depth_roi()

        if median_depth is not None:
            smoothed_depth = median_depth
        else:
            smoothed_depth = self.tracker.get_smoothed_depth()

        preferred_depth = clamp_depth_to_preference(
            smoothed_depth,
            self.config.params.TARGET_FOCUS_DISTANCE_M,
            self.config.params.MIN_FOCUS_DISTANCE_M,
        )

        u, v = self.center_uv
        x_cam, y_cam = pixel_to_camera_xy(
            u,
            v,
            preferred_depth,
            self.config.params.CAMERA_FX,
            self.config.params.CAMERA_FY,
            self.config.params.CAMERA_CX,
            self.config.params.CAMERA_CY,
        )

        return x_cam, y_cam, preferred_depth

    def compute_target_world_position(
        self, drone_position: np.ndarray, has_odom: bool
    ) -> Optional[np.ndarray]:
        target_cam = self.compute_target_xy()

        if target_cam is None or not has_odom:
            return None

        x_cam, y_cam, z_cam = target_cam

        target_world = drone_position + np.array([z_cam, -x_cam, -y_cam])

        return target_world
