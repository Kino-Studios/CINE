import time
from collections import deque

import numpy as np


class StateTracker:
    def __init__(self, look_duration_s: float = 10.0, depth_window_size: int = 15):
        self.look_duration_s = float(look_duration_s)
        self.depth_window_size = int(depth_window_size)
        self.current_target_id: int | None = None
        self.last_target_switch_time_s = time.time()
        self.recent_depth_samples = deque(maxlen=self.depth_window_size)
        self.last_center_uv: tuple[float, float] | None = None

    def should_switch_target(self) -> bool:
        now_s = time.time()

        return (now_s - self.last_target_switch_time_s) >= self.look_duration_s

    def set_target_observation(self, center_uv: tuple[float, float]):
        self.last_center_uv = center_uv

    def record_depth(self, depth_value_m: float):
        if depth_value_m <= 0 or np.isnan(depth_value_m):
            return
        self.recent_depth_samples.append(float(depth_value_m))

    def get_smoothed_depth(self) -> float | None:
        if len(self.recent_depth_samples) == 0:
            return None

        return float(np.median(np.array(self.recent_depth_samples)))

    def rotate_to_next_target(self) -> int:
        self.last_target_switch_time_s = time.time()
        self.recent_depth_samples.clear()
        if self.current_target_id is None:
            self.current_target_id = 0
        else:
            self.current_target_id += 1

        return int(self.current_target_id)
