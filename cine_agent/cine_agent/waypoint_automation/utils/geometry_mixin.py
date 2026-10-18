class GeometryMixin:
    def _normalize3(self, v):
        x, y, z = float(v[0]), float(v[1]), float(v[2])
        n = (x * x + y * y + z * z) ** 0.5
        if n <= 1e-9:
            return (0.0, 0.0, 0.0), 0.0

        return (x / n, y / n, z / n), n

    def _clamp_waypoint_to_bounds(self, wp):
        x, y, z = wp
        ox, oy = (self.origin_offset or (0.0, 0.0, 0.0))[:2]
        x_min = self.world_bounds_min[0] + ox
        x_max = self.world_bounds_max[0] + ox
        y_min = self.world_bounds_min[1] + oy
        y_max = self.world_bounds_max[1] + oy

        x = max(x_min, min(x_max, float(x)))
        y = max(y_min, min(y_max, float(y)))
        original_z = float(z)
        if self.enable_z_clamp:
            ceiling = (
                float(self.z_ceiling_override_m)
                if self.z_ceiling_override_m > 0.0
                else min(self.world_bounds_max[2], self.virtual_ceiling_m)
            )
            z = max(self.world_bounds_min[2], min(ceiling, float(z)))
            z = max(self.min_altitude_m, min(self.max_altitude_m, z))
            if abs(z - original_z) > 1e-6:
                self.get_logger().debug(f"Z clamped from {original_z:.2f} to {z:.2f}")
        else:
            z = max(self.world_bounds_min[2], max(self.min_altitude_m, float(z)))

        return (x, y, z)

    def _rel_to_world(self, target_rel):
        origin = self.origin_offset if self.origin_offset is not None else (0.0, 0.0, 0.0)
        target_world = (
            target_rel[0] + origin[0],
            target_rel[1] + origin[1],
            target_rel[2] + origin[2],
        )

        return self._clamp_waypoint_to_bounds(target_world)

    def _distance_to(self, target_world):
        dx = self.current_pos[0] - target_world[0]
        dy = self.current_pos[1] - target_world[1]
        dz = self.current_pos[2] - target_world[2]

        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _closest_waypoint_index(self, target_world):
        if self.active_waypoints is None:
            return None
        best_idx = None
        best_dist_sq = float("inf")
        tx, ty, tz = (
            float(target_world[0]),
            float(target_world[1]),
            float(target_world[2]),
        )
        for i, wp_rel in enumerate(self.active_waypoints):
            wp_world = self._rel_to_world(wp_rel)
            dx = float(wp_world[0]) - tx
            dy = float(wp_world[1]) - ty
            dz = float(wp_world[2]) - tz
            dist_sq = dx * dx + dy * dy + dz * dz
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_idx = i

        return best_idx
