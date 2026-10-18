import math


class PathBuilderMixin:
    def _build_room_orbit(self):
        radius = max(self.min_orbit_radius_m, 1.0)
        radius = min(radius, 1.5)
        num_pts = 16
        pts = []

        for i in range(num_pts):
            t = 2.0 * math.pi * i / num_pts
            x = radius * math.cos(t)
            y = radius * math.sin(t)
            z = self.orbit_altitude_m + 0.6 * math.sin(t)
            z = max(self.min_altitude_m, min(self.max_altitude_m, z))
            pts.append((x, y, z))

        return pts

    def _estimate_waypoint_cost(self, candidate_world, target_world, reference_world):
        logger = self.get_logger()
        if not bool(self.enable_cost_optimization):
            logger.debug("Waypoint cost optimization disabled, using fallback cost")

            return self._fallback_waypoint_cost(candidate_world, target_world, reference_world)
        if candidate_world is None or target_world is None or reference_world is None:
            logger.debug(
                "Waypoint cost missing world info (candidate=%s target=%s reference=%s)",
                candidate_world,
                target_world,
                reference_world,
            )

            return self._fallback_waypoint_cost(candidate_world, target_world, reference_world)
        if self.cost_calculator is None:
            logger.debug("Cost calculator unavailable, using fallback cost")

            return self._fallback_waypoint_cost(candidate_world, target_world, reference_world)

        cost = self.cost_calculator.estimate_position_cost_at_position(
            float(candidate_world[0]),
            float(candidate_world[1]),
            float(candidate_world[2]),
            float(target_world[0]),
            float(target_world[1]),
            float(target_world[2]),
            reference_drone_x=float(reference_world[0]),
            reference_drone_y=float(reference_world[1]),
            reference_drone_z=float(reference_world[2]),
        )
        if cost is None or not math.isfinite(cost):
            fallback_cost = self._fallback_waypoint_cost(
                candidate_world, target_world, reference_world
            )
            logger.debug(
                "CostCalculator estimate invalid (%s), fallback cost=%s",
                cost,
                fallback_cost,
            )

            return fallback_cost

        return cost

    def _fallback_waypoint_cost(self, candidate_world, target_world, reference_world):
        if candidate_world is None or target_world is None:
            return None

        try:
            total = math.dist(candidate_world, target_world)
        except (ValueError, TypeError):
            return None

        if reference_world is not None:
            try:
                total += math.dist(candidate_world, reference_world)
            except (ValueError, TypeError) as error:
                self.get_logger().debug(f"fallback cost skipped reference distance: {error}")

        return total

    def _sample_cubic_bezier(self, p0, p1, p2, p3, n_points):
        n_points = int(n_points)
        if n_points <= 0:
            return []

        out = []
        for i in range(1, n_points + 1):
            t = i / (n_points + 1)
            u = 1.0 - t
            b0 = u * u * u
            b1 = 3.0 * u * u * t
            b2 = 3.0 * u * t * t
            b3 = t * t * t
            out.append(
                (
                    b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
                    b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1],
                    b0 * p0[2] + b1 * p1[2] + b2 * p2[2] + b3 * p3[2],
                )
            )

        return out

    def _build_bezier_transition(self, p0, p3, prev_point=None, next_point=None):
        if self.bezier_transition_points <= 0 or self.bezier_blend_length_m <= 0.0:
            return []

        dx = p3[0] - p0[0]
        dy = p3[1] - p0[1]
        dz = p3[2] - p0[2]
        if (dx * dx + dy * dy + dz * dz) ** 0.5 < 1e-3:
            return []

        start_dir = (
            (p0[0] - prev_point[0], p0[1] - prev_point[1], p0[2] - prev_point[2])
            if prev_point is not None
            else (dx, dy, dz)
        )
        end_dir = (
            (next_point[0] - p3[0], next_point[1] - p3[1], next_point[2] - p3[2])
            if next_point is not None
            else (dx, dy, dz)
        )

        v0_dir, v0_mag = self._normalize3(start_dir)
        if v0_mag < 1e-6:
            v0_dir, _ = self._normalize3((dx, dy, dz))
        v3_dir, v3_mag = self._normalize3(end_dir)
        if v3_mag < 1e-6:
            v3_dir, _ = self._normalize3((dx, dy, dz))

        alpha = float(self.bezier_blend_length_m)
        beta = float(self.bezier_blend_length_m)
        p1 = (
            p0[0] + alpha * v0_dir[0],
            p0[1] + alpha * v0_dir[1],
            p0[2],
        )
        p2 = (
            p3[0] - beta * v3_dir[0],
            p3[1] - beta * v3_dir[1],
            p3[2],
        )
        transition = self._sample_cubic_bezier(p0, p1, p2, p3, self.bezier_transition_points)
        self.get_logger().debug(
            f"Bezier transition: p0={p0} p3={p3} blend={self.bezier_blend_length_m:.2f} points={len(transition)}"
        )

        return [self._clamp_waypoint_to_bounds(wp) for wp in transition]

    def _resolve_focus_distance(self, obj_depth, use_anchor=True):
        if (
            use_anchor
            and self.anchor_focus_distance is not None
            and self.anchor_focus_distance > 0.0
        ):
            return float(self.anchor_focus_distance)
        if self.focus_depth_m is not None and self.focus_depth_m > 0.0:
            return float(self.focus_depth_m)
        if obj_depth is not None and obj_depth > 0.0:
            return float(obj_depth)

        return float(max(self.min_focus_distance_m, self.min_orbit_radius_m))

    def _path_length_between(self, static_world, start_idx, end_idx):
        if not static_world or end_idx <= start_idx:
            return 0.0
        length = 0.0
        for i in range(start_idx, end_idx):
            a = static_world[i]
            b = static_world[i + 1]
            dx = float(b[0]) - float(a[0])
            dy = float(b[1]) - float(a[1])
            dz = float(b[2]) - float(a[2])
            length += math.sqrt(dx * dx + dy * dy + dz * dz)

        return float(length)

    def _resolve_framing_altitude_from_anchors(
        self, start_wp, end_wp, target_world=None, desired_distance=None
    ):
        if self.focus_altitude_m > 0.0 and math.isfinite(self.focus_altitude_m):
            return float(self.focus_altitude_m)
        z = 0.5 * (float(start_wp[2]) + float(end_wp[2]))
        z = max(z, self.min_altitude_m)

        return float(z)

    def _build_anchor_arc(
        self,
        start_wp,
        end_wp,
        target_world,
        desired_distance,
        altitude_m,
        path_length_m,
        prev_wp=None,
        next_wp=None,
    ):
        if target_world is None:
            return []
        tx, ty, tz = (
            float(target_world[0]),
            float(target_world[1]),
            float(target_world[2]),
        )
        desired = max(float(desired_distance), float(self.min_orbit_radius_m))

        v0 = (float(start_wp[0]) - tx, float(start_wp[1]) - ty, float(start_wp[2]) - tz)
        v1 = (float(end_wp[0]) - tx, float(end_wp[1]) - ty, float(end_wp[2]) - tz)
        v0_len = math.sqrt(v0[0] * v0[0] + v0[1] * v0[1] + v0[2] * v0[2])
        v1_len = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2])
        if v0_len < 1e-6 or v1_len < 1e-6:
            return []

        v0n = (v0[0] / v0_len, v0[1] / v0_len, v0[2] / v0_len)
        v1n = (v1[0] / v1_len, v1[1] / v1_len, v1[2] / v1_len)

        dot = max(-1.0, min(1.0, v0n[0] * v1n[0] + v0n[1] * v1n[1] + v0n[2] * v1n[2]))
        theta = math.acos(dot)
        spacing = max(0.15, float(self.arc_waypoint_spacing_m))
        total_len = max(1e-6, theta * desired)
        num_total = max(7, int(math.ceil(total_len / spacing)) + 1)

        def _project_to_sphere(candidate):
            dx = float(candidate[0]) - tx
            dy = float(candidate[1]) - ty
            dz = float(candidate[2]) - tz
            n = math.sqrt(dx * dx + dy * dy + dz * dz)
            if n <= 1e-6:
                return candidate
            scale = desired / n

            return (tx + dx * scale, ty + dy * scale, tz + dz * scale)

        if theta < 1e-6:
            candidate = _project_to_sphere(
                (tx + v0n[0] * desired, ty + v0n[1] * desired, tz + v0n[2] * desired)
            )

            return [self._clamp_waypoint_to_bounds(candidate)]

        sin_theta = math.sin(theta)
        arc_pts = []
        for i in range(num_total):
            t = i / (num_total - 1)
            s0 = math.sin((1.0 - t) * theta) / sin_theta
            s1 = math.sin(t * theta) / sin_theta
            vx = s0 * v0n[0] + s1 * v1n[0]
            vy = s0 * v0n[1] + s1 * v1n[1]
            vz = s0 * v0n[2] + s1 * v1n[2]
            candidate = (tx + vx * desired, ty + vy * desired, tz + vz * desired)
            candidate = self._clamp_waypoint_to_bounds(candidate)
            candidate = _project_to_sphere(candidate)
            candidate = self._clamp_waypoint_to_bounds(candidate)
            arc_pts.append(candidate)

        return arc_pts

    def _score_arc_points(self, arc_pts, target_world):
        if target_world is None or self.cost_calculator is None:
            return float("inf")
        if not arc_pts or len(arc_pts) < 2:
            return float("inf")

        tx, ty, tz = (
            float(target_world[0]),
            float(target_world[1]),
            float(target_world[2]),
        )
        total = 0.0
        ref = arc_pts[0]
        for wp in arc_pts[1:]:
            cost = self.cost_calculator.estimate_position_cost_at_position(
                float(wp[0]),
                float(wp[1]),
                float(wp[2]),
                tx,
                ty,
                tz,
                reference_drone_x=float(ref[0]),
                reference_drone_y=float(ref[1]),
                reference_drone_z=float(ref[2]),
            )
            if cost is None or not math.isfinite(cost):
                return float("inf")
            total += float(cost)
            ref = wp

        return float(total)

    def _resolve_framing_altitude(self, base_pos):
        if self.focus_altitude_m > 0.0 and math.isfinite(self.focus_altitude_m):
            return float(self.focus_altitude_m)
        if base_pos is None:
            return float(self.orbit_altitude_m)

        return float(base_pos[2])

    def _enforce_focus_distance(
        self, candidate, target_world, desired_distance, altitude_m, tolerance_m
    ):
        if target_world is None:
            return candidate

        tx, ty, tz = (
            float(target_world[0]),
            float(target_world[1]),
            float(target_world[2]),
        )
        x, y, _ = candidate
        z = float(altitude_m)
        dx = x - tx
        dy = y - ty
        dz = z - tz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if not math.isfinite(dist):
            return candidate

        desired = float(desired_distance)
        min_dist = math.sqrt(self.min_orbit_radius_m * self.min_orbit_radius_m + dz * dz)
        if desired < min_dist:
            desired = min_dist

        if abs(dist - desired) <= tolerance_m:
            return (x, y, z)

        xy_dist = math.hypot(dx, dy)
        if xy_dist < 1e-6:
            ref = self.current_pos or self.start_pos
            if ref is not None:
                dx = float(ref[0]) - tx
                dy = float(ref[1]) - ty
                xy_dist = math.hypot(dx, dy)
        if xy_dist < 1e-6:
            dx = math.cos(self.current_yaw)
            dy = math.sin(self.current_yaw)
            xy_dist = math.hypot(dx, dy)

        desired_xy = math.sqrt(max(desired * desired - dz * dz, 0.0))
        if desired_xy < self.min_orbit_radius_m:
            desired_xy = float(self.min_orbit_radius_m)
        scale = desired_xy / max(xy_dist, 1e-6)

        return (tx + dx * scale, ty + dy * scale, z)

    def generate_cinematographic_waypoints(self):
        if self.start_pos is None:
            return
        if self.detected_object is None:
            target_world = self.target_world or self.last_target_world
            if target_world is None and self.simulate_object:
                base_pos = self.current_pos if self.current_pos is not None else self.start_pos
                if base_pos is not None:
                    offset = self.simulated_object_world_offset
                    target_world = (
                        float(base_pos[0]) + float(offset[0]),
                        float(base_pos[1]) + float(offset[1]),
                        float(base_pos[2]) + float(offset[2]),
                    )
                    self.target_world = target_world
                    self.last_target_world = target_world
            if target_world is None:
                return

        if self.detected_object is None:
            depth = (
                float(self.focus_depth_m)
                if self.focus_depth_m is not None
                else max(self.min_focus_distance_m, self.min_orbit_radius_m)
            )

            self.detected_object = {
                "center_x": float(self.cx),
                "center_y": float(self.cy),
                "depth": depth,
                "norm_x": 0.5,
                "norm_y": 0.5,
            }

        if self.anchor_start_idx is None or self.anchor_end_idx is None:
            self._set_anchors(
                target_world=self.target_world, obj_depth=self.detected_object["depth"]
            )
        if self.anchor_start_idx is None or self.anchor_end_idx is None:
            return

        obj_depth = (
            float(self.focus_depth_m)
            if self.focus_depth_m is not None
            else float(self.detected_object["depth"])
        )
        center_x = float(self.detected_object["center_x"])
        center_y = float(self.detected_object["center_y"])
        norm_x = self.detected_object.get("norm_x", None)
        norm_y = self.detected_object.get("norm_y", None)

        if norm_x is not None and norm_y is not None:
            u = float(norm_x) * float(self.image_width)
            v = float(norm_y) * float(self.image_height)
        else:
            u = center_x
            v = center_y

        if self.fx > 0 and self.fy > 0:
            obj_x_m = (u - self.cx) / self.fx * obj_depth
            obj_y_m = (v - self.cy) / self.fy * obj_depth
        else:
            obj_x_m = 0.0
            obj_y_m = 0.0

        base_pos = self.current_pos if self.current_pos is not None else self.start_pos
        if base_pos is None:
            return

        target_world = self.target_world
        if target_world is None:
            target_world, _ = self._compute_object_world(self.detected_object)

        if target_world is None:
            center_x = float(self.detected_object["center_x"])
            center_y = float(self.detected_object["center_y"])
            norm_x = self.detected_object.get("norm_x", None)
            norm_y = self.detected_object.get("norm_y", None)

            if norm_x is not None and norm_y is not None:
                u = float(norm_x) * float(self.image_width)
                v = float(norm_y) * float(self.image_height)
            else:
                u = center_x
                v = center_y

            if self.fx > 0 and self.fy > 0:
                obj_x_m = (u - self.cx) / self.fx * obj_depth
                obj_y_m = (v - self.cy) / self.fy * obj_depth
            else:
                obj_x_m = 0.0
                obj_y_m = 0.0

            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            target_world = (
                base_pos[0] + cos_yaw * obj_x_m - sin_yaw * obj_y_m,
                base_pos[1] + sin_yaw * obj_x_m + cos_yaw * obj_y_m,
                base_pos[2],
            )
        self.last_target_world = target_world

        static_world = [self._rel_to_world(wp) for wp in self._scaled_static_waypoints()]
        start_idx = int(max(0, min(self.anchor_start_idx, len(static_world) - 2)))
        end_idx = int(max(start_idx + 1, min(self.anchor_end_idx, len(static_world) - 1)))
        self.viz_data["static_world"] = static_world
        self.viz_data["start_idx"] = start_idx
        self.viz_data["end_idx"] = end_idx

        start_wp = static_world[start_idx]
        end_wp = static_world[end_idx]

        anchor_dist_start = math.dist(start_wp, target_world)
        anchor_dist_end = math.dist(end_wp, target_world)
        anchor_distance = max(anchor_dist_start, anchor_dist_end)
        focus_distance = max(float(anchor_distance), float(self.min_orbit_radius_m))
        self.anchor_focus_distance = float(focus_distance)
        self._publish_anchor_focus_distance()
        base_z = self._resolve_framing_altitude_from_anchors(
            start_wp,
            end_wp,
            target_world=target_world,
            desired_distance=focus_distance,
        )
        base_z = max(self.min_altitude_m, min(self.max_altitude_m, base_z))
        path_length = self._path_length_between(static_world, start_idx, end_idx)

        tx, ty, tz = (
            float(target_world[0]),
            float(target_world[1]),
            float(target_world[2]),
        )
        dz = base_z - tz
        desired = focus_distance
        radius_xy = math.sqrt(max(desired * desired - dz * dz, 0.0))
        radius_xy = max(self.min_orbit_radius_m, radius_xy)

        def _project_to_circle(wp):
            dx = float(wp[0]) - tx
            dy = float(wp[1]) - ty
            xy_dist = math.hypot(dx, dy)
            if xy_dist < 1e-6:
                dx = 1.0
                dy = 0.0
                xy_dist = 1.0
            scale = radius_xy / xy_dist if radius_xy > 1e-6 else 0.0
            candidate = (tx + dx * scale, ty + dy * scale, base_z)
            candidate = self._clamp_waypoint_to_bounds(candidate)
            dist = math.dist(candidate, (tx, ty, tz))
            if math.isfinite(dist) and abs(dist - desired) > 1e-3:
                dx = float(candidate[0]) - tx
                dy = float(candidate[1]) - ty
                xy_dist = math.hypot(dx, dy)
                if xy_dist > 1e-6:
                    scale = radius_xy / xy_dist if radius_xy > 1e-6 else 0.0
                    candidate = (tx + dx * scale, ty + dy * scale, base_z)
                    candidate = self._clamp_waypoint_to_bounds(candidate)

            return candidate

        arc_start = _project_to_circle(start_wp)
        arc_end = _project_to_circle(end_wp)
        self.viz_data["arc_start"] = arc_start
        self.viz_data["arc_end"] = arc_end

        prev_static = static_world[start_idx - 1] if start_idx - 1 >= 0 else None
        next_static = static_world[end_idx + 1] if end_idx + 1 < len(static_world) else None
        if prev_static is None and len(static_world) >= 2:
            fallback_ref = (
                static_world[min(start_idx + 1, len(static_world) - 1)]
                if start_idx + 1 < len(static_world)
                else static_world[start_idx]
            )

            prev_static = self._extrapolate_transition_point(
                static_world[start_idx], fallback_ref, -self.bezier_blend_length_m
            )

        if next_static is None and len(static_world) >= 2:
            fallback_ref = (
                static_world[max(end_idx - 1, 0)] if end_idx - 1 >= 0 else static_world[end_idx]
            )

            next_static = self._extrapolate_transition_point(
                static_world[end_idx], fallback_ref, self.bezier_blend_length_m
            )

        cinematographic_waypoints = self._build_anchor_arc(
            arc_start,
            arc_end,
            target_world,
            focus_distance,
            base_z,
            path_length,
            prev_wp=prev_static,
            next_wp=next_static,
        )

        has_cinematic_arc = bool(cinematographic_waypoints)
        self.generated_arc_length = len(cinematographic_waypoints) if has_cinematic_arc else 0

        if cinematographic_waypoints:
            self.anchor_start_world = arc_start
            self.anchor_end_world = arc_end

        self.viz_data["cinematographic_waypoints"] = cinematographic_waypoints
        self._debug_arc(cinematographic_waypoints, target_world, desired)

        actual_focus_distance = focus_distance
        if cinematographic_waypoints:
            waypoint_dists = [
                math.dist(wp, target_world) for wp in cinematographic_waypoints if wp is not None
            ]
            if waypoint_dists:
                max_arc_dist = max(waypoint_dists)
                if math.isfinite(max_arc_dist):
                    actual_focus_distance = max(actual_focus_distance, max_arc_dist)
        if actual_focus_distance != self.anchor_focus_distance:
            self.anchor_focus_distance = float(actual_focus_distance)
            self._publish_anchor_focus_distance()

        suffix = static_world[end_idx + 1 :]

        transition_in = []
        transition_out = []
        if cinematographic_waypoints:
            if prev_static is not None:
                prev_prev = static_world[start_idx - 2] if start_idx - 2 >= 0 else None
                next_dynamic = (
                    cinematographic_waypoints[2] if len(cinematographic_waypoints) > 1 else None
                )

                transition_in = self._build_bezier_transition(
                    prev_static,
                    cinematographic_waypoints[1],
                    prev_point=prev_prev,
                    next_point=next_dynamic,
                )

            first_suffix = suffix[0] if suffix else None
            next_suffix = suffix[1] if len(suffix) > 1 else None

            if first_suffix is not None:
                cine_waypoints_len = len(cinematographic_waypoints)
                prev_dynamic = (
                    cinematographic_waypoints[cine_waypoints_len - 2]
                    if cine_waypoints_len > 1
                    else None
                )
                prev_prev_dynamic = (
                    cinematographic_waypoints[cine_waypoints_len - 3]
                    if cine_waypoints_len > 2
                    else None
                )

                transition_out = self._build_bezier_transition(
                    prev_dynamic,
                    first_suffix,
                    prev_point=prev_prev_dynamic,
                    next_point=next_suffix,
                )

        if not has_cinematic_arc:
            self.last_cinematic_waypoints = []
            self.dynamic_waypoints = None
            self.viz_data["transition_in"] = transition_in
            self.viz_data["transition_out"] = transition_out
            self.viz_data["combined_world"] = static_world
            self.get_logger().debug(
                f"Cinematic arc unavailable (start={start_idx} end={end_idx}) using static path len={len(static_world)}"
            )

            return
        else:
            if suffix:
                suffix_altitudes = [f"{wp[2]:.3f}" for wp in suffix[:5]]
                self.get_logger().info(f"Suffix altitudes (first 5): {suffix_altitudes}")

            combined_world = (
                static_world[:start_idx]
                + transition_in
                + cinematographic_waypoints
                + transition_out
                + suffix
            )
            if cinematographic_waypoints:
                arc_altitudes = [wp[2] for wp in cinematographic_waypoints]
                self.get_logger().info(
                    f"Arc altitude range: {min(arc_altitudes):.3f} - {max(arc_altitudes):.3f} m"
                )
                post_arc_start = (
                    len(static_world[:start_idx])
                    + len(transition_in)
                    + len(cinematographic_waypoints)
                    + len(transition_out)
                )
                post_arc = combined_world[post_arc_start : post_arc_start + 5]
                if post_arc:
                    alt_list = [f"{wp[2]:.3f}" for wp in post_arc]
                    self.get_logger().info(f"Post-arc waypoint altitudes: {alt_list}")

            self.viz_data["transition_in"] = transition_in
            self.viz_data["transition_out"] = transition_out
            self.viz_data["combined_world"] = combined_world

        origin = self.origin_offset if self.origin_offset is not None else (0.0, 0.0, 0.0)
        combined_world = self._remove_anchor_points_from_waypoints(combined_world, origin=origin)
        self.dynamic_waypoints = [
            (wp[0] - origin[0], wp[1] - origin[1], wp[2] - origin[2]) for wp in combined_world
        ]
        self.last_cinematic_waypoints = cinematographic_waypoints
        self.get_logger().debug(
            f"Dynamic arc details start_idx={start_idx} end_idx={end_idx} combined_len={len(combined_world)} "
            f"arc_pts={len(cinematographic_waypoints)} transition_in={len(transition_in)} transition_out={len(transition_out)} focus_dist={float(focus_distance):.3f}"
        )

    def _build_orbit_segment(self, center_world, radius, z, num_pts=16, reference_world=None):
        radius = max(self.min_orbit_radius_m, float(radius))
        z = max(self.min_altitude_m, min(self.max_altitude_m, float(z)))
        num_pts = int(max(1, num_pts))

        reference = reference_world
        if reference is None:
            reference = self.current_pos if self.current_pos is not None else self.start_pos

        best_radius = float(radius)
        if bool(self.enable_cost_optimization) and reference is not None:
            radius_samples = int(max(1, self.candidate_radius_samples))
            r_scale_min, r_scale_max = [float(v) for v in self.candidate_radius_scale_range]
            r_scale_min, r_scale_max = (
                min(r_scale_min, r_scale_max),
                max(r_scale_min, r_scale_max),
            )

            r0 = float(radius)
            r_min = max(self.min_orbit_radius_m, r0 * r_scale_min)
            r_max = max(r_min, r0 * r_scale_max)
            radii = (
                [r_min + (r_max - r_min) * k / (radius_samples - 1) for k in range(radius_samples)]
                if radius_samples > 1
                else [r0]
            )

            best_total = float("inf")
            for rr in radii:
                total = 0.0
                ref = reference
                for i in range(num_pts):
                    t = 2.0 * math.pi * i / num_pts
                    candidate = self._clamp_waypoint_to_bounds(
                        (
                            center_world[0] + rr * math.cos(t),
                            center_world[1] + rr * math.sin(t),
                            z,
                        )
                    )
                    c = self._estimate_waypoint_cost(candidate, center_world, ref)
                    if c is None or not math.isfinite(c):
                        total = float("inf")
                        break
                    total += float(c)
                    ref = candidate

                if total < best_total:
                    best_total = total
                    best_radius = float(rr)

        waypoints = []
        for i in range(num_pts):
            t = 2.0 * math.pi * i / num_pts
            x = center_world[0] + best_radius * math.cos(t)
            y = center_world[1] + best_radius * math.sin(t)
            waypoints.append(self._clamp_waypoint_to_bounds((x, y, z)))

        return waypoints

    def _merge_framing_with_static(self, rel_waypoints, detection):
        if not rel_waypoints or detection is None:
            return rel_waypoints

        origin = self.origin_offset if self.origin_offset is not None else (0.0, 0.0, 0.0)
        wps_world = [
            (wp[0] + origin[0], wp[1] + origin[1], wp[2] + origin[2]) for wp in rel_waypoints
        ]

        center_world, obj_depth = self._compute_object_world(detection)
        if center_world is None:
            return rel_waypoints
        self.last_target_world = center_world

        idx_anchor = min(
            range(len(wps_world)),
            key=lambda i: (
                (wps_world[i][0] - center_world[0]) ** 2 + (wps_world[i][1] - center_world[1]) ** 2
            ),
        )
        anchor_wp = wps_world[idx_anchor]
        focus_distance = self._resolve_focus_distance(obj_depth)
        dz = float(anchor_wp[2]) - float(center_world[2])
        radius_xy = math.sqrt(max(focus_distance * focus_distance - dz * dz, 0.0))
        orbit_radius = max(self.min_orbit_radius_m, radius_xy)
        orbit = self._build_orbit_segment(
            center_world, orbit_radius, anchor_wp[2], reference_world=anchor_wp
        )

        overlap_thresh = 0.35

        def too_close(wp):
            for o in orbit:
                dx = wp[0] - o[0]
                dy = wp[1] - o[1]
                dz = wp[2] - o[2]
                if (dx * dx + dy * dy + dz * dz) ** 0.5 < overlap_thresh:
                    return True

            return False

        head = [wp for wp in wps_world[: idx_anchor + 1] if not too_close(wp)]
        tail = [wp for wp in wps_world[idx_anchor + 1 :] if not too_close(wp)]
        merged_world = head + orbit + tail

        return [(wp[0] - origin[0], wp[1] - origin[1], wp[2] - origin[2]) for wp in merged_world]

    def _build_framing_arc(self, obj_world, anchor_wp, radius, prev_wp=None, next_wp=None):
        cx, cy, _ = obj_world
        ax, ay, az = anchor_wp

        dist_oa = math.hypot(ax - cx, ay - cy)
        desired_r = min(radius, 0.4 * dist_oa) if dist_oa > 1e-3 else radius
        radius = max(self.min_orbit_radius_m, desired_r)
        z = max(self.min_altitude_m, min(self.max_altitude_m, az))

        def _dir_from(a, b):
            return (a[0] - b[0], a[1] - b[1])

        approach = _dir_from(anchor_wp, prev_wp) if prev_wp is not None else None
        depart = _dir_from(next_wp, anchor_wp) if next_wp is not None else None

        def _angle(v):
            return math.atan2(v[1], v[0])

        approach_ang = _angle(approach) if approach else None
        depart_ang = _angle(depart) if depart else approach_ang

        start_angle = math.atan2(ay - cy, ax - cx)
        end_angle = start_angle
        if next_wp is not None:
            end_angle = math.atan2(next_wp[1] - cy, next_wp[0] - cx)

        delta = (end_angle - start_angle + math.pi) % (2 * math.pi) - math.pi

        if approach is not None:
            obj_vec = (cx - ax, cy - ay)
            cross = approach[0] * obj_vec[1] - approach[1] * obj_vec[0]
            turn_sign = 1.0 if cross > 0.0 else -1.0
            if delta * turn_sign < 0:
                delta = delta - math.copysign(2 * math.pi, delta)
        num_pts = 7

        arc_pts = []

        for i in range(num_pts):
            t = i / (num_pts - 1)
            ang = start_angle + t * delta
            x = cx + radius * math.cos(ang)
            y = cy + radius * math.sin(ang)
            arc_pts.append(self._clamp_waypoint_to_bounds((x, y, z)))

        return arc_pts
