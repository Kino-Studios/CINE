import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class WaypointSequencerMixin:
    def check_and_send_waypoints(self):
        if self.start_pos is not None and self.waypoints_sent:
            return

        if self.start_pos is not None and self.ready_time is not None:
            elapsed = (self.get_clock().now() - self.ready_time).nanoseconds / 1e9
            if elapsed >= self.STARTUP_DELAY or self.velocity_settled:
                if self.active_waypoints is None:
                    self._initialize_waypoint_sequence()
                self._step_waypoint_sequence()

    def _should_scale_static_waypoints(self):
        return (
            self.waypoint_xy_scale != 1.0
            or self.waypoint_y_extra_scale != 1.0
            or self.waypoint_z_scale != 1.0
        )

    def _scaled_static_waypoints(self):
        waypoints = list(self.static_waypoints)
        if self._should_scale_static_waypoints():
            sx = self.waypoint_xy_scale
            sy = self.waypoint_xy_scale * self.waypoint_y_extra_scale
            sz = self.waypoint_z_scale
            waypoints = [(wp[0] * sx, wp[1] * sy, wp[2] * sz) for wp in waypoints]
            self.get_logger().debug(
                f"Scaling static path (count={len(self.static_waypoints)}): xy_scale={sx:.2f} y_extra={self.waypoint_y_extra_scale:.2f} z_scale={sz:.2f}"
            )

        return waypoints

    def _initialize_waypoint_sequence(self):
        if self.baseline_mode:
            waypoints_to_send = self._scaled_static_waypoints()
        else:
            if self.use_dynamic_waypoints and self.dynamic_waypoints is None:
                if (
                    self.detected_object is not None
                    or self.target_world is not None
                    or self.last_target_world is not None
                ):
                    self.get_logger().info("Generating dynamic waypoints for active path")
                    self.generate_cinematographic_waypoints()
                if self.dynamic_waypoints is None:
                    self.get_logger().warn(
                        "Dynamic waypoints unavailable; falling back to static path"
                    )
                    self.use_dynamic_waypoints = False
                    if self.detected_object is not None or self.target_world is not None:
                        self.pending_detection_merge = True

            if self.use_dynamic_waypoints and self.dynamic_waypoints is not None:
                waypoints_to_send = self.dynamic_waypoints
            else:
                waypoints_to_send = self._scaled_static_waypoints()

            if (
                (not self.use_dynamic_waypoints)
                and self.pending_detection_merge
                and self.detected_object is not None
            ):
                waypoints_to_send = self._merge_framing_with_static(
                    waypoints_to_send, self.detected_object
                )
                self.pending_detection_merge = False
            elif self.simulate_object and (not self.use_dynamic_waypoints):
                waypoints_to_send = self._insert_simulated_object_orbit(waypoints_to_send)

        origin = self.origin_offset if self.origin_offset is not None else (0.0, 0.0, 0.0)
        waypoints_to_send = self._remove_anchor_points_from_waypoints(
            waypoints_to_send, origin=origin
        )

        dedup_min_sep = self.min_waypoint_separation_m
        if self.use_dynamic_waypoints and self.dynamic_waypoints is not None:
            blend_gap = max(0.02, float(self.bezier_blend_length_m) * 0.5)
            dedup_min_sep = min(self.min_waypoint_separation_m, blend_gap)
        waypoints_to_send = self._deduplicate_waypoints(
            waypoints_to_send, min_separation=dedup_min_sep
        )
        if not self.use_dynamic_waypoints:
            waypoints_to_send = self._apply_descent_tail(waypoints_to_send)

        self.active_waypoints = self._limit_consecutive_waypoint_jumps(waypoints_to_send)

        self.viz_data["static_world"] = [self._rel_to_world(wp) for wp in self.active_waypoints]

        self.current_target_index = 0
        self.last_published_index = -1
        self.last_progress_time = None
        self.last_progress_dist = None
        self.waypoints_sent = False
        self._update_framing_indices()
        if not self.baseline_mode:
            self._insert_focus_entry_waypoints()
            self._update_framing_indices()
        framing_active = self._is_framing_idx(self.current_target_index)
        self._publish_framing_states(framing_active)

        if self.current_pos is not None and self.active_waypoints:
            skipped = 0
            min_start_threshold = max(
                self._current_distance_threshold(),
                float(self.min_first_waypoint_distance_m),
            )
            while self.current_target_index < len(self.active_waypoints):
                if (
                    self.framing_start_idx is not None
                    and self.current_target_index >= self.framing_start_idx
                ):
                    break
                target_world = self._rel_to_world(self.active_waypoints[self.current_target_index])
                dist = self._distance_to(target_world)
                threshold = min_start_threshold
                if self.current_target_index == 0:
                    threshold = min(threshold, max(dist * 0.9, self.PROGRESS_EPS_M))
                if dist > threshold:
                    break
                self.current_target_index += 1
                skipped += 1
            if skipped > 0:
                self.last_published_index = self.current_target_index - 1
                total_waypoints = (
                    len(self.active_waypoints) if self.active_waypoints is not None else 0
                )
                if self.framing_start_idx is not None and self.framing_end_idx is not None:
                    range_desc = f"{self.framing_start_idx}-{self.framing_end_idx}"
                else:
                    range_desc = "unset"
                overshoot_note = ""
                if (
                    self.framing_start_idx is not None
                    and self.current_target_index > self.framing_start_idx
                ):
                    overshoot_note = " [warn: skip may have jumped past framing start]"
                self.get_logger().warn(
                    f"Skipped {skipped} near waypoints at init; now at {self.current_target_index} "
                    f"(min_start_threshold={min_start_threshold:.2f}, framing range={range_desc}, total_waypoints={total_waypoints})"
                    f"{overshoot_note}"
                )
        self._reset_and_prefill_queue()

    def _limit_consecutive_waypoint_jumps(self, waypoints):
        if not waypoints or self.max_waypoint_jump_m <= 0:
            return waypoints

        out = [waypoints[0]]
        for wp in waypoints[1:]:
            prev = out[-1]
            dx = wp[0] - prev[0]
            dy = wp[1] - prev[1]
            dz = wp[2] - prev[2]
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5

            if dist <= self.max_waypoint_jump_m:
                out.append(wp)
                continue

            steps = max(1, int(math.ceil(dist / self.max_waypoint_jump_m)))
            for s in range(1, steps + 1):
                t = s / steps
                out.append((prev[0] + t * dx, prev[1] + t * dy, prev[2] + t * dz))

        return out

    def _insert_focus_entry_waypoints(self):
        if (
            not self.active_waypoints
            or self.focus_entry_decel_waypoints <= 0
            or self.anchor_start_world is None
        ):
            return
        self._update_framing_indices()
        start_idx = self.framing_start_idx
        if start_idx is None or start_idx <= 0 or start_idx >= len(self.active_waypoints):
            return
        prev_wp = self.active_waypoints[start_idx - 1]
        entry_wp = self.active_waypoints[start_idx]
        inserts = []
        for i in range(self.focus_entry_decel_waypoints):
            t = (i + 1) / (self.focus_entry_decel_waypoints + 1)
            inserts.append(tuple(prev_wp[j] + t * (entry_wp[j] - prev_wp[j]) for j in range(3)))
        self.active_waypoints[start_idx:start_idx] = inserts
        self._update_framing_indices()

    def _apply_descent_tail(self, waypoints):
        if not waypoints or len(waypoints) < 3:
            return waypoints

        tail_count = max(3, int(len(waypoints) * 0.3))
        start_idx = max(0, len(waypoints) - tail_count)
        start_z = waypoints[start_idx][2]
        target_z = max(self.min_altitude_m, self.world_bounds_min[2])

        if start_z <= target_z + 1e-3:
            return waypoints

        adjusted = list(waypoints)
        for i in range(len(waypoints) - start_idx):
            t = (i + 1) / (len(waypoints) - start_idx)
            z = start_z + t * (target_z - start_z)
            wp = adjusted[start_idx + i]
            adjusted[start_idx + i] = (wp[0], wp[1], z)

        return adjusted

    def _deduplicate_waypoints(self, waypoints, min_separation=0.3):
        if not waypoints:
            return waypoints
        out = []
        prev = None
        for wp in waypoints:
            if prev is not None:
                dx = wp[0] - prev[0]
                dy = wp[1] - prev[1]
                dz = wp[2] - prev[2]
                if (dx * dx + dy * dy + dz * dz) ** 0.5 < min_separation:
                    continue
            out.append(wp)
            prev = wp

        return out

    def _remove_anchor_points_from_waypoints(self, waypoints, origin=None):
        if not waypoints:
            return waypoints
        anchors = []
        if self.anchor_start_world is not None:
            anchors.append(self.anchor_start_world)
        if self.anchor_end_world is not None:
            anchors.append(self.anchor_end_world)
        if not anchors:
            return waypoints
        if origin is None:
            origin = (0.0, 0.0, 0.0)

        normalized_anchors = [
            (float(a[0]) - origin[0], float(a[1]) - origin[1], float(a[2]) - origin[2])
            for a in anchors
        ]
        tolerance = max(
            0.01,
            min(
                float(self.min_waypoint_separation_m) * 0.5,
                float(self.bezier_blend_length_m) * 0.4 + 0.01,
            ),
        )

        def _is_anchor(wp):
            for anchor in normalized_anchors:
                if math.dist(wp, anchor) <= tolerance:
                    return True

            return False

        return [wp for wp in waypoints if not _is_anchor(wp)]

    def _current_distance_threshold(self):
        radius = max(float(self.waypoint_radius_m), 0.001)
        if self._is_framing_idx(self.current_target_index):
            spacing = (
                float(self.arc_waypoint_spacing_m) if self.arc_waypoint_spacing_m > 0.0 else radius
            )
            arc_threshold = max(radius, min(0.9 * spacing, spacing))

            return max(arc_threshold, self.min_goal_distance_m)

        return max(radius, self.min_goal_distance_m)

    def _light_avoidance_weight(self) -> float:
        if (
            not self.light_avoidance_enabled
            or self.light_distance is None
            or self.light_influence_distance_m <= 0.0
        ):
            return 0.0
        weight = (
            self.light_influence_distance_m - self.light_distance
        ) / self.light_influence_distance_m
        weight = max(0.0, min(1.0, weight))

        return math.sqrt(weight)

    def _light_avoidance_yaw(self):
        if self.light_vector is None:
            return None
        dx, dy = float(self.light_vector[0]), float(self.light_vector[1])
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return None

        return math.atan2(-dy, -dx)

    def _blend_angles(self, base: float, target: float, weight: float) -> float:
        if weight <= 0.0:
            return base
        if weight >= 1.0:
            return target
        diff = ((target - base + math.pi) % (2.0 * math.pi)) - math.pi

        return base + diff * weight

    def _current_target_reached(self):
        if self.active_waypoints is None or self.current_pos is None:
            return False
        if self.current_target_index >= len(self.active_waypoints):
            return True
        target_world = self._rel_to_world(self.active_waypoints[self.current_target_index])

        return self._distance_to(target_world) <= self._current_distance_threshold()

    def _publish_framing_active(self, active):
        active = bool(active)
        if self.framing_active == active:
            return
        prev_idx = getattr(self, "current_target_index", 0) or 0
        total_waypoints = len(self.active_waypoints) if self.active_waypoints is not None else 0
        if self.framing_start_idx is not None and self.framing_end_idx is not None:
            range_desc = f"{self.framing_start_idx}-{self.framing_end_idx}"
        else:
            range_desc = "unset"
        if active:
            self.get_logger().info(
                f"FRAMING STARTED at waypoint {prev_idx} (range: {range_desc}, total_waypoints: {total_waypoints})"
            )
        else:
            self.get_logger().info(
                f"FRAMING ENDED at waypoint {prev_idx} (range was: {range_desc}, total_waypoints: {total_waypoints})"
            )
        msg = Bool()
        msg.data = active
        self.framing_active_pub.publish(msg)
        self.framing_active = active

    def _publish_baseline_framing_active(self, active):
        active = bool(active)
        if self.baseline_framing_active == active:
            return
        self.get_logger().info(f"Baseline framing active state changed to {active}")
        msg = Bool()
        msg.data = active
        self.baseline_framing_pub.publish(msg)
        self.baseline_framing_active = active

    def _publish_framing_states(self, framing_active):
        if self.baseline_mode:
            self._publish_framing_active(False)
            self._publish_baseline_framing_active(framing_active)
        else:
            self._publish_framing_active(framing_active)
            self._publish_baseline_framing_active(False)

    def _publish_waypoint(self, target_world, idx: int):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = float(target_world[0])
        goal.pose.position.y = float(target_world[1])
        goal.pose.position.z = float(target_world[2])

        yaw = self.current_yaw

        if self._is_framing_idx(idx):
            look_at = self.last_target_world or self.target_world

            if look_at is not None:
                dx = float(look_at[0]) - float(target_world[0])
                dy = float(look_at[1]) - float(target_world[1])
                yaw = math.atan2(dy, dx)
        else:
            if self.light_avoidance_enabled:
                avoidance_weight = self._light_avoidance_weight()
                avoidance_yaw = self._light_avoidance_yaw()
                if avoidance_yaw is not None and avoidance_weight > 0.0:
                    blend = min(
                        1.0,
                        float(self.light_avoidance_gain) * (avoidance_weight + 0.25),
                    )
                    yaw = self._blend_angles(yaw, avoidance_yaw, blend)

        qw = math.cos(0.5 * yaw)
        qz = math.sin(0.5 * yaw)
        goal.pose.orientation.w = qw
        goal.pose.orientation.z = qz
        self.goal_pub.publish(goal)
        self.sent_waypoints_history.append(target_world)

        if idx == self.current_target_index:
            self.last_progress_time = self.get_clock().now()
            self.last_progress_dist = self._distance_to(target_world)
            self.stuck_attempts = 0

    def _prefill_waypoint_queue(self):
        if self.active_waypoints is None or self.current_pos is None:
            return
        lookahead = self.WAYPOINT_LOOKAHEAD
        if self.use_dynamic_waypoints and self.dynamic_waypoints is not None:
            lookahead = self.dynamic_waypoint_lookahead
        lookahead = max(1, int(lookahead))
        lookahead = min(2, lookahead)

        while (
            self.last_published_index < len(self.active_waypoints) - 1
            and (self.last_published_index - self.current_target_index + 1) < lookahead
        ):
            next_idx = self.last_published_index + 1
            target_world = self._rel_to_world(self.active_waypoints[next_idx])
            self._publish_waypoint(target_world, next_idx)
            self.last_published_index = next_idx

    def _reset_and_prefill_queue(self):
        if self.active_waypoints is None:
            return
        self.last_published_index = self.current_target_index - 1
        self._prefill_waypoint_queue()

    def _step_waypoint_sequence(self):
        if self.active_waypoints is None or self.current_pos is None:
            self._publish_framing_states(False)

            return

        if self.current_target_index >= len(self.active_waypoints):
            self.waypoints_sent = True
            self._publish_framing_states(False)

            return

        framing_active = self._is_framing_idx(self.current_target_index)
        self._publish_framing_states(framing_active)

        target_world = self._rel_to_world(self.active_waypoints[self.current_target_index])

        dx = self.current_pos[0] - target_world[0]
        dy = self.current_pos[1] - target_world[1]
        dz = self.current_pos[2] - target_world[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        threshold = self._current_distance_threshold()
        self.get_logger().debug(
            f"Waypoint {self.current_target_index} dist={dist:.3f} threshold={threshold:.3f}"
        )

        now = self.get_clock().now()
        if self.last_progress_time is None:
            self.last_progress_time = now
            self.last_progress_dist = dist

        if dist < (self.last_progress_dist or 0.0) - self.PROGRESS_EPS_M:
            self.last_progress_time = now
            self.last_progress_dist = dist
        else:
            if self.enable_progress_skip:
                elapsed_stuck = (now - self.last_progress_time).nanoseconds / 1e9
                if elapsed_stuck >= self.PROGRESS_TIMEOUT_S and dist > (
                    self.waypoint_radius_m * 0.5
                ):
                    self.get_logger().warn(
                        f"No progress toward waypoint {self.current_target_index} for {elapsed_stuck:.1f}s; skipping it"
                    )
                    self.current_target_index += 1
                    self.last_progress_time = None
                    self.last_progress_dist = None
                    self.stuck_attempts = 0
                    self._prefill_waypoint_queue()

                    return

        if dist <= threshold:
            total_waypoints = len(self.active_waypoints) if self.active_waypoints is not None else 0
            max_idx = max(total_waypoints - 1, 0)
            idx_label = f"{self.current_target_index}/{max_idx}"
            framing_tag = " [FRAMING]" if framing_active else ""
            self.get_logger().info(
                f"Waypoint {idx_label} reached{framing_tag}: dist={dist:.3f} <= threshold={threshold:.3f}"
            )
            self.current_target_index += 1
            self.last_progress_time = None
            self.last_progress_dist = None
            self.stuck_attempts = 0
            self._prefill_waypoint_queue()
            if self.current_target_index >= len(self.active_waypoints):
                self.waypoints_sent = True

            return

        self._prefill_waypoint_queue()
