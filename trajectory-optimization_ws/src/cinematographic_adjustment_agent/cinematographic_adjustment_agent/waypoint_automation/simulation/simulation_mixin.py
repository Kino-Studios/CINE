import math

from std_msgs.msg import Float32MultiArray


class SimulationMixin:
    def publish_simulated_detection(self):
        simulated_detection = Float32MultiArray()
        simulated_detection.data = [1.0, 320.0, 240.0, 1.5, 5000.0, 0.05, 0.9, 0.5, 0.5]
        self.simulated_detection_pub.publish(simulated_detection)
        if not self.simulate_object:
            return
        if not self.simulated_detection_active and self.detected_object is not None:
            return
        parsed = self._parse_detection(simulated_detection)
        if parsed is None:
            return
        now = self.get_clock().now()
        if self.object_detection_time is not None:
            elapsed = (now - self.object_detection_time).nanoseconds / 1e9
            if elapsed < self.DETECTION_DEBOUNCE:
                return
        self.detected_object = parsed
        self.object_detection_time = now
        self.simulated_detection_active = True
        self.use_dynamic_waypoints = True
        self.pending_detection_merge = True
        self._set_anchors(
            target_world=self.target_world, obj_depth=self.detected_object["depth"]
        )
        if self.dynamic_waypoints is None or not self.use_dynamic_waypoints:
            self._rebuild_active_with_detection()

    def _build_simulated_object_waypoints(self, obj_world, radius, base_z):
        radius = max(self.min_orbit_radius_m, radius)
        num_pts = 16
        waypoints = []
        z = max(self.min_altitude_m, min(self.max_altitude_m, base_z))
        for i in range(num_pts):
            t = 2.0 * math.pi * i / num_pts
            x = obj_world[0] + radius * math.cos(t)
            y = obj_world[1] + radius * math.sin(t)
            waypoints.append(self._clamp_waypoint_to_bounds((x, y, z)))
        return waypoints

    def _compute_simulated_object_position(self, anchor_wp):
        """
        Place a simulated object just to the right of the initial trajectory direction,
        with an optional absolute offset. This keeps the orbit circular and offset
        from the nominal path.
        """
        base_x, base_y, base_z = anchor_wp
        yaw = self.current_yaw

        dx_side = math.sin(yaw) * self.simulated_object_side_offset_m
        dy_side = -math.cos(yaw) * self.simulated_object_side_offset_m

        sign_y = 1.0 if base_y >= 0.0 else -1.0
        dy_out = sign_y * self.simulated_object_outward_offset_m

        obj_world = (
            base_x + self.simulated_object_world_offset[0] + dx_side,
            base_y + self.simulated_object_world_offset[1] + dy_side + dy_out,
            base_z + self.simulated_object_world_offset[2],
        )
        obj_world = (
            obj_world[0],
            obj_world[1],
            max(self.min_altitude_m, min(self.max_altitude_m, obj_world[2])),
        )
        return obj_world

    def _insert_simulated_object_orbit(self, waypoints):
        if not waypoints:
            return waypoints


        idx_max_y = max(range(len(waypoints)), key=lambda i: waypoints[i][1])
        anchor_wp = waypoints[idx_max_y]
        obj_world = self._compute_simulated_object_position(anchor_wp)
        self.last_target_world = obj_world
        prev_wp = waypoints[idx_max_y - 1] if idx_max_y - 1 >= 0 else None
        next_wp = waypoints[idx_max_y + 1] if idx_max_y + 1 < len(waypoints) else None
        orbit = self._build_framing_arc(
            obj_world,
            anchor_wp,
            self.simulated_object_radius_m,
            prev_wp=prev_wp,
            next_wp=next_wp,
        )
        return waypoints[: idx_max_y + 1] + orbit + waypoints[idx_max_y + 1 :]
