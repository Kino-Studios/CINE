import math

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


class AnchorMixin:
    def _set_anchors(self, target_world=None, obj_depth=None):
        if self.current_pos is None or self.origin_offset is None:
            return

        static_world = [self._rel_to_world(wp) for wp in self._scaled_static_waypoints()]
        if target_world is None and self.detected_object is not None:
            target_world, _ = self._compute_object_world(self.detected_object)

        if target_world is not None:
            desired_distance = self._resolve_focus_distance(obj_depth, use_anchor=False)
            start_idx, end_idx = self._select_anchor_pair(
                static_world,
                target_world,
                desired_distance,
            )
            dist_start = math.dist(static_world[start_idx], target_world)
            dist_end = math.dist(static_world[end_idx], target_world)

            if math.isfinite(dist_start) and math.isfinite(dist_end):
                self.anchor_focus_distance = max(dist_start, dist_end)
            else:
                self.anchor_focus_distance = None
            self.anchor_start_world = static_world[start_idx]
            self.anchor_end_world = static_world[end_idx]
            self.anchor_source = 'static_pair'
            self.get_logger().debug(
                f'Anchors static_pair source={self.anchor_source} start={start_idx} end={end_idx} focus_dist={self.anchor_focus_distance} '
                f'target={tuple(target_world)} start_world={tuple(self.anchor_start_world)} end_world={tuple(self.anchor_end_world)}'
            )
        else:
            start_idx = 0
            if self.active_waypoints is not None and 0 <= int(self.current_target_index) < len(
                self.active_waypoints
            ):
                start_idx = int(self.current_target_index)
            else:
                start_idx = min(
                    range(len(static_world)),
                    key=lambda i: (
                        (static_world[i][0] - self.current_pos[0]) ** 2
                        + (static_world[i][1] - self.current_pos[1]) ** 2
                        + (static_world[i][2] - self.current_pos[2]) ** 2
                    ),
                )
            end_idx = len(static_world) - 1
            self.anchor_start_world = static_world[start_idx]
            self.anchor_end_world = static_world[end_idx]
            self.anchor_source = 'fallback'
            self.anchor_focus_distance = None
            self.get_logger().debug(
                f'Anchors fallback source={self.anchor_source} start={start_idx} end={end_idx} '
                f'start_world={tuple(self.anchor_start_world)} end_world={tuple(self.anchor_end_world)}'
            )

        self.anchor_start_idx = int(start_idx)
        self.anchor_end_idx = int(end_idx)
        self.anchors_set = True
        self._publish_anchor_points()
        self._publish_anchor_focus_distance()

    def _segment_circle_intersection(self, p0, p1, target_world, desired_distance):
        tx, ty, tz = (
            float(target_world[0]),
            float(target_world[1]),
            float(target_world[2]),
        )

        def _dist_at(t):
            x = p0[0] + t * (p1[0] - p0[0])
            y = p0[1] + t * (p1[1] - p0[1])
            z = p0[2] + t * (p1[2] - p0[2])
            dx = x - tx
            dy = y - ty
            dz = z - tz

            return math.sqrt(dx * dx + dy * dy + dz * dz), (x, y, z)

        d0, _ = _dist_at(0.0)
        d1, _ = _dist_at(1.0)
        f0 = d0 - desired_distance
        f1 = d1 - desired_distance

        if not (math.isfinite(f0) and math.isfinite(f1)):
            return None
        if abs(f0) <= 1e-6:
            _, p = _dist_at(0.0)

            return p
        if abs(f1) <= 1e-6:
            _, p = _dist_at(1.0)

            return p
        if f0 * f1 > 0.0:
            return None

        t0 = 0.0
        t1 = 1.0
        for _ in range(12):
            tm = 0.5 * (t0 + t1)
            dm, _ = _dist_at(tm)
            fm = dm - desired_distance
            if f0 * fm <= 0.0:
                t1 = tm
                f1 = fm
            else:
                t0 = tm
                f0 = fm
        _, p = _dist_at(0.5 * (t0 + t1))

        return p

    def _compute_anchor_crossings(self, static_world, target_world, desired_distance):
        n = len(static_world)
        if n < 2:
            return None

        desired = max(float(desired_distance), float(self.min_orbit_radius_m))
        distances = []
        for wp in static_world:
            dx = float(wp[0]) - float(target_world[0])
            dy = float(wp[1]) - float(target_world[1])
            dz = float(wp[2]) - float(target_world[2])
            distances.append(math.sqrt(dx * dx + dy * dy + dz * dz))

        closest_idx = int(min(range(n), key=lambda i: distances[i]))
        left_anchor = None
        left_idx = None
        for i in range(closest_idx, 0, -1):
            p0 = static_world[i - 1]
            p1 = static_world[i]
            hit = self._segment_circle_intersection(p0, p1, target_world, desired)
            if hit is not None:
                left_anchor = hit
                left_idx = i - 1
                break

        right_anchor = None
        right_idx = None
        for i in range(closest_idx, n - 1):
            p0 = static_world[i]
            p1 = static_world[i + 1]
            hit = self._segment_circle_intersection(p0, p1, target_world, desired)
            if hit is not None:
                right_anchor = hit
                right_idx = i + 1
                break

        if left_anchor is None or right_anchor is None:
            return None

        if left_idx is None or right_idx is None:
            return None
        if right_idx <= left_idx:
            return None

        return (left_idx, right_idx, left_anchor, right_anchor)

    def _select_anchor_pair(self, static_world, target_world, desired_distance):
        n = len(static_world)
        if n < 2:
            return 0, max(0, n - 1)

        tx, ty, tz = (
            float(target_world[0]),
            float(target_world[1]),
            float(target_world[2]),
        )
        desired = max(float(desired_distance), float(self.min_orbit_radius_m))
        distances = []
        for wp in static_world:
            dx = float(wp[0]) - tx
            dy = float(wp[1]) - ty
            dz = float(wp[2]) - tz
            distances.append(math.sqrt(dx * dx + dy * dy + dz * dz))

        closest_idx = int(min(range(n), key=lambda i: distances[i]))
        min_gap = max(1, int(self.anchor_min_gap))
        balance_weight = 1.5

        best_pair = (0, min_gap)
        best_cost = float('inf')
        for i in range(n - 1):
            if i > closest_idx:
                break
            di = distances[i]
            if not math.isfinite(di) or di < self.min_orbit_radius_m:
                continue
            for j in range(i + min_gap, n):
                if j < closest_idx:
                    continue
                dj = distances[j]
                if not math.isfinite(dj) or dj < self.min_orbit_radius_m:
                    continue
                cost = abs(di - desired) + abs(dj - desired) + balance_weight * abs(di - dj)
                if cost < best_cost:
                    best_cost = cost
                    best_pair = (i, j)

        return best_pair

    def _publish_anchor_points(self):
        if not self.anchors_set or self.anchor_start_idx is None or self.anchor_end_idx is None:
            return
        static_world = [self._rel_to_world(wp) for wp in self._scaled_static_waypoints()]
        if not static_world:
            return
        start_idx = int(max(0, min(self.anchor_start_idx, len(static_world) - 1)))
        end_idx = int(max(start_idx, min(self.anchor_end_idx, len(static_world) - 1)))
        start_wp = static_world[start_idx]
        end_wp = static_world[end_idx]
        now = self.get_clock().now().to_msg()
        start_msg = PointStamped()
        start_msg.header.stamp = now
        start_msg.header.frame_id = self.frame_id
        start_msg.point.x = float(start_wp[0])
        start_msg.point.y = float(start_wp[1])
        start_msg.point.z = float(start_wp[2])
        self.anchor_start_pub.publish(start_msg)
        end_msg = PointStamped()
        end_msg.header.stamp = now
        end_msg.header.frame_id = self.frame_id
        end_msg.point.x = float(end_wp[0])
        end_msg.point.y = float(end_wp[1])
        end_msg.point.z = float(end_wp[2])
        self.anchor_end_pub.publish(end_msg)
        self._publish_anchor_markers(start_wp, end_wp, now)

    def _publish_anchor_markers(self, start_wp, end_wp, now_msg):
        scale = max(0.01, float(self.anchor_marker_scale_m))
        markers = MarkerArray()

        seq = getattr(self, '_anchor_marker_seq', 0) + 1
        self._anchor_marker_seq = seq
        start_marker = Marker()
        start_marker.header.stamp = now_msg
        start_marker.header.frame_id = self.frame_id
        start_marker.ns = 'cinematography_anchors'
        start_marker.id = seq * 2
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.orientation.w = 1.0
        start_marker.pose.position.x = float(start_wp[0])
        start_marker.pose.position.y = float(start_wp[1])
        start_marker.pose.position.z = float(start_wp[2])
        start_marker.scale.x = scale
        start_marker.scale.y = scale
        start_marker.scale.z = scale
        start_marker.color.r = 1.0
        start_marker.color.g = 0.2
        start_marker.color.b = 0.2
        start_marker.color.a = 0.9
        markers.markers.append(start_marker)

        end_marker = Marker()
        end_marker.header.stamp = now_msg
        end_marker.header.frame_id = self.frame_id
        end_marker.ns = 'cinematography_anchors'
        end_marker.id = seq * 2 + 1
        end_marker.type = Marker.SPHERE
        end_marker.action = Marker.ADD
        end_marker.pose.orientation.w = 1.0
        end_marker.pose.position.x = float(end_wp[0])
        end_marker.pose.position.y = float(end_wp[1])
        end_marker.pose.position.z = float(end_wp[2])
        end_marker.scale.x = scale
        end_marker.scale.y = scale
        end_marker.scale.z = scale
        end_marker.color.r = 0.2
        end_marker.color.g = 0.2
        end_marker.color.b = 1.0
        end_marker.color.a = 0.9
        markers.markers.append(end_marker)

        self.anchor_marker_pub.publish(markers)

    def _publish_anchor_focus_distance(self):
        if self.anchor_distance_pub is None or self.anchor_focus_distance is None:
            return
        if not math.isfinite(self.anchor_focus_distance) or self.anchor_focus_distance <= 0.0:
            return
        self.get_logger().info(
            f'Publishing anchor focus distance: {self.anchor_focus_distance:.3f} m '
            f'(source={self.anchor_source} start={self.anchor_start_idx} end={self.anchor_end_idx})'
        )
        msg = Float32()
        msg.data = float(self.anchor_focus_distance)
        self.anchor_distance_pub.publish(msg)

    def _find_candidate_indices(self, target_world):
        if self.active_waypoints is None:
            return []

        candidates = []
        for i, wp in enumerate(self.active_waypoints):
            rel = self._rel_to_world(wp)
            dist_sq = (
                (rel[0] - target_world[0]) ** 2
                + (rel[1] - target_world[1]) ** 2
                + (rel[2] - target_world[2]) ** 2
            )
            candidates.append((i, dist_sq))

        candidates.sort(key=lambda x: x[1])
        if not candidates:
            return []

        min_dist_sq = candidates[0][1]
        threshold_sq = min_dist_sq + 0.25

        return [c[0] for c in candidates if c[1] <= threshold_sq]

    def _update_framing_indices(self):
        self.framing_start_idx = None
        self.framing_end_idx = None
        if self.active_waypoints is None:
            return
        if not self.anchors_set:
            return
        if self.anchor_start_world is None or self.anchor_end_world is None:
            return

        start_candidates = self._find_candidate_indices(self.anchor_start_world)
        end_candidates = self._find_candidate_indices(self.anchor_end_world)

        if not start_candidates or not end_candidates:
            return

        expected_len = getattr(self, 'generated_arc_length', 0)
        best_pair = None
        best_cost = float('inf')

        for s in start_candidates:
            for e in end_candidates:
                if e > s:
                    diff = e - s
                    cost = abs(diff - expected_len)
                    if cost < best_cost:
                        best_cost = cost
                        best_pair = (s, e)

        if best_pair is not None:
            self.framing_start_idx, self.framing_end_idx = best_pair
        else:
            s = start_candidates[0]
            e = end_candidates[0]
            if e < s:
                s, e = e, s
            self.framing_start_idx = s
            self.framing_end_idx = e

        self.get_logger().info(
            f'Framing indices updated start={self.framing_start_idx} end={self.framing_end_idx} '
            f'(len={self.framing_end_idx - self.framing_start_idx}, expected={expected_len}, '
            f'mode={"baseline" if getattr(self, "baseline_mode", False) else "cinematic"})'
        )

    def _is_framing_idx(self, idx):
        if self.framing_start_idx is None or self.framing_end_idx is None:
            return False

        return self.framing_start_idx <= idx <= self.framing_end_idx
