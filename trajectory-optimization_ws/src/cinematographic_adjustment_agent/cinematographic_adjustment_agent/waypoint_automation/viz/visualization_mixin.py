import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class VisualizationMixin:
    def _publish_target_marker(self):
        target = self.target_world or self.last_target_world
        if target is None:
            return
        now = self.get_clock().now().to_msg()
        scale = max(0.01, float(self.target_marker_scale_m))
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.frame_id
        marker.ns = "cinematography_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float(target[0])
        marker.pose.position.y = float(target[1])
        marker.pose.position.z = float(target[2])
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = 0.8
        marker.color.g = 0.1
        marker.color.b = 0.9
        marker.color.a = 0.9
        self.target_marker_pub.publish(marker)

    def _debug_arc(self, arc_pts, target_world, desired_distance):
        if not self.debug_arc:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "cinematography_arc"
        marker.id = 0
        if not arc_pts:
            marker.action = Marker.DELETE
            self.arc_marker_pub.publish(marker)
            return

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = max(0.01, float(self.debug_arc_marker_scale))
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.9
        for wp in arc_pts:
            p = Point()
            p.x = float(wp[0])
            p.y = float(wp[1])
            p.z = float(wp[2])
            marker.points.append(p)
        self.arc_marker_pub.publish(marker)

        if target_world is None:
            return
        now = self.get_clock().now()
        if self.last_arc_log_time is not None:
            elapsed = (now - self.last_arc_log_time).nanoseconds / 1e9
            if elapsed < self.debug_arc_log_period_s:
                return
        self.last_arc_log_time = now

        desired = float(desired_distance)
        errors = []
        for wp in arc_pts:
            dist = math.dist(wp, target_world)
            if math.isfinite(dist):
                errors.append(abs(dist - desired))
        if not errors:
            return
        max_err = max(errors)
        mean_err = sum(errors) / len(errors)
        self.get_logger().info(
            f"Arc debug: points={len(arc_pts)} desired={desired:.3f} "
            f"max_err={max_err:.3f} mean_err={mean_err:.3f}"
        )

    def _publish_debug_visualization(self):
        if not self.enable_debug_viz:
            return

        markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = self.frame_id
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        now = self.get_clock().now().to_msg()

        if self.viz_static_path and self.viz_data["static_world"] is not None:
            markers.markers.extend(
                self._create_path_markers(
                    self.viz_data["static_world"],
                    "static_path",
                    (0.5, 0.5, 0.5, 0.4),
                    0.02,
                    now,
                    0,
                )
            )

        if self.viz_arc_only and self.viz_data["cinematographic_waypoints"] is not None:
            arc_pts = self.viz_data["cinematographic_waypoints"]
            arc_path_color = (
                (0.0, 1.0, 0.0, 1.0) if self.framing_active else (1.0, 0.0, 0.0, 1.0)
            )
            arc_point_color = (
                (0.0, 1.0, 0.0, 0.8) if self.framing_active else (1.0, 0.0, 0.0, 0.8)
            )
            markers.markers.extend(
                self._create_path_markers(
                    arc_pts,
                    "arc_path",
                    arc_path_color,
                    0.06,
                    now,
                    100,
                )
            )
            for i, wp in enumerate(arc_pts):
                markers.markers.append(
                    self._create_waypoint_sphere(
                        wp,
                        200 + i,
                        "arc_waypoints",
                        arc_point_color,
                        self._waypoint_diameter(),
                        now,
                    )
                )

        if self.viz_transitions:
            transition_in = self.viz_data["transition_in"] or []
            if transition_in:
                markers.markers.extend(
                    self._create_path_markers(
                        transition_in,
                        "transition_in",
                        (1.0, 1.0, 0.0, 0.8),
                        0.04,
                        now,
                        300,
                    )
                )
            transition_out = self.viz_data["transition_out"] or []
            if transition_out:
                markers.markers.extend(
                    self._create_path_markers(
                        transition_out,
                        "transition_out",
                        (1.0, 0.5, 0.0, 0.8),
                        0.04,
                        now,
                        400,
                    )
                )
        if self.viz_data["arc_start"] is not None:
            markers.markers.append(
                self._create_waypoint_sphere(
                    self.viz_data["arc_start"],
                    500,
                    "arc_anchors",
                    (1.0, 0.0, 0.0, 1.0),
                    0.12,
                    now,
                )
            )
            markers.markers.append(
                self._create_text_marker(
                    self.viz_data["arc_start"],
                    "ARC START",
                    501,
                    "arc_labels",
                    (1.0, 1.0, 1.0, 1.0),
                    0.15,
                    now,
                )
            )

        if self.viz_data["arc_end"] is not None:
            markers.markers.append(
                self._create_waypoint_sphere(
                    self.viz_data["arc_end"],
                    510,
                    "arc_anchors",
                    (0.0, 0.0, 1.0, 1.0),
                    0.12,
                    now,
                )
            )
            markers.markers.append(
                self._create_text_marker(
                    self.viz_data["arc_end"],
                    "ARC END",
                    511,
                    "arc_labels",
                    (1.0, 1.0, 1.0, 1.0),
                    0.15,
                    now,
                )
            )

        if self.viz_dynamic_path and self.viz_data["combined_world"] is not None:
            markers.markers.extend(
                self._create_path_markers(
                    self.viz_data["combined_world"],
                    "combined_path",
                    (0.0, 0.8, 1.0, 0.6),
                    0.03,
                    now,
                    600,
                )
            )

        if self.viz_active_path and self.active_waypoints is not None:
            origin = (
                self.origin_offset
                if self.origin_offset is not None
                else (0.0, 0.0, 0.0)
            )
            active_world = [
                (wp[0] + origin[0], wp[1] + origin[1], wp[2] + origin[2])
                for wp in self.active_waypoints
            ]
            markers.markers.extend(
                self._create_path_markers(
                    active_world,
                    "active_path",
                    (1.0, 0.0, 1.0, 1.0),
                    0.05,
                    now,
                    700,
                )
            )
            if 0 <= self.current_target_index < len(active_world):
                current_wp = active_world[self.current_target_index]
                markers.markers.append(
                    self._create_waypoint_sphere(
                        current_wp,
                        800,
                        "active_targets",
                        (1.0, 0.0, 1.0, 1.0),
                        self._waypoint_diameter(),
                        now,
                    )
                )
                markers.markers.append(
                    self._create_text_marker(
                        current_wp,
                        f"CURRENT #{self.current_target_index}",
                        801,
                        "active_labels",
                        (1.0, 1.0, 0.0, 1.0),
                        0.2,
                        now,
                    )
                )
            if (
                self.active_waypoints
                and self.last_published_index >= self.current_target_index
            ):
                queued_world = active_world[
                    self.current_target_index : self.last_published_index + 1
                ]
                if queued_world:
                    markers.markers.extend(
                        self._create_path_markers(
                            queued_world,
                            "waypoint_queue",
                            (1.0, 0.6, 0.0, 0.9),
                            0.04,
                            now,
                            8000,
                        )
                    )
                    for i, wp in enumerate(queued_world):
                        markers.markers.append(
                            self._create_waypoint_sphere(
                                wp,
                                8100 + i,
                                "queue_points",
                                (1.0, 0.6, 0.0, 0.8),
                                self._waypoint_diameter() * 0.4,
                                now,
                            )
                        )

        self.debug_markers_pub.publish(markers)
        self._publish_transition_viz(now)

    def _publish_transition_viz(self, stamp):
        if self.transition_viz_pub is None:
            return
        markers = MarkerArray()
        paths = [
            (
                self.viz_data.get("transition_in", []),
                "transition_viz_in",
                (0.8, 0.0, 1.0, 1.0),
                0.04,
                6000,
            ),
            (
                self.viz_data.get("transition_out", []),
                "transition_viz_out",
                (1.0, 0.3, 0.0, 1.0),
                0.04,
                6010,
            ),
        ]
        for pts, ns, color, width, base_id in paths:
            if not pts:
                continue
            markers.markers.extend(
                self._create_path_markers(pts, ns, color, width, stamp, base_id)
            )
        self.transition_viz_pub.publish(markers)

    def _waypoint_diameter(self):
        """Convert the configured waypoint radius into the diameter expected by Marker scales."""
        return max(0.001, 2.0 * float(self.waypoint_radius_m))

    def _create_path_markers(
        self, waypoints, namespace, color, line_width, stamp, marker_id
    ):
        markers = []

        line_marker = Marker()
        line_marker.header.stamp = stamp
        line_marker.header.frame_id = self.frame_id
        line_marker.ns = namespace
        line_marker.id = int(marker_id)
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = float(line_width)
        line_marker.color.r = float(color[0])
        line_marker.color.g = float(color[1])
        line_marker.color.b = float(color[2])
        line_marker.color.a = float(color[3])
        line_marker.pose.orientation.w = 1.0

        for wp in waypoints:
            p = Point()
            p.x = float(wp[0])
            p.y = float(wp[1])
            p.z = float(wp[2])
            line_marker.points.append(p)

        markers.append(line_marker)
        return markers

    def _create_waypoint_sphere(
        self, position, marker_id, namespace, color, scale, stamp
    ):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(scale)
        marker.scale.y = float(scale)
        marker.scale.z = float(scale)
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        return marker

    def _create_text_marker(
        self, position, text, marker_id, namespace, color, scale, stamp, offset_z=0.2
    ):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2]) + float(offset_z)
        marker.pose.orientation.w = 1.0
        marker.scale.z = float(scale)
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        marker.text = text
        return marker
