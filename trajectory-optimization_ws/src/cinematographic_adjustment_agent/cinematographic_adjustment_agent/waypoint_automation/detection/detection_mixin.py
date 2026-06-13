import math

from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Float32, Float32MultiArray


class DetectionMixin:
    def light_callback(self, msg: Vector3Stamped):
        if not self.light_avoidance_enabled:
            self.light_vector = None
            self.light_distance = None
            return

        vec = (float(msg.vector.x), float(msg.vector.y), float(msg.vector.z))
        self.light_vector = vec
        norm = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])
        self.light_distance = norm if norm > 1e-6 else None

        try:
            self.cost_calculator.update_light_vector(msg)
        except AttributeError as error:
            self.get_logger().debug(f"light vector update skipped: {error}")

    def focus_depth_callback(self, msg: Float32):
        try:
            self.focus_depth_m = float(getattr(msg, "data", None))
        except (TypeError, ValueError) as error:
            self.get_logger().warn(f"focus_depth_m nulled; bad Float32 data field: {error}")
            self.focus_depth_m = None

    def _is_published_target(self, stamp):
        last = self._last_published_target_stamp
        if last is None:
            return False
        return last.sec == stamp.sec and last.nanosec == stamp.nanosec

    def _publish_target_world(self):
        target = self.target_world or self.last_target_world
        if target is None:
            return
        msg = PointStamped()
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.point.x = float(target[0])
        msg.point.y = float(target[1])
        msg.point.z = float(target[2])
        self.target_world_pub.publish(msg)
        self._last_published_target_stamp = stamp

    def target_world_callback(self, msg: PointStamped):
        if self._is_published_target(msg.header.stamp):
            self._last_published_target_stamp = None
            return
        try:
            p = msg.point
            self.target_world = (float(p.x), float(p.y), float(p.z))
            self.last_target_world = self.target_world
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warn(f"target world parse failed: {error}")
            self.target_world = None
            self.last_target_world = None
            return

        try:
            self._set_anchors(target_world=self.target_world, obj_depth=self.focus_depth_m)
        except (AttributeError, TypeError, ValueError, IndexError) as error:
            self.get_logger().warn(f"anchor update failed; nulling target world: {error}")
            self.target_world = None
            self.last_target_world = None

    def _parse_detection(self, msg: Float32MultiArray):
        if msg is None or msg.data is None:
            return None

        data = [float(x) for x in msg.data]
        if len(data) < 1:
            return None

        num_objects = int(round(data[0]))
        if num_objects >= 1 and len(data) >= 4:
            center_x = data[1]
            center_y = data[2]
            depth_m = data[3]
            if not (depth_m > 0.1 and math.isfinite(depth_m)):
                return None

            norm_x = None
            norm_y = None
            if len(data) >= 9:
                norm_x = data[7]
                norm_y = data[8]

            return {
                "center_x": center_x,
                "center_y": center_y,
                "depth": depth_m,
                "norm_x": norm_x,
                "norm_y": norm_y,
            }

        if len(data) >= 3:
            x_center = data[0]
            y_center = data[1]
            depth_m = data[2]
            if not (depth_m > 0.1 and math.isfinite(depth_m)):
                return None
            return {
                "center_x": x_center,
                "center_y": y_center,
                "depth": depth_m,
                "norm_x": x_center if 0.0 <= x_center <= 1.0 else None,
                "norm_y": y_center if 0.0 <= y_center <= 1.0 else None,
            }

        return None

    def _compute_object_world(self, detection):
        if detection is None:
            return None
        obj_depth = float(detection["depth"])
        center_x = float(detection["center_x"])
        center_y = float(detection["center_y"])
        norm_x = detection.get("norm_x", None)
        norm_y = detection.get("norm_y", None)

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
            return None, obj_depth

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        world_dx = cos_yaw * obj_x_m - sin_yaw * obj_y_m
        world_dy = sin_yaw * obj_x_m + cos_yaw * obj_y_m
        center_world = (base_pos[0] + world_dx, base_pos[1] + world_dy, base_pos[2])
        self.last_target_world = center_world
        return center_world, obj_depth

    def detection_callback(self, msg: Float32MultiArray):
        if self.baseline_mode:
            return
        now = self.get_clock().now()
        if self.object_detection_time is not None:
            elapsed = (now - self.object_detection_time).nanoseconds / 1e9
            if elapsed < self.DETECTION_DEBOUNCE:
                return

        parsed = self._parse_detection(msg)
        if parsed is None:
            return

        self.detected_object = parsed
        self.object_detection_time = now
        self.simulated_detection_active = False
        self.use_dynamic_waypoints = True
        self.pending_detection_merge = True
        obj_depth = float(parsed.get("depth", 0.0))
        computed = self._compute_object_world(parsed)
        if computed is not None:
            target_world_pos, computed_depth = computed
            if target_world_pos is not None:
                self.target_world = target_world_pos
                self.last_target_world = target_world_pos
            if computed_depth is not None:
                obj_depth = computed_depth
        self._publish_target_world()
        self._set_anchors(target_world=self.target_world, obj_depth=obj_depth)
        self._rebuild_active_with_detection()

    def _rebuild_active_with_detection(self):
        if self.start_pos is None or self.origin_offset is None:
            return

        use_dynamic = bool(self.use_dynamic_waypoints_on_detection)
        if use_dynamic:
            self.generate_cinematographic_waypoints()
            success = bool(self.last_cinematic_waypoints)
            self.use_dynamic_waypoints = success
            self.pending_detection_merge = not success
            if not success:
                self.get_logger().warn(
                    "Dynamic waypoints requested but unavailable; inserting anchor splicing"
                )
        else:
            self.use_dynamic_waypoints = False
            self.pending_detection_merge = True

        self.active_waypoints = None
        self.waypoints_sent = False
        self.current_target_index = 0
        self.last_published_index = -1
        self._initialize_waypoint_sequence()
