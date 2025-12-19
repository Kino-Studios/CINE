#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math
import time


class RoomWaypointSender(Node):
    
    def __init__(self):
        super().__init__('room_waypoint_sender')
        
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/move_base_simple/goal',
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone_0_odometry',
            self.odom_callback,
            10
        )
        # Some setups publish visual odom instead; subscribe to both for robustness.
        self.visual_odom_sub = self.create_subscription(
            Odometry,
            '/drone_0_visual_odom',
            self.odom_callback,
            10
        )
        
        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/object_detections',
            self.detection_callback,
            10
        )
        
        self.start_pos = None
        self.waypoints_sent = False
        self.ready_time = None
        self.STARTUP_DELAY = 5.0
        self.velocity_settled = False
        self.VELOCITY_THRESHOLD = 0.01        
        self.object_detection_time = None
        self.DETECTION_DEBOUNCE = 2.0        
        self.DISTANCE_THRESHOLD_M = 0.5
        self.current_pos = None

        # Camera model for converting detections to meters (pinhole).
        # These defaults match the simulated detector unless overridden.
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_fx', 387.229248046875)
        self.declare_parameter('camera_fy', 387.229248046875)
        self.declare_parameter('camera_cx', 320.5)
        self.declare_parameter('camera_cy', 240.5)
        self.declare_parameter('orbit_altitude_m', 1.5)
        self.declare_parameter('min_altitude_m', 0.2)
        self.declare_parameter('max_altitude_m', 3.0)
        self.declare_parameter('min_orbit_radius_m', 0.6)
        self.declare_parameter('max_waypoint_jump_m', 1.0)
        self.declare_parameter('world_bounds_min', [-10.0, -10.0, 0.0])
        self.declare_parameter('world_bounds_max', [10.0, 10.0, 5.0])

        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.fx = float(self.get_parameter('camera_fx').value)
        self.fy = float(self.get_parameter('camera_fy').value)
        self.cx = float(self.get_parameter('camera_cx').value)
        self.cy = float(self.get_parameter('camera_cy').value)
        self.orbit_altitude_m = float(self.get_parameter('orbit_altitude_m').value)
        self.min_altitude_m = float(self.get_parameter('min_altitude_m').value)
        self.max_altitude_m = float(self.get_parameter('max_altitude_m').value)
        self.min_orbit_radius_m = float(self.get_parameter('min_orbit_radius_m').value)
        self.max_waypoint_jump_m = float(self.get_parameter('max_waypoint_jump_m').value)
        self.world_bounds_min = [float(v) for v in self.get_parameter('world_bounds_min').value]
        self.world_bounds_max = [float(v) for v in self.get_parameter('world_bounds_max').value]

        self.static_waypoints = [ 
            (0.1, 0.3, 0.3),     
            (0.15, 0.8, 0.32),  
            (0.2, 1.3, 0.35),    
            (0.25, 1.8, 0.38), 
            (0.3, 2.3, 0.42),  
            (0.35, 2.8, 0.48), 
            (0.4, 3.2, 0.58), 
            (0.45, 3.5, 0.70), 
            (0.5, 3.6, 0.80),  
            (0.5, 3.5, 0.82),  
            (0.48, 3.2, 0.78),   
            (0.45, 2.8, 0.70),  
            (0.4, 2.3, 0.58),  
            (0.3, 1.8, 0.48),   
            (0.2, 1.2, 0.38),  
            (0.05, 0.6, 0.32), 
            (-0.1, 0.0, 0.30),   
            (-0.25, -0.6, 0.30),  
            (-0.35, -1.2, 0.32),
            (-0.40, -1.8, 0.38),   
            (-0.42, -2.3, 0.48), 
            (-0.40, -2.8, 0.58),  
            (-0.35, -3.2, 0.70),
            (-0.25, -3.5, 0.80),
            (-0.20, -3.6, 0.82),  
            (-0.15, -3.5, 0.78),
            (-0.10, -3.2, 0.70),  
            (-0.05, -2.8, 0.58), 
            (0.05, -2.3, 0.48), 
            (0.15, -1.8, 0.38), 
            (0.25, -1.2, 0.32),  
            (0.3, -0.6, 0.30),
        ]
        
        self.dynamic_waypoints = None
        self.detected_object = None
        self.use_dynamic_waypoints = False
        self.active_waypoints = None
        self.active_waypoint_index = 0
        self.active_waypoint_sent = False
        
        self.fake_detection_pub = self.create_publisher(
            Float32MultiArray,
            '/object_detections',
            10
        )
        
        self.timer = self.create_timer(0.5, self.check_and_send_waypoints)
        
        self.get_logger().info('[SENDER] Room waypoint sender initialized - cinematographic object tracking enabled')
        self.get_logger().info('[SENDER] Waiting for drone odometry and object detections...')
    
    def odom_callback(self, msg):
        self.current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        velocity_mag = (vx**2 + vy**2 + vz**2)**0.5
        
        if self.start_pos is None:
            self.start_pos = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            )
            self.ready_time = self.get_clock().now()
            self.get_logger().info(f'[ODOMETRY] Drone starting position: ({self.start_pos[0]:.2f}, {self.start_pos[1]:.2f}, {self.start_pos[2]:.2f})')
            self.get_logger().info(f'[ODOMETRY] Waiting for velocity to settle below {self.VELOCITY_THRESHOLD} m/s...')

        if velocity_mag < self.VELOCITY_THRESHOLD:
            
            if not self.velocity_settled:
                self.velocity_settled = True
                self.get_logger().info(f'[ODOMETRY] Velocity settled: {velocity_mag:.4f} m/s')
        else:
            self.velocity_settled = False
    
    def detection_callback(self, msg):
        try:
            # Once we start executing a waypoint sequence, ignore further detections to avoid
            # swapping waypoint sets mid-flight (planner instability).
            if self.active_waypoints is not None:
                return

            det = self._parse_detection(msg)
            if det is None:
                return

            self.detected_object = det

            if self.start_pos is not None and not self.waypoints_sent:
                now = self.get_clock().now()
                if self.object_detection_time is None:
                    self.generate_cinematographic_waypoints()
                    self.use_dynamic_waypoints = True
                    self.object_detection_time = now
                else:
                    elapsed = (now - self.object_detection_time).nanoseconds / 1e9
                    if elapsed >= self.DETECTION_DEBOUNCE:
                        self.generate_cinematographic_waypoints()
                        self.object_detection_time = now
        
        except Exception as e:
            self.get_logger().warn(f'Error processing: {e}')

    def _parse_detection(self, msg: Float32MultiArray):
        """
        Supports two schemas:
        1) SimulatedObjectDetector: [num_objects, center_x_px, center_y_px, depth_m, area, coverage, centering, norm_x, norm_y, ...]
        2) Legacy/handmade: [x_center, y_center, depth_m, bbox_width, bbox_height, (confidence)]
        """
        if msg is None or msg.data is None:
            return None
        data = [float(x) for x in msg.data]
        if len(data) < 1:
            return None

        # Schema 1: count-prefixed list.
        num_objects = int(round(data[0]))
        if num_objects >= 1 and len(data) >= 4:
            # First object starts at index 1.
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
                'center_x': center_x,
                'center_y': center_y,
                'depth': depth_m,
                'norm_x': norm_x,
                'norm_y': norm_y,
            }

        # Schema 2: direct fields (often normalized [0..1] for x/y).
        if len(data) >= 3:
            x_center = data[0]
            y_center = data[1]
            depth_m = data[2]
            if not (depth_m > 0.1 and math.isfinite(depth_m)):
                return None
            return {
                'center_x': x_center,
                'center_y': y_center,
                'depth': depth_m,
                'norm_x': x_center if 0.0 <= x_center <= 1.0 else None,
                'norm_y': y_center if 0.0 <= y_center <= 1.0 else None,
            }

        return None

    def process_detection_synchronously(self, detection_msg):
        """Process detection message directly without waiting for async callback."""
        try:
            det = self._parse_detection(detection_msg)
            if det is None:
                return
            self.detected_object = det
            self.get_logger().info(
                f"[CINEMATOGRAPHY] Object detected at ({det['center_x']:.2f}, {det['center_y']:.2f}) depth={det['depth']:.2f}m"
            )

            self.generate_cinematographic_waypoints()
            self.use_dynamic_waypoints = True
            self.object_detection_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().warn(f'Error in process_detection_synchronously: {e}')

    def generate_cinematographic_waypoints(self):
        if self.detected_object is None or self.start_pos is None:
            return
        
        obj_depth = float(self.detected_object['depth'])
        center_x = float(self.detected_object['center_x'])
        center_y = float(self.detected_object['center_y'])
        norm_x = self.detected_object.get('norm_x', None)
        norm_y = self.detected_object.get('norm_y', None)

        # Convert center to pixel coords if normalized was provided.
        if norm_x is not None and norm_y is not None:
            u = float(norm_x) * float(self.image_width)
            v = float(norm_y) * float(self.image_height)
        else:
            u = center_x
            v = center_y

        # Convert to meters in camera frame (approx; assumes camera aligned with world frame).
        if self.fx > 0 and self.fy > 0:
            obj_x_m = (u - self.cx) / self.fx * obj_depth
            obj_y_m = (v - self.cy) / self.fy * obj_depth
        else:
            obj_x_m = 0.0
            obj_y_m = 0.0
        
        circle_radius = max(float(self.min_orbit_radius_m), obj_depth * 0.4)
        
        num_waypoints = 16
        
        arc_span = 2 * math.pi / 3
        start_angle = -arc_span / 2
        
        cinematographic_waypoints = []
        
        for i in range(num_waypoints):
            angle = start_angle + (arc_span * i / (num_waypoints - 1))
            
            dx_circle = circle_radius * math.cos(angle)
            dy_circle = circle_radius * math.sin(angle)
            
            # Small composition bias based on normalized position if available.
            thirds_offset_x_m = 0.0
            thirds_offset_y_m = 0.0
            if norm_x is not None and norm_y is not None:
                thirds_offset_x_m = (float(norm_x) - 0.5) * 0.15
                thirds_offset_y_m = (float(norm_y) - 0.5) * 0.15

            wp_x_m = obj_x_m + dx_circle + thirds_offset_x_m
            wp_y_m = obj_y_m + dy_circle + thirds_offset_y_m

            # Depth is NOT altitude; keep a stable flight altitude for the orbit.
            wp_z_m = max(self.min_altitude_m, min(self.max_altitude_m, self.orbit_altitude_m))

            abs_x = self.start_pos[0] + wp_x_m
            abs_y = self.start_pos[1] + wp_y_m
            abs_z = wp_z_m
            
            safe = self._clamp_waypoint_to_bounds((abs_x, abs_y, abs_z))
            cinematographic_waypoints.append(safe)
        
        self.dynamic_waypoints = cinematographic_waypoints
        
        self.publish_simulated_detection()
        self.get_logger().info(
            f"[CINEMATOGRAPHY] Generated {len(cinematographic_waypoints)} arc waypoints "
            f"(radius={circle_radius:.2f}m, depth={obj_depth:.2f}m, z={self.orbit_altitude_m:.2f}m)"
        )

    def _clamp_waypoint_to_bounds(self, wp):
        x, y, z = wp
        x = max(self.world_bounds_min[0], min(self.world_bounds_max[0], float(x)))
        y = max(self.world_bounds_min[1], min(self.world_bounds_max[1], float(y)))
        z = max(self.world_bounds_min[2], min(self.world_bounds_max[2], float(z)))
        z = max(self.min_altitude_m, min(self.max_altitude_m, z))
        return (x, y, z)
    
    def publish_simulated_detection(self):
        
        fake_detection = Float32MultiArray()
        fake_detection.data = [
            1.0,      # num_objects
            320.0,    # center_x (px)
            240.0,    # center_y (px)
            1.5,      # depth (m)
            5000.0,   # area (px)
            0.05,     # coverage
            0.9,      # centering_score
            0.5,      # norm_x
            0.5,      # norm_y
        ]
        
        self.fake_detection_pub.publish(fake_detection)
    
    def check_and_send_waypoints(self):
        if self.start_pos is not None and not self.waypoints_sent and self.velocity_settled:
            elapsed = (self.get_clock().now() - self.ready_time).nanoseconds / 1e9
            if elapsed >= self.STARTUP_DELAY:
                if self.active_waypoints is None:
                    self.get_logger().info(f'[SENDER] Startup delay complete ({elapsed:.1f}s). Preparing waypoint sequence...')
                    self._initialize_waypoint_sequence()
                self._step_waypoint_sequence()
        elif self.start_pos is not None and not self.waypoints_sent:
            elapsed = (self.get_clock().now() - self.ready_time).nanoseconds / 1e9
            self.get_logger().debug(f'[SENDER] Waiting: start_pos={self.start_pos is not None}, waypoints_sent={self.waypoints_sent}, velocity_settled={self.velocity_settled}, elapsed={elapsed:.1f}s/{self.STARTUP_DELAY}s')
    
    def _initialize_waypoint_sequence(self):
        # First, ensure dynamic waypoints are generated by directly calling arc generation
        if self.start_pos is not None and self.detected_object is None:
            # Generate arc waypoints for simulated object
            fake_detection = Float32MultiArray()
            fake_detection.data = [
                1.0,      # num_objects
                320.0,    # center_x (px)
                240.0,    # center_y (px)
                1.5,      # depth (m)
                5000.0,   # area (px)
                0.05,     # coverage
                0.9,      # centering_score
                0.5,      # norm_x
                0.5,      # norm_y
            ]
            # Directly process the detection instead of publishing
            self.process_detection_synchronously(fake_detection)
        
        if self.use_dynamic_waypoints and self.dynamic_waypoints is not None:
            waypoints_to_send = self.dynamic_waypoints
            mode = "CINEMATOGRAPHIC"
            self.get_logger().info(f'[SENDER] Sending {len(waypoints_to_send)} cinematographic arc waypoints')
        else:
            absolute_waypoints = [
                (self.start_pos[0] + dx, self.start_pos[1] + dy, self.start_pos[2] + dz)
                for dx, dy, dz in self.static_waypoints
            ]
            waypoints_to_send = absolute_waypoints
            mode = "STATIC ELLIPTICAL"
            self.get_logger().info(f'[SENDER] Sending {len(waypoints_to_send)} static elliptical waypoints')

        clamped = [self._clamp_waypoint_to_bounds(wp) for wp in waypoints_to_send]
        self.active_waypoints = self._limit_consecutive_waypoint_jumps(clamped)
        self.active_waypoint_index = 0
        self.active_waypoint_sent = False
        self.get_logger().info(f'[SENDER] Waypoint mode={mode}, sequential send enabled (threshold={self.DISTANCE_THRESHOLD_M}m)')

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
                out.append(
                    (
                        prev[0] + t * dx,
                        prev[1] + t * dy,
                        prev[2] + t * dz,
                    )
                )
        if len(out) != len(waypoints):
            self.get_logger().info(f'[SENDER] Inserted {len(out) - len(waypoints)} intermediate waypoints (max_jump={self.max_waypoint_jump_m}m)')
        return out

    def _step_waypoint_sequence(self):
        if self.active_waypoints is None or self.current_pos is None:
            return

        if self.active_waypoint_index >= len(self.active_waypoints):
            self.get_logger().info(f'[SENDER] Completed waypoint sequence ({len(self.active_waypoints)} waypoints)')
            self.waypoints_sent = True
            self.timer.cancel()
            return

        target = self.active_waypoints[self.active_waypoint_index]
        dx = self.current_pos[0] - target[0]
        dy = self.current_pos[1] - target[1]
        dz = self.current_pos[2] - target[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5

        if not self.active_waypoint_sent:
            x, y, z = target
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'world'
            goal.pose.position.x = float(x)
            goal.pose.position.y = float(y)
            goal.pose.position.z = float(z)
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.active_waypoint_sent = True
            self.get_logger().info(
                f'[SENDER] Sent waypoint {self.active_waypoint_index + 1}/{len(self.active_waypoints)}: '
                f'({x:.2f}, {y:.2f}, {z:.2f})'
            )
            return

        if dist <= self.DISTANCE_THRESHOLD_M:
            self.get_logger().info(
                f'[SENDER] Reached waypoint {self.active_waypoint_index + 1}/{len(self.active_waypoints)} (dist={dist:.2f}m)'
            )
            self.active_waypoint_index += 1
            self.active_waypoint_sent = False
            return

def main(args=None):
    rclpy.init(args=args)
    sender = RoomWaypointSender()
    
    try:
        rclpy.spin(sender)
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
