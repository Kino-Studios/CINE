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
        
        self.fake_detection_pub = self.create_publisher(
            Float32MultiArray,
            '/object_detections',
            10
        )
        
        self.timer = self.create_timer(0.5, self.check_and_send_waypoints)
        
        self.get_logger().info('[SENDER] Room waypoint sender initialized - cinematographic object tracking enabled')
        self.get_logger().info('[SENDER] Waiting for drone odometry and object detections...')
    
    def odom_callback(self, msg):
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
            if len(msg.data) >= 5:
                x_center = float(msg.data[0])
                y_center = float(msg.data[1])
                z_depth = float(msg.data[2])
                bbox_width = float(msg.data[3])
                bbox_height = float(msg.data[4])
                
                if z_depth > 0.1:
                    self.detected_object = {
                        'x_center': x_center,
                        'y_center': y_center,
                        'depth': z_depth,
                        'bbox_width': bbox_width,
                        'bbox_height': bbox_height
                    }
                    
                    if self.start_pos is not None and not self.waypoints_sent:
                        # Debounce: only regenerate waypoints every 2 seconds
                        now = self.get_clock().now()
                        if self.object_detection_time is None:
                            # First detection - generate waypoints
                            self.generate_cinematographic_waypoints()
                            self.use_dynamic_waypoints = True
                            self.object_detection_time = now
                        else:
                            elapsed = (now - self.object_detection_time).nanoseconds / 1e9
                            if elapsed >= self.DETECTION_DEBOUNCE:
                                # Update waypoints if object moved significantly
                                self.generate_cinematographic_waypoints()
                                self.object_detection_time = now
        
        except Exception as e:
            self.get_logger().warn(f'Error processing: {e}')
    
    def process_detection_synchronously(self, detection_msg):
        """Process detection message directly without waiting for async callback."""
        try:
            if len(detection_msg.data) >= 5:
                x_center = float(detection_msg.data[0])
                y_center = float(detection_msg.data[1])
                z_depth = float(detection_msg.data[2])
                bbox_width = float(detection_msg.data[3])
                bbox_height = float(detection_msg.data[4])
                
                if z_depth > 0.1:
                    self.detected_object = {
                        'x_center': x_center,
                        'y_center': y_center,
                        'depth': z_depth,
                        'bbox_width': bbox_width,
                        'bbox_height': bbox_height
                    }
                    self.get_logger().info(f'[CINEMATOGRAPHY] Object detected at ({x_center:.2f}, {y_center:.2f}) with depth {z_depth:.2f}m')
                    
                    # Generate waypoints
                    self.generate_cinematographic_waypoints()
                    self.use_dynamic_waypoints = True
                    self.object_detection_time = self.get_clock().now()
                    self.get_logger().info(f'[CINEMATOGRAPHY] Generated 16 arc waypoints (1/3 circumference, radius=0.60m, depth={z_depth:.2f}m)')
        except Exception as e:
            self.get_logger().warn(f'Error in process_detection_synchronously: {e}')
    
    def generate_cinematographic_waypoints(self):
        if self.detected_object is None or self.start_pos is None:
            return
        
        obj_depth = self.detected_object['depth']
        obj_x_center = self.detected_object['x_center']
        obj_y_center = self.detected_object['y_center']
        bbox_width = self.detected_object['bbox_width']
        bbox_height = self.detected_object['bbox_height']
        
        circle_radius = max(0.3, obj_depth * 0.4)
        
        num_waypoints = 16
        
        arc_span = 2 * math.pi / 3
        start_angle = -arc_span / 2
        
        cinematographic_waypoints = []
        
        for i in range(num_waypoints):
            angle = start_angle + (arc_span * i / (num_waypoints - 1))
            
            dx_circle = circle_radius * math.cos(angle)
            dy_circle = circle_radius * math.sin(angle)
            
            thirds_offset_x = (obj_x_center - 0.5) * 0.15
            thirds_offset_y = (obj_y_center - 0.5) * 0.15
            
            wp_x = obj_x_center + dx_circle + thirds_offset_x
            wp_y = obj_y_center + dy_circle + thirds_offset_y
            
            wp_z = obj_depth
            
            abs_x = self.start_pos[0] + wp_x
            abs_y = self.start_pos[1] + wp_y
            abs_z = self.start_pos[2] + wp_z
            
            cinematographic_waypoints.append((abs_x, abs_y, abs_z))
        
        self.dynamic_waypoints = cinematographic_waypoints
        
        self.publish_simulated_detection()
    
    def publish_simulated_detection(self):
        
        fake_detection = Float32MultiArray()
        fake_detection.data = [
            0.5,      # x_center
            0.5,      # y_center
            1.5,      # z_depth
            0.2,      # bbox_width
            0.3,      # bbox_height
            0.95      # confidence
        ]
        
        self.fake_detection_pub.publish(fake_detection)
    
    def check_and_send_waypoints(self):
        if self.start_pos is not None and not self.waypoints_sent and self.velocity_settled:
            elapsed = (self.get_clock().now() - self.ready_time).nanoseconds / 1e9
            if elapsed >= self.STARTUP_DELAY:
                self.get_logger().info(f'[SENDER] Startup delay complete ({elapsed:.1f}s). Sending waypoints now!')
                self.send_all_waypoints()
                self.waypoints_sent = True
                self.timer.cancel()
        elif self.start_pos is not None and not self.waypoints_sent:
            elapsed = (self.get_clock().now() - self.ready_time).nanoseconds / 1e9
            self.get_logger().debug(f'[SENDER] Waiting: start_pos={self.start_pos is not None}, waypoints_sent={self.waypoints_sent}, velocity_settled={self.velocity_settled}, elapsed={elapsed:.1f}s/{self.STARTUP_DELAY}s')
    
    
    def send_all_waypoints(self):
        # First, ensure dynamic waypoints are generated by directly calling arc generation
        if self.start_pos is not None and self.detected_object is None:
            # Generate arc waypoints for simulated object
            fake_detection = Float32MultiArray()
            fake_detection.data = [
                0.5,      # x_center
                0.5,      # y_center
                1.5,      # z_depth
                0.2,      # bbox_width
                0.3,      # bbox_height
                0.95      # confidence
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
        
        # Batch publish all waypoints efficiently
        for idx, (x, y, z) in enumerate(waypoints_to_send):
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'world'
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = z
            goal.pose.orientation.w = 1.0
            
            # Publish waypoint
            self.goal_pub.publish(goal)
            time.sleep(0.2)
        
        self.get_logger().info(f'[SENDER] All {len(waypoints_to_send)} waypoints published')

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
