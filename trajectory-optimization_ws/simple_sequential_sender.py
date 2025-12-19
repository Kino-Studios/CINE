import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import time

class SequentialWaypointSender(Node):
    def __init__(self):
        super().__init__('sequential_waypoint_sender')
        
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.odom_sub = self.create_subscription(Odometry, '/drone_0_visual_odom', self.odom_callback, 10)
        
        self.current_pos = None
        self.current_waypoint_idx = 0
        self.waypoints = self.generate_circular_waypoints()
        self.waypoint_sent = False
        self.start_time = None
        
        self.DISTANCE_THRESHOLD = 0.5
        
        self.timer = self.create_timer(0.5, self.check_and_send_next)
    
    def generate_circular_waypoints(self):
        center_x, center_y = 0.5, 0.5
        radius = 0.6
        z_height = 1.5
        num_points = 16
        
        start_angle = -np.pi/3
        end_angle = np.pi/3
        angles = np.linspace(start_angle, end_angle, num_points)
        
        waypoints = []
        for angle in angles:
            x = center_x + radius * np.cos(angle)
            y = center_y + radius * np.sin(angle)
            waypoints.append((x, y, z_height))
        
        return waypoints
    
    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
    
    def check_and_send_next(self):
        if self.current_pos is None:
            return
        
        if self.start_time is None:
            self.start_time = time.time()
        
        if self.current_waypoint_idx >= len(self.waypoints):
            elapsed = time.time() - self.start_time
            self.timer.cancel()

            return
        
        target = np.array(self.waypoints[self.current_waypoint_idx])
        distance = np.linalg.norm(self.current_pos - target)
        
        if not self.waypoint_sent:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'world'
            goal.pose.position.x = target[0]
            goal.pose.position.y = target[1]
            goal.pose.position.z = target[2]
            goal.pose.orientation.w = 1.0
            
            self.goal_pub.publish(goal)
            self.waypoint_sent = True
        
        elif distance < self.DISTANCE_THRESHOLD:
            self.current_waypoint_idx += 1
            self.waypoint_sent = False
        else:
            self.get_logger().debug(f'[PROGRESS] Distance to waypoint {self.current_waypoint_idx+1}: {distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    sender = SequentialWaypointSender()
    
    try:
        rclpy.spin(sender)
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
