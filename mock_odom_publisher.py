#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class MockOdomPublisher(Node):
    def __init__(self):
        super().__init__('mock_odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/imu_propagate', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time_step = 0

    def timer_callback(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'
        
        # Simulate drone at origin with zero velocity
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 1.0  # 1 meter above ground
        
        # Identity quaternion (no rotation)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Zero velocity
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        
        self.publisher_.publish(msg)
        self.time_step += 1

def main(args=None):
    rclpy.init(args=args)
    node = MockOdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
