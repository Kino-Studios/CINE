import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/move_base_simple/goal',
            10
        )
        
        self.waypoints = [
            (5.0, 0.0, 1.5),
            (5.0, 5.0, 1.5),
            (0.0, 5.0, 1.5),
            (-5.0, 5.0, 1.5),
            (-5.0, 0.0, 1.5),
            (0.0, 0.0, 1.5),
        ]
        
        self.current_waypoint = 0
        
        time.sleep(2)
        
        self.send_next_waypoint()
        self.timer = self.create_timer(3.0, self.timer_callback)
        self.get_logger().info('Waypoint publisher started, sending waypoints every 3 seconds')
        
    def send_next_waypoint(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('All waypoints completed!')
            return
        
        waypoint = self.waypoints[self.current_waypoint]
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        msg.pose.position.x = waypoint[0]
        msg.pose.position.y = waypoint[1]
        msg.pose.position.z = waypoint[2]
        
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(msg)
        
        self.get_logger().info(f'Sent waypoint {self.current_waypoint + 1}/{len(self.waypoints)}: '
                              f'x={waypoint[0]}, y={waypoint[1]}, z={waypoint[2]}')
        
        self.current_waypoint += 1
    
    def timer_callback(self):
        if self.current_waypoint < len(self.waypoints):
            self.send_next_waypoint()

def main():
    rclpy.init()
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()