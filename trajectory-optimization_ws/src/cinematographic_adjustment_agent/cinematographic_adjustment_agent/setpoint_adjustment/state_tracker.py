import time
import numpy as np
from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from std_msgs.msg import Int32

class StateTracker(Node):
    def __init__(self, look_duration_s: float = 10.0, depth_window_size: int = 15):
        super().__init__('state_tracker')
        
        self.look_duration_s = look_duration_s
        self.depth_window_size = depth_window_size
        self.current_target_id = None
        self.last_target_switch_time_s = time.time()
        self.recent_depth_samples = deque(maxlen=depth_window_size)
        self.last_center_uv = None
        
        self.center_sub = self.create_subscription(
            PolygonStamped, '/tracker/center_point', self.center_callback, 10)
        self.depth_sub = self.create_subscription(
            PolygonStamped, '/tracker/depth', self.depth_callback, 10)
        
        self.target_id_pub = self.create_publisher(Int32, '/tracker/target_id', 10)
        
        self.timer = self.create_timer(1.0, self.check_target_switch)
        
        self.declare_parameter('look_duration_s', look_duration_s)
        self.declare_parameter('depth_window_size', depth_window_size)

    def center_callback(self, msg):
        if len(msg.polygon.points) > 0:
            self.set_target_observation((msg.polygon.points[0].x, msg.polygon.points[0].y))

    def depth_callback(self, msg):
        if len(msg.polygon.points) > 0:
            depth_value = msg.polygon.points[0].z
            self.record_depth(depth_value)

    def check_target_switch(self):
        if self.should_switch_target():
            self.rotate_to_next_target()

    def should_switch_target(self) -> bool:
        now_s = time.time()
        look_duration = self.get_parameter('look_duration_s').value
        return (now_s - self.last_target_switch_time_s) >= look_duration

    def set_target_observation(self, center_uv):
        self.last_center_uv = center_uv

    def record_depth(self, depth_value_m: float):
        if depth_value_m <= 0 or np.isnan(depth_value_m):
            return
        self.recent_depth_samples.append(float(depth_value_m))

    def get_smoothed_depth(self) -> float | None:
        if len(self.recent_depth_samples) == 0:
            return None
        return float(np.median(np.array(self.recent_depth_samples)))

    def rotate_to_next_target(self):
        self.last_target_switch_time_s = time.time()
        self.recent_depth_samples.clear()
        
        if self.current_target_id is None:
            self.current_target_id = 0
        else:
            self.current_target_id += 1
        
        msg = Int32()
        msg.data = self.current_target_id
        self.target_id_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    tracker = StateTracker()
    
    rclpy.spin(tracker)
    
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()