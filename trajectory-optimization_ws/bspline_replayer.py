#!/usr/bin/env python3
"""
Replays a recorded B-spline trajectory from JSON file.
Publishes the exact same control points for reproducible experiments.
"""

import rclpy
from rclpy.node import Node
from traj_utils.msg import Bspline
from geometry_msgs.msg import Point
import json
import sys
from pathlib import Path


class BsplineReplayer(Node):
    def __init__(self, trajectory_file='reference_trajectory.json', use_cinematic=False):
        super().__init__('bspline_replayer')
        self.trajectory_file = trajectory_file
        self.use_cinematic = use_cinematic
        self.trajectory_published = False
        
        # ALWAYS publish to the raw bspline topic
        # In baseline mode: traj_server reads from /drone_0_planning/bspline directly
        # In cinematic mode: filter reads from /drone_0_planning/bspline, outputs to /drone_0_planning/cinematic_bspline, then traj_server reads that
        topic_name = '/drone_0_planning/bspline'
        
        if use_cinematic:
            self.get_logger().info('Replay mode: CINEMATIC (trajectory will be filtered)')
        else:
            self.get_logger().info('Replay mode: BASELINE (no filter)')
        
        # Publisher
        self.publisher = self.create_publisher(Bspline, topic_name, 10)
        
        # Load trajectory data
        self.trajectory_data = self.load_trajectory()
        
        # Wait for traj_server to be ready, then publish
        self.timer = self.create_timer(2.0, self.publish_trajectory)
        
        self.get_logger().info(f'Loaded trajectory from: {trajectory_file}')
        self.get_logger().info(f'  Control points: {len(self.trajectory_data["pos_pts"])}')
        self.get_logger().info(f'  Order: {self.trajectory_data["order"]}')
        self.get_logger().info('Ready to replay trajectory...')
    
    def load_trajectory(self):
        """Load B-spline trajectory from JSON file"""
        trajectory_path = Path(self.trajectory_file)
        
        if not trajectory_path.exists():
            self.get_logger().error(f'Trajectory file not found: {trajectory_path.absolute()}')
            raise FileNotFoundError(f'Trajectory file not found: {trajectory_path}')
        
        with open(trajectory_path, 'r') as f:
            data = json.load(f)
        
        return data
    
    def publish_trajectory(self):
        """Publish the recorded B-spline trajectory"""
        if self.trajectory_published:
            return
        
        # Create message
        msg = Bspline()
        msg.order = self.trajectory_data['order']
        msg.drone_id = self.trajectory_data['drone_id']
        msg.traj_id = self.trajectory_data['traj_id']
        
        # Set start time to now
        msg.start_time = self.get_clock().now().to_msg()
        
        # Add control points
        for pt_list in self.trajectory_data['pos_pts']:
            pt = Point()
            pt.x = float(pt_list[0])
            pt.y = float(pt_list[1])
            pt.z = float(pt_list[2])
            msg.pos_pts.append(pt)
        
        # Add knots
        msg.knots = [float(k) for k in self.trajectory_data['knots']]
        
        # Publish
        self.publisher.publish(msg)
        
        self.get_logger().info('✓ Trajectory published successfully!')
        self.get_logger().info('  traj_server will now execute this exact trajectory')
        
        self.trajectory_published = True
        
        # Keep publishing for a few more times to ensure it's received
        self.publish_count = 0
        self.timer.cancel()
        self.timer = self.create_timer(0.5, self.republish_trajectory)
    
    def republish_trajectory(self):
        """Republish a few times to ensure reception"""
        self.publish_count += 1
        
        if self.publish_count < 5:
            # Recreate message with updated timestamp
            msg = Bspline()
            msg.order = self.trajectory_data['order']
            msg.drone_id = self.trajectory_data['drone_id']
            msg.traj_id = self.trajectory_data['traj_id']
            msg.start_time = self.get_clock().now().to_msg()
            
            for pt_list in self.trajectory_data['pos_pts']:
                pt = Point()
                pt.x = float(pt_list[0])
                pt.y = float(pt_list[1])
                pt.z = float(pt_list[2])
                msg.pos_pts.append(pt)
            
            msg.knots = [float(k) for k in self.trajectory_data['knots']]
            self.publisher.publish(msg)
        else:
            self.get_logger().info('Replay publishing complete. Keeping node alive...')
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    trajectory_file = 'reference_trajectory.json'
    use_cinematic = False
    
    if len(sys.argv) > 1:
        trajectory_file = sys.argv[1]
    if len(sys.argv) > 2 and sys.argv[2] == '--cinematic':
        use_cinematic = True
    
    replayer = BsplineReplayer(trajectory_file, use_cinematic)
    
    try:
        rclpy.spin(replayer)
    except KeyboardInterrupt:
        pass
    finally:
        replayer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
