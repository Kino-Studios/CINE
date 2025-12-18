#!/usr/bin/env python3
"""
Records B-spline control points from ego_planner for later replay.
Saves the exact mathematical trajectory for reproducible experiments.
"""

import rclpy
from rclpy.node import Node
from traj_utils.msg import Bspline
import json
import sys
from pathlib import Path


class BsplineRecorder(Node):
    def __init__(self, output_file='reference_trajectory.json'):
        super().__init__('bspline_recorder')
        self.output_file = output_file
        self.trajectory_received = False
        
        # Subscribe to baseline B-spline 
        # ego_planner publishes to planning/bspline which is remapped to /drone_0_planning/bspline
        self.subscription = self.create_subscription(
            Bspline,
            '/drone_0_planning/bspline',
            self.bspline_callback,
            10
        )
        
        self.get_logger().info('='*50)
        self.get_logger().info(f'B-spline Recorder Started')
        self.get_logger().info(f'Output file: {output_file}')
        self.get_logger().info(f'Subscribed to: /planning/bspline')
        self.get_logger().info('Waiting for first trajectory...')
        self.get_logger().info('='*50)
    
    def bspline_callback(self, msg):
        """Save the first B-spline trajectory received"""
        self.get_logger().info(f'!!! CALLBACK TRIGGERED !!!')
        if self.trajectory_received:
            return  # Only record the first trajectory
        
        self.get_logger().info('✓ B-spline trajectory received!')
        self.get_logger().info(f'  Order: {msg.order}')
        self.get_logger().info(f'  Control points: {len(msg.pos_pts)}')
        self.get_logger().info(f'  Knots: {len(msg.knots)}')
        
        # Extract control points
        pos_pts = []
        for pt in msg.pos_pts:
            pos_pts.append([pt.x, pt.y, pt.z])
        
        # Extract knots
        knots = list(msg.knots)
        
        # Create trajectory data structure
        trajectory_data = {
            'order': msg.order,
            'pos_pts': pos_pts,
            'knots': knots,
            'drone_id': msg.drone_id,
            'traj_id': msg.traj_id,
            'start_time': {
                'sec': msg.start_time.sec,
                'nanosec': msg.start_time.nanosec
            }
        }
        
        # Save to JSON file
        output_path = Path(self.output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info(f'Attempting to save to: {output_path.absolute()}')
        try:
            with open(output_path, 'w') as f:
                json.dump(trajectory_data, f, indent=2)
            self.get_logger().info(f'✓ File written successfully!')
        except Exception as e:
            self.get_logger().error(f'✗ Failed to write file: {e}')
        
        self.get_logger().info(f'✓ Trajectory recorded successfully!')
        self.get_logger().info(f'  Control points: {len(pos_pts)}')
        self.get_logger().info(f'  Knots: {len(knots)}')
        self.get_logger().info(f'  Order: {msg.order}')
        self.get_logger().info(f'  Saved to: {output_path.absolute()}')
        
        self.trajectory_received = True
        
        self.get_logger().info(f'Recording complete. Waiting for cleanup...')


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    
    # Get output file from command line or use default
    output_file = sys.argv[1] if len(sys.argv) > 1 else 'reference_trajectory.json'
    
    recorder = BsplineRecorder(output_file)
    recorder.get_logger().info(f'Recorder initialized, waiting for messages on /drone_0_planning/bspline...')
    
    try:
        # Wait up to 40 seconds for trajectory
        timeout_start = recorder.get_clock().now()
        spin_count = 0
        while rclpy.ok() and not recorder.trajectory_received:
            rclpy.spin_once(recorder, timeout_sec=0.1)
            spin_count += 1
            elapsed = (recorder.get_clock().now() - timeout_start).nanoseconds / 1e9
            if spin_count % 100 == 0:  # Log every 10 seconds
                recorder.get_logger().info(f'Spinning... {elapsed:.1f}s elapsed, trajectory_received={recorder.trajectory_received}')
            if elapsed > 40:
                recorder.get_logger().error('Timeout: No trajectory received after 40 seconds')
                break
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
