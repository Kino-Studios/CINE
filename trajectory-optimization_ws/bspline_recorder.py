import json
import sys
import os
from pathlib import Path
import rclpy
from rclpy.node import Node
from traj_utils.msg import Bspline

class BsplineRecorder(Node):
    def __init__(self, output_file='reference_trajectory.json', mode: str = 'baseline'):
        super().__init__('bspline_recorder')
        self.output_file = output_file
        self.trajectory_received = False

        mode = mode.lower()
        if mode not in ('baseline', 'cinematic'):
            self.get_logger().warn(f"Unknown mode '{mode}', defaulting to baseline")
            mode = 'baseline'

        topic = '/drone_0_planning/cinematic_bspline' if mode == 'cinematic' else '/drone_0_planning/bspline'

        self.subscription = self.create_subscription(
            Bspline,
            topic,
            self.bspline_callback,
            10
        )
        
    def bspline_callback(self, msg):
        if self.trajectory_received:
            return
        
        pos_pts = []
        for pt in msg.pos_pts:
            pos_pts.append([pt.x, pt.y, pt.z])
        
        knots = list(msg.knots)
        
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
        
        output_path = Path(self.output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        try:
            with open(output_path, 'w') as f:
                json.dump(trajectory_data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"failed to create json file: {e}")
        
        self.trajectory_received = True

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    
    output_file = sys.argv[1] if len(sys.argv) > 1 else 'reference_trajectory.json'
    mode = os.getenv("BSPLINE_RECORDER_MODE", "baseline")
    if len(sys.argv) > 2:
        mode = sys.argv[2]

    recorder = BsplineRecorder(output_file, mode)
    
    try:
        timeout_start = recorder.get_clock().now()
        spin_count = 0

        while rclpy.ok() and not recorder.trajectory_received:
            rclpy.spin_once(recorder, timeout_sec=0.1)
            spin_count += 1
            elapsed = (recorder.get_clock().now() - timeout_start).nanoseconds / 1e9

            if spin_count % 100 == 0:
                recorder.get_logger().info(f'spinning: trajectory_received={recorder.trajectory_received}')
            if elapsed > 40:
                recorder.get_logger().error('timeout error, no trajectory saved')
                break
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
