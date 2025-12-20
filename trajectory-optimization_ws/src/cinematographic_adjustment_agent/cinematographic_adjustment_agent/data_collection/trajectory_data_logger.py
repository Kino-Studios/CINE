#!/usr/bin/env python3

import csv
import rclpy
import rclpy.time
from rclpy.node import Node
from traj_utils.msg import Bspline
from nav_msgs.msg import Odometry
import numpy as np
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass
import signal
import sys


class UniformBspline:
    """Python implementation of ego_planner's UniformBspline using de Boor's algorithm"""
    
    def __init__(self, control_points, order, knots):
        """
        Args:
            control_points: np.array of shape (N, 3) - control points
            order: int - B-spline order (degree + 1)
            knots: np.array - knot vector
        """
        self.control_points = control_points  # (N, 3)
        self.p = order  # order
        self.u = knots  # knot vector
        self.n = len(control_points) - 1
        self.m = self.n + self.p + 1
    
    def evaluateDeBoor(self, u):
        """
        Evaluate B-spline at parameter u using de Boor's algorithm.
        Direct translation of ego_planner's UniformBspline::evaluateDeBoor()
        """
        # Clamp u to valid range
        ub = np.clip(u, self.u[self.p], self.u[self.m - self.p])
        
        # Determine which knot span [u_k, u_{k+1}] contains ub
        k = self.p
        while k < len(self.u) - 1:
            if self.u[k + 1] >= ub:
                break
            k += 1
        
        # Initialize de Boor points with relevant control points
        d = []
        for i in range(self.p + 1):
            idx = k - self.p + i
            if 0 <= idx < len(self.control_points):
                d.append(self.control_points[idx].copy())
            else:
                d.append(np.zeros(3))
        
        # de Boor's recursion
        for r in range(1, self.p + 1):
            for i in range(self.p, r - 1, -1):
                knot_idx_1 = i + k - self.p
                knot_idx_2 = i + 1 + k - r
                
                if knot_idx_1 < len(self.u) and knot_idx_2 < len(self.u):
                    denom = self.u[knot_idx_2] - self.u[knot_idx_1]
                    if abs(denom) > 1e-10:
                        alpha = (ub - self.u[knot_idx_1]) / denom
                        d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i]
        
        return d[self.p]
    
    def evaluateDeBoorT(self, t):
        """
        Evaluate B-spline at time t.
        Direct translation of ego_planner's evaluateDeBoorT(t) = evaluateDeBoor(t + u[p])
        """
        return self.evaluateDeBoor(t + self.u[self.p])
    
    def getTimeSum(self):
        """Get total duration of the B-spline trajectory"""
        return self.u[self.m - self.p] - self.u[self.p]
    
    def getDerivative(self):
        """
        Get derivative B-spline (velocity from position B-spline).
        Translation of ego_planner's UniformBspline::getDerivative()
        """
        # Compute derivative control points
        n = len(self.control_points)
        derivative_control_points = np.zeros((n - 1, 3))
        
        for i in range(n - 1):
            denom = self.u[i + self.p + 1] - self.u[i + 1]
            if abs(denom) > 1e-10:
                derivative_control_points[i] = self.p * (self.control_points[i + 1] - self.control_points[i]) / denom
            else:
                derivative_control_points[i] = np.zeros(3)
        
        # Derivative B-spline has order p-1 and uses inner knots
        derivative_knots = self.u[1:-1]  # Remove first and last knot
        
        return UniformBspline(derivative_control_points, self.p - 1, derivative_knots)


@dataclass
class CinematicConstraints:
    FOCUS_MAX_VELOCITY_MPS: float = 1.0
    FOCUS_MAX_ACCEL_MPS2: float = 1.5
    DEFAULT_MAX_VELOCITY_MPS: float = 2.0
    DEFAULT_MAX_ACCEL_MPS2: float = 3.0
    MIN_DT: float = 0.05


class TrajectoryDataLogger(Node):
    
    def __init__(self, mode='sim', run_number=None):
        super().__init__('trajectory_data_logger')
        
        self.constraints = CinematicConstraints()
        self.baseline_data = []
        self.constrained_data = []
        self.current_baseline_bspline = None
        self.current_constrained_bspline = None
        self.logging_active = False
        self.global_start_time = None  # Global time when logging started
        self.start_position = None  # Store first position as origin for relative coordinates
        
        # Support different modes: 'sim' for normal runs, 'baseline' and 'constrained' for replay runs
        self.mode = mode
        self.run_number = run_number
        
        output_dir = Path('./trajectory_data')
        output_dir.mkdir(exist_ok=True, parents=True)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = output_dir
        
        # Subscribe to B-spline messages to evaluate planned trajectories
        self.declare_parameter("baseline_bspline_topic", "/drone_0_planning/bspline")
        self.declare_parameter("cinematic_bspline_topic", "/drone_0_planning/cinematic_bspline")
        baseline_topic = str(self.get_parameter("baseline_bspline_topic").value)
        cinematic_topic = str(self.get_parameter("cinematic_bspline_topic").value)

        self.baseline_bspline_sub = self.create_subscription(
            Bspline,
            baseline_topic,
            self.baseline_bspline_callback,
            10
        )
        
        self.constrained_bspline_sub = self.create_subscription(
            Bspline,
            cinematic_topic,
            self.constrained_bspline_callback,
            10
        )
        
        # Sample at 50 Hz to get very dense trajectory data
        self.timer = self.create_timer(0.02, self.log_data)
        
        self.get_logger().info('Trajectory data logger initialized')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info('Sampling rate: 50 Hz (every 0.02s)')
    
    def baseline_bspline_callback(self, msg):
        """Store latest baseline B-spline and start logging"""
        self.current_baseline_bspline = msg
        if not self.logging_active:
            self.logging_active = True
            self.global_start_time = self.get_clock().now()
            self.get_logger().info(f'[LOGGER] Received first B-spline! Started logging trajectory data. '
                                 f'B-spline has {len(msg.pos_pts)} control points')
        else:
            # Log periodic updates
            if len(self.baseline_data) % 100 == 0:
                self.get_logger().debug(f'[LOGGER] Received B-spline #{len(self.baseline_data)//100}')
    
    def constrained_bspline_callback(self, msg):
        """Store latest constrained B-spline"""
        self.current_constrained_bspline = msg
    
    def log_data(self):
        """Evaluate B-spline trajectories at 20 Hz to get planned position/velocity"""
        if not self.logging_active:
            return
        
        current_time = self.get_clock().now()
        global_elapsed = (current_time - self.global_start_time).nanoseconds / 1e9
        
        # Evaluate baseline B-spline at current time
        if self.current_baseline_bspline:
            pos, vel = self._evaluate_bspline(self.current_baseline_bspline, current_time)
            if pos is not None:
                # Store first position as origin
                if self.start_position is None:
                    self.start_position = pos.copy()
                    self.get_logger().info(f'Origin set to: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')
                
                # Store positions relative to start (so all plots start at 0,0,0)
                rel_pos = pos - self.start_position
                speed = np.linalg.norm(vel)
                baseline_entry = {
                    'time': global_elapsed,
                    'planned_x': rel_pos[0],
                    'planned_y': rel_pos[1],
                    'planned_z': rel_pos[2],
                    'planned_vx': vel[0],
                    'planned_vy': vel[1],
                    'planned_vz': vel[2],
                    'planned_speed': speed,
                }
                self.baseline_data.append(baseline_entry)

        if self.current_constrained_bspline:
            pos, vel = self._evaluate_bspline(self.current_constrained_bspline, current_time)
            if pos is not None and self.start_position is not None:
                rel_pos = pos - self.start_position
                speed = np.linalg.norm(vel)
                constrained_entry = {
                    'time': global_elapsed,
                    'planned_x': rel_pos[0],
                    'planned_y': rel_pos[1],
                    'planned_z': rel_pos[2],
                    'planned_vx': vel[0],
                    'planned_vy': vel[1],
                    'planned_vz': vel[2],
                    'planned_speed': speed,
                }
                self.constrained_data.append(constrained_entry)
        
        if len(self.baseline_data) % 500 == 0:
            constrained_info = f'Constrained: {len(self.constrained_data)} samples' if self.constrained_data else 'No constrained data'
            self.get_logger().info(
                f'Logged {len(self.baseline_data)} baseline samples | {constrained_info}'
            )
    
    def _evaluate_bspline(self, bspline_msg, current_ros_time):
        """
        Evaluate B-spline position and velocity at current ROS time.
        Uses the B-spline's start_time to compute time offset.
        Returns (position, velocity) or (None, None) if out of range.
        """
        try:
            order = bspline_msg.order
            knots = np.array(bspline_msg.knots, dtype=float)
            control_points = np.array([[p.x, p.y, p.z] for p in bspline_msg.pos_pts], dtype=float)
            
            if len(control_points) == 0:
                return None, None
            
            # Compute time since this B-spline's start_time
            bspline_start = rclpy.time.Time.from_msg(bspline_msg.start_time)
            time_offset = (current_ros_time - bspline_start).nanoseconds / 1e9
            
            # Create position B-spline
            pos_bspline = UniformBspline(control_points, order, knots)
            
            # Get valid time range
            time_sum = pos_bspline.getTimeSum()
            if time_offset < 0 or time_offset > time_sum:
                # Outside valid range - trajectory not active yet or expired
                return None, None
            
            # Evaluate position
            position = pos_bspline.evaluateDeBoorT(time_offset)
            
            # Get derivative B-spline for velocity
            vel_bspline = pos_bspline.getDerivative()
            velocity = vel_bspline.evaluateDeBoorT(time_offset)
            
            return position, velocity
            
        except Exception as e:
            self.get_logger().error(f'Error evaluating B-spline: {e}')
            return None, None
    
    def save_data(self):
        if len(self.baseline_data) == 0:
            self.get_logger().warn('No data to save')
            return
        
        # Determine filename based on mode
        if self.mode == 'baseline' and self.run_number is not None:
            filename = f'baseline_replay_run{self.run_number}_{self.timestamp}.csv'
        elif self.mode == 'constrained' and self.run_number is not None:
            filename = f'constrained_replay_run{self.run_number}_{self.timestamp}.csv'
        elif self.mode == 'constrained':
            # Constrained mode without run number (direct data collection)
            filename = f'cinematic_sim_{self.timestamp}.csv'
        else:
            # Default to baseline_sim for 'sim' mode or backward compatibility
            filename = f'baseline_sim_{self.timestamp}.csv'
        
        baseline_path = self.output_dir / filename
        self._write_csv(baseline_path, self.baseline_data)
        self.get_logger().info(f'Saved {len(self.baseline_data)} baseline data points to: {baseline_path}')
        
        # Only save constrained data if it was actually collected
        if len(self.constrained_data) > 0:
            constrained_path = self.output_dir / f'constrained_sim_{self.timestamp}.csv'
            self._write_csv(constrained_path, self.constrained_data)
            self.get_logger().info(f'Saved {len(self.constrained_data)} constrained data points to: {constrained_path}')
            
            # Print comparison statistics
            baseline_speeds = np.array([float(r["planned_speed"]) for r in self.baseline_data], dtype=float)
            constrained_speeds = np.array([float(r["planned_speed"]) for r in self.constrained_data], dtype=float)
            self.get_logger().info(
                f'Baseline - Mean speed: {float(np.mean(baseline_speeds)):.3f} m/s, '
                f'Max: {float(np.max(baseline_speeds)):.3f} m/s'
            )
            self.get_logger().info(
                f'Constrained - Mean speed: {float(np.mean(constrained_speeds)):.3f} m/s, '
                f'Max: {float(np.max(constrained_speeds)):.3f} m/s'
            )
            self.get_logger().info(
                f'Speed reduction: {float(np.mean(baseline_speeds) - np.mean(constrained_speeds)):.3f} m/s'
            )
        else:
            # Only baseline data collected
            baseline_speeds = np.array([float(r["planned_speed"]) for r in self.baseline_data], dtype=float)
            self.get_logger().info(
                f'Baseline - Mean speed: {float(np.mean(baseline_speeds)):.3f} m/s, '
                f'Max: {float(np.max(baseline_speeds)):.3f} m/s'
            )
            self.get_logger().info('No constrained data collected (cinematic filter not running)')

    def _write_csv(self, path: Path, rows: list[dict]):
        if not rows:
            return
        fieldnames = list(rows[0].keys())
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)



def main(args=None):
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Log trajectory data')
    parser.add_argument('--mode', type=str, default='sim', 
                       choices=['sim', 'baseline', 'constrained'],
                       help='Logging mode: sim (normal), baseline (replay), or constrained (replay)')
    parser.add_argument('--run-number', type=int, default=None,
                       help='Run number for replay mode')
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    logger = TrajectoryDataLogger(mode=parsed_args.mode, run_number=parsed_args.run_number)
    
    # Set up signal handler for graceful shutdown
    def signal_handler(sig, frame):
        logger.get_logger().info('Received shutdown signal...')
        logger.save_data()
        logger.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        logger.get_logger().info('Shutting down...')
        logger.save_data()
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
