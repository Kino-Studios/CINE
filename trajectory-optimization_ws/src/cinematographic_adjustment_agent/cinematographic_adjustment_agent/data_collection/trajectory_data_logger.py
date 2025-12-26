#!/usr/bin/env python3

import csv
import os
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
import atexit


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
        self.odom_data = []
        self.current_baseline_bspline = None
        self.current_constrained_bspline = None
        self.current_odom = None
        self.time0 = None
        self.origin_position = None
        self.planned_time0_s = None
        self.last_logged_t = None
        self.sample_period_s = 0.02
        
        self.mode = mode
        self.run_number = run_number
        
        # Output directory: allow override via TRAJECTORY_DATA_DIR, otherwise use current CWD/trajectory_data
        default_dir = Path.cwd() / "trajectory_data"
        output_dir = Path(os.environ.get("TRAJECTORY_DATA_DIR", default_dir))
        output_dir.mkdir(exist_ok=True, parents=True)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = output_dir

        # Precompute filenames so autosaves are consistent
        if self.mode == 'baseline':
            base_name = (
                f'baseline_replay_run{self.run_number}_{self.timestamp}.csv'
                if self.run_number is not None
                else f'baseline_sim_{self.timestamp}.csv'
            )
            odom_prefix = 'baseline_odom'
        elif self.mode == 'constrained':
            base_name = f'baseline_in_constrained_sim_{self.timestamp}.csv'
            odom_prefix = 'cinematic_odom'
        else:
            base_name = f'baseline_sim_{self.timestamp}.csv'
            odom_prefix = 'baseline_odom'

        cinematic_name = (
            f'cinematic_replay_run{self.run_number}_{self.timestamp}.csv'
            if self.run_number is not None
            else f'cinematic_sim_{self.timestamp}.csv'
        )

        self.baseline_path = self.output_dir / base_name
        self.cinematic_path = self.output_dir / cinematic_name
        self.odom_path = self.output_dir / f'{odom_prefix}_sim_{self.timestamp}.csv'

        self.declare_parameter("baseline_bspline_topic", "/drone_0_planning/bspline")
        self.declare_parameter("cinematic_bspline_topic", "/drone_0_planning/cinematic_bspline")
        # Default odom topic matches ego-planner single_run_in_sim launch (visual_slam/odom)
        self.declare_parameter("odom_topic", "/visual_slam/odom")
        baseline_topic = str(self.get_parameter("baseline_bspline_topic").value)
        cinematic_topic = str(self.get_parameter("cinematic_bspline_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)

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

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            50
        )

        self._have_odom = False
        self._have_baseline = False
        self._warned_no_odom = False
        self._warned_no_baseline = False

        # Autosave periodically so data is flushed even if interrupted
        self.save_timer = self.create_timer(2.0, self._autosave)
        self.health_timer = self.create_timer(5.0, self._health_check)
        atexit.register(self._autosave)
        self.get_logger().info(
            f"Trajectory data logger initialized | Output dir: {self.output_dir} | "
            f"Baseline topic: {baseline_topic} | Cinematic topic: {cinematic_topic} | Odom topic: {odom_topic}"
        )
        
        self.get_logger().info('Trajectory data logger initialized')
    
    def baseline_bspline_callback(self, msg):
        self.current_baseline_bspline = msg
        self._have_baseline = True
        if not getattr(self, "_logged_baseline_info", False):
            self.get_logger().info(
                f"[LOGGER] Received first B-spline! Started logging trajectory data. "
                f"B-spline has {len(msg.pos_pts)} control points"
            )
            self._logged_baseline_info = True
    
    def constrained_bspline_callback(self, msg):
        self.current_constrained_bspline = msg

    def odom_callback(self, msg: Odometry):
        self._have_odom = True
        self.current_odom = msg
        if not getattr(self, "_logged_odom_info", False):
            pos = msg.pose.pose.position
            self.get_logger().info(
                f"Origin set to: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
            )
            self._logged_odom_info = True
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        if self.time0 is None:
            self.time0 = stamp
            pos0 = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                dtype=float,
            )
            self.origin_position = pos0.copy()
            self.get_logger().info(
                f'[LOGGER] Received first odometry; origin set to ({pos0[0]:.2f}, {pos0[1]:.2f}, {pos0[2]:.2f})'
            )

        t = (stamp - self.time0).nanoseconds / 1e9
        if self.last_logged_t is not None and (t - self.last_logged_t) < (self.sample_period_s * 0.8):
            return
        self.last_logged_t = t

        pos = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=float,
        )
        vel = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
            dtype=float,
        )

        origin = self.origin_position if self.origin_position is not None else pos
        rel_pos = pos - origin
        speed = float(np.linalg.norm(vel))

        self.odom_data.append(
            {
                'time': float(t),
                'odom_x': float(rel_pos[0]),
                'odom_y': float(rel_pos[1]),
                'odom_z': float(rel_pos[2]),
                'odom_vx': float(vel[0]),
                'odom_vy': float(vel[1]),
                'odom_vz': float(vel[2]),
                'odom_speed': speed,
            }
        )

        if self.mode != 'constrained':
            self._log_planned_baseline(stamp, t)
        else:
            self._log_planned_baseline(stamp, t)
            self._log_planned_constrained(stamp, t)

        if len(self.odom_data) % 500 == 0:
            self.get_logger().info(f'Logged {len(self.odom_data)} odom samples')

    def _log_planned_baseline(self, stamp: rclpy.time.Time, t: float):
        if self.current_baseline_bspline is None:
            return

        position, velocity = self._evaluate_bspline(self.current_baseline_bspline, stamp)
        if position is None or velocity is None:
            return

        if self.planned_time0_s is None:
            self.planned_time0_s = float(t)
        t_rel = float(t) - float(self.planned_time0_s)

        origin = self.origin_position if self.origin_position is not None else position
        rel_pos = position - origin
        speed = float(np.linalg.norm(velocity))

        self.baseline_data.append(
            {
                'time': float(t_rel),
                'traj_id': int(self.current_baseline_bspline.traj_id),
                'planned_x': float(rel_pos[0]),
                'planned_y': float(rel_pos[1]),
                'planned_z': float(rel_pos[2]),
                'planned_vx': float(velocity[0]),
                'planned_vy': float(velocity[1]),
                'planned_vz': float(velocity[2]),
                'planned_speed': speed,
            }
        )

    def _log_planned_constrained(self, stamp: rclpy.time.Time, t: float):
        if self.current_constrained_bspline is None:
            return

        position, velocity = self._evaluate_bspline(self.current_constrained_bspline, stamp)
        if position is None or velocity is None:
            return

        if self.planned_time0_s is None:
            self.planned_time0_s = float(t)
        t_rel = float(t) - float(self.planned_time0_s)

        origin = self.origin_position if self.origin_position is not None else position
        rel_pos = position - origin
        speed = float(np.linalg.norm(velocity))

        self.constrained_data.append(
            {
                'time': float(t_rel),
                'traj_id': int(self.current_constrained_bspline.traj_id),
                'planned_x': float(rel_pos[0]),
                'planned_y': float(rel_pos[1]),
                'planned_z': float(rel_pos[2]),
                'planned_vx': float(velocity[0]),
                'planned_vy': float(velocity[1]),
                'planned_vz': float(velocity[2]),
                'planned_speed': speed,
            }
        )
    
    def _evaluate_bspline(self, bspline_msg, current_ros_time):
        try:
            order = bspline_msg.order
            knots = np.array(bspline_msg.knots, dtype=float)
            control_points = np.array([[p.x, p.y, p.z] for p in bspline_msg.pos_pts], dtype=float)
            
            if len(control_points) == 0:
                return None, None
            
            bspline_start = rclpy.time.Time.from_msg(bspline_msg.start_time)
            time_offset = (current_ros_time - bspline_start).nanoseconds / 1e9
            
            pos_bspline = UniformBspline(control_points, order, knots)
            
            time_sum = pos_bspline.getTimeSum()
            if time_offset < 0 or time_offset > time_sum:
                return None, None
            
            position = pos_bspline.evaluateDeBoorT(time_offset)
            
            vel_bspline = pos_bspline.getDerivative()
            velocity = vel_bspline.evaluateDeBoorT(time_offset)
            
            return position, velocity
            
        except Exception as e:
            self.get_logger().error(f'Error evaluating B-spline: {e}')
            return None, None
    
    def save_data(self, log_empty: bool = True):
        if len(self.odom_data) == 0 and len(self.baseline_data) == 0 and len(self.constrained_data) == 0:
            if log_empty:
                self.get_logger().warn('No data to save')
            return

        if len(self.baseline_data) > 0:
            self._write_csv(self.baseline_path, self.baseline_data)
            self.get_logger().info(f'Saved {len(self.baseline_data)} planned baseline points to: {self.baseline_path}')

        if len(self.constrained_data) > 0:
            self._write_csv(self.cinematic_path, self.constrained_data)
            self.get_logger().info(f'Saved {len(self.constrained_data)} planned cinematic points to: {self.cinematic_path}')

        if len(self.odom_data) > 0:
            self._write_csv(self.odom_path, self.odom_data)
            self.get_logger().info(f'Saved {len(self.odom_data)} odometry points to: {self.odom_path}')

    def _autosave(self):
        # Quiet autosave to ensure CSVs land even if interrupted
        self.save_data(log_empty=False)

    def _write_csv(self, path: Path, rows: list[dict]):
        if not rows:
            return
        fieldnames = list(rows[0].keys())
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)

    def _health_check(self):
        if not self._have_odom and not self._warned_no_odom:
            self.get_logger().warn(
                f"No odometry received yet on topic '{self.get_parameter('odom_topic').value}'"
            )
            self._warned_no_odom = True
        if not self._have_baseline and not self._warned_no_baseline:
            self.get_logger().warn(
                f"No baseline B-spline received yet on topic '{self.get_parameter('baseline_bspline_topic').value}'"
            )
            self._warned_no_baseline = True


def main(args=None):
    import argparse
    
    parser = argparse.ArgumentParser(description='Log trajectory data')
    parser.add_argument('--mode', type=str, default='sim', 
                       choices=['sim', 'baseline', 'constrained'],
                       help='Logging mode: sim/baseline (planned baseline), or constrained (planned baseline + cinematic)')
    parser.add_argument('--run-number', type=int, default=None,
                       help='Run number for replay mode')
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    mode = parsed_args.mode
    if mode == 'sim':
        mode = 'baseline'
    logger = TrajectoryDataLogger(mode=mode, run_number=parsed_args.run_number)

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
    except Exception as e:
        logger.get_logger().error(f'Unhandled error: {e}')
        logger.save_data()
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
