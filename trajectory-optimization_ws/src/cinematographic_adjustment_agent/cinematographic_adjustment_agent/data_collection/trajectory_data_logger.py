import atexit
import csv
import os
import rclpy
import rclpy.time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from traj_utils.msg import Bspline
from nav_msgs.msg import Odometry
import numpy as np
from pathlib import Path
from datetime import datetime
import signal
import sys

# Add repo root to sys.path to find 'configs'
here = Path(__file__).resolve()
for parent in [here] + list(here.parents):
    if (parent / "configs").is_dir():
        python_constants_path = parent / "configs" / "python-constants"
        if str(python_constants_path) not in sys.path:
            sys.path.insert(0, str(python_constants_path))
        break

from constants import CinematicConstraints, SimulationParams


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
        n = len(self.control_points)
        derivative_control_points = np.zeros((n - 1, 3))

        for i in range(n - 1):
            denom = self.u[i + self.p + 1] - self.u[i + 1]
            if abs(denom) > 1e-10:
                derivative_control_points[i] = (
                    self.p * (self.control_points[i + 1] - self.control_points[i]) / denom
                )
            else:
                derivative_control_points[i] = np.zeros(3)

        # Derivative B-spline has order p-1 and uses inner knots
        derivative_knots = self.u[1:-1]  # Remove first and last knot

        return UniformBspline(derivative_control_points, self.p - 1, derivative_knots)


class TrajectoryDataLogger(Node):

    def __init__(self, mode='sim', run_number=None):
        super().__init__('trajectory_data_logger')

        self.constraints = CinematicConstraints()
        self.baseline_data = []
        self.constrained_data = []
        self.current_baseline_bspline = None
        self.current_constrained_bspline = None
        self.current_odom = None
        self.logging_active = False
        self.global_start_time = None  # Global time when logging started
        self.start_position = None  # Store first position as origin for relative coordinates

        self.mode = mode
        self.run_number = run_number

        workspace_root = next(
            (p for p in Path(__file__).resolve().parents if p.name == "trajectory-optimization_ws"),
            Path.cwd(),
        )
        default_dir = workspace_root / SimulationParams.TRAJECTORY_DATA_DIR
        output_dir = Path(default_dir)
        output_dir.mkdir(exist_ok=True, parents=True)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = output_dir

        drone_id = SimulationParams.DRONE_ID
        self.declare_parameter("baseline_bspline_topic", f"/drone_{drone_id}_planning/bspline")
        self.declare_parameter("cinematic_bspline_topic", f"/drone_{drone_id}_planning/cinematic_bspline")
        self.declare_parameter("odom_topic", f"/drone_{drone_id}_visual_slam/odom")
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

        self.timer = self.create_timer(0.02, self.log_data)
        self._saved_once = False
        self._autosave_timer = self.create_timer(2.0, self._autosave)
        atexit.register(lambda: self._save_on_exit("process exit"))

        self.get_logger().info('Trajectory data logger initialized')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info('Sampling rate: 50 Hz (every 0.02s)')

    def baseline_bspline_callback(self, msg):
        """Store latest baseline B-spline and start logging"""
        self.current_baseline_bspline = msg
        if not self.logging_active:
            self.logging_active = True
            self.global_start_time = self.get_clock().now()
            self.get_logger().info(
                f'[LOGGER] Received first B-spline! Started logging trajectory data. '
                f'B-spline has {len(msg.pos_pts)} control points'
            )
        else:
            if len(self.baseline_data) % 100 == 0:
                self.get_logger().debug(f'[LOGGER] Received B-spline #{len(self.baseline_data)//100}')

    def constrained_bspline_callback(self, msg):
        """Store latest constrained B-spline"""
        self.current_constrained_bspline = msg

    def odom_callback(self, msg: Odometry):
        """Store latest odometry and start logging when first received"""
        self.current_odom = msg
        if not self.logging_active:
            self.logging_active = True
            self.global_start_time = self.get_clock().now()
            pos = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ])
            self.start_position = pos.copy()
            self.get_logger().info(
                f'[LOGGER] Received first odometry; origin set to ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})'
            )

    def log_data(self):
        """Evaluate B-spline trajectories at 20 Hz to get planned position/velocity"""
        if not self.logging_active or self.current_odom is None:
            return

        current_time = self.get_clock().now()
        global_elapsed = (current_time - self.global_start_time).nanoseconds / 1e9

        msg = self.current_odom
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])

        if self.start_position is None:
            self.start_position = pos.copy()
            self.get_logger().info(f'[LOGGER] Origin set to: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

        rel_pos = pos - self.start_position
        speed = np.linalg.norm(vel)
        planned_pos = None
        planned_vel = None
        if self.current_baseline_bspline is not None:
            planned_pos, planned_vel = self._evaluate_bspline(self.current_baseline_bspline, current_time)
        if planned_pos is not None and self.start_position is not None:
            planned_pos = planned_pos - self.start_position

        odom_entry = {
            'time': global_elapsed,
            'odom_x': rel_pos[0],
            'odom_y': rel_pos[1],
            'odom_z': rel_pos[2],
            'odom_vx': vel[0],
            'odom_vy': vel[1],
            'odom_vz': vel[2],
            'odom_speed': speed,
            'planned_x': planned_pos[0] if planned_pos is not None else None,
            'planned_y': planned_pos[1] if planned_pos is not None else None,
            'planned_z': planned_pos[2] if planned_pos is not None else None,
            'planned_vx': planned_vel[0] if planned_vel is not None else None,
            'planned_vy': planned_vel[1] if planned_vel is not None else None,
            'planned_vz': planned_vel[2] if planned_vel is not None else None,
            'planned_speed': float(np.linalg.norm(planned_vel)) if planned_vel is not None else None,
        }
        self.baseline_data.append(odom_entry)

        if len(self.baseline_data) % 500 == 0:
            constrained_info = (
                f'Constrained: {len(self.constrained_data)} samples' if self.constrained_data else 'No constrained data'
            )
            self.get_logger().info(
                f'Logged {len(self.baseline_data)} baseline samples | {constrained_info}'
            )

    def _evaluate_bspline(self, bspline_msg, current_ros_time):
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
        # Constrained runs should emit a single cinematic file (no extra constrained_* output).
        if self.mode == 'constrained':
            if len(self.constrained_data) == 0 and len(self.baseline_data) == 0:
                self.get_logger().warn('No data to save')
                return

            filename = (
                f'cinematic_sim_{self.timestamp}.csv'
                if self.run_number is None
                else f'cinematic_replay_run{self.run_number}_{self.timestamp}.csv'
            )
            data_to_save = self.constrained_data if len(self.constrained_data) > 0 else self.baseline_data
            path = self.output_dir / filename
            self._write_csv(path, data_to_save)
            self.get_logger().info(f'Saved {len(data_to_save)} cinematic data points to: {path}')
            return

        if len(self.baseline_data) == 0:
            self.get_logger().warn('No data to save')
            return

        # Determine filename based on mode (baseline / sim)
        if self.mode == 'baseline' and self.run_number is not None:
            filename = f'baseline_replay_run{self.run_number}_{self.timestamp}.csv'
        else:
            filename = f'baseline_sim_{self.timestamp}.csv'

        baseline_path = self.output_dir / filename
        self._write_csv(baseline_path, self.baseline_data)
        self.get_logger().info(f'Saved {len(self.baseline_data)} baseline data points to: {baseline_path}')

        # Only save constrained data in non-constrained mode for comparison
        if len(self.constrained_data) > 0:
            constrained_path = self.output_dir / f'constrained_sim_{self.timestamp}.csv'
            self._write_csv(constrained_path, self.constrained_data)
            self.get_logger().info(f'Saved {len(self.constrained_data)} constrained data points to: {constrained_path}')

    def _autosave(self):
        if self._saved_once:
            return
        if len(self.baseline_data) == 0 and len(self.constrained_data) == 0:
            return
        self.save_data()

    def _save_on_exit(self, reason: str):
        if self._saved_once:
            return
        self._saved_once = True
        try:
            self.get_logger().info(f"Saving trajectory data ({reason})")
            self.save_data()
        except Exception:
            pass

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
    parsed_args, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    logger = TrajectoryDataLogger(mode=parsed_args.mode, run_number=parsed_args.run_number)

    # Set up signal handler for graceful shutdown
    def signal_handler(sig, frame):
        logger.get_logger().info('Received shutdown signal...')
        logger._save_on_exit(f"signal {sig}")
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        logger.get_logger().info('Shutting down...')
        logger._save_on_exit("KeyboardInterrupt")
    except ExternalShutdownException:
        logger.get_logger().info('External shutdown requested')
    finally:
        logger._save_on_exit("finalizer")
        logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
