import atexit
import csv
import math
import os
import rclpy
import rclpy.time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from traj_utils.msg import Bspline
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float32
import numpy as np
from pathlib import Path
from datetime import datetime
import signal

from cinematographic_adjustment_agent.utils.paths import add_config_path

add_config_path()

from constants import CinematicConstraints, SimulationParams, WaypointParams


class UniformBspline:

    def __init__(self, control_points, order, knots):
        self.control_points = control_points
        self.p = order
        self.u = knots
        self.n = len(control_points) - 1
        self.m = self.n + self.p + 1

    def evaluateDeBoor(self, u):
        ub = np.clip(u, self.u[self.p], self.u[self.m - self.p])


        k = self.p
        while k < len(self.u) - 1:
            if self.u[k + 1] >= ub:
                break
            k += 1


        d = []
        for i in range(self.p + 1):
            idx = k - self.p + i
            if 0 <= idx < len(self.control_points):
                d.append(self.control_points[idx].copy())
            else:
                d.append(np.zeros(3))


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

        return self.evaluateDeBoor(t + self.u[self.p])

    def getTimeSum(self):

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


        derivative_knots = self.u[1:-1]

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
        self.global_start_time = None
        self._global_start_ros_time: rclpy.time.Time | None = None
        self.start_position = None

        self.mode = mode
        self.run_number = run_number

        self.declare_parameter("distance_target_m", SimulationParams.FOCUS_DEPTH_M)
        self.declare_parameter("distance_tolerance_m", 0.15)
        self.target_distance_m = float(self.get_parameter("distance_target_m").value)
        self.distance_tolerance_m = float(self.get_parameter("distance_tolerance_m").value)
        self.target_world = None
        self._distance_rows: list[list[str]] = []
        self._distance_saved = False
        self._distance_stats = {
            "count": 0,
            "abs_error_sum": 0.0,
            "abs_error_max": 0.0,
            "max_tolerance_violation": 0.0,
            "within_count": 0,
        }
        self._framing_target_distance: float | None = None


        self._cinematic_framing_active = False
        self._framing_active = False

        self._anchor_start_world: tuple[float, float, float] | None = None
        self._anchor_end_world: tuple[float, float, float] | None = None
        self._expected_target_distance: float | None = None

        workspace_root = next(
            (p for p in Path(__file__).resolve().parents if p.name == "trajectory-optimization_ws"),
            Path.cwd(),
        )
        default_dir = workspace_root / SimulationParams.TRAJECTORY_DATA_DIR
        output_dir = Path(default_dir)
        output_dir.mkdir(exist_ok=True, parents=True)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = output_dir
        self.distance_output_path = self.output_dir / f"distance_{self.mode}_{self.timestamp}.csv"

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

        self.target_world_sub = self.create_subscription(
            PointStamped,
            "/cinematography/target_world",
            self.target_world_callback,
            10,
        )


        self.cinematic_framing_sub = self.create_subscription(
            Bool,
            "/cinematography/framing_active",
            self.cinematic_framing_callback,
            10,
        )

        self.anchor_start_sub = self.create_subscription(
            PointStamped,
            "/cinematography/anchor_start",
            self.anchor_start_callback,
            10,
        )

        self.anchor_end_sub = self.create_subscription(
            PointStamped,
            "/cinematography/anchor_end",
            self.anchor_end_callback,
            10,
        )

        self.anchor_distance_topic = str(
            self.declare_parameter(
                "anchor_focus_distance_topic",
                "/cinematography/anchor_focus_distance",
            ).value
        )
        self.anchor_distance_sub = self.create_subscription(
            Float32,
            self.anchor_distance_topic,
            self.anchor_focus_distance_callback,
            10,
        )

        self.timer = self.create_timer(0.02, self.log_data)
        self._saved_once = False
        self._autosave_timer = self.create_timer(0.05, self._autosave)
        atexit.register(lambda: self._save_on_exit("process exit"))

        self.get_logger().info('Trajectory data logger initialized')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(f'Mode: {self.mode}')
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
        if self._global_start_ros_time is None:
            header_stamp = msg.header.stamp
            if header_stamp.sec != 0 or header_stamp.nanosec != 0:
                try:
                    self._global_start_ros_time = rclpy.time.Time.from_msg(header_stamp)
                except Exception:
                    pass

    def log_data(self):
        """Evaluate B-spline trajectories at 20 Hz to get planned position/velocity"""
        if not self.logging_active or self.current_odom is None:
            return

        current_time = self.get_clock().now()
        msg = self.current_odom
        odom_time = None
        if msg is not None:
            try:
                header_stamp = msg.header.stamp
                odom_time = rclpy.time.Time.from_msg(header_stamp)
            except Exception:
                odom_time = None
        if self._global_start_ros_time is None:
            self._global_start_ros_time = odom_time
        if odom_time is not None and self._global_start_ros_time is not None:
            global_elapsed = (odom_time - self._global_start_ros_time).nanoseconds / 1e9
        elif self.global_start_time is not None:
            global_elapsed = (current_time - self.global_start_time).nanoseconds / 1e9
        else:
            global_elapsed = 0.0

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
            'cinematic_framing_active': self._cinematic_framing_active,
        }
        self.baseline_data.append(odom_entry)


        if self._framing_active and self.target_world is not None:
            framing_mode = 'cinematic' if self._cinematic_framing_active else 'baseline'
            self._record_distance_entry(global_elapsed, pos, framing_mode)

        if len(self.baseline_data) % 500 == 0:
            constrained_info = (
                f'Constrained: {len(self.constrained_data)} samples' if self.constrained_data else 'No constrained data'
            )
            framing_status = "FRAMING" if self._framing_active else "not framing"
            framing_type = " (cinematic)" if self._framing_active else ""
            self.get_logger().info(
                f'Logged {len(self.baseline_data)} baseline samples | {constrained_info} | {framing_status}{framing_type}'
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

    def target_world_callback(self, msg: PointStamped):
        self.target_world = (
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
        )
        self._recompute_expected_target_distance()

    def cinematic_framing_callback(self, msg: Bool):
        """Handle cinematic framing active state changes"""
        active = bool(msg.data)
        if active == self._cinematic_framing_active:
            return

        self.get_logger().info(f"Cinematic framing: {self._cinematic_framing_active} -> {active}")

        prev_overall = self._framing_active
        self._cinematic_framing_active = active
        self._update_framing_state(prev_overall)

    def anchor_start_callback(self, msg: PointStamped):
        self._anchor_start_world = (
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
        )
        self._recompute_expected_target_distance()

    def anchor_end_callback(self, msg: PointStamped):
        self._anchor_end_world = (
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
        )
        self._recompute_expected_target_distance()

    def anchor_focus_distance_callback(self, msg: Float32):
        distance = float(msg.data)
        if not math.isfinite(distance) or distance <= 0.0:
            return

        self.get_logger().info(f"Received anchor focus distance: {distance:.3f}m")
        self._expected_target_distance = distance


        if self._framing_active:
            if self._framing_target_distance is None:
                self._framing_target_distance = distance
                self.get_logger().info(f"Set framing target distance to {distance:.3f}m")

    def _update_framing_state(self, prev_overall):
        """Update overall framing state based on cinematic framing status"""
        current_overall = self._cinematic_framing_active


        if current_overall and not prev_overall:
            self._framing_target_distance = self._expected_target_distance
            framing_type = 'cinematic'
            self.get_logger().info(
                f"ENTERING FRAMING ({framing_type}) - target distance: {self._framing_target_distance}"
            )


        elif not current_overall and prev_overall:
            framing_type = 'cinematic'
            self.get_logger().info(f"EXITING FRAMING ({framing_type})")


        self._framing_active = current_overall

    def _recompute_expected_target_distance(self):
        """Compute expected target distance from anchor points"""
        expected = None
        if (
            self.target_world is not None
            and self._anchor_start_world is not None
            and self._anchor_end_world is not None
        ):
            start_dist = math.dist(self._anchor_start_world, self.target_world)
            end_dist = math.dist(self._anchor_end_world, self.target_world)
            focus_distance = max(start_dist, end_dist)
            min_radius = float(WaypointParams.MIN_ORBIT_RADIUS_M)
            focus_distance = max(focus_distance, min_radius)
            if math.isfinite(focus_distance) and focus_distance > 0.0:
                expected = focus_distance
                self.get_logger().info(
                    f"Computed expected distance from anchors: {expected:.3f}m "
                    f"(start={start_dist:.3f}m, end={end_dist:.3f}m)"
                )

        self._expected_target_distance = expected


        if self._framing_active and expected is not None:
            if self._framing_target_distance is None:
                self._framing_target_distance = expected

    def _record_distance_entry(self, time_value: float, drone_pos: np.ndarray, framing_mode: str):
        """Record a distance measurement during framing"""
        if self.target_world is None:
            return

        dx = drone_pos[0] - self.target_world[0]
        dy = drone_pos[1] - self.target_world[1]
        dz = drone_pos[2] - self.target_world[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        target_distance = self._framing_target_distance
        if target_distance is None:

            target_distance = distance
            self._framing_target_distance = target_distance
            self.get_logger().info(
                f"Initialized framing target distance to current: {target_distance:.3f}m"
            )

        error = distance - target_distance
        abs_error = abs(error)
        within = abs_error <= self.distance_tolerance_m

        self._distance_rows.append([
            framing_mode,  # 'cinematic' or 'baseline'
            f"{time_value:.9f}",
            f"{drone_pos[0]:.9f}",
            f"{drone_pos[1]:.9f}",
            f"{drone_pos[2]:.9f}",
            f"{self.target_world[0]:.9f}",
            f"{self.target_world[1]:.9f}",
            f"{self.target_world[2]:.9f}",
            f"{distance:.9f}",
            f"{target_distance:.9f}",
            f"{error:.9f}",
            f"{abs_error:.9f}",
            '1' if within else '0',
        ])

        stats = self._distance_stats
        stats["count"] += 1
        stats["abs_error_sum"] += abs_error
        stats["abs_error_max"] = max(stats["abs_error_max"], abs_error)
        violation = max(abs_error - self.distance_tolerance_m, 0.0)
        stats["max_tolerance_violation"] = max(stats["max_tolerance_violation"], violation)
        stats["within_count"] += 1 if within else 0

    def _write_distance_csv(self):
        if not self._distance_rows:
            return

        fieldnames = [
            "framing_mode",  # 'cinematic' or 'baseline'
            "time",
            "drone_x",
            "drone_y",
            "drone_z",
            "object_x",
            "object_y",
            "object_z",
            "distance",
            "target_distance",
            "error",
            "abs_error",
            "within_tolerance",
        ]

        self.distance_output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.distance_output_path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(fieldnames)
            writer.writerows(self._distance_rows)


        if self._saved_once:
            self._log_distance_summary()

    def _log_distance_summary(self):
        stats = self._distance_stats
        if stats["count"] == 0:
            self.get_logger().info("No distance data recorded during framing")
            return

        avg_abs = stats["abs_error_sum"] / stats["count"]
        within_pct = stats["within_count"] / stats["count"] * 100.0
        outside_pct = 100.0 - within_pct
        max_violation = stats["max_tolerance_violation"]

        self.get_logger().info(
            f"\n{'='*70}\n"
            f"DISTANCE ANALYSIS SUMMARY (mode={self.mode})\n"
            f"{'='*70}\n"
            f"  Samples:              {stats['count']}\n"
            f"  Mean |error|:         {avg_abs:.4f} m\n"
            f"  Max |error|:          {stats['abs_error_max']:.4f} m\n"
            f"  Within tolerance:     {within_pct:.1f}% (±{self.distance_tolerance_m:.2f} m)\n"
            f"  Outside tolerance:    {outside_pct:.1f}%\n"
            f"  Max violation:        {max_violation:.4f} m\n"
            f"  Output:               {self.distance_output_path.name}\n"
            f"{'='*70}"
        )

    def save_data(self):

        if self.mode == 'constrained':
            if len(self.constrained_data) == 0 and len(self.baseline_data) == 0:
                self.get_logger().warn('No data to save')
                self._write_distance_csv()
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
            self._write_distance_csv()
            return

        if len(self.baseline_data) == 0:
            self.get_logger().warn('No data to save')
            self._write_distance_csv()
            return


        if self.mode == 'baseline' and self.run_number is not None:
            filename = f'baseline_replay_run{self.run_number}_{self.timestamp}.csv'
        else:
            filename = f'baseline_sim_{self.timestamp}.csv'

        baseline_path = self.output_dir / filename
        self._write_csv(baseline_path, self.baseline_data)
        self.get_logger().info(f'Saved {len(self.baseline_data)} baseline data points to: {baseline_path}')


        if len(self.constrained_data) > 0:
            constrained_path = self.output_dir / f'constrained_sim_{self.timestamp}.csv'
            self._write_csv(constrained_path, self.constrained_data)
            self.get_logger().info(f'Saved {len(self.constrained_data)} constrained data points to: {constrained_path}')

        self._write_distance_csv()

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


    parser = argparse.ArgumentParser(description='Log trajectory data')
    parser.add_argument('--mode', type=str, default='sim',
                       choices=['sim', 'baseline', 'constrained'],
                       help='Logging mode: sim (normal), baseline (replay), or constrained (replay)')
    parser.add_argument('--run-number', type=int, default=None,
                       help='Run number for replay mode')
    parsed_args, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    logger = TrajectoryDataLogger(mode=parsed_args.mode, run_number=parsed_args.run_number)


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
