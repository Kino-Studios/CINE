import math

import numpy as np
from cine_config.constants import Constraints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TrapezoidalProfile:
    def __init__(self, max_velocity: float, max_acceleration: float):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

    def compute_profile(
        self, start_pos: np.ndarray, target_pos: np.ndarray, current_vel: np.ndarray, dt: float
    ) -> tuple[np.ndarray, np.ndarray]:

        displacement = target_pos - start_pos
        distance = np.linalg.norm(displacement)
        if distance < 1e-6:
            return target_pos.copy(), np.zeros(3)

        direction = displacement / distance
        desired_speed = min(self.max_velocity, math.sqrt(2.0 * self.max_acceleration * distance))
        desired_vel = direction * desired_speed
        vel_error = desired_vel - current_vel
        vel_error_mag = np.linalg.norm(vel_error)
        if vel_error_mag < 1e-6:
            accel = np.zeros(3)
        else:
            max_vel_change = self.max_acceleration * dt
            if vel_error_mag <= max_vel_change:
                accel = vel_error / dt
            else:
                accel = (vel_error / vel_error_mag) * self.max_acceleration

        next_vel = current_vel + accel * dt
        vel_mag = np.linalg.norm(next_vel)
        if vel_mag > self.max_velocity:
            next_vel = (next_vel / vel_mag) * self.max_velocity

        next_pos = start_pos + current_vel * dt + 0.5 * accel * dt * dt
        if np.linalg.norm(next_pos - target_pos) < 0.01:
            next_pos = target_pos.copy()
            next_vel = np.zeros(3)

        return next_pos, next_vel


class TrapezoidalSetpointFilter:
    def __init__(
        self,
        node: Node,
        input_topic: str = '/position_cmd',
        output_topic: str = '/position_cmd_smooth',
        max_velocity: float | None = None,
        max_acceleration: float | None = None,
    ):

        self.node = node

        max_vel = max_velocity if max_velocity is not None else Constraints.MAX_VELOCITY
        max_accel = (
            max_acceleration if max_acceleration is not None else Constraints.MAX_ACCELERATION
        )

        self.profile = TrapezoidalProfile(max_vel, max_accel)
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.has_odom = False

        self.odom_sub = node.create_subscription(Odometry, '/odometry', self.odom_callback, 20)
        self.cmd_sub = node.create_subscription(
            PoseStamped, '/cinematographic/pos_cmd_geo', self.position_cmd_callback, 10
        )
        self.cmd_pub = node.create_publisher(PoseStamped, '/cinematographic/trap_pos_cmd_geo', 10)

        self.publish_rate = 50.0
        self.timer = node.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.target_cmd: PoseStamped | None = None

    def odom_callback(self, msg: Odometry):
        self.current_pos = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        )
        self.current_vel = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        )
        self.has_odom = True

    def position_cmd_callback(self, msg: PoseStamped):
        if not (
            math.isnan(msg.pose.position.x)
            or math.isnan(msg.pose.position.y)
            or math.isnan(msg.pose.position.z)
        ):
            self.target_cmd = msg

    def timer_callback(self):
        if not self.has_odom or self.target_cmd is None:
            return

        target_pos = np.array(
            [
                self.target_cmd.pose.position.x,
                self.target_cmd.pose.position.y,
                self.target_cmd.pose.position.z,
            ]
        )

        dt = 1.0 / self.publish_rate
        next_pos, next_vel = self.profile.compute_profile(
            self.current_pos, target_pos, self.current_vel, dt
        )

        smoothed_cmd = PoseStamped()
        smoothed_cmd.header.stamp = self.node.get_clock().now().to_msg()
        smoothed_cmd.header.frame_id = self.target_cmd.header.frame_id
        smoothed_cmd.pose.position.x = float(next_pos[0])
        smoothed_cmd.pose.position.y = float(next_pos[1])
        smoothed_cmd.pose.position.z = float(next_pos[2])

        vel_mag = np.linalg.norm(next_vel)
        if vel_mag > 0.01:
            vel_dir = next_vel / vel_mag
            yaw = math.atan2(vel_dir[1], vel_dir[0])

            smoothed_cmd.pose.orientation.x = 0.0
            smoothed_cmd.pose.orientation.y = 0.0
            smoothed_cmd.pose.orientation.z = math.sin(yaw / 2.0)
            smoothed_cmd.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            smoothed_cmd.pose.orientation = self.target_cmd.pose.orientation

        self.cmd_pub.publish(smoothed_cmd)
        self.current_pos = next_pos
        self.current_vel = next_vel
