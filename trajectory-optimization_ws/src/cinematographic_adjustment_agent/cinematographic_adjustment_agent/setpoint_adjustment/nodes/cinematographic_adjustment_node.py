import sys
from pathlib import Path

here = Path(__file__).resolve()
for parent in [here] + list(here.parents):
    if (parent / "configs").is_dir():
        python_constants_path = parent / "configs" / "python-constants"
        if str(python_constants_path) not in sys.path:
            sys.path.insert(0, str(python_constants_path))
        break

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PolygonStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from cinematography_msgs.msg import CinematographyCosts

from constants import SensorNoiseParams
from cinematographic_adjustment_agent.cost_functions.cost_calculator import CostCalculator
from cinematographic_adjustment_agent.setpoint_sender.waypoint_sender import WaypointPublisher
from cinematographic_adjustment_agent.sensor_noise.noisy_odometry import NoisyOdometryNode

from ..utils.config_loader import ConfigLoader
from ..utils.state_tracker import StateTracker
from ..controllers.kinematics_controller import KinematicsController
from ..controllers.target_computer import TargetComputer
from ..controllers.waypoint_selector import WaypointSelector


class CinematographicPlannerNode(Node):
    def __init__(self):
        super().__init__("cinematographic_planner_node")

        self.bridge = CvBridge()
        self.has_odom = False
        self.drone_position = np.array([0.0, 0.0, 0.0])
        self.drone_orientation = np.array([0.0, 0.0, 0.0, 0.0])
        self.has_focus = False

        self.last_tick_time_s = None

        self.config = ConfigLoader(self)
        self.config.load_parameters()

        self.cost_calculator = CostCalculator(self)
        self.tracker = StateTracker(
            look_duration_s=self.config.params.LOOK_DURATION_S,
            depth_window_size=15
        )
        self.target_computer = TargetComputer(self.config, self.tracker, self.cost_calculator)
        self.kinematics_controller = KinematicsController(self.config)
        self.waypoint_selector = WaypointSelector(self, self.config)

        self.setup_subscriptions()

        self.cost_pub = self.create_publisher(
            CinematographyCosts,
            "/cinematography/cinematography_costs",
            10,
        )
        self.waypoint_pub_node = WaypointPublisher(self, self.config.params.WAYPOINTS_TOPIC)
        self.timer = self.create_timer(1.0 / max(1.0, self.config.params.WAYPOINT_RATE_HZ), self.tick)

    def setup_subscriptions(self) -> None:
        self.center_sub = self.create_subscription(
            PolygonStamped, self.config.params.CENTER_TOPIC, self.center_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, self.config.params.DEPTH_TOPIC, self.depth_callback, 10
        )
        self.bounding_box_sub = self.create_subscription(
            PolygonStamped, self.config.params.BBOX_TOPIC, self.bounding_box_callback, 10
        )
        self.dimensions_sub = self.create_subscription(
            PolygonStamped, self.config.params.DIMENSIONS_TOPIC, self.dimensions_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.config.params.ODOM_TOPIC, self.odom_callback, 10
        )
        if self.config.light_avoidance_enabled:
            self.light_vector_sub = self.create_subscription(
                Vector3Stamped, self.config.params.LIGHT_VECTOR_TOPIC, self.light_vector_callback, 10
            )
        else:
            self.light_vector_sub = None

        self.image_sub = self.create_subscription(
            Image, self.config.params.IMAGE_TOPIC, self.image_callback, 10
        )

    def center_callback(self, msg: PolygonStamped):
        if len(msg.polygon.points) == 0:
            return
        u = int(msg.polygon.points[0].x)
        v = int(msg.polygon.points[0].y)
        self.target_computer.update_center(u, v)
        self.cost_calculator.update_center_point(msg)

    def bounding_box_callback(self, msg: PolygonStamped):
        self.has_focus = len(msg.polygon.points) > 0
        self.cost_calculator.update_bounding_box(msg)

    def dimensions_callback(self, msg: PolygonStamped):
        self.cost_calculator.update_dimensions(msg)

    def odom_callback(self, msg: Odometry):
        self.drone_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.drone_orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.has_odom = True
        self.cost_calculator.update_odometry(msg)

    def light_vector_callback(self, msg: Vector3Stamped):
        if not self.config.light_avoidance_enabled:
            return
        self.cost_calculator.update_light_vector(msg)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.cost_calculator.update_bgr_image(cv_image)
        except Exception:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.cost_calculator.update_bgr_image(cv_image)
            except Exception as e:
                self.get_logger().warn(f'Image callback error: {e}')

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.cost_calculator.update_depth_image(depth)

            center_uv = self.target_computer.center_uv
            if center_uv is not None:
                u, v = center_uv
                h, w = depth.shape[:2]
                uu = max(0, min(u, w - 1))
                vv = max(0, min(v, h - 1))
                z = float(depth[vv, uu])
                if z > 0 and not np.isnan(z):
                    self.tracker.record_depth(z)
        except Exception as e:
            self.get_logger().warn(f'Depth callback error: {e}')

    def _dt(self) -> float:
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.last_tick_time_s is None:
            self.last_tick_time_s = now_s
            return 1.0 / max(1.0, self.config.params.WAYPOINT_RATE_HZ)
        dt = now_s - self.last_tick_time_s
        self.last_tick_time_s = now_s
        return float(max(1e-3, min(dt, 0.5)))

    def publish_costs(self):
        cost_result = self.cost_calculator.calculate_cost()
        if cost_result is not None and isinstance(cost_result, dict):
            total_cost = cost_result.get('total_cost', 0.0)
            msg = CinematographyCosts()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.light_cost = float(cost_result.get('lighting_cost', 0.0))
            msg.framing_cost = float(cost_result.get('framing_cost', 0.0))
            msg.smoothness_cost = float(cost_result.get('smoothness_cost', 0.0))
            msg.distance_cost = float(cost_result.get('distance_cost', 0.0))
            msg.total_cost = float(total_cost)
            self.cost_pub.publish(msg)

    def tick(self):
        dt = self._dt()

        if self.tracker.should_switch_target():
            self.tracker.rotate_to_next_target()

        self.publish_costs()

        now = self.get_clock().now()
        current_time_s = now.nanoseconds / 1e9

        target_world = self.target_computer.compute_target_world_position(self.drone_position, self.has_odom)
        if target_world is None:
            return

        self.waypoint_selector.optimize_orbit_parameters_if_needed(
            current_time=current_time_s,
            target_world=target_world,
            drone_position=self.drone_position,
            cost_estimation_func=self.cost_calculator.estimate_position_cost_at_position
        )

        opt_center = self.waypoint_selector.optimized_center
        opt_radius = self.waypoint_selector.optimized_radius

        tx, ty = float(target_world[0]), float(target_world[1])
        if opt_center is not None and opt_radius is not None:
            center_xy = opt_center
            radius = float(opt_radius)
        else:
            center_xy = (tx, ty)
            radius = float(self.config.params.ORBIT_RADIUS_M)

        desired = self.waypoint_selector.select_next_waypoint(
            center_xy=center_xy,
            radius_m=radius,
            altitude_m=float(self.config.params.CONSTANT_ALTITUDE_M),
            target_world=target_world,
            drone_position=self.drone_position,
            cost_estimation_func=self.cost_calculator.estimate_position_cost_at_position,
            dt=dt,
        )
        if desired is None:
            return

        next_pos = self.kinematics_controller.next_kinematic_step(desired, self.drone_position, dt, self.has_focus)
        self.waypoint_pub_node.publish_single_waypoint(float(next_pos[0]), float(next_pos[1]), float(next_pos[2]))


def main(args=None):
    rclpy.init(args=args)
    planner_node = CinematographicPlannerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(planner_node)

    noise_node = None
    if SensorNoiseParams.ENABLE_SENSOR_NOISE:
        noise_node = NoisyOdometryNode()
        executor.add_node(noise_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if noise_node is not None:
            noise_node.destroy_node()
        planner_node.destroy_node()
        rclpy.shutdown()
