import sys
from pathlib import Path

# Add repo root to sys.path to find 'configs'
here = Path(__file__).resolve()
for parent in [here] + list(here.parents):
    if (parent / "configs").is_dir():
        python_constants_path = parent / "configs" / "python-constants"
        if str(python_constants_path) not in sys.path:
            sys.path.insert(0, str(python_constants_path))
        break

import rclpy
from cinematography_msgs.msg import CinematographyCosts
from geometry_msgs.msg import PointStamped, PolygonStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool

from constants import CinematographyParams, LightAvoidanceParams
from .cost_calculator import CostCalculator


class CostFunctionCalculatorNode(Node):
    def __init__(self):
        super().__init__("cost_function_calculator")
        self.params = CinematographyParams()
        self.light_avoidance_enabled = bool(
            self.declare_parameter(
                "light_avoidance_enabled", LightAvoidanceParams.ENABLED
            ).value
        )
        self._framing_active = False
        self.cost_calculator = CostCalculator(self)
        self.cost_calculator.update_framing_active(self._framing_active)
        self._last_odom: Odometry | None = None
        self.declare_parameter("frame_id", "map")

        self.declare_parameter("image_width", self.params.IMAGE_WIDTH)
        self.declare_parameter("image_height", self.params.IMAGE_HEIGHT)
        self.declare_parameter("camera_fx", self.params.CAMERA_FX)
        self.declare_parameter("camera_fy", self.params.CAMERA_FY)
        self.declare_parameter("camera_cx", self.params.CAMERA_CX)
        self.declare_parameter("camera_cy", self.params.CAMERA_CY)
        self.declare_parameter("depth_topic", self.params.DEPTH_TOPIC)
        self.declare_parameter("center_topic", self.params.CENTER_TOPIC)
        self.declare_parameter("bbox_topic", self.params.BBOX_TOPIC)
        self.declare_parameter("dimensions_topic", self.params.DIMENSIONS_TOPIC)
        self.declare_parameter("odom_topic", self.params.ODOM_TOPIC)
        self.declare_parameter("light_vector_topic", self.params.LIGHT_VECTOR_TOPIC)
        self.declare_parameter("publish_topic", "/cinematography/cinematography_costs")
        self.declare_parameter("publish_rate_hz", 10.0)

        depth_topic = str(self.get_parameter("depth_topic").value)
        center_topic = str(self.get_parameter("center_topic").value)
        bbox_topic = str(self.get_parameter("bbox_topic").value)
        dimensions_topic = str(self.get_parameter("dimensions_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        light_vector_topic = str(self.get_parameter("light_vector_topic").value)
        publish_topic = str(self.get_parameter("publish_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self._depth_sub = self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        self._center_sub = self.create_subscription(PolygonStamped, center_topic, self._center_cb, 10)
        self._bbox_sub = self.create_subscription(PolygonStamped, bbox_topic, self._bbox_cb, 10)
        self._dims_sub = self.create_subscription(PolygonStamped, dimensions_topic, self._dims_cb, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        if self.light_avoidance_enabled:
            self._light_sub = self.create_subscription(
                Vector3Stamped, light_vector_topic, self._light_cb, 10
            )
        else:
            self._light_sub = None

        self._pub = self.create_publisher(CinematographyCosts, publish_topic, 10)
        self._focus_depth_pub = self.create_publisher(Float32, "/cinematography/focus_depth_m", 10)
        self._focus_center_depth_pub = self.create_publisher(Float32, "/cinematography/focus_center_depth_m", 10)
        self._focus_roi_depth_pub = self.create_publisher(Float32, "/cinematography/focus_roi_depth_median_m", 10)
        self._target_world_pub = self.create_publisher(PointStamped, "/cinematography/target_world", 10)
        self._timer = self.create_timer(1.0 / max(1.0, publish_rate_hz), self._tick)
        self._framing_sub = self.create_subscription(
            Bool,
            "/cinematography/framing_active",
            self._framing_cb,
            10,
        )

    def _depth_cb(self, msg: Image):
        try:
            from cv_bridge import CvBridge

            bridge = getattr(self, "_bridge", None)
            if bridge is None:
                bridge = CvBridge()
                self._bridge = bridge
            depth = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.cost_calculator.update_depth_image(depth)
        except Exception:
            return

    def _center_cb(self, msg: PolygonStamped):
        self.cost_calculator.update_center_point(msg)

    def _bbox_cb(self, msg: PolygonStamped):
        self.cost_calculator.update_bounding_box(msg)

    def _dims_cb(self, msg: PolygonStamped):
        self.cost_calculator.update_dimensions(msg)

    def _odom_cb(self, msg: Odometry):
        self.cost_calculator.update_odometry(msg)
        self._last_odom = msg

    def _light_cb(self, msg: Vector3Stamped):
        self.cost_calculator.update_light_vector(msg)

    def _framing_cb(self, msg: Bool):
        active = bool(msg.data) if msg is not None else False
        self._framing_active = active
        self.cost_calculator.update_framing_active(active)

    def _tick(self):
        now_msg = self.get_clock().now().to_msg()
        frame_id = str(self.get_parameter("frame_id").value)

        center_depth = self.cost_calculator.center_depth
        if center_depth is not None:
            msg = Float32()
            msg.data = float(center_depth)
            self._focus_center_depth_pub.publish(msg)

        roi_depth = self.cost_calculator.get_median_depth_roi()
        if roi_depth is not None:
            msg = Float32()
            msg.data = float(roi_depth)
            self._focus_roi_depth_pub.publish(msg)

        focus_depth = roi_depth if roi_depth is not None else center_depth
        if focus_depth is not None:
            msg = Float32()
            msg.data = float(focus_depth)
            self._focus_depth_pub.publish(msg)

        target_world = self._estimate_target_world_position(focus_depth)
        self.cost_calculator.update_target_world(target_world)
        if target_world is not None:
            msg = PointStamped()
            msg.header.stamp = now_msg
            msg.header.frame_id = frame_id
            msg.point.x = float(target_world[0])
            msg.point.y = float(target_world[1])
            msg.point.z = float(target_world[2])
            self._target_world_pub.publish(msg)

        result = self.cost_calculator.calculate_cost()
        if result is None:
            return

        msg = CinematographyCosts()
        msg.header.stamp = now_msg
        msg.header.frame_id = frame_id
        msg.light_cost = float(result.get("lighting_cost", 0.0))
        msg.framing_cost = float(result.get("framing_cost", 0.0))
        msg.smoothness_cost = float(result.get("smoothness_cost", 0.0))
        msg.distance_cost = float(result.get("distance_cost", 0.0))
        msg.total_cost = float(result.get("total_cost", 0.0))
        msg.yaw_setpoint = float(result.get("yaw_setpoint", 0.0))
        msg.has_cinematic_yaw = bool(result.get("has_cinematic_yaw", False))
        msg.pitch_setpoint = float(result.get("pitch_setpoint", 0.0))
        msg.roll_setpoint = float(result.get("roll_setpoint", 0.0))
        msg.yaw_error = float(result.get("yaw_error", 0.0))
        msg.pitch_error = float(result.get("pitch_error", 0.0))
        msg.roll_error = float(result.get("roll_error", 0.0))
        msg.actual_yaw = float(result.get("actual_yaw", 0.0))
        msg.actual_pitch = float(result.get("actual_pitch", 0.0))
        msg.actual_roll = float(result.get("actual_roll", 0.0))
        self.get_logger().debug(
            f"Attitude yaw(set={msg.yaw_setpoint:.2f}, act={msg.actual_yaw:.2f}), "
            f"pitch(set={msg.pitch_setpoint:.2f}, act={msg.actual_pitch:.2f}), "
            f"roll(set={msg.roll_setpoint:.2f}, act={msg.actual_roll:.2f})"
        )
        self._pub.publish(msg)

    def _estimate_target_world_position(self, depth_m: float | None):
        if depth_m is None or not float(depth_m) > 0.0:
            return None
        if self._last_odom is None:
            return None
        if self.cost_calculator.center_point is None or len(self.cost_calculator.center_point.polygon.points) == 0:
            return None

        u = float(self.cost_calculator.center_point.polygon.points[0].x)
        v = float(self.cost_calculator.center_point.polygon.points[0].y)

        fx = float(self.get_parameter("camera_fx").value)
        fy = float(self.get_parameter("camera_fy").value)
        cx = float(self.get_parameter("camera_cx").value)
        cy = float(self.get_parameter("camera_cy").value)

        if fx <= 0.0 or fy <= 0.0:
            return None

        x_cam = (u - cx) * float(depth_m) / fx
        y_cam = (v - cy) * float(depth_m) / fy
        z_cam = float(depth_m)

        drone_pos = self._last_odom.pose.pose.position
        # Same convention used in `cinematographic_adjustment_node.py`.
        target_world = (
            float(drone_pos.x) + z_cam,
            float(drone_pos.y) - x_cam,
            float(drone_pos.z) - y_cam,
        )
        return target_world


def main(args=None):
    rclpy.init(args=args)
    node = CostFunctionCalculatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
