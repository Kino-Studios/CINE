import rclpy
from cinematography_msgs.msg import CinematographyCosts
from geometry_msgs.msg import PolygonStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

from ..config.planner_params import CinematographyParams
from .cost_calculator import CostCalculator


class CostFunctionCalculatorNode(Node):
    def __init__(self):
        super().__init__("cost_function_calculator")
        self.params = CinematographyParams.from_env()
        self.cost_calculator = CostCalculator(self)

        self.declare_parameter("image_width", self.params.image_width)
        self.declare_parameter("image_height", self.params.image_height)
        self.declare_parameter("depth_topic", self.params.depth_topic)
        self.declare_parameter("center_topic", self.params.center_topic)
        self.declare_parameter("bbox_topic", self.params.bbox_topic)
        self.declare_parameter("dimensions_topic", self.params.dimensions_topic)
        self.declare_parameter("odom_topic", self.params.odom_topic)
        self.declare_parameter("light_vector_topic", self.params.light_vector_topic)
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
        self._light_sub = self.create_subscription(Vector3Stamped, light_vector_topic, self._light_cb, 10)

        self._pub = self.create_publisher(CinematographyCosts, publish_topic, 10)
        self._timer = self.create_timer(1.0 / max(1.0, publish_rate_hz), self._tick)

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

    def _light_cb(self, msg: Vector3Stamped):
        self.cost_calculator.update_light_vector(msg)

    def _tick(self):
        result = self.cost_calculator.calculate_cost()
        if result is None:
            return

        msg = CinematographyCosts()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.light_cost = float(result.get("lighting_cost", 0.0))
        msg.framing_cost = float(result.get("framing_cost", 0.0))
        msg.smoothness_cost = float(result.get("smoothness_cost", 0.0))
        msg.distance_cost = float(result.get("distance_cost", 0.0))
        msg.total_cost = float(result.get("total_cost", 0.0))
        self._pub.publish(msg)


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
