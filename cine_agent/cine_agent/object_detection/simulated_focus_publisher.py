import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

from cine_msgs.msg import CinematographyCosts


class SimulatedFocusPublisher(Node):
    def __init__(self):
        super().__init__('simulated_focus_publisher')

        self.frame_id = str(self.declare_parameter('frame_id', 'world').value)
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 10.0).value)

        self.odom_topic = str(self.declare_parameter('odom_topic', '/odom').value)
        self.use_odom = bool(self.declare_parameter('use_odom', True).value)

        self.target_xyz = tuple(
            float(v) for v in self.declare_parameter('target_xyz', [2.34, 4.9, 1.35]).value
        )
        self.target_offset_xyz = tuple(
            float(v) for v in self.declare_parameter('target_offset_xyz', [2.34, 4.9, 1.35]).value
        )

        self.focus_depth_m = float(self.declare_parameter('focus_depth_m', 1.5).value)

        self._last_odom: Odometry | None = None
        self._odom_origin: tuple[float, float, float] | None = None
        self._target_world: tuple[float, float, float] | None = None
        self._odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        self._focus_depth_pub = self.create_publisher(Float32, '/cinematography/focus_depth_m', 10)
        self._target_world_pub = self.create_publisher(
            PointStamped, '/cinematography/target_world', 10
        )
        self._costs_pub = self.create_publisher(
            CinematographyCosts, '/cinematography/cinematography_costs', 10
        )

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self._timer = self.create_timer(period, self._tick)

    def _odom_cb(self, msg: Odometry):
        self._last_odom = msg
        if self._odom_origin is None:
            p = msg.pose.pose.position
            self._odom_origin = (float(p.x), float(p.y), float(p.z))
            ox, oy, oz = self.target_offset_xyz
            self._target_world = (
                self._odom_origin[0] + ox,
                self._odom_origin[1] + oy,
                self._odom_origin[2] + oz,
            )

    def _target_from_odom(self) -> tuple[float, float, float] | None:
        if not self.use_odom or self._last_odom is None:
            return None
        if self._target_world is not None:
            return self._target_world
        p = self._last_odom.pose.pose.position
        ox, oy, oz = self.target_offset_xyz

        return (float(p.x) + ox, float(p.y) + oy, float(p.z) + oz)

    def _tick(self):
        now = self.get_clock().now().to_msg()

        depth_msg = Float32()
        depth_msg.data = float(self.focus_depth_m)
        self._focus_depth_pub.publish(depth_msg)

        target = self._target_from_odom() or self.target_xyz
        target_msg = PointStamped()
        target_msg.header.stamp = now
        target_msg.header.frame_id = self.frame_id
        target_msg.point.x = float(target[0])
        target_msg.point.y = float(target[1])
        target_msg.point.z = float(target[2])
        self._target_world_pub.publish(target_msg)

        costs = CinematographyCosts()
        costs.header.stamp = now
        costs.header.frame_id = self.frame_id
        costs.light_cost = 0.0
        costs.framing_cost = 0.0
        costs.smoothness_cost = 0.0
        costs.distance_cost = 0.0
        costs.total_cost = 0.0
        self._costs_pub.publish(costs)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedFocusPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
