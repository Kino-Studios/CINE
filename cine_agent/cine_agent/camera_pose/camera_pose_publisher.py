import rclpy
from cine_config.config_loader import ConfigLoader
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from cine_agent.camera_pose.camera_pose import Pose, compose_camera_pose


def load_body_to_camera() -> Pose:
    config = ConfigLoader()
    translation = config.get(
        'camera', 'camera_params', 'body_to_camera_translation', [0.05, 0.0, 0.0]
    )
    quaternion = config.get(
        'camera', 'camera_params', 'body_to_camera_quaternion', [-0.5, 0.5, -0.5, 0.5]
    )

    return Pose(*(float(value) for value in (*translation, *quaternion)))


class CameraPosePublisher(Node):
    def __init__(self):
        super().__init__('camera_pose_publisher')
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('odometry_topic', '/vio/odometry')
        drone_id = int(self.get_parameter('drone_id').value)
        odometry_topic = str(self.get_parameter('odometry_topic').value)

        self._body_to_camera = load_body_to_camera()
        self._publisher = self.create_publisher(PoseStamped, f'drone_{drone_id}_camera_pose', 10)
        self._subscription = self.create_subscription(
            Odometry, odometry_topic, self._on_odometry, 10
        )

    def _on_odometry(self, odometry: Odometry):
        body_pose = odometry.pose.pose
        camera = compose_camera_pose(
            Pose(
                body_pose.position.x,
                body_pose.position.y,
                body_pose.position.z,
                body_pose.orientation.x,
                body_pose.orientation.y,
                body_pose.orientation.z,
                body_pose.orientation.w,
            ),
            self._body_to_camera,
        )

        message = PoseStamped()
        message.header = odometry.header
        message.pose.position.x = camera.x
        message.pose.position.y = camera.y
        message.pose.position.z = camera.z
        message.pose.orientation.x = camera.qx
        message.pose.orientation.y = camera.qy
        message.pose.orientation.z = camera.qz
        message.pose.orientation.w = camera.qw
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
