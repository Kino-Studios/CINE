import rclpy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node

from cine_agent.setpoint_sender.setpoint_conversion import position_command_to_setpoint


class PlannerSetpointAdapter(Node):
    def __init__(self):
        super().__init__('planner_setpoint_adapter')
        self.declare_parameter('drone_id', 0)
        drone_id = int(self.get_parameter('drone_id').value)

        self._publisher = self.create_publisher(
            PoseStamped, '/cinematography/translational_position_setpoints', 10
        )
        self._subscription = self.create_subscription(
            PositionCommand, f'drone_{drone_id}_planning/pos_cmd', self._on_position_command, 10
        )

    def _on_position_command(self, command: PositionCommand):
        setpoint = position_command_to_setpoint(
            command.position.x, command.position.y, command.position.z, command.yaw
        )
        if setpoint is None:
            self.get_logger().warning('Dropped position command with non-finite values')

            return

        message = PoseStamped()
        message.header.stamp = command.header.stamp
        message.header.frame_id = 'map'
        message.pose.position.x = setpoint.x
        message.pose.position.y = setpoint.y
        message.pose.position.z = setpoint.z
        message.pose.orientation.x = setpoint.qx
        message.pose.orientation.y = setpoint.qy
        message.pose.orientation.z = setpoint.qz
        message.pose.orientation.w = setpoint.qw
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerSetpointAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
