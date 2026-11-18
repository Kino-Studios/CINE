"""Takes the simulated drone from link-up to the mission trigger, through MAVROS.

Runs the flight starter state machine (`flight_starter_logic.FlightStarter`) at 10 Hz of
sim time. Each step builds an observation from /mavros/state, /mavros/local_position/odom
and /vio/odometry and carries out the action the machine returns: MAVROS services for
mode, arming, takeoff and the EKF source set, setpoints through the Pi's setpoint relay,
and a pose on /traj_start_trigger to start the mission. Exits 0 when the machine reaches
DONE, 1 when it ends in FAILED, once the vehicle is disarmed or a bounded wait has passed.
"""

import math
import sys
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from cine_gazebo.flight_starter_logic import (
    GPS_SOURCE_SET,
    LAND_MODE,
    FlightStarter,
    StarterConfig,
    StarterState,
)
from cine_gazebo.sitl_observation import (
    LAND_SWITCH_TIMEOUT_S,
    CommandGate,
    LinkState,
    OdometrySample,
    build_observation,
    land_request,
    load_excitation_pattern,
    yaw_of,
)

MAV_CMD_SET_EKF_SOURCE_SET = 42007
SETPOINT_TOPIC = '/cinematography/translational_position_setpoints'
TRIGGER_TOPIC = '/traj_start_trigger'
SETPOINT_FRAME = 'map'

STEP_PERIOD_S = 0.1
SPIN_TIMEOUT_S = 0.1
# Wall time, not sim time: a stopped simulator must not keep the process alive forever.
DISARM_WAIT_WALL_S = 120.0
# Lets the trigger message leave before the node is destroyed.
EXIT_FLUSH_WALL_S = 1.0
HOLD_STATES = (
    StarterState.SETTLING,
    StarterState.SWITCHING_SOURCE,
    StarterState.CONFIRMING_HOLD,
)


def _odometry_sample(message, received_s):
    pose = message.pose.pose
    linear = message.twist.twist.linear

    return OdometrySample(
        position=(pose.position.x, pose.position.y, pose.position.z),
        orientation=(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
        linear_velocity=(linear.x, linear.y, linear.z),
        received_s=received_s,
    )


class SitlFlightStarter(Node):
    def __init__(self):
        super().__init__('sitl_flight_starter')
        share_directory = Path(get_package_share_directory('cine_gazebo'))
        nav_source = self.declare_parameter('nav_source', 'sim').value
        pattern_file = self.declare_parameter(
            'excitation_pattern_file',
            str(share_directory / 'config' / 'excitation_pattern.yaml'),
        ).value
        self._excitation_pattern = load_excitation_pattern(pattern_file)
        self._starter = FlightStarter(
            StarterConfig(nav_source=nav_source, excitation_pattern=self._excitation_pattern)
        )

        self.exit_code = None
        self._state = None
        self._autopilot_odometry = None
        self._autopilot_received_s = None
        self._vio_odometry = None
        self._vio_received_s = None
        # ArduPilot does not stream the active EKF source set, so it is tracked from the
        # answers to MAV_CMD_SET_EKF_SOURCE_SET. The autopilot boots on set 1.
        self._ekf_source_set = GPS_SOURCE_SET
        self._land_switch_requested_s = None
        self._last_source_set_refusal = None
        self._land_fallback_logged = False
        self._target_index = None
        self._hold_setpoint = None
        self._last_message = None
        self._disarm_wait_started_wall_s = None
        self._gate = CommandGate()

        self.create_subscription(State, '/mavros/state', self._on_state, 10)
        self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self._on_autopilot_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, '/vio/odometry', self._on_vio_odometry, qos_profile_sensor_data
        )
        # Same QoS as the relay's subscription in cine_sensing_setpoint/mavros_setpoints.py.
        relay_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._setpoint_publisher = self.create_publisher(PoseStamped, SETPOINT_TOPIC, relay_qos)
        self._trigger_publisher = self.create_publisher(PoseStamped, TRIGGER_TOPIC, 10)
        self._set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self._arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self._command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        # Under sim time the timer only fires once /clock is running.
        self.create_timer(STEP_PERIOD_S, self._on_step)
        self.get_logger().info(f'flight starter running with nav_source={nav_source}')

    def run(self):
        while rclpy.ok() and self.exit_code is None:
            rclpy.spin_once(self, timeout_sec=SPIN_TIMEOUT_S)
            self._check_disarm_wait()
        flush_deadline = time.monotonic() + EXIT_FLUSH_WALL_S
        while rclpy.ok() and time.monotonic() < flush_deadline:
            rclpy.spin_once(self, timeout_sec=SPIN_TIMEOUT_S)

        # rclpy shutting down before the machine finished is a failure too.

        return 1 if self.exit_code is None else self.exit_code

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, message):
        self._state = message

    def _on_autopilot_odometry(self, message):
        self._autopilot_odometry = message
        self._autopilot_received_s = self._now_s()

    def _on_vio_odometry(self, message):
        self._vio_odometry = message
        self._vio_received_s = self._now_s()

    def _observe(self, now_s):
        state = self._state
        odometry = self._autopilot_odometry
        vio_odometry = self._vio_odometry

        return build_observation(
            now_s=now_s,
            link=(
                LinkState(connected=state.connected, mode=state.mode, armed=state.armed)
                if state is not None
                else None
            ),
            autopilot=(
                _odometry_sample(odometry, self._autopilot_received_s)
                if odometry is not None
                else None
            ),
            vio=(
                _odometry_sample(vio_odometry, self._vio_received_s)
                if vio_odometry is not None
                else None
            ),
            ekf_source_set=self._ekf_source_set,
            excitation_pattern=self._excitation_pattern,
            target_index=self._target_index,
            hold_setpoint=self._hold_setpoint,
        )

    def _on_step(self):
        if self.exit_code is not None or self._disarm_wait_started_wall_s is not None:
            return
        action = self._starter.step(self._observe(self._now_s()))
        if action.message and action.message != self._last_message:
            self.get_logger().info(action.message)
            self._last_message = action.message
        self._execute(action)
        state = self._starter.state
        if state in HOLD_STATES and self._hold_setpoint is not None:
            self._publish_setpoint(self._hold_setpoint)
        if state is StarterState.DONE:
            self.get_logger().info('flight starter done')
            self.exit_code = 0
        elif state is StarterState.FAILED:
            self.get_logger().error(f'flight starter failed: {self._starter.failure_message}')
            self._disarm_wait_started_wall_s = time.monotonic()
            self._check_disarm_wait()

    def _check_disarm_wait(self):
        if self._disarm_wait_started_wall_s is None or self.exit_code is not None:
            return
        if self._state is None or not self._state.armed:
            self.get_logger().info('vehicle is disarmed; exiting')
            self.exit_code = 1
        elif time.monotonic() - self._disarm_wait_started_wall_s > DISARM_WAIT_WALL_S:
            self.get_logger().error(
                f'vehicle still armed {DISARM_WAIT_WALL_S:g} s after the failure; exiting anyway'
            )
            self.exit_code = 1

    def _execute(self, action):
        if action.kind == 'set_mode':
            self._request_mode(action.mode)
        elif action.kind == 'arm':
            self._request_arm()
        elif action.kind == 'takeoff':
            self._request_takeoff(action.altitude_m)
        elif action.kind == 'setpoint':
            self._fly_to_excitation_point(action.setpoint)
        elif action.kind == 'hold_position':
            self._capture_hold_setpoint()
        elif action.kind == 'switch_source':
            self._capture_hold_setpoint()
            self._request_source_set(action.source_set)
        elif action.kind == 'trigger':
            self._publish_trigger()
        elif action.kind == 'land':
            # 7a's contract: select the GPS source set first, then LAND. The machine
            # repeats the land action until the mode reads LAND.
            self._land(self._now_s())

    def _land(self, now_s):
        if self._ekf_source_set != GPS_SOURCE_SET and self._land_switch_requested_s is None:
            self._land_switch_requested_s = now_s
        if land_request(self._ekf_source_set, now_s, self._land_switch_requested_s) == (
            'switch_source'
        ):
            self._request_source_set(GPS_SOURCE_SET)

            return
        if self._ekf_source_set != GPS_SOURCE_SET and not self._land_fallback_logged:
            self.get_logger().warning(
                f'source set {GPS_SOURCE_SET} not accepted within {LAND_SWITCH_TIMEOUT_S:g} s '
                f'({self._last_source_set_refusal or "no answer"}); landing on source set '
                f'{self._ekf_source_set}'
            )
            self._land_fallback_logged = True
        self._request_mode(LAND_MODE)

    def _call(self, kind, client, request, on_response):
        now_s = self._now_s()
        if not self._gate.may_send(kind, now_s):
            return
        if not client.service_is_ready():
            self.get_logger().warning(
                f'{client.srv_name} is not available yet', throttle_duration_sec=5.0
            )

            return
        self._gate.sent(kind, now_s)
        future = client.call_async(request)
        future.add_done_callback(lambda done: self._on_answer(kind, client, done, on_response))

    def _on_answer(self, kind, client, future, on_response):
        self._gate.answered(kind)
        try:
            response = future.result()
        except Exception as exc:
            # The machine re-emits the command, so a failed call is logged and retried.
            self.get_logger().error(f'{client.srv_name} call failed: {exc}')
            if kind == 'switch_source':
                self._last_source_set_refusal = f'call failed: {exc}'

            return
        on_response(response)

    def _request_mode(self, mode):
        def on_response(response):
            if not response.mode_sent:
                self.get_logger().warning(f'autopilot refused mode {mode}')

        self._call(
            'set_mode', self._set_mode_client, SetMode.Request(custom_mode=mode), on_response
        )

    def _request_arm(self):
        def on_response(response):
            if not response.success:
                self.get_logger().warning(
                    f'autopilot refused to arm (MAV_RESULT {response.result})'
                )

        self._call('arm', self._arming_client, CommandBool.Request(value=True), on_response)

    def _request_takeoff(self, altitude_m):
        def on_response(response):
            if not response.success:
                self.get_logger().warning(
                    f'autopilot refused takeoff to {altitude_m:g} m (MAV_RESULT {response.result})'
                )

        # ArduCopter in GUIDED reads only the altitude, in metres above home.
        request = CommandTOL.Request(altitude=float(altitude_m))
        self._call('takeoff', self._takeoff_client, request, on_response)

    def _request_source_set(self, source_set):
        def on_response(response):
            if response.success:
                self._ekf_source_set = source_set
                self.get_logger().info(f'switched to source set {source_set}')
            else:
                self._last_source_set_refusal = (
                    f'autopilot refused source set {source_set}, MAV_RESULT {response.result}'
                )
                self.get_logger().warning(self._last_source_set_refusal)

        request = CommandLong.Request(command=MAV_CMD_SET_EKF_SOURCE_SET, param1=float(source_set))
        self._call('switch_source', self._command_client, request, on_response)

    def _fly_to_excitation_point(self, setpoint):
        self._target_index = self._excitation_pattern.index(tuple(setpoint))
        self._publish_setpoint(setpoint)

    def _capture_hold_setpoint(self):
        # Hold where the vehicle is when it is told to stop (or, if it was already at rest,
        # when the switch starts), so the hold check measures the estimate moving, not the
        # vehicle finishing its last excitation leg.
        if self._hold_setpoint is not None or self._autopilot_odometry is None:
            return
        sample = _odometry_sample(self._autopilot_odometry, self._autopilot_received_s)
        self._hold_setpoint = (*sample.position, yaw_of(sample.orientation))
        self._target_index = None

    def _publish_setpoint(self, setpoint):
        x, y, z, yaw = setpoint
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = SETPOINT_FRAME
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        message.pose.position.z = float(z)
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        self._setpoint_publisher.publish(message)

    def _publish_trigger(self):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = SETPOINT_FRAME
        if self._autopilot_odometry is not None:
            message.pose = self._autopilot_odometry.pose.pose
        self._trigger_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SitlFlightStarter()
    exit_code = 1
    try:
        exit_code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
