"""ROS 2 parameter shim for the setpoint-adjustment node.

This ``ConfigLoader`` is a different layer from
``cine_config.config_loader``. The latter is the low-level JSON
loader (a process singleton that parses the config files). This class sits above
it: given a ROS 2 ``Node``, it *declares* node parameters seeded from the
dataclasses that the JSON loader populated, then reads back the (possibly
launch-overridden) parameter values. The two are intentionally not merged --
one owns file parsing, the other owns ROS parameter lifecycle.
"""

from cine_config.constants import (
    CinematographyParams,
    LightAvoidanceParams,
    UserRequestedParams,
    WaypointParams,
)
from rclpy.node import Node


class ConfigLoader:
    def __init__(self, node: Node):
        self.node = node
        self.params = CinematographyParams()
        self.light_avoidance_enabled = False
        self.thirds_strength = UserRequestedParams.THIRDS_STRENGTH
        self.thirds_use_nearest_anchor = UserRequestedParams.THIRDS_USE_NEAREST_ANCHOR
        self.candidate_angle_window_rad = WaypointParams.CANDIDATE_ANGLE_WINDOW_RAD
        self.candidate_angle_samples = WaypointParams.CANDIDATE_ANGLE_SAMPLES
        self.candidate_radius_samples = WaypointParams.CANDIDATE_RADIUS_SAMPLES
        self.candidate_radius_range_m = WaypointParams.CANDIDATE_RADIUS_RANGE_M
        self.max_waypoint_step_m = WaypointParams.MAX_WAYPOINT_STEP_M

    def load_parameters(self) -> None:
        """Declares and loads all ROS 2 parameters necessary for the node."""
        self.light_avoidance_enabled = bool(
            self.node.declare_parameter(
                'light_avoidance_enabled', LightAvoidanceParams.ENABLED
            ).value
        )

        self.node.declare_parameter('target_focus_distance_m', self.params.TARGET_FOCUS_DISTANCE_M)
        self.node.declare_parameter('min_focus_distance_m', self.params.MIN_FOCUS_DISTANCE_M)
        self.node.declare_parameter('constant_altitude_m', self.params.CONSTANT_ALTITUDE_M)
        self.node.declare_parameter('orbit_radius_m', self.params.ORBIT_RADIUS_M)
        self.node.declare_parameter('angular_velocity_rps', self.params.ANGULAR_VELOCITY_RPS)
        self.node.declare_parameter('look_duration_s', self.params.LOOK_DURATION_S)
        self.node.declare_parameter('waypoint_rate_hz', self.params.WAYPOINT_RATE_HZ)
        self.node.declare_parameter('num_initial_waypoints', self.params.NUM_INITIAL_WAYPOINTS)
        self.node.declare_parameter('image_width', self.params.IMAGE_WIDTH)
        self.node.declare_parameter('image_height', self.params.IMAGE_HEIGHT)
        self.node.declare_parameter('camera_fx', self.params.CAMERA_FX)
        self.node.declare_parameter('camera_fy', self.params.CAMERA_FY)
        self.node.declare_parameter('camera_cx', self.params.CAMERA_CX)
        self.node.declare_parameter('camera_cy', self.params.CAMERA_CY)
        self.node.declare_parameter('waypoints_topic', self.params.WAYPOINTS_TOPIC)
        self.node.declare_parameter('odom_topic', self.params.ODOM_TOPIC)
        self.node.declare_parameter('depth_topic', self.params.DEPTH_TOPIC)
        self.node.declare_parameter('center_topic', self.params.CENTER_TOPIC)
        self.node.declare_parameter('bbox_topic', self.params.BBOX_TOPIC)
        self.node.declare_parameter('dimensions_topic', self.params.DIMENSIONS_TOPIC)
        self.node.declare_parameter('light_vector_topic', self.params.LIGHT_VECTOR_TOPIC)
        self.node.declare_parameter('image_topic', self.params.IMAGE_TOPIC)
        self.node.declare_parameter('thirds_strength', UserRequestedParams.THIRDS_STRENGTH)
        self.node.declare_parameter(
            'thirds_use_nearest_anchor', UserRequestedParams.THIRDS_USE_NEAREST_ANCHOR
        )
        self.node.declare_parameter(
            'candidate_angle_window_rad', WaypointParams.CANDIDATE_ANGLE_WINDOW_RAD
        )
        self.node.declare_parameter(
            'candidate_angle_samples', WaypointParams.CANDIDATE_ANGLE_SAMPLES
        )
        self.node.declare_parameter(
            'candidate_radius_samples', WaypointParams.CANDIDATE_RADIUS_SAMPLES
        )
        self.node.declare_parameter(
            'candidate_radius_range_m', WaypointParams.CANDIDATE_RADIUS_RANGE_M
        )
        self.node.declare_parameter('max_waypoint_step_m', WaypointParams.MAX_WAYPOINT_STEP_M)

        self.params.TARGET_FOCUS_DISTANCE_M = float(
            self.node.get_parameter('target_focus_distance_m').value
        )
        self.params.MIN_FOCUS_DISTANCE_M = float(
            self.node.get_parameter('min_focus_distance_m').value
        )
        self.params.CONSTANT_ALTITUDE_M = float(
            self.node.get_parameter('constant_altitude_m').value
        )
        self.params.ORBIT_RADIUS_M = float(self.node.get_parameter('orbit_radius_m').value)
        self.params.ANGULAR_VELOCITY_RPS = float(
            self.node.get_parameter('angular_velocity_rps').value
        )
        self.params.LOOK_DURATION_S = float(self.node.get_parameter('look_duration_s').value)
        self.params.WAYPOINT_RATE_HZ = float(self.node.get_parameter('waypoint_rate_hz').value)
        self.params.NUM_INITIAL_WAYPOINTS = int(
            self.node.get_parameter('num_initial_waypoints').value
        )
        self.params.IMAGE_WIDTH = int(self.node.get_parameter('image_width').value)
        self.params.IMAGE_HEIGHT = int(self.node.get_parameter('image_height').value)
        self.params.CAMERA_FX = float(self.node.get_parameter('camera_fx').value)
        self.params.CAMERA_FY = float(self.node.get_parameter('camera_fy').value)
        self.params.CAMERA_CX = float(self.node.get_parameter('camera_cx').value)
        self.params.CAMERA_CY = float(self.node.get_parameter('camera_cy').value)
        self.params.WAYPOINTS_TOPIC = str(self.node.get_parameter('waypoints_topic').value)
        self.params.ODOM_TOPIC = str(self.node.get_parameter('odom_topic').value)
        self.params.DEPTH_TOPIC = str(self.node.get_parameter('depth_topic').value)
        self.params.CENTER_TOPIC = str(self.node.get_parameter('center_topic').value)
        self.params.BBOX_TOPIC = str(self.node.get_parameter('bbox_topic').value)
        self.params.DIMENSIONS_TOPIC = str(self.node.get_parameter('dimensions_topic').value)
        self.params.LIGHT_VECTOR_TOPIC = str(self.node.get_parameter('light_vector_topic').value)
        self.params.IMAGE_TOPIC = str(self.node.get_parameter('image_topic').value)
        self.thirds_strength = float(self.node.get_parameter('thirds_strength').value)
        self.thirds_use_nearest_anchor = bool(
            self.node.get_parameter('thirds_use_nearest_anchor').value
        )
        self.candidate_angle_window_rad = float(
            self.node.get_parameter('candidate_angle_window_rad').value
        )
        self.candidate_angle_samples = int(self.node.get_parameter('candidate_angle_samples').value)
        self.candidate_radius_samples = int(
            self.node.get_parameter('candidate_radius_samples').value
        )
        self.candidate_radius_range_m = [
            float(v) for v in self.node.get_parameter('candidate_radius_range_m').value
        ]
        self.max_waypoint_step_m = float(self.node.get_parameter('max_waypoint_step_m').value)
