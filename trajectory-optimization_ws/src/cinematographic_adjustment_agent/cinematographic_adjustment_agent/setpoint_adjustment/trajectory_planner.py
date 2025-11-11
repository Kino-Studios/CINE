import math
import numpy as np
from typing import List, Tuple, Optional, Callable
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PolygonStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Header
from cv_bridge import CvBridge
from cost_calculator import CostCalculator

class OrbitWaypointGenerator(Node):
    def __init__(self):
        super().__init__('orbit_waypoint_generator')
        
        self.bridge = CvBridge()
        self.camera_info = None
        self.current_pose = None
        self.depth_image = None
        self.waypoints = []
        self.waypoint_index = 0
        self.publishing_active = False
        
        self.cost_calculator = CostCalculator(self)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/vio/pose', self.pose_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.bbox_sub = self.create_subscription(
            PolygonStamped, '/tracker/bounding_box', self.bbox_callback, 10)
        self.center_sub = self.create_subscription(
            PolygonStamped, '/tracker/center_point', self.center_callback, 10)
        self.dims_sub = self.create_subscription(
            PolygonStamped, '/tracker/dimensions', self.dims_callback, 10)
        
        self.setpoint_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)
        
        self.timer = self.create_timer(0.1, self.publish_waypoint)
        
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('angular_velocity', 0.1)
        self.declare_parameter('duration', 60.0)
        self.declare_parameter('waypoint_rate', 10.0)
        self.declare_parameter('optimize_cost', True)
        self.declare_parameter('altitude', 1.5)
        
    def camera_info_callback(self, msg):
        self.camera_info = msg
        
    def pose_callback(self, msg):
        self.current_pose = msg
        
        if not self.publishing_active and self.camera_info is not None:
            self.generate_waypoints()
            self.publishing_active = True
        
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if self.cost_calculator:
            self.cost_calculator.update_depth_image(self.depth_image)
    
    def bbox_callback(self, msg):
        if self.cost_calculator:
            self.cost_calculator.update_bounding_box(msg)
    
    def center_callback(self, msg):
        if self.cost_calculator:
            self.cost_calculator.update_center_point(msg)
    
    def dims_callback(self, msg):
        if self.cost_calculator:
            self.cost_calculator.update_dimensions(msg)
    
    def get_depth_at_pixel(self, u: int, v: int) -> Optional[float]:
        if self.depth_image is None:
            return None
        h, w = self.depth_image.shape[:2]
        if 0 <= u < w and 0 <= v < h:
            depth = self.depth_image[v, u]
            if depth > 0:
                return float(depth)
        return None
    
    def get_camera_intrinsics(self) -> Tuple[float, float, float, float]:
        if self.camera_info is None:
            return 458.654, 457.296, 320.0, 240.0
        K = self.camera_info.k
        return K[0], K[4], K[2], K[5]
    
    def get_current_position(self) -> Tuple[float, float, float]:
        if self.current_pose is None:
            return 0.0, 0.0, 1.5
        p = self.current_pose.pose.position
        return p.x, p.y, p.z
    
    def evaluate_cost_at_position(self, x: float, y: float, z: float) -> Optional[float]:
        if self.camera_info is None or self.current_pose is None:
            return None
        
        if not hasattr(self, 'cost_calculator') or self.cost_calculator is None:
            return None
        
        if self.cost_calculator.center_point is None or len(self.cost_calculator.center_point.polygon.points) == 0:
            return None
        
        center_point = self.cost_calculator.center_point.polygon.points[0]
        target_world_x = center_point.x if hasattr(center_point, 'x') else 0.0
        target_world_y = center_point.y if hasattr(center_point, 'y') else 0.0
        target_world_z = center_point.z if hasattr(center_point, 'z') else z
        
        cost = self.cost_calculator.estimate_cost_at_position(
            x, y, z,
            target_world_x, target_world_y, target_world_z
        )
        
        return cost
    
    def generate_waypoints(self):
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        radius = self.get_parameter('radius').value
        angular_velocity = self.get_parameter('angular_velocity').value
        duration = self.get_parameter('duration').value
        waypoint_rate = self.get_parameter('waypoint_rate').value
        optimize = self.get_parameter('optimize_cost').value
        altitude = self.get_parameter('altitude').value
        
        center = (center_x, center_y)
        
        if optimize:
            center, radius = optimize_orbit_center_and_radius(
                center, radius, altitude, self.evaluate_cost_at_position)
            
            self.waypoints = generate_cost_optimized_orbit_waypoints(
                center, radius, angular_velocity, duration, waypoint_rate,
                altitude, self.evaluate_cost_at_position)
        else:
            self.waypoints = generate_orbit_waypoints_xy(
                center, radius, angular_velocity, duration, waypoint_rate, altitude)
        
        self.waypoint_index = 0
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints')
    
    def publish_waypoint(self):
        if not self.publishing_active or len(self.waypoints) == 0:
            return
        
        if self.waypoint_index >= len(self.waypoints):
            self.waypoint_index = 0
        
        x, y, z, dt = self.waypoints[self.waypoint_index]
        
        msg = PositionTarget()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        yaw = math.atan2(center_y - y, center_x - x)
        msg.yaw = yaw
        
        self.setpoint_pub.publish(msg)
        
        self.waypoint_index += 1

def generate_orbit_waypoints_xy(
    center_xy_m: Tuple[float, float],
    radius_m: float,
    angular_velocity_rps: float,
    duration_s: float,
    waypoint_rate_hz: float,
    altitude_m: float,
) -> List[Tuple[float, float, float, float]]:
    cx, cy = center_xy_m
    num_points = max(1, int(duration_s * waypoint_rate_hz))
    dt = 1.0 / waypoint_rate_hz
    waypoints = []
    theta = 0.0

    for _ in range(num_points):
        x = cx + radius_m * math.cos(theta)
        y = cy + radius_m * math.sin(theta)
        waypoints.append((x, y, altitude_m, dt))
        theta += 2.0 * math.pi * angular_velocity_rps * dt
    return waypoints

def generate_cost_optimized_orbit_waypoints(
    center_xy_m: Tuple[float, float],
    initial_radius_m: float,
    angular_velocity_rps: float,
    duration_s: float,
    waypoint_rate_hz: float,
    altitude_m: float,
    cost_function: Callable[[float, float, float], Optional[float]],
    num_radius_samples: int = 5,
    radius_range: Tuple[float, float] = (0.8, 2.0),
) -> List[Tuple[float, float, float, float]]:

    cx, cy = center_xy_m
    num_points = max(1, int(duration_s * waypoint_rate_hz))
    dt = 1.0 / waypoint_rate_hz
    waypoints = []
    
    theta = 0.0
    current_radius = initial_radius_m
    
    for i in range(num_points):
        radius_candidates = np.linspace(
            max(radius_range[0], current_radius * 0.7),
            min(radius_range[1], current_radius * 1.3),
            num_radius_samples
        )
        
        best_radius = current_radius
        best_cost = float('inf')
        
        for test_radius in radius_candidates:
            test_x = cx + test_radius * math.cos(theta)
            test_y = cy + test_radius * math.sin(theta)
            
            cost = cost_function(test_x, test_y, altitude_m)
            if cost is not None and cost < best_cost:
                best_cost = cost
                best_radius = test_radius
        
        current_radius = best_radius
        x = cx + current_radius * math.cos(theta)
        y = cy + current_radius * math.sin(theta)
        waypoints.append((x, y, altitude_m, dt))
        
        theta += 2.0 * math.pi * angular_velocity_rps * dt
    
    return waypoints

def optimize_orbit_center_and_radius(
    initial_center_xy: Tuple[float, float],
    initial_radius: float,
    altitude_m: float,
    cost_function: Callable[[float, float, float], Optional[float]],
    search_radius: float = 0.5,
    num_samples: int = 9,
) -> Tuple[Tuple[float, float], float]:
    best_center = initial_center_xy
    best_radius = initial_radius
    best_cost = float('inf')
    
    cx, cy = initial_center_xy
    x_samples = np.linspace(cx - search_radius, cx + search_radius, int(np.sqrt(num_samples)))
    y_samples = np.linspace(cy - search_radius, cy + search_radius, int(np.sqrt(num_samples)))
    
    radius_candidates = np.linspace(max(0.5, initial_radius * 0.7), initial_radius * 1.3, 5)
    
    for test_x in x_samples:
        for test_y in y_samples:
            for test_radius in radius_candidates:
                cost = cost_function(test_x, test_y, altitude_m)
                
                if cost is not None and cost < best_cost:
                    best_cost = cost
                    best_center = (float(test_x), float(test_y))
                    best_radius = float(test_radius)
    
    return best_center, best_radius

def pixel_to_camera_xy(
    u: float,
    v: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Tuple[float, float]:
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return float(x), float(y)

def clamp_depth_to_preference(depth_m: float | None, preferred_m: float, min_m: float) -> float:
    if depth_m is None:
        return max(preferred_m, min_m)
    return max(depth_m, min_m)

def main(args=None):
    rclpy.init(args=args)
    generator = OrbitWaypointGenerator()
    
    rclpy.spin(generator)
    
    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()