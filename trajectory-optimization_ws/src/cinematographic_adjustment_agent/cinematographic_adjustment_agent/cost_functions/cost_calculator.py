import math
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import Image

workspace_root = None
constants_path = None

def set_constants_path(path):
    global constants_path
    constants_path = path

def get_constants():
    import sys
    if constants_path and constants_path not in sys.path:
        sys.path.insert(0, constants_path)
    from constants import ROSParams, UserRequestedParams, ImageParams
    return ROSParams, UserRequestedParams, ImageParams

class CostCalculator:
    def __init__(self, node, constants_path=None):
        if constants_path:
            set_constants_path(constants_path)
        
        ROSParams, UserRequestedParams, ImageParams = get_constants()
        
        self.node = node
        self.bridge = CvBridge()
        self.bounding_box = None
        self.dimensions = None
        self.center_point = None
        self.depth_image = None
        self.center_depth = None
        
        self.ROSParams = ROSParams
        self.UserRequestedParams = UserRequestedParams
        self.ImageParams = ImageParams
        
        node.declare_parameter('focus_object_id', UserRequestedParams.FOCUS_OBJECT_ID)
        node.declare_parameter('huber_delta_px', 6.0)
        node.declare_parameter('deadband_px', 4.0)
        node.declare_parameter('w_framing', 0.056)
        node.declare_parameter('w_dist', 4.0)
        node.declare_parameter('w_near', 25.0)
        node.declare_parameter('w_stab', 0.2)
    
    def update_bounding_box(self, msg: PolygonStamped):
        self.bounding_box = msg
        self._update_center_depth()
    
    def update_dimensions(self, msg: PolygonStamped):
        self.dimensions = msg
    
    def update_center_point(self, msg: PolygonStamped):
        self.center_point = msg
        self._update_center_depth()
    
    def update_depth_image(self, depth_image):
        self.depth_image = depth_image
        self._update_center_depth()
    
    def _update_center_depth(self):
        if self.depth_image is None:
            return
        
        center_x, center_y = None, None
        
        if self.center_point is not None and len(self.center_point.polygon.points) > 0:
            center_x = int(self.center_point.polygon.points[0].x)
            center_y = int(self.center_point.polygon.points[0].y)
        elif self.bounding_box is not None and len(self.bounding_box.polygon.points) > 0:
            points = self.bounding_box.polygon.points
            if len(points) >= 2:
                x_coords = [p.x for p in points]
                y_coords = [p.y for p in points]
                center_x = int(np.mean(x_coords))
                center_y = int(np.mean(y_coords))
        
        if center_x is not None and center_y is not None:
            h, w = self.depth_image.shape[:2]
            center_x = max(0, min(center_x, w - 1))
            center_y = max(0, min(center_y, h - 1))
            
            depth_value = self.depth_image[center_y, center_x]
            
            if depth_value > 0 and not np.isnan(depth_value):
                self.center_depth = float(depth_value)
            else:
                self.center_depth = None
    
    def get_depth_at_point(self, x, y):
        if self.depth_image is None:
            return None
        
        h, w = self.depth_image.shape[:2]
        
        x = int(max(0, min(x, w - 1)))
        y = int(max(0, min(y, h - 1)))
        
        depth_value = self.depth_image[y, x]
        
        if depth_value > 0 and not np.isnan(depth_value):
            return float(depth_value)
        else:
            return None
    
    def get_median_depth_roi(self):
        if self.depth_image is None or self.bounding_box is None:
            return None
        
        if len(self.bounding_box.polygon.points) < 2:
            return None
        
        h, w = self.depth_image.shape[:2]
        
        points = self.bounding_box.polygon.points
        x_coords = [p.x for p in points]
        y_coords = [p.y for p in points]
        
        x_min = int(max(0, min(x_coords)))
        x_max = int(min(w - 1, max(x_coords)))
        y_min = int(max(0, min(y_coords)))
        y_max = int(min(h - 1, max(y_coords)))
        
        if x_min >= x_max or y_min >= y_max:
            return None
        
        depth_region = self.depth_image[y_min:y_max+1, x_min:x_max+1]
        
        valid_depths = depth_region[(depth_region > 0) & ~np.isnan(depth_region)]
        
        if len(valid_depths) == 0:
            return None
        
        median_depth = float(np.median(valid_depths))
        
        return median_depth
    
    def huber_function(self, r, delta=6.0):
        a = abs(r)
        return 0.5 * math.pow(a, 2) if a <= delta else delta * (a - 0.5 * delta)
    
    def calculate_cost(self):
        if (self.bounding_box is None or 
            self.dimensions is None or 
            self.center_point is None or 
            self.depth_image is None or
            self.center_depth is None
        ):
            return None

        u_star = self.ImageParams.IMAGE_WIDTH * self.UserRequestedParams.THIRDS_ANCHOR[0]
        v_star = self.ImageParams.IMAGE_HEIGHT * self.UserRequestedParams.THIRDS_ANCHOR[1]
        
        if len(self.center_point.polygon.points) > 0:
            u_current = self.center_point.polygon.points[0].x
            v_current = self.center_point.polygon.points[0].y
        elif len(self.bounding_box.polygon.points) > 0:
            points = self.bounding_box.polygon.points
            if len(points) >= 2:
                x_coords = [p.x for p in points]
                y_coords = [p.y for p in points]
                u_current = np.mean(x_coords)
                v_current = np.mean(y_coords)
            else:
                return None
        else:
            return None
        
        deadband = float(self.node.get_parameter('deadband_px').value)
        delta_px = float(self.node.get_parameter('huber_delta_px').value)
        
        du = u_current - u_star
        du = 0.0 if abs(du) < deadband else du
        
        dv = v_current - v_star
        dv = 0.0 if abs(dv) < deadband else dv
        
        w_framing = float(self.node.get_parameter('w_framing').value)
        framing_cost = w_framing * (self.huber_function(du, delta_px) + self.huber_function(dv, delta_px))

        d_star = self.UserRequestedParams.DESIRED_FOCUS_DISTANCE_M
        d_min = self.UserRequestedParams.FOCUS_DISTANCE_MIN_M

        Z = self.get_median_depth_roi()
        if Z is None:
            return None
        Z = float(Z)
        
        J_dist = math.pow(Z - d_star, 2)
        J_near = math.pow(d_min - Z, 2) if Z < d_min else 0.0
        if not hasattr(self, "_prev_Z"):
            self._prev_Z = Z
        J_stab = math.pow(Z - self._prev_Z, 2)
        self._prev_Z = Z

        w_dist = float(self.node.get_parameter('w_dist').value)
        w_near = float(self.node.get_parameter('w_near').value)
        w_stab = float(self.node.get_parameter('w_stab').value)
        depth_cost = (w_dist * J_dist) + (w_near * J_near) + (w_stab * J_stab)

        cost = (depth_cost + framing_cost)
        
        return cost
    
    def estimate_cost_at_position(self, drone_x: float, drone_y: float, drone_z: float, 
                                  target_world_x: float, target_world_y: float, target_world_z: float) -> Optional[float]:
        """
        Estimate cost if drone were at position (drone_x, drone_y, drone_z) looking at target.
        
        This is a simplified estimation that uses:
        1. Distance to target (depth cost)
        2. Estimated framing based on geometry
        
        Args:
            drone_x, drone_y, drone_z: Hypothetical drone position in world frame
            target_world_x, target_world_y, target_world_z: Target position in world frame
            
        Returns:
            Estimated cost or None if insufficient data
        """
        if self.depth_image is None or self.center_point is None:
            return None
        
        # Calculate distance to target
        dx = target_world_x - drone_x
        dy = target_world_y - drone_y
        dz = target_world_z - drone_z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Depth cost
        d_star = self.UserRequestedParams.DESIRED_FOCUS_DISTANCE_M
        d_min = self.UserRequestedParams.FOCUS_DISTANCE_MIN_M
        
        J_dist = math.pow(distance - d_star, 2)
        J_near = math.pow(d_min - distance, 2) if distance < d_min else 0.0
        
        # Stability cost (use current depth as reference)
        if hasattr(self, "_prev_Z") and self._prev_Z is not None:
            J_stab = 0.0
        else:
            prev_Z = getattr(self, "_prev_Z", distance)
            J_stab = math.pow(distance - prev_Z, 2)
        
        w_dist = float(self.node.get_parameter('w_dist').value)
        w_near = float(self.node.get_parameter('w_near').value)
        w_stab = float(self.node.get_parameter('w_stab').value)
        depth_cost = (w_dist * J_dist) + (w_near * J_near) + (w_stab * J_stab)
        
        # add proper object estimation later
        fx = self.ImageParams.IMAGE_WIDTH / 2.0
        fy = self.ImageParams.IMAGE_HEIGHT / 2.0
        
        if len(self.center_point.polygon.points) > 0:
            u_current = self.center_point.polygon.points[0].x
            v_current = self.center_point.polygon.points[0].y
        else:
            u_current = self.ImageParams.IMAGE_WIDTH / 2.0
            v_current = self.ImageParams.IMAGE_HEIGHT / 2.0
        
        u_star = self.ImageParams.IMAGE_WIDTH * self.UserRequestedParams.THIRDS_ANCHOR[0]
        v_star = self.ImageParams.IMAGE_HEIGHT * self.UserRequestedParams.THIRDS_ANCHOR[1]
        
        du_est = u_current - u_star
        dv_est = v_current - v_star
        
        deadband = float(self.node.get_parameter('deadband_px').value)
        delta_px = float(self.node.get_parameter('huber_delta_px').value)
        
        du_est = 0.0 if abs(du_est) < deadband else du_est
        dv_est = 0.0 if abs(dv_est) < deadband else dv_est
        
        w_framing = float(self.node.get_parameter('w_framing').value)
        framing_cost = w_framing * (self.huber_function(du_est, delta_px) + self.huber_function(dv_est, delta_px))
        
        cost = depth_cost + framing_cost
        return cost

