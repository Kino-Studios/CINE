import math
import sys
import os
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import Image

workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))))
constants_path = os.path.join(workspace_root, 'configs', 'user_params')
if constants_path not in sys.path:
    sys.path.insert(0, constants_path)

from constants import ROSParams, UserRequestedParams, ImageParams

class CostFunctionCalculator(Node):
    def __init__(self):
        super().__init__('cost_function_calculator')
        
        self.bridge = CvBridge()
        
        self.bounding_box = None
        self.dimensions = None
        self.center_point = None
        self.depth_image = None
        self.center_depth = None
        
        self.declare_parameter('focus_object_id', UserRequestedParams.FOCUS_OBJECT_ID)
        self.declare_parameter('huber_delta_px', 6.0)
        self.declare_parameter('deadband_px', 4.0)
        self.declare_parameter('w_framing', 0.056)
        self.declare_parameter('w_dist',    4.0)
        self.declare_parameter('w_near',    25.0)
        self.declare_parameter('w_stab',    0.2)
        
        self.focus_object_id = self.get_parameter('focus_object_id').value

        self.bounding_box_subscriber = self.create_subscription(
            PolygonStamped,
            '/object_detection/bounding_box',
            self.bounding_box_callback,
            qos_profile=ROSParams.QOS_PROFILE
        )

        self.dimensions_subscriber = self.create_subscription(
            PolygonStamped,
            '/object_detection/dimensions',
            self.dimensions_callback,
            qos_profile=ROSParams.QOS_PROFILE
        )

        self.center_point_subscriber = self.create_subscription(
            PolygonStamped,
            '/object_detection/center_point',
            self.center_point_callback,
            qos_profile=ROSParams.QOS_PROFILE
        )
        
        self.depth_subscriber = self.create_subscription(
            Image,
            '/depth/image_raw',
            self.depth_callback,
            qos_profile=ROSParams.QOS_PROFILE
        )
    
    def bounding_box_callback(self, msg):
        self.bounding_box = msg
        self._update_center_depth()
    
    def dimensions_callback(self, msg):
        self.dimensions = msg
    
    def center_point_callback(self, msg):
        self.center_point = msg
        self._update_center_depth()
    
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self._update_center_depth()
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
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

    def _calculate_cost_function(self):
        if (self.bounding_box is None or 
            self.dimensions is None or 
            self.center_point is None or 
            self.depth_image is None or
            self.center_depth is None
        ):
            return None

        u_star = ImageParams.IMAGE_WIDTH * UserRequestedParams.THIRDS_ANCHOR[0]
        v_star = ImageParams.IMAGE_HEIGHT * UserRequestedParams.THIRDS_ANCHOR[1]
        
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
        
        deadband = float(self.get_parameter('deadband_px').value)
        delta_px = float(self.get_parameter('huber_delta_px').value)
        
        du = u_current - u_star
        du = 0.0 if abs(du) < deadband else du
        
        dv = v_current - v_star
        dv = 0.0 if abs(dv) < deadband else dv
        
        w_framing = float(self.get_parameter('w_framing').value)
        framing_cost = w_framing * (self.huber_function(du, delta_px) + self.huber_function(dv, delta_px))

        # calculate the depth cost
        d_star = UserRequestedParams.DESIRED_FOCUS_DISTANCE_M
        d_min = UserRequestedParams.FOCUS_DISTANCE_MIN_M

        Z = float(self.get_depth_at_point())
        
        J_dist = (Z - d_star) ** 2
        J_near = math.pow(d_min - Z, 2) if Z < d_min else 0.0
        if not hasattr(self, "_prev_Z"):
            self._prev_Z = Z
        J_stab = math.pow(Z - self._prev_Z, 2)
        self._prev_Z = Z

        w_dist = float(self.get_parameter('w_dist').value)
        w_near = float(self.get_parameter('w_near').value)
        w_stab = float(self.get_parameter('w_stab').value)
        depth_cost = (w_dist * J_dist) + (w_near * J_near) + (w_stab * J_stab)

        # total cost function
        cost = (depth_cost + framing_cost);
        
        return cost

def main(args=None):
    rclpy.init(args=args)
    node = CostFunctionCalculator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()