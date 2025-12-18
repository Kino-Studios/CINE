#!/usr/bin/env python3
"""
Simulated Object Detector for Cinematographic Analysis

Detects objects in the depth image and provides metrics for cinematographic quality:
- Object position in frame (for rule of thirds)
- Object distance and size
- Whether objects are centered/framed well
- Number of objects in view
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2


class SimulatedObjectDetector(Node):
    def __init__(self):
        super().__init__('simulated_object_detector')
        
        self.bridge = CvBridge()
        
        # Camera parameters (from camera.yaml)
        self.declare_parameter('cx', 320.5)
        self.declare_parameter('cy', 240.5)
        self.declare_parameter('fx', 387.229248046875)
        self.declare_parameter('fy', 387.229248046875)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        
        # Detection parameters
        self.declare_parameter('min_object_area', 500)  # Minimum pixels for object
        self.declare_parameter('depth_threshold', 0.3)  # Depth difference threshold (m)
        self.declare_parameter('max_detection_range', 10.0)  # Maximum detection distance (m)
        
        self.min_area = self.get_parameter('min_object_area').value
        self.depth_thresh = self.get_parameter('depth_threshold').value
        self.max_range = self.get_parameter('max_detection_range').value
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            'depth/image_raw',
            self.depth_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Float32MultiArray,
            'object_detections',
            10
        )
        
        self.get_logger().info('Simulated Object Detector initialized')
        self.get_logger().info(f'Camera: {self.width}x{self.height}, f=({self.fx:.2f}, {self.fy:.2f})')
        
    def depth_callback(self, msg):
        """Process depth image and detect objects"""
        try:
            # Convert ROS Image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Detect objects using depth segmentation
            detections = self.detect_objects(depth_image)
            
            if detections:
                # Publish detections
                self.publish_detections(detections)
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')
    
    def detect_objects(self, depth_image):
        """
        Detect objects in depth image using clustering
        
        Returns list of detections:
        [center_x, center_y, depth, area_pixels, frame_coverage, centering_score]
        """
        # Clean up depth image
        depth_clean = np.copy(depth_image)
        depth_clean[np.isnan(depth_clean)] = 0
        depth_clean[np.isinf(depth_clean)] = 0
        depth_clean[depth_clean > self.max_range] = 0
        
        # Convert to 8-bit for OpenCV processing
        depth_normalized = np.zeros_like(depth_clean, dtype=np.uint8)
        valid_mask = depth_clean > 0
        if np.any(valid_mask):
            depth_min = depth_clean[valid_mask].min()
            depth_max = depth_clean[valid_mask].max()
            if depth_max > depth_min:
                depth_normalized[valid_mask] = ((depth_clean[valid_mask] - depth_min) / 
                                                 (depth_max - depth_min) * 255).astype(np.uint8)
        
        # Apply morphological operations to group nearby pixels
        kernel = np.ones((5, 5), np.uint8)
        depth_morph = cv2.morphologyEx(depth_normalized, cv2.MORPH_CLOSE, kernel)
        
        # Find contours (objects)
        contours, _ = cv2.findContours(depth_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area < self.min_area:
                continue
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate center
            center_x = x + w / 2
            center_y = y + h / 2
            
            # Get average depth of object
            mask = np.zeros(depth_clean.shape, dtype=np.uint8)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            object_depths = depth_clean[mask > 0]
            
            if len(object_depths) == 0:
                continue
            
            avg_depth = np.median(object_depths[object_depths > 0])
            
            if avg_depth <= 0 or avg_depth > self.max_range:
                continue
            
            # Calculate cinematographic metrics
            
            # 1. Frame coverage (what % of frame does object occupy)
            frame_coverage = area / (self.width * self.height)
            
            # 2. Centering score (0-1, where 1 is perfectly centered)
            # Based on rule of thirds: objects at 1/3 or 2/3 positions are good
            norm_x = center_x / self.width
            norm_y = center_y / self.height
            
            # Distance from rule of thirds positions (0.33, 0.5, 0.67)
            thirds_x = min(abs(norm_x - 0.33), abs(norm_x - 0.5), abs(norm_x - 0.67))
            thirds_y = min(abs(norm_y - 0.33), abs(norm_y - 0.5), abs(norm_y - 0.67))
            
            # Centering score: lower distance = better composition
            centering_score = 1.0 - (thirds_x + thirds_y) / 2.0
            
            detections.append({
                'center_x': float(center_x),
                'center_y': float(center_y),
                'depth': float(avg_depth),
                'area_pixels': float(area),
                'frame_coverage': float(frame_coverage),
                'centering_score': float(centering_score),
                'normalized_x': float(norm_x),
                'normalized_y': float(norm_y)
            })
        
        return detections
    
    def publish_detections(self, detections):
        """Publish detection data"""
        msg = Float32MultiArray()
        
        # Format: [num_objects, obj1_data..., obj2_data..., ...]
        # Each object: [center_x, center_y, depth, area, coverage, centering_score, norm_x, norm_y]
        
        data = [float(len(detections))]
        
        for det in detections:
            data.extend([
                det['center_x'],
                det['center_y'],
                det['depth'],
                det['area_pixels'],
                det['frame_coverage'],
                det['centering_score'],
                det['normalized_x'],
                det['normalized_y']
            ])
        
        msg.data = data
        self.detection_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    detector = SimulatedObjectDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
