import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, PolygonStamped
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2


class SimulatedObjectDetector(Node):
    def __init__(self):
        super().__init__('simulated_object_detector')
        
        self.bridge = CvBridge()
        
        self.declare_parameter('cx', 576.0)
        self.declare_parameter('cy', 324.0)
        self.declare_parameter('fx', 525.0)
        self.declare_parameter('fy', 525.0)
        self.declare_parameter('width', 1152)
        self.declare_parameter('height', 648)
        
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        
        self.declare_parameter('min_object_area', 500)
        self.declare_parameter('depth_threshold', 0.3)
        self.declare_parameter('max_detection_range', 10.0)
        
        self.min_area = self.get_parameter('min_object_area').value
        self.depth_thresh = self.get_parameter('depth_threshold').value
        self.max_range = self.get_parameter('max_detection_range').value
        
        self.depth_sub = self.create_subscription(
            Image,
            'depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.detection_pub = self.create_publisher(
            Float32MultiArray,
            'object_detections',
            10
        )

        self.center_pub = self.create_publisher(
            PolygonStamped,
            '/object_detection/center_point',
            10
        )

        self.bbox_pub = self.create_publisher(
            PolygonStamped,
            '/object_detection/bounding_box',
            10
        )

        self.dims_pub = self.create_publisher(
            PolygonStamped,
            '/object_detection/dimensions',
            10
        )
        
        self.get_logger().info('Simulated Object Detector initialized')
        self.get_logger().info(f'Camera: {self.width}x{self.height}, f=({self.fx:.2f}, {self.fy:.2f})')
        
    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            detections = self.detect_objects(depth_image)
            
            if detections:
                self.publish_detections(detections)
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')
    
    def detect_objects(self, depth_image):
        depth_clean = np.copy(depth_image)
        depth_clean[np.isnan(depth_clean)] = 0
        depth_clean[np.isinf(depth_clean)] = 0
        depth_clean[depth_clean > self.max_range] = 0
        
        depth_normalized = np.zeros_like(depth_clean, dtype=np.uint8)
        valid_mask = depth_clean > 0

        if np.any(valid_mask):
            depth_min = depth_clean[valid_mask].min()
            depth_max = depth_clean[valid_mask].max()
            if depth_max > depth_min:

                depth_normalized[valid_mask] = ((depth_clean[valid_mask] - depth_min) / 
                                                 (depth_max - depth_min) * 255).astype(np.uint8)
        
        kernel = np.ones((5, 5), np.uint8)
        depth_morph = cv2.morphologyEx(depth_normalized, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(depth_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area < self.min_area:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            
            center_x = x + w / 2
            center_y = y + h / 2
            
            mask = np.zeros(depth_clean.shape, dtype=np.uint8)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            object_depths = depth_clean[mask > 0]
            
            if len(object_depths) == 0:
                continue
            
            avg_depth = np.median(object_depths[object_depths > 0])
            
            if avg_depth <= 0 or avg_depth > self.max_range:
                continue
            
            frame_coverage = area / (self.width * self.height)
            
            norm_x = center_x / self.width
            norm_y = center_y / self.height
            
            thirds_x = min(abs(norm_x - 0.33), abs(norm_x - 0.5), abs(norm_x - 0.67))
            thirds_y = min(abs(norm_y - 0.33), abs(norm_y - 0.5), abs(norm_y - 0.67))
            
            centering_score = 1.0 - (thirds_x + thirds_y) / 2.0
            
            detections.append({
                'center_x': float(center_x),
                'center_y': float(center_y),
                'depth': float(avg_depth),
                'area_pixels': float(area),
                'frame_coverage': float(frame_coverage),
                'centering_score': float(centering_score),
                'normalized_x': float(norm_x),
                'normalized_y': float(norm_y),
                'bbox_x': float(x),
                'bbox_y': float(y),
                'bbox_w': float(w),
                'bbox_h': float(h),
            })
        
        return detections
    
    def publish_detections(self, detections):
        msg = Float32MultiArray()
        
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

        primary = max(detections, key=lambda d: d.get('area_pixels', 0.0))
        self._publish_polygons(primary)

    def _publish_polygons(self, det):
        now = self.get_clock().now().to_msg()

        center = PolygonStamped()
        center.header.stamp = now
        center.header.frame_id = "camera"
        p = Point32()
        p.x = float(det["center_x"])
        p.y = float(det["center_y"])
        p.z = float(det["depth"])
        center.polygon.points = [p]
        self.center_pub.publish(center)

        bbox = PolygonStamped()
        bbox.header.stamp = now
        bbox.header.frame_id = "camera"
        x = float(det["bbox_x"])
        y = float(det["bbox_y"])
        w = float(det["bbox_w"])
        h = float(det["bbox_h"])
        p0 = Point32()
        p0.x, p0.y, p0.z = x, y, 0.0
        p1 = Point32()
        p1.x, p1.y, p1.z = x + w, y + h, 0.0
        bbox.polygon.points = [p0, p1]
        self.bbox_pub.publish(bbox)

        dims = PolygonStamped()
        dims.header.stamp = now
        dims.header.frame_id = "camera"
        d = Point32()
        d.x = float(w)
        d.y = float(h)
        d.z = float(det["area_pixels"])
        dims.polygon.points = [d]
        self.dims_pub.publish(dims)


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
