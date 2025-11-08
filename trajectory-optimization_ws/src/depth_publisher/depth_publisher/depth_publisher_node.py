import sys
import os

user_site = os.path.expanduser('~/.local/lib/python3.12/site-packages')
if user_site in sys.path:
    sys.path.remove(user_site)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class DepthPublisherNode(Node):
    def __init__(self):
        super().__init__('depth_publisher_node')
        
        self.bridge = CvBridge()
        self.processing = False
        
        self.depth_model = None
        self.transform = None
        self.device = None
        
        self.initialize_depth_model()
        
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.depth_publisher = self.create_publisher(
            Image,
            '/depth/image_raw',
            10
        )
    
    def initialize_depth_model(self):
        try:
            venv_site = os.path.expanduser('~/bspline-nvidia-isaac-tsdf/venv/lib/python3.12/site-packages')
            depth_anything_path = os.path.expanduser('~/bspline-nvidia-isaac-tsdf/src/Depth-Anything')
            
            if venv_site not in sys.path:
                sys.path.insert(0, venv_site)
            if depth_anything_path not in sys.path:
                sys.path.insert(0, depth_anything_path)
            
            import torch
            import torch.nn.functional as F
            from torchvision.transforms import Compose
            from depth_anything.dpt import DepthAnything
            from depth_anything.util.transform import Resize, NormalizeImage, PrepareForNet
            
            torch._dynamo.config.suppress_errors = True
            torch.set_float32_matmul_precision('high')
            
            self.torch = torch
            self.F = F
            
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
            
            if self.device == 'cuda':
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True
                torch.backends.cudnn.benchmark = True
            
            original_dir = os.getcwd()
            os.chdir(depth_anything_path)
            try:
                self.depth_model = DepthAnything.from_pretrained('LiheYoung/depth_anything_vits14').to(self.device).eval()
            finally:
                os.chdir(original_dir)
            
            if hasattr(torch, 'compile'):
                self.depth_model = torch.compile(self.depth_model, mode='reduce-overhead')
            
            total_params = sum(param.numel() for param in self.depth_model.parameters())
            self.get_logger().info(f'Model loaded: {total_params / 1e6:.2f}M parameters')
            
            self.transform = Compose([
                Resize(
                    width=518,
                    height=518,
                    resize_target=False,
                    keep_aspect_ratio=True,
                    ensure_multiple_of=14,
                    resize_method='lower_bound',
                    image_interpolation_method=cv2.INTER_CUBIC,
                ),
                NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
                PrepareForNet(),
            ])
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.depth_model = None

    def image_callback(self, msg):
        if self.processing:
            self.get_logger().debug('Skipping frame')
            return
        if self.depth_model is None:
            self.get_logger().error('Depth failure!', throttle_duration_sec=5.0)
            return
            
        try:
            self.processing = True
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = cv_image.shape[:2]
            
            image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) / 255.0
            
            image_tensor = self.transform({'image': image_rgb})['image']
            image_tensor = self.torch.from_numpy(image_tensor).unsqueeze(0).to(self.device)
            
            with self.torch.no_grad():
                depth = self.depth_model(image_tensor)
            
            depth = self.F.interpolate(depth[None], (h, w), mode='bilinear', align_corners=False)[0, 0]
            
            depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
            depth = depth.cpu().numpy().astype(np.uint8)
            
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='mono8')
            depth_msg.header = msg.header
            depth_msg.header.frame_id = 'depth_camera'
            self.depth_publisher.publish(depth_msg)
                    
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.processing = False

def main(args=None):
    rclpy.init(args=args)
    node = DepthPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()