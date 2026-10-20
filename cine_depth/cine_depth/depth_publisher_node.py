import os
import sys
from pathlib import Path

# Add repo root to sys.path to find 'configs'
here = Path(__file__).resolve()
for parent in [here] + list(here.parents):
    if (parent / 'configs').is_dir():
        python_constants_path = parent / 'configs' / 'python-constants'
        if str(python_constants_path) not in sys.path:
            sys.path.insert(0, str(python_constants_path))
        break

user_site = os.path.expanduser('~/.local/lib/python3.12/site-packages')
if user_site in sys.path:
    sys.path.remove(user_site)

import cv2
import numpy as np
import rclpy
from constants import DepthAnythingParams
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthPublisherNode(Node):
    def __init__(self):
        super().__init__('depth_publisher_node')

        self.bridge = CvBridge()
        self.processing = False

        self.depth_model = None
        self.transform = None
        self.device = None

        self.declare_parameter('depth_encoding', '32FC1')
        self.declare_parameter('depth_scale', 1.0)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('depth_anything_path', '')
        self.declare_parameter('venv_site_path', '')

        self.depth_encoding = self.get_parameter('depth_encoding').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.max_depth = self.get_parameter('max_depth').value

        self.initialize_depth_model()

        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.depth_publisher = self.create_publisher(Image, '/depth/image_raw', 10)

    def initialize_depth_model(self):
        try:
            configured_depth_anything = str(self.get_parameter('depth_anything_path').value).strip()
            configured_venv_site = str(self.get_parameter('venv_site_path').value).strip()

            repo_root = None
            here = Path(__file__).resolve()
            for parent in [here] + list(here.parents):
                if (parent / 'src' / 'Depth-Anything').is_dir() and (parent / 'venv').is_dir():
                    repo_root = parent
                    break

            py_ver = f'python{sys.version_info.major}.{sys.version_info.minor}'
            default_depth_anything = (repo_root / 'src' / 'Depth-Anything') if repo_root else None
            default_venv_site = (
                (repo_root / 'venv' / 'lib' / py_ver / 'site-packages') if repo_root else None
            )

            depth_anything_path = (
                configured_depth_anything
                or DepthAnythingParams.DEPTH_ANYTHING_PATH
                or (str(default_depth_anything) if default_depth_anything else '')
            )
            venv_site = (
                configured_venv_site
                or DepthAnythingParams.VENV_SITE_PATH
                or (str(default_venv_site) if default_venv_site else '')
            )

            if not depth_anything_path:
                raise RuntimeError(
                    'Depth-Anything path not set; provide ROS param `depth_anything_path` or set it in configs/json_constants/user_params/params.json.'
                )
            if not venv_site:
                raise RuntimeError(
                    'Venv site-packages path not set; provide ROS param `venv_site_path` or set it in configs/json_constants/user_params/params.json.'
                )

            if venv_site not in sys.path:
                sys.path.insert(0, venv_site)
            if depth_anything_path not in sys.path:
                sys.path.insert(0, depth_anything_path)

            import torch
            import torch.nn.functional as F
            from depth_anything.dpt import DepthAnything
            from depth_anything.util.transform import NormalizeImage, PrepareForNet, Resize
            from torchvision.transforms import Compose

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
                self.depth_model = (
                    DepthAnything.from_pretrained('LiheYoung/depth_anything_vits14')
                    .to(self.device)
                    .eval()
                )
            finally:
                os.chdir(original_dir)

            if hasattr(torch, 'compile'):
                self.depth_model = torch.compile(self.depth_model, mode='reduce-overhead')

            total_params = sum(param.numel() for param in self.depth_model.parameters())
            self.get_logger().info(f'Model loaded: {total_params / 1e6:.2f}M parameters')

            self.transform = Compose(
                [
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
                ]
            )

        except Exception as e:
            self.get_logger().error(f'Failed to initialize: {str(e)}')
            import traceback

            self.get_logger().error(traceback.format_exc())
            self.depth_model = None

    def image_callback(self, msg):
        self.get_logger().info('Received image callback')
        if self.processing:
            self.get_logger().debug('Skipping frame')

            return
        if self.depth_model is None:
            self.get_logger().error('Depth failure!')

            return

        try:
            self.processing = True
            self.get_logger().info(f'Processing image {msg.width}x{msg.height}')

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = cv_image.shape[:2]

            image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) / 255.0

            image_tensor = self.transform({'image': image_rgb})['image']
            image_tensor = self.torch.from_numpy(image_tensor).unsqueeze(0).to(self.device)

            with self.torch.no_grad():
                depth = self.depth_model(image_tensor)

            if len(depth.shape) == 2:
                depth = depth.unsqueeze(0).unsqueeze(0)
            elif len(depth.shape) == 3:
                depth = depth.unsqueeze(0)

            depth = self.F.interpolate(depth, size=(h, w), mode='bilinear', align_corners=False)
            depth = depth[0, 0]

            depth_np = depth.cpu().numpy()

            if depth_np.shape != (h, w):
                self.get_logger().warn(f'Shape mismatch: ({h}, {w}) got {depth_np.shape}')
                depth_np = cv2.resize(depth_np, (w, h), interpolation=cv2.INTER_LINEAR)

            if self.depth_encoding == '32FC1':
                depth_min = depth_np.min()
                depth_max = depth_np.max()

                if depth_max > depth_min:
                    depth_normalized = (depth_np - depth_min) / (depth_max - depth_min)
                    depth_meters = depth_normalized * self.max_depth * self.depth_scale
                else:
                    depth_meters = np.zeros_like(depth_np)

                depth_meters = depth_meters.astype(np.float32)

                depth_msg = self.bridge.cv2_to_imgmsg(depth_meters, encoding='32FC1')
            else:
                depth_normalized = (depth_np - depth_np.min()) / (
                    depth_np.max() - depth_np.min() + 1e-8
                )
                depth_uint8 = (depth_normalized * 255.0).astype(np.uint8)
                depth_msg = self.bridge.cv2_to_imgmsg(depth_uint8, encoding='mono8')

            depth_msg.header = msg.header
            depth_msg.header.frame_id = 'depth_camera'
            self.depth_publisher.publish(depth_msg)

            self.get_logger().debug('Published depth image')

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
