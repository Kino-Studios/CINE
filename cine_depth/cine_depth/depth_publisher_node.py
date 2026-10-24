import sys
import types

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

import cine_config
from cine_depth.depth_scaling import metric_depth_metres, relative_depth_metres

# Apache-2.0, like DA3-SMALL and DA3-BASE; the larger non-metric DA3 models are CC BY-NC 4.0.
DEFAULT_MODEL_ID = 'depth-anything/DA3METRIC-LARGE'


def _import_depth_anything_3():
    # depth_anything_3.api imports its point-cloud and COLMAP export helpers at load time,
    # and those need pycolmap, which has no wheel for Linux on ARM (Jetson, Raspberry Pi).
    # The node never exports, so an empty stand-in takes the export package's place.
    export_stand_in = types.ModuleType('depth_anything_3.utils.export')

    def export(*args, **kwargs):
        raise RuntimeError('cine_depth does not install the Depth Anything 3 exporters')

    export_stand_in.export = export
    sys.modules.setdefault('depth_anything_3.utils.export', export_stand_in)
    from depth_anything_3.api import DepthAnything3

    return DepthAnything3


class DepthPublisherNode(Node):
    def __init__(self):
        super().__init__('depth_publisher_node')

        self.bridge = CvBridge()
        self.processing = False

        self.depth_model = None
        self.device = None

        self.declare_parameter('model_id', DEFAULT_MODEL_ID)
        self.declare_parameter('process_resolution', 504)
        self.declare_parameter('depth_encoding', '32FC1')
        self.declare_parameter('depth_scale', 1.0)
        self.declare_parameter('max_depth', 10.0)

        self.model_id = self.get_parameter('model_id').value
        self.process_resolution = int(self.get_parameter('process_resolution').value)
        self.depth_encoding = self.get_parameter('depth_encoding').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.max_depth = self.get_parameter('max_depth').value
        # Only DA3METRIC models output depth that converts to metres with the focal length.
        self.model_is_metric = 'METRIC' in self.model_id.upper()

        camera = cine_config.load('camera/camera-params')['camera_params']
        self.camera_focal_px = (camera['camera_fx'] + camera['camera_fy']) / 2.0
        self.calibration_image_width = camera['image_width']

        self.initialize_depth_model()

        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.depth_publisher = self.create_publisher(Image, '/depth/image_raw', 10)

    def initialize_depth_model(self):
        try:
            import torch

            model_class = _import_depth_anything_3()
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.depth_model = model_class.from_pretrained(self.model_id).to(self.device).eval()

            total_params = sum(param.numel() for param in self.depth_model.parameters())
            self.get_logger().info(
                f'Model {self.model_id} loaded on {self.device}: '
                f'{total_params / 1e6:.2f}M parameters'
            )
        except Exception as error:
            self.get_logger().error(f'Failed to initialize: {error}')
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

            image_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            height, width = image_rgb.shape[:2]

            prediction = self.depth_model.inference(
                [image_rgb], process_res=self.process_resolution
            )
            network_depth = prediction.depth[0]

            if self.model_is_metric:
                depth_metres = metric_depth_metres(
                    network_depth, self.camera_focal_px, self.calibration_image_width
                )
            else:
                depth_metres = relative_depth_metres(network_depth, self.max_depth)
            depth_metres = cv2.resize(
                depth_metres * self.depth_scale, (width, height), interpolation=cv2.INTER_LINEAR
            )

            if self.depth_encoding == '32FC1':
                depth_msg = self.bridge.cv2_to_imgmsg(
                    depth_metres.astype(np.float32), encoding='32FC1'
                )
            else:
                depth_normalized = (depth_metres - depth_metres.min()) / (
                    depth_metres.max() - depth_metres.min() + 1e-8
                )
                depth_uint8 = (depth_normalized * 255.0).astype(np.uint8)
                depth_msg = self.bridge.cv2_to_imgmsg(depth_uint8, encoding='mono8')

            depth_msg.header = msg.header
            depth_msg.header.frame_id = 'depth_camera'
            self.depth_publisher.publish(depth_msg)

            self.get_logger().debug('Published depth image')

        except Exception as error:
            self.get_logger().error(f'Error: {error}')
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
