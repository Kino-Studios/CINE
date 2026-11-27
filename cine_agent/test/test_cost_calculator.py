import math
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

mock_rclpy = MagicMock()
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['nav_msgs'] = MagicMock()
sys.modules['nav_msgs.msg'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['cine_msgs'] = MagicMock()
sys.modules['cine_msgs.msg'] = MagicMock()
sys.modules['traj_utils'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()
sys.modules['cv2'] = MagicMock()

import numpy as np
import pytest
from cine_agent.cost_functions.cost_calculator import CostCalculator
from cine_config.constants import (
    CostParams,
    ImageParams,
    SimulationParams,
)
from geometry_msgs.msg import PolygonStamped


@pytest.fixture
def mock_node():
    node = MagicMock()
    node.anchor_focus_distance = None

    param_mapping = {
        'deadband_px': CostParams.DEADBAND_PX,
        'huber_delta_px': CostParams.HUBER_DELTA_PX,
        'w_framing': CostParams.W_FRAMING,
        'w_dist': CostParams.W_DIST,
        'w_near': CostParams.W_NEAR,
        'w_stab': CostParams.W_STAB,
        'w_smooth': CostParams.W_SMOOTH,
        'w_light': CostParams.W_LIGHT,
        'light_avoidance_enabled': False,
        'image_width': ImageParams.IMAGE_WIDTH,
        'image_height': ImageParams.IMAGE_HEIGHT,
        'framing_vertical_weight': CostParams.FRAMING_VERTICAL_WEIGHT,
        'light_saturation_threshold': CostParams.LIGHT_SATURATION_THRESHOLD,
        'light_tau': CostParams.LIGHT_TAU,
        'w_saturation': CostParams.W_SATURATION,
        'use_simulated_light': SimulationParams.USE_SIMULATED_LIGHT,
        'camera_fx': 500.0,
        'focus_yaw_hold_deadband_rad': math.radians(2.0),
    }

    node.get_parameter.side_effect = lambda name: MagicMock(value=param_mapping.get(name, 0.0))
    node.get_clock().now().nanoseconds = 10**9

    return node


def test_camera_forward_vector_alignment(mock_node):
    calculator = CostCalculator(mock_node)
    calculator.orientation = np.array([0.0, 0.0, 0.0, 1.0])
    forward = calculator._camera_forward_vector()
    np.testing.assert_allclose(forward, [0.0, 0.0, 1.0], atol=1e-7)

    s = math.sin(math.radians(45))
    c = math.cos(math.radians(45))
    calculator.orientation = np.array([0.0, s, 0.0, c])
    forward = calculator._camera_forward_vector()
    np.testing.assert_allclose(forward, [1.0, 0.0, 0.0], atol=1e-7)


def test_acceleration_smoothness_cost(mock_node):
    calculator = CostCalculator(mock_node)
    calculator.angular_velocity = np.array([1.0, 0.0, 0.0])
    calculator._prev_ang_vel = np.array([0.0, 0.0, 0.0])
    calculator.orientation = np.array([0.0, 0.0, 0.0, 1.0])
    calculator.depth_image = np.zeros((10, 10))
    calculator.center_depth = 5.0

    msg = PolygonStamped()
    from geometry_msgs.msg import Point32

    p = Point32()
    p.x, p.y = 320.0, 240.0
    msg.polygon.points = [p]
    calculator.center_point = msg

    costs = calculator.calculate_cost()
    assert costs is not None
    assert costs['smoothness_cost'] == pytest.approx(float(CostParams.W_SMOOTH))


class SingleMessageLogger:
    """Takes one message string and keyword options only, like rclpy's RcutilsLogger."""

    def __init__(self):
        self.messages = []

    def debug(self, message, **kwargs):
        self.messages.append(message)

    def info(self, message, **kwargs):
        self.messages.append(message)


def test_calculate_cost_logs_preformatted_detail_line_through_rclpy_style_logger(mock_node):
    logger = SingleMessageLogger()
    mock_node.get_logger.return_value = logger
    calculator = CostCalculator(mock_node)
    calculator.orientation = np.array([0.0, 0.0, 0.0, 1.0])
    calculator.depth_image = np.zeros((10, 10))
    calculator.center_depth = 5.0

    msg = PolygonStamped()
    from geometry_msgs.msg import Point32

    p = Point32()
    p.x, p.y = 320.0, 240.0
    msg.polygon.points = [p]
    calculator.center_point = msg

    costs = calculator.calculate_cost()

    assert costs is not None
    assert logger.messages[-1].startswith('CostCalculator detail: framing=')


def test_calculate_cost_returns_none_for_one_bbox_point_through_rclpy_style_logger(mock_node):
    mock_node.get_logger.return_value = SingleMessageLogger()
    calculator = CostCalculator(mock_node)
    calculator.depth_image = np.zeros((10, 10))

    msg = PolygonStamped()
    from geometry_msgs.msg import Point32

    msg.polygon.points = [Point32()]
    calculator.bounding_box = msg

    assert calculator.calculate_cost() is None


def test_feature_tracking_center_depth(mock_node):
    calculator = CostCalculator(mock_node)
    calculator.depth_image = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)

    msg = PolygonStamped()
    from geometry_msgs.msg import Point32

    p = Point32()
    p.x, p.y = 1.0, 1.0
    msg.polygon.points = [p]
    calculator.center_point = msg

    calculator._update_center_depth()
    assert calculator.center_depth == pytest.approx(4.0)


def test_calibration_parameter_integration(mock_node):
    calculator = CostCalculator(mock_node)

    res = calculator._compute_cost_components(
        drone_x=0.0,
        drone_y=0.0,
        drone_z=0.0,
        target_world_x=1.0,
        target_world_y=0.0,
        target_world_z=1.0,
        reference_drone_x=0.0,
        reference_drone_y=0.0,
        reference_drone_z=-1.0,
        include_framing=True,
        include_lighting=False,
    )

    framing_cost = res[3]
    assert framing_cost > 0.0

    def get_param_double_fx(name):
        params = {
            'w_framing': CostParams.W_FRAMING,
            'camera_fx': 1000.0,
            'huber_delta_px': CostParams.HUBER_DELTA_PX,
            'deadband_px': 0.0,
            'w_dist': CostParams.W_DIST,
            'w_near': CostParams.W_NEAR,
            'w_stab': CostParams.W_STAB,
            'w_smooth': CostParams.W_SMOOTH,
        }

        return MagicMock(value=params.get(name, 1.0))

    mock_node.get_parameter.side_effect = get_param_double_fx
    res_double = calculator._compute_cost_components(
        drone_x=0.0,
        drone_y=0.0,
        drone_z=0.0,
        target_world_x=1.0,
        target_world_y=0.0,
        target_world_z=1.0,
        reference_drone_x=0.0,
        reference_drone_y=0.0,
        reference_drone_z=-1.0,
        include_framing=True,
        include_lighting=False,
    )
    assert res_double[3] > framing_cost


def _position_cost(calculator):
    return calculator.estimate_position_cost_at_position(
        drone_x=0.0,
        drone_y=0.0,
        drone_z=0.0,
        target_world_x=1.0,
        target_world_y=0.0,
        target_world_z=1.0,
        reference_drone_x=0.0,
        reference_drone_y=0.0,
        reference_drone_z=-1.0,
    )


def _framing_component(calculator):
    return calculator._compute_cost_components(
        drone_x=0.0,
        drone_y=0.0,
        drone_z=0.0,
        target_world_x=1.0,
        target_world_y=0.0,
        target_world_z=1.0,
        reference_drone_x=0.0,
        reference_drone_y=0.0,
        reference_drone_z=-1.0,
        include_framing=True,
        include_lighting=False,
    )[3]


def test_position_cost_rises_by_framing_cost_when_framing_is_on(mock_node):
    calculator = CostCalculator(mock_node)
    calculator._get_bool_parameter = lambda name, default: (
        name == ('include_framing_in_position_cost')
    )
    with_framing = _position_cost(calculator)

    calculator._get_bool_parameter = lambda name, default: False
    without_framing = _position_cost(calculator)

    assert with_framing - without_framing == pytest.approx(_framing_component(calculator))
    assert _framing_component(calculator) > 0.0


def test_center_depth_reads_the_depth_pixel_under_a_detection_box_in_camera_pixels(mock_node):
    from geometry_msgs.msg import Point32

    param_values = {
        'detection_image_width': 1152,
        'detection_image_height': 648,
    }
    original_side_effect = mock_node.get_parameter.side_effect
    mock_node.get_parameter.side_effect = lambda name: (
        MagicMock(value=param_values[name]) if name in param_values else original_side_effect(name)
    )
    calculator = CostCalculator(mock_node)
    depth_image = np.full((360, 640), 9.0, dtype=np.float32)
    depth_image[178:183, 318:323] = 4.0
    calculator.depth_image = depth_image

    msg = PolygonStamped()
    center = Point32()
    center.x, center.y = 576.0, 324.0
    msg.polygon.points = [center]
    calculator.center_point = msg

    calculator._update_center_depth()

    assert calculator.center_depth == 4.0


def test_subject_beyond_the_depth_cameras_far_clip_gets_no_depth(mock_node):
    from geometry_msgs.msg import Point32

    calculator = CostCalculator(mock_node)
    calculator.depth_image = np.array([[1.0, 2.0], [3.0, np.inf]], dtype=np.float32)
    msg = PolygonStamped()
    center = Point32()
    center.x, center.y = 1.0, 1.0
    msg.polygon.points = [center]
    calculator.center_point = msg

    calculator._update_center_depth()

    assert calculator.center_depth is None


def _calculator_with_mostly_sky_around_subject(mock_node):
    # geometry_msgs is mocked, so Point32() hands back one shared object; distinct corners
    # need plain records.
    calculator = CostCalculator(mock_node)
    depth_image = np.full((40, 40), np.inf, dtype=np.float32)
    depth_image[20, 20] = 6.0
    calculator.depth_image = depth_image
    msg = PolygonStamped()
    msg.polygon.points = [SimpleNamespace(x=10.0, y=10.0), SimpleNamespace(x=30.0, y=30.0)]
    calculator.bounding_box = msg

    return calculator


def test_subject_box_median_depth_ignores_sky_beyond_the_far_clip(mock_node):
    calculator = _calculator_with_mostly_sky_around_subject(mock_node)

    assert calculator.get_median_depth_object() == 6.0


def test_subject_neighbourhood_median_depth_ignores_sky_beyond_the_far_clip(mock_node):
    calculator = _calculator_with_mostly_sky_around_subject(mock_node)

    assert calculator.get_median_depth_roi() == 6.0


def _calculator_with_thin_subject_in_front_of_a_hedge(mock_node):
    # A person 9 m away fills about half of their detection box; the hedge behind them
    # fills the rest, as in the simulator's cine_field.
    calculator = CostCalculator(mock_node)
    depth_image = np.full((360, 640), 17.0, dtype=np.float32)
    depth_image[150:201, 310:329] = 9.0
    calculator.depth_image = depth_image
    # geometry_msgs is mocked, so every PolygonStamped() is one shared object.
    calculator.bounding_box = SimpleNamespace(
        polygon=SimpleNamespace(
            points=[SimpleNamespace(x=300.0, y=145.0), SimpleNamespace(x=338.0, y=205.0)],
        ),
    )
    calculator.center_point = SimpleNamespace(
        polygon=SimpleNamespace(points=[SimpleNamespace(x=319.0, y=175.0)]),
    )

    return calculator


def test_subject_box_depth_reads_the_subject_not_the_background_behind_it(mock_node):
    calculator = _calculator_with_thin_subject_in_front_of_a_hedge(mock_node)

    assert calculator.get_median_depth_object() == 9.0


def test_subject_neighbourhood_depth_reads_the_subject_not_the_background_behind_it(mock_node):
    calculator = _calculator_with_thin_subject_in_front_of_a_hedge(mock_node)

    assert calculator.get_median_depth_roi() == 9.0


def _detection_message():
    from geometry_msgs.msg import Point32

    msg = PolygonStamped()
    center = Point32()
    center.x, center.y = 576.0, 324.0
    msg.polygon.points = [center]

    return msg


def _calculator_seeing_subject_at(mock_node, seen_at_s):
    original_side_effect = mock_node.get_parameter.side_effect
    mock_node.get_parameter.side_effect = lambda name: (
        MagicMock(value=0.5) if name == 'subject_detection_stale_s' else original_side_effect(name)
    )
    calculator = CostCalculator(mock_node)
    mock_node.get_clock().now().nanoseconds = int(seen_at_s * 1e9)
    calculator.update_center_point(_detection_message())
    calculator.update_bounding_box(_detection_message())
    calculator.update_dimensions(_detection_message())
    calculator.center_depth = 3.0

    return calculator


def test_subject_last_detected_longer_ago_than_stale_time_is_forgotten(mock_node):
    calculator = _calculator_seeing_subject_at(mock_node, seen_at_s=10.0)
    mock_node.get_clock().now().nanoseconds = int(10.6 * 1e9)

    calculator.forget_stale_subject()

    assert calculator.center_point is None
    assert calculator.bounding_box is None
    assert calculator.dimensions is None
    assert calculator.center_depth is None


def test_subject_detected_within_stale_time_is_kept(mock_node):
    calculator = _calculator_seeing_subject_at(mock_node, seen_at_s=10.0)
    mock_node.get_clock().now().nanoseconds = int(10.4 * 1e9)

    calculator.forget_stale_subject()

    assert calculator.center_point is not None
    assert calculator.center_depth == 3.0
