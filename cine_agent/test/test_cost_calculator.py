import math
import sys
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
