"""The cost nodes start and compute against real rclpy, not a MagicMock.

Each check runs in a fresh interpreter because test_cost_calculator.py replaces rclpy in
sys.modules for the rest of the session. The package source goes first on PYTHONPATH so
the check sees this tree rather than the last colcon install.
"""

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    shutil.which('ros2') is None,
    reason='needs ROS 2; runs inside cine-gazebo-sim:dev',
)

PACKAGE_SOURCE = Path(__file__).resolve().parents[1]

CONSTRUCT_COST_FUNCTION_NODE = """
import rclpy
from cine_agent.cost_functions.cost_functions import CostFunctionCalculatorNode

rclpy.init()
CostFunctionCalculatorNode().destroy_node()
rclpy.shutdown()
"""

CALCULATE_COST_WITH_NODE_LOGGER = """
import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.node import Node
from cine_agent.cost_functions.cost_calculator import CostCalculator

rclpy.init()
node = Node('cost_calculator_check')
calculator = CostCalculator(node)
calculator.orientation = np.array([0.0, 0.0, 0.0, 1.0])
calculator.depth_image = np.zeros((10, 10))
calculator.center_depth = 5.0
center = PolygonStamped()
center.polygon.points = [Point32(x=320.0, y=240.0)]
calculator.center_point = center
assert calculator.calculate_cost() is not None
node.destroy_node()
rclpy.shutdown()
"""

TICK_FORGETS_STALE_SUBJECT = """
import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.parameter import Parameter
from cine_agent.cost_functions.cost_functions import CostFunctionCalculatorNode

rclpy.init()
node = CostFunctionCalculatorNode()
node.set_parameters([Parameter('subject_detection_stale_s', Parameter.Type.DOUBLE, -1.0)])
center = PolygonStamped()
center.polygon.points = [Point32(x=320.0, y=240.0)]
node._center_cb(center)
node._tick()
assert node.cost_calculator.center_point is None
node.destroy_node()
rclpy.shutdown()
"""


def _run_against_package_source(script):
    environment = dict(
        os.environ,
        PYTHONPATH=os.pathsep.join([str(PACKAGE_SOURCE), os.environ.get('PYTHONPATH', '')]),
    )

    return subprocess.run(
        [sys.executable, '-c', script],
        env=environment,
        capture_output=True,
        text=True,
        timeout=60,
    )


def test_cost_function_node_starts_with_real_rclpy():
    result = _run_against_package_source(CONSTRUCT_COST_FUNCTION_NODE)

    assert result.returncode == 0, result.stderr


def test_calculate_cost_completes_with_real_rclpy_node_logger():
    result = _run_against_package_source(CALCULATE_COST_WITH_NODE_LOGGER)

    assert result.returncode == 0, result.stderr


def test_cost_node_tick_forgets_subject_once_detection_is_stale():
    result = _run_against_package_source(TICK_FORGETS_STALE_SUBJECT)

    assert result.returncode == 0, result.stderr
