import ast
from pathlib import Path

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'gazebo_sim.launch.py'


def _launched_node_names():
    tree = ast.parse(LAUNCH_FILE.read_text())

    return {
        keyword.value.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'Node'
        for keyword in node.keywords
        if keyword.arg == 'name' and isinstance(keyword.value, ast.Constant)
    }


def test_simulator_starts_contract_nodes_under_their_contract_names():
    # The live contract check credits a topic only to the node name CINE-Interfaces lists.
    assert {
        'depth_publisher_node',
        'camera_pose_publisher',
        'planner_setpoint_adapter',
        'vins_estimator',
        'vio_publisher',
        'bb_extractor',
        'mavros_setpoints',
    } <= _launched_node_names()
