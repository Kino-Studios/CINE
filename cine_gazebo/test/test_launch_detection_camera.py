import ast
from pathlib import Path

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'gazebo_sim.launch.py'


def _tree():
    return ast.parse(LAUNCH_FILE.read_text())


def _simulated_detection_camera():
    assignment = next(
        node
        for node in ast.walk(_tree())
        if isinstance(node, ast.Assign)
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id == 'DETECTION_CAMERA'
    )

    return ast.literal_eval(assignment.value)


def test_simulated_detection_camera_is_the_gazebo_camera_at_full_resolution():
    assert {name: float(value) for name, value in _simulated_detection_camera().items()} == {
        'detection_camera_fx': 497.5985575,
        'detection_camera_fy': 497.5985575,
        'detection_camera_cx': 576.0,
        'detection_camera_cy': 324.0,
        'detection_image_width': 1152.0,
        'detection_image_height': 648.0,
    }


def test_mission_passes_the_detection_camera_to_the_cinematic_run_launch():
    spread_names = {
        value.id
        for node in ast.walk(_tree())
        if isinstance(node, ast.Dict)
        for key, value in zip(node.keys, node.values)
        if key is None and isinstance(value, ast.Name)
    }

    assert 'DETECTION_CAMERA' in spread_names


def test_mission_turns_off_the_cinematic_intent_node():
    passed = {
        key.value: value.value
        for node in ast.walk(_tree())
        if isinstance(node, ast.Dict)
        for key, value in zip(node.keys, node.values)
        if isinstance(key, ast.Constant) and isinstance(value, ast.Constant)
    }

    assert passed['start_cinematic_intent_node'] == 'false'
