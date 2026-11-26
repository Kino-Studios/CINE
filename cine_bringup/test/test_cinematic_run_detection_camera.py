import ast
from pathlib import Path

import pytest

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'cinematic_run.launch.py'
DETECTION_ARGUMENT_BY_PARAMETER = {
    'camera_fx': 'detection_camera_fx',
    'camera_fy': 'detection_camera_fy',
    'camera_cx': 'detection_camera_cx',
    'camera_cy': 'detection_camera_cy',
    'detection_image_width': 'detection_image_width',
    'detection_image_height': 'detection_image_height',
}


def _tree():
    return ast.parse(LAUNCH_FILE.read_text())


def _function(name):
    return next(
        node
        for node in ast.walk(_tree())
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _hardware_defaults_by_argument():
    dictionary = next(
        node
        for node in ast.walk(_function('_detection_camera_arguments'))
        if isinstance(node, ast.Dict)
    )

    return {key.value: ast.unparse(value) for key, value in zip(dictionary.keys, dictionary.values)}


def _argument_by_parameter():
    rows = next(
        node
        for node in ast.walk(_function('_detection_parameters'))
        if isinstance(node, ast.Tuple) and all(isinstance(row, ast.Tuple) for row in node.elts)
    )

    return {row.elts[0].value: row.elts[1].value for row in rows.elts}


def _node_spreads_detection_parameters(executable):
    node = next(
        call
        for call in ast.walk(_tree())
        if isinstance(call, ast.Call) and getattr(call.func, 'id', None) == 'Node'
        for keyword in call.keywords
        if keyword.arg == 'executable'
        and isinstance(keyword.value, ast.Constant)
        and keyword.value.value == executable
    )
    parameters = next(keyword.value for keyword in node.keywords if keyword.arg == 'parameters')

    return any(
        isinstance(element, ast.Starred) and ast.unparse(element.value) == '_detection_parameters()'
        for element in parameters.elts
    )


def test_detection_parameters_read_each_value_from_its_launch_argument():
    assert _argument_by_parameter() == DETECTION_ARGUMENT_BY_PARAMETER


@pytest.mark.parametrize(
    'executable',
    ['cost_function_calculator', 'cinematographic_adjustment_node'],
)
def test_node_with_a_cost_calculator_gets_the_detection_camera(executable):
    assert _node_spreads_detection_parameters(executable)


def test_detection_camera_arguments_default_to_the_hardware_camera():
    assert _hardware_defaults_by_argument() == {
        'detection_camera_fx': 'hardware_camera.CAMERA_FX',
        'detection_camera_fy': 'hardware_camera.CAMERA_FY',
        'detection_camera_cx': 'hardware_camera.CAMERA_CX',
        'detection_camera_cy': 'hardware_camera.CAMERA_CY',
        'detection_image_width': 'hardware_camera.IMAGE_WIDTH',
        'detection_image_height': 'hardware_camera.IMAGE_HEIGHT',
    }
