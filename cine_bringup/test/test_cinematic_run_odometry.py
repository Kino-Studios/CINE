import ast
from pathlib import Path

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'cinematic_run.launch.py'


def _launch_calls(function_name):
    return [
        node
        for node in ast.walk(ast.parse(LAUNCH_FILE.read_text()))
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == function_name
    ]


def _declared_default(argument_name):
    return next(
        keyword.value.value
        for call in _launch_calls('DeclareLaunchArgument')
        if call.args
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == argument_name
        for keyword in call.keywords
        if keyword.arg == 'default_value'
    )


def _launch_configuration_names_by_variable():
    return {
        node.targets[0].id: node.value.args[0].value
        for node in ast.walk(ast.parse(LAUNCH_FILE.read_text()))
        if isinstance(node, ast.Assign)
        and isinstance(node.targets[0], ast.Name)
        and isinstance(node.value, ast.Call)
        and getattr(node.value.func, 'id', None) == 'LaunchConfiguration'
    }


def _launch_argument_read_by(expression):
    if isinstance(expression, ast.Name):
        return _launch_configuration_names_by_variable()[expression.id]

    return expression.args[0].value


def _cost_calculator_remappings():
    cost_calculator = next(
        call
        for call in _launch_calls('Node')
        for keyword in call.keywords
        if keyword.arg == 'executable'
        and isinstance(keyword.value, ast.Constant)
        and keyword.value.value == 'cost_function_calculator'
    )
    remappings = next(
        keyword.value for keyword in cost_calculator.keywords if keyword.arg == 'remappings'
    )

    return {
        (pair.elts[0].value, _launch_argument_read_by(pair.elts[1])) for pair in remappings.elts
    }


def test_cost_calculator_defaults_to_imu_propagate_on_hardware():
    assert _declared_default('cost_calculator_odometry_topic') == '/imu_propagate'


def test_cost_calculator_reads_its_odometry_from_the_launch_argument():
    assert ('/imu_propagate', 'cost_calculator_odometry_topic') in _cost_calculator_remappings()
