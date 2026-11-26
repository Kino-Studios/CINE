import ast
from pathlib import Path

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'cinematic_run.launch.py'


def _calls(function_name):
    return [
        node
        for node in ast.walk(ast.parse(LAUNCH_FILE.read_text()))
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == function_name
    ]


def _intent_node_condition():
    node = next(
        call
        for call in _calls('Node')
        for keyword in call.keywords
        if keyword.arg == 'name'
        and isinstance(keyword.value, ast.Constant)
        and keyword.value.value == 'cinematic_intent_node'
    )

    return ast.unparse(next(kw.value for kw in node.keywords if kw.arg == 'condition'))


def test_intent_node_condition_requires_the_start_argument():
    assert 'start_cinematic_intent_node' in _intent_node_condition()


def test_start_argument_defaults_to_true_so_hardware_is_unchanged():
    defaults = [
        keyword.value.value
        for call in _calls('DeclareLaunchArgument')
        if call.args
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == 'start_cinematic_intent_node'
        for keyword in call.keywords
        if keyword.arg == 'default_value'
    ]

    assert defaults == ['true']
