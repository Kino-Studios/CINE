import ast
from pathlib import Path

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'cinematic_run.launch.py'


def _tree():
    return ast.parse(LAUNCH_FILE.read_text())


def _flight_type_argument_default():
    for call in ast.walk(_tree()):
        if (
            isinstance(call, ast.Call)
            and getattr(call.func, 'id', None) == 'DeclareLaunchArgument'
            and call.args
            and getattr(call.args[0], 'value', None) == 'flight_type'
        ):
            return next(
                keyword.value.value for keyword in call.keywords if keyword.arg == 'default_value'
            )

    return None


def _planner_flight_type_value():
    for dictionary in ast.walk(_tree()):
        if isinstance(dictionary, ast.Dict):
            for key, value in zip(dictionary.keys, dictionary.values):
                if getattr(key, 'value', None) == 'flight_type':
                    return ast.unparse(value)

    return None


def test_cinematic_run_passes_flight_type_launch_argument_to_planner():
    assert _flight_type_argument_default() == '2'
    assert _planner_flight_type_value() != 'str(2)'
    assert 'flight_type' in _planner_flight_type_value()
