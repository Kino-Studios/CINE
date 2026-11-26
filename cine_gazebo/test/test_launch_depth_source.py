import ast
from pathlib import Path

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'gazebo_sim.launch.py'


def _tree():
    return ast.parse(LAUNCH_FILE.read_text())


def _declared_default(name):
    for call in ast.walk(_tree()):
        if (
            isinstance(call, ast.Call)
            and getattr(call.func, 'id', None) == 'DeclareLaunchArgument'
            and getattr(call.args[0], 'value', None) == name
        ):
            return next(
                keyword.value.value for keyword in call.keywords if keyword.arg == 'default_value'
            )

    return None


def _module_constant(name):
    for node in _tree().body:
        if isinstance(node, ast.Assign) and [target.id for target in node.targets] == [name]:
            return ast.literal_eval(node.value)

    raise AssertionError(f'{name} is not a module-level literal in {LAUNCH_FILE}')


def _image_bridge_arguments():
    return [
        ast.unparse(keyword.value)
        for call in ast.walk(_tree())
        if isinstance(call, ast.Call)
        and getattr(call.func, 'id', None) == 'Node'
        and any(
            keyword.arg == 'executable' and getattr(keyword.value, 'value', None) == 'image_bridge'
            for keyword in call.keywords
        )
        for keyword in call.keywords
        if keyword.arg == 'arguments'
    ]


def test_simulator_depth_is_the_default_depth_source():
    assert _declared_default('depth_source') == 'gazebo'


def test_run_record_names_the_depth_source():
    assert 'depth_source' in _module_constant('RECORDED_LAUNCH_ARGUMENTS')


def test_depth_image_is_bridged_where_the_planner_reads_it():
    assert "[f'/{DEPTH_TOPIC}']" in _image_bridge_arguments()
