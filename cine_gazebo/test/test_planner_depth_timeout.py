import ast
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
GAZEBO_LAUNCH_FILE = REPOSITORY_ROOT / 'cine_gazebo' / 'launch' / 'gazebo_sim.launch.py'
MISSION_LAUNCH_FILE = REPOSITORY_ROOT / 'cine_bringup' / 'launch' / 'cinematic_run.launch.py'
PLANNER_LAUNCH_FILE = (
    REPOSITORY_ROOT
    / 'third_party'
    / 'ego_planner_swarm'
    / 'src'
    / 'planner'
    / 'plan_manage'
    / 'launch'
    / 'advanced_param.launch.py'
)
# Gap between occupancy-map updates in simulated seconds, measured from launch.log of
# eval/runs/modal/2026-11-24_21-16-49_mission_sim_calm_4e71 (depth network on CPU).
SLOWEST_MEASURED_MAP_UPDATE_GAP_S = 1.7


def _tree(path):
    return ast.parse(path.read_text())


def _module_constant(path, name):
    for node in _tree(path).body:
        if isinstance(node, ast.Assign) and [target.id for target in node.targets] == [name]:
            return ast.literal_eval(node.value)

    raise AssertionError(f'{name} is not a module-level literal in {path}')


def _dictionary_value_source(path, key):
    for dictionary in ast.walk(_tree(path)):
        if isinstance(dictionary, ast.Dict):
            for dictionary_key, value in zip(dictionary.keys, dictionary.values):
                if getattr(dictionary_key, 'value', None) == key:
                    return ast.unparse(value)

    return None


def _launch_argument_default_source(path, name):
    for call in ast.walk(_tree(path)):
        if (
            isinstance(call, ast.Call)
            and getattr(call.func, 'id', None) == 'LaunchConfiguration'
            and call.args
            and getattr(call.args[0], 'value', None) == name
        ):
            return next(
                ast.unparse(keyword.value) for keyword in call.keywords if keyword.arg == 'default'
            )

    return None


def test_gazebo_timeout_outlasts_the_slowest_measured_map_update_gap():
    assert _module_constant(GAZEBO_LAUNCH_FILE, 'PLANNER_ODOM_DEPTH_TIMEOUT') > (
        SLOWEST_MEASURED_MAP_UPDATE_GAP_S
    )


def test_gazebo_launch_passes_its_timeout_to_the_mission():
    assert (
        _dictionary_value_source(GAZEBO_LAUNCH_FILE, 'planner_odom_depth_timeout')
        == 'str(PLANNER_ODOM_DEPTH_TIMEOUT)'
    )


def test_mission_keeps_the_upstream_timeout_by_default():
    assert _launch_argument_default_source(MISSION_LAUNCH_FILE, 'planner_odom_depth_timeout') == (
        "'1.0'"
    )


def test_mission_passes_its_timeout_to_the_planner_launch():
    assert (
        _dictionary_value_source(MISSION_LAUNCH_FILE, 'odom_depth_timeout')
        == 'planner_odom_depth_timeout'
    )


def test_planner_launch_keeps_the_upstream_timeout_by_default():
    assert _launch_argument_default_source(PLANNER_LAUNCH_FILE, 'odom_depth_timeout') == '1.0'


def test_planner_launch_sets_the_grid_map_timeout_parameter():
    assert (
        _dictionary_value_source(PLANNER_LAUNCH_FILE, 'grid_map/odom_depth_timeout')
        == 'odom_depth_timeout'
    )
