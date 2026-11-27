import ast
import json
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
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
SCORING_FILE = REPOSITORY_ROOT / 'cine_config' / 'config' / 'eval' / 'scoring.json'


def _planner_parameter(name):
    # The launch file imports ROS, so its literals are read from the source instead.
    for dictionary in ast.walk(ast.parse(PLANNER_LAUNCH_FILE.read_text())):
        if isinstance(dictionary, ast.Dict):
            for key, value in zip(dictionary.keys, dictionary.values):
                if getattr(key, 'value', None) == name:
                    return ast.literal_eval(value)

    raise AssertionError(f'{name} is not a literal parameter in {PLANNER_LAUNCH_FILE}')


def test_planner_pads_obstacles_by_at_least_the_drone_radius():
    airframe_radius_m = json.loads(SCORING_FILE.read_text())['clearance']['airframe_radius_m']

    assert _planner_parameter('grid_map/obstacles_inflation') >= airframe_radius_m
