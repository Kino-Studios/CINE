import ast
from pathlib import Path

import yaml

PACKAGE_DIRECTORY = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_DIRECTORY / 'launch' / 'gazebo_sim.launch.py'
WAYPOINTS_FILE = PACKAGE_DIRECTORY / 'config' / 'mission_waypoints.yaml'
# GridMap centres x and y on the origin and starts z at grid_map/ground_height, which
# advanced_param.launch.py sets to this.
PLANNER_GROUND_HEIGHT = -0.01
MARGIN_M = 1.0


def _launch_constant(name):
    # The launch file imports ROS, so its literals are read from the source instead.
    for node in ast.parse(LAUNCH_FILE.read_text()).body:
        if isinstance(node, ast.Assign) and [target.id for target in node.targets] == [name]:
            return ast.literal_eval(node.value)

    raise AssertionError(f'{name} is not a module-level literal in {LAUNCH_FILE}')


def _waypoints():
    return yaml.safe_load(WAYPOINTS_FILE.read_text())


def test_every_waypoint_lies_inside_the_planner_map_horizontally_with_margin():
    half_x = _launch_constant('PLANNER_MAP_SIZE_X') / 2.0
    half_y = _launch_constant('PLANNER_MAP_SIZE_Y') / 2.0

    assert all(
        abs(x) <= half_x - MARGIN_M and abs(y) <= half_y - MARGIN_M for x, y, _ in _waypoints()
    )


def test_every_waypoint_lies_below_the_planner_ceiling_with_margin():
    ceiling = _launch_constant('PLANNER_VIRTUAL_CEILING')

    assert max(z for _, _, z in _waypoints()) <= ceiling - MARGIN_M


def test_the_planner_ceiling_lies_inside_the_planner_map():
    map_top = PLANNER_GROUND_HEIGHT + _launch_constant('PLANNER_MAP_SIZE_Z')

    assert _launch_constant('PLANNER_VIRTUAL_CEILING') <= map_top
