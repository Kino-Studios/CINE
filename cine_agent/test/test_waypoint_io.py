"""The waypoint loader reads the mission file that the Gazebo launch passes in.

The pure-Python CI job has no ROS, so a fake geometry_msgs stands in while the loader is
imported; the loader itself needs only a logger and a place to store the points.
"""

import sys
from pathlib import Path
from types import ModuleType
from unittest.mock import MagicMock, patch

_FAKE_ROS_MODULES = {
    'geometry_msgs': ModuleType('geometry_msgs'),
    'geometry_msgs.msg': ModuleType('geometry_msgs.msg'),
}
_FAKE_ROS_MODULES['geometry_msgs.msg'].PoseArray = object

with patch.dict(sys.modules, _FAKE_ROS_MODULES):
    from cine_agent.waypoint_automation.io.waypoint_io import WaypointIOMixin

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MISSION_FILE = REPOSITORY_ROOT / 'cine_gazebo' / 'config' / 'mission_waypoints.yaml'


class LoaderHost(WaypointIOMixin):
    def __init__(self):
        self.static_waypoints = []
        self.logger = MagicMock()

    def get_logger(self):
        return self.logger


def test_yaml_waypoint_file_loads_the_mission_points():
    host = LoaderHost()

    host._load_waypoints_from_file(str(MISSION_FILE))

    assert host.static_waypoints == [(0, 3, 2.5), (-5.1, 6, 3), (-5.1, 13, 3), (0, 14, 3)]
