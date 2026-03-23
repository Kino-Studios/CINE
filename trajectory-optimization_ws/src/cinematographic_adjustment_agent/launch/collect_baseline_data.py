# NOTE: Data collection script - waypoint dispatch via this launch file is not yet functional.
# Use the standalone collect_baseline_data.py script directly for data collection.

from launch import LaunchDescription
from launch_ros.actions import Node
import sys
from pathlib import Path

here = Path(__file__).resolve()
for parent in [here] + list(here.parents):
    if (parent / "configs").is_dir():
        python_constants_path = parent / "configs" / "python-constants"
        if str(python_constants_path) not in sys.path:
            sys.path.insert(0, str(python_constants_path))
        break

from constants import SimulationParams, WaypointParams

def generate_launch_description():
    drone_id = SimulationParams.DRONE_ID
    odom_topic = "visual_slam/odom"
    baseline_topic = "planning/bspline"
    cinematic_topic = "planning/cinematic_bspline"

    baseline_collection = Node(
        package="cinematographic_adjustment_agent",
        executable="baseline_collection",
        name="baseline_collection",
        namespace=f"drone_{drone_id}",
        output="screen",
        parameters=[
            {
                "odom_topic": odom_topic,
                "baseline_bspline_topic": baseline_topic,
                "cinematic_bspline_topic": cinematic_topic,
                "frame_id": WaypointParams.FRAME_ID,
                "target_xyz": SimulationParams.TARGET_XYZ,
                "focus_depth_m": SimulationParams.FOCUS_DEPTH_M,
                "waypoints_topic": "",
                "waypoints_file": "",
            }
        ],
    )

    return LaunchDescription(
        [
            baseline_collection,
        ]
    )
