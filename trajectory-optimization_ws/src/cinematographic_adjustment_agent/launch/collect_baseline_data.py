# TODO: make a python script that uses terminal to launch certain commands (needs to be launched here first though). also add proper baseline / cinematic switching and clean codebase

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
    odom_topic = f"/drone_{drone_id}_visual_slam/odom"
    baseline_topic = f"/drone_{drone_id}_planning/bspline"
    cinematic_topic = f"/drone_{drone_id}_planning/cinematic_bspline"

    baseline_collection = Node(
        package="cinematographic_adjustment_agent",
        executable="baseline_collection",
        name="baseline_collection",
        output="screen",
        parameters=[
            {
                "odom_topic": odom_topic,
                "baseline_bspline_topic": baseline_topic,
                "cinematic_bspline_topic": cinematic_topic,
                "frame_id": WaypointParams.FRAME_ID,
                "target_xyz": SimulationParams.TARGET_XYZ,
                "focus_depth_m": SimulationParams.FOCUS_DEPTH_M,
            }
        ],
    )

    return LaunchDescription(
        [
            baseline_collection,
        ]
    )
