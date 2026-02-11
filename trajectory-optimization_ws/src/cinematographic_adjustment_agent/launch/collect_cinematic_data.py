# TODO: make a python script with cinematographic launching and formalize it (make it work as intended, currently lossy structure)

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

    cinematic_collection = Node(
        package="cinematographic_adjustment_agent",
        executable="cinematic_collection",
        name="cinematic_collection",
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
                "use_simulated_focus": SimulationParams.USE_SIMULATED_FOCUS,
                "use_simulated_light": SimulationParams.USE_SIMULATED_LIGHT,
                "waypoints_topic": "",
                "waypoints_file": "",
            }
        ],
    )

    return LaunchDescription(
        [
            cinematic_collection,
        ]
    )
