# TODO: this kind of works but waypoints arent being sent here. figure out the fix here and for the cinematograpihc data collection

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    drone_id_arg = DeclareLaunchArgument(
        "drone_id",
        default_value=str(SimulationParams.DRONE_ID),
        description="Drone ID for namespacing",
    )
    waypoints_topic_arg = DeclareLaunchArgument(
        "waypoints_topic",
        default_value="",
        description="Optional PoseArray topic to override default waypoints",
    )
    waypoints_file_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value="",
        description="Optional CSV file path for static waypoints",
    )

    drone_id = LaunchConfiguration("drone_id")
    waypoints_topic = LaunchConfiguration("waypoints_topic")
    waypoints_file = LaunchConfiguration("waypoints_file")

    odom_topic = "visual_slam/odom"
    baseline_topic = "planning/bspline"
    cinematic_topic = "planning/cinematic_bspline"

    baseline_collection = Node(
         package="cinematographic_adjustment_agent",
         executable="baseline_collection",
         name="baseline_collection",
         namespace=["drone_", drone_id],
         output="screen",
         parameters=[
             {
                 "odom_topic": odom_topic,
                 "baseline_bspline_topic": baseline_topic,
                 "cinematic_bspline_topic": cinematic_topic,
                 "frame_id": WaypointParams.FRAME_ID,
                 "target_xyz": SimulationParams.TARGET_XYZ,
                 "focus_depth_m": SimulationParams.FOCUS_DEPTH_M,
                 "waypoints_topic": waypoints_topic,
                 "waypoints_file": waypoints_file,
                 "baseline_mode": True,
             }
         ],
     )

    return LaunchDescription(
        [
            drone_id_arg,
            waypoints_topic_arg,
            waypoints_file_arg,
            baseline_collection,
         ]
     )
