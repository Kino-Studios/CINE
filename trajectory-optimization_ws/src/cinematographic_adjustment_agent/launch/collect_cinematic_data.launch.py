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
        description="Drone ID for namespacing"
    )

    use_sim_focus_arg = DeclareLaunchArgument(
        "use_simulated_focus",
        default_value=str(SimulationParams.USE_SIMULATED_FOCUS),
        description="Whether to use simulated focus"
    )

    use_sim_light_arg = DeclareLaunchArgument(
        "use_simulated_light",
        default_value=str(SimulationParams.USE_SIMULATED_LIGHT),
        description="Whether to use simulated light"
    )

    drone_id = LaunchConfiguration("drone_id")
    use_simulated_focus = LaunchConfiguration("use_simulated_focus")
    use_simulated_light = LaunchConfiguration("use_simulated_light")

    # Topic construction
    odom_topic = ["/drone_", drone_id, "/visual_slam/odom"]
    baseline_topic = ["/drone_", drone_id, "/planning/bspline"]
    cinematic_topic = ["/drone_", drone_id, "/planning/cinematic_bspline"]

    cinematic_collection = Node(
        package="cinematographic_adjustment_agent",
        executable="cinematic_collection",
        name="cinematic_collection",
        output="screen",
        parameters=[
            {
                "odom_topic": odom_topic,
                "baseline_bspline_topic": baseline_topic,
                "cinematic_bspline_topic": cinematic_topic,
                "frame_id": WaypointParams.FRAME_ID,
                "target_xyz": SimulationParams.TARGET_XYZ,
                "focus_depth_m": SimulationParams.FOCUS_DEPTH_M,
                "use_simulated_focus": use_simulated_focus,
                "use_simulated_light": use_simulated_light,
            }
        ],
    )

    return LaunchDescription(
        [
            drone_id_arg,
            use_sim_focus_arg,
            use_sim_light_arg,
            cinematic_collection,
        ]
    )
