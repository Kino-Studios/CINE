from cine_config.constants import SimulationParams, WaypointParams
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value=str(SimulationParams.DRONE_ID),
        description='Drone ID for namespacing',
    )
    waypoints_topic_arg = DeclareLaunchArgument(
        'waypoints_topic',
        default_value='',
        description='Optional PoseArray topic to override default waypoints',
    )
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='',
        description='Optional CSV file path for static waypoints',
    )

    drone_id = LaunchConfiguration('drone_id')
    waypoints_topic = LaunchConfiguration('waypoints_topic')
    waypoints_file = LaunchConfiguration('waypoints_file')
    odom_topic = ['/drone_', drone_id, '/visual_slam/odom']
    baseline_topic = ['/drone_', drone_id, '/planning/bspline']
    cinematic_topic = ['/drone_', drone_id, '/planning/cinematic_bspline']

    cinematic_collection = Node(
        package='cine_agent',
        executable='cinematic_collection',
        name='cinematic_collection',
        namespace=['drone_', drone_id],
        output='screen',
        parameters=[
            {
                'odom_topic': odom_topic,
                'baseline_bspline_topic': ['/drone_', drone_id, '/planning/bspline'],
                'cinematic_bspline_topic': cinematic_topic,
                'frame_id': WaypointParams.FRAME_ID,
                'target_xyz': SimulationParams.TARGET_XYZ,
                'focus_depth_m': SimulationParams.FOCUS_DEPTH_M,
                'use_simulated_focus': SimulationParams.USE_SIMULATED_FOCUS,
                'use_simulated_light': SimulationParams.USE_SIMULATED_LIGHT,
                'waypoints_topic': waypoints_topic,
                'waypoints_file': waypoints_file,
            }
        ],
    )

    return LaunchDescription(
        [
            drone_id_arg,
            waypoints_topic_arg,
            waypoints_file_arg,
            cinematic_collection,
        ]
    )
