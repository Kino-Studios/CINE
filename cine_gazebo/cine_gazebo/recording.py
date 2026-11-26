"""Names each Gazebo run's ROS 2 bag and its topics; free of ROS imports so CI can test it."""

from datetime import datetime


def bag_directory_name(now: datetime, nav_source: str, world: str, wind: str) -> str:
    return f'{now:%Y-%m-%d_%H-%M-%S}_{nav_source}_{world}_{wind}'


def recorded_topics(drone_id: int) -> list[str]:
    return [
        '/gazebo/drone_pose',
        '/gazebo/subject_pose',
        # The estimates the flight is judged against that truth.
        '/vio/odometry',
        '/mavros/local_position/odom',
        '/cinematography/drone_odometry',
        '/imu_propagate',
        # Every setpoint stage: the planner, the cinematography agent, then the
        # relay's output to MAVROS (names as mavros_setpoints.py publishes them).
        f'drone_{drone_id}_planning/pos_cmd',
        '/cinematography/translational_position_setpoints',
        '/cinematography/translational_velocity_setpoints',
        '/cinematography/translational_acceleration_setpoints',
        '/cinematography/attitude_position_setpoints',
        '/cinematography/attitude_velocity_setpoints',
        '/mavros/setpoint_position/local',
        '/mavros/setpoint_velocity/cmd_vel',
        '/mavros/setpoint_accel/accel',
        '/mavros/setpoint_attitude/attitude',
        '/mavros/setpoint_attitude/cmd_vel',
        '/cinematography/focus_target_distance',
        '/object_detection/center_point',
        '/object_detection/bounding_box',
        '/object_detection/dimensions',
        # cine_eval scores focus and motion from these; without them those checks are skipped.
        '/cinematography/framing_active',
        '/cinematography/subject_position',
        '/mavros/imu/data_raw',
        '/imu_bridge/data_raw',
    ]
