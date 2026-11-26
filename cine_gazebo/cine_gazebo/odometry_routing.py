"""Picks the odometry topic each mission node reads in the simulator, per nav_source.

MAVROS publishes best effort, which reliable subscribers never connect to, so with
nav_source sim they read odometry_relay's reliable republish instead.
"""

from dataclasses import dataclass

MAVROS_ODOMETRY_TOPIC = '/mavros/local_position/odom'
RELAYED_ODOMETRY_TOPIC = '/cinematography/drone_odometry'
# About 1 s at MAVROS's ~10 Hz; readers keep 1 to 10, so a deeper queue only adds stale poses.
RELAY_HISTORY_DEPTH = 10


@dataclass(frozen=True)
class RelayReliability:
    subscriber: str
    publisher: str


RELAY_RELIABILITY = RelayReliability(subscriber='BEST_EFFORT', publisher='RELIABLE')


@dataclass(frozen=True)
class MissionOdometry:
    # cinematic_run.launch.py's odometry_topic: planner, intent node and camera pose read it.
    odometry_topic: str
    cost_calculator_odometry_topic: str
    starts_relay: bool


MISSION_ODOMETRY = {
    'sim': MissionOdometry(
        odometry_topic=RELAYED_ODOMETRY_TOPIC,
        cost_calculator_odometry_topic=RELAYED_ODOMETRY_TOPIC,
        starts_relay=True,
    ),
    'vins': MissionOdometry(
        odometry_topic='/vio/odometry',
        cost_calculator_odometry_topic='/imu_propagate',
        starts_relay=False,
    ),
}


def mission_odometry(nav_source: str) -> MissionOdometry:
    if nav_source not in MISSION_ODOMETRY:
        raise ValueError(
            f"Unknown nav_source '{nav_source}'; choose one of {sorted(MISSION_ODOMETRY)}",
        )

    return MISSION_ODOMETRY[nav_source]
