"""Picks the planner's depth source in the simulator: `gazebo` bridges exact rendered depth,
`network` runs the depth network on the RGB image, and a smoke flight needs neither."""

from dataclasses import dataclass

VALID_DEPTH_SOURCES = ('gazebo', 'network')


@dataclass(frozen=True)
class DepthRouting:
    # The Gazebo depth camera in the drone model, and the bridge carrying its image.
    renders_simulator_depth: bool
    starts_depth_network: bool


def depth_routing(depth_source: str, mission: bool) -> DepthRouting:
    if depth_source not in VALID_DEPTH_SOURCES:
        raise ValueError(
            f"Unknown depth_source '{depth_source}'; choose one of {sorted(VALID_DEPTH_SOURCES)}",
        )

    return DepthRouting(
        renders_simulator_depth=mission and depth_source == 'gazebo',
        starts_depth_network=mission and depth_source == 'network',
    )
