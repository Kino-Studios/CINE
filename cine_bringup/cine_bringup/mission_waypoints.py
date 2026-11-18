import numbers

import yaml

MAX_WAYPOINTS = 5


def load_mission_waypoints(path: str) -> list[tuple[float, float, float]]:
    with open(path, encoding='utf-8') as waypoints_file:
        loaded = yaml.safe_load(waypoints_file)

    if loaded is None:
        raise ValueError(f'waypoints file {path} is empty')

    if not isinstance(loaded, list):
        raise ValueError(f'waypoints file {path} must be a list of [x, y, z] points')

    if len(loaded) > MAX_WAYPOINTS:
        raise ValueError(f'at most {MAX_WAYPOINTS} waypoints are supported, got {len(loaded)}')

    return [_to_point(point, index) for index, point in enumerate(loaded)]


def _to_point(point, index: int) -> tuple[float, float, float]:
    is_triple = isinstance(point, list) and len(point) == 3
    is_numeric = is_triple and all(
        isinstance(value, numbers.Real) and not isinstance(value, bool) for value in point
    )
    if not is_numeric:
        raise ValueError(f'waypoint {index} must be 3 numbers [x, y, z], got {point!r}')

    return (float(point[0]), float(point[1]), float(point[2]))
