"""Print one CINE constant for shell scripts: ``python3 -m cine_config.get_config DRONE_ID``."""

import sys

from cine_config.constants import (
    CinematicConstraints,
    CinematographyParams,
    SimulationParams,
    WaypointParams,
)


def main():
    if len(sys.argv) < 2:
        sys.exit(1)

    param_name = sys.argv[1]

    for params_class in [
        SimulationParams,
        WaypointParams,
        CinematicConstraints,
        CinematographyParams,
    ]:
        if hasattr(params_class, param_name):
            value = getattr(params_class, param_name)

            if isinstance(value, bool):
                print('1' if value else '0')
            else:
                print(value)

            return

    print(f'error: {param_name} not found', file=sys.stderr)
    sys.exit(1)


if __name__ == '__main__':
    main()
