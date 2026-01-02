import sys
from pathlib import Path

here = Path(__file__).resolve().parent
if str(here) not in sys.path:
    sys.path.insert(0, str(here))

from constants import SimulationParams, WaypointParams, CinematicConstraints, CinematographyParams

def main():
    if len(sys.argv) < 2:
        sys.exit(1)
    
    param_name = sys.argv[1]
    
    for cls in [SimulationParams, WaypointParams, CinematicConstraints, CinematographyParams]:
        if hasattr(cls, param_name):
            val = getattr(cls, param_name)

            if isinstance(val, bool):
                print("1" if val else "0")
            else:
                print(val)

            return
            
    print(f"error: {param_name} not found", file=sys.stderr)
    sys.exit(1)

if __name__ == "__main__":
    main()
