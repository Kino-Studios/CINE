import os
import sys
from pathlib import Path


def add_config_path() -> None:
    here = Path(__file__).resolve()
    for parent in [here] + list(here.parents):
        configs_dir = parent / "configs"
        if configs_dir.is_dir():
            python_constants_path = configs_dir / "python-constants"
            if str(python_constants_path) not in sys.path:
                sys.path.insert(0, str(python_constants_path))

            return

    raise RuntimeError("Unable to locate configs/python-constants for constants import.")


def repo_root_from(file_path: str, max_levels_up: int = 12) -> str:
    cur = os.path.abspath(os.path.dirname(file_path))

    for _ in range(int(max_levels_up)):
        if os.path.isdir(os.path.join(cur, "configs")):
            return cur

        if os.path.isdir(os.path.join(cur, "trajectory-optimization_ws")):
            return cur

        parent = os.path.dirname(cur)

        if parent == cur:
            break

        cur = parent

    return os.path.abspath(os.path.dirname(file_path))


def workspace_root_from(file_path: str, levels_up: int = 6) -> str:
    return repo_root_from(file_path, max_levels_up=max(1, int(levels_up) + 6))
