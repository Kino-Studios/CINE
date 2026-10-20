"""Shared CINE configuration: JSON files installed to ``share/cine_config/config``."""

import json
import os
from pathlib import Path

CONFIG_ROOT_ENV = 'CINE_CONFIG_ROOT'


def config_root() -> Path:
    """Directory holding the JSON config tree.

    ``$CINE_CONFIG_ROOT`` wins when set; otherwise the installed package share.
    """
    override = os.environ.get(CONFIG_ROOT_ENV)
    if override:
        return Path(override)

    # Imported here so the loader works without a ROS environment when the override is set.
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory('cine_config')) / 'config'


def load(relative_path: str) -> dict:
    """Parse ``<config root>/<relative_path>.json``, e.g. ``load('camera/camera-params')``."""
    path = config_root() / f'{relative_path}.json'
    try:
        text = path.read_text()
    except FileNotFoundError as exc:
        raise FileNotFoundError(f'CINE config file not found: {path}') from exc

    try:
        return json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f'CINE config file is not valid JSON: {path}: {exc}') from exc
