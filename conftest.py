"""Lets bare ``pytest`` from the repo root read the source config tree without a ROS install."""

import os
from pathlib import Path

os.environ.setdefault(
    'CINE_CONFIG_ROOT', str(Path(__file__).resolve().parent / 'cine_config' / 'config')
)
