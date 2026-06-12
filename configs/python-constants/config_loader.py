"""Singleton loader for the CINE JSON configuration files.

This is the low-level configuration layer: it reads every JSON file under
``configs/json_constants`` once, caches the parsed dictionaries on a process
singleton, and exposes ``get``/``get_section`` accessors. Missing config *files*
are treated as fatal misconfiguration and raise ``FileNotFoundError``; missing
*keys* fall back to the per-call ``default`` (callers depend on this for
optional settings).

Boundary note: this loader is distinct from the ``ConfigLoader`` in
``cinematographic_adjustment_agent/.../setpoint_adjustment/utils/config_loader.py``.
That class is a ROS 2 parameter shim that declares and reads node parameters; it
consumes the dataclasses this module populates. The two are intentionally not
merged (see that module's docstring).
"""

import json
import logging
import os
from typing import Any, Dict

try:
    from .paths import repo_root_from
except (ImportError, ValueError):
    from paths import repo_root_from

logger = logging.getLogger(__name__)

CONFIG_FILES: Dict[str, str] = {
    "camera": "camera/camera-params.json",
    "costs": "cost_functions/cost-params.json",
    "light": "light_avoidance/orientation-params.json",
    "mlp": "mlp/mlp-params.json",
    "topics": "topics/topic-params.json",
    "defaults": "trajectory_constraints/defaults.json",
    "focus": "trajectory_constraints/focus.json",
    "params": "user_params/params.json",
    "waypoints": "waypoint_generation/waypoint-params.json",
    "sensing": "sensing/sensory-params.json",
}


class ConfigLoader:
    _instance = None
    _configs: Dict[str, Any] = {}

    def __new__(cls) -> "ConfigLoader":
        if cls._instance is None:
            cls._instance = super(ConfigLoader, cls).__new__(cls)
            cls._instance._load_all_configs()

        return cls._instance

    def _load_all_configs(self) -> None:
        root = repo_root_from(__file__)
        config_dir = os.path.join(root, "configs", "json_constants")

        for key, relative_path in CONFIG_FILES.items():
            full_path = os.path.join(config_dir, relative_path)

            if not os.path.isfile(full_path):
                raise FileNotFoundError(
                    f"Required config file for '{key}' not found: {full_path}"
                )

            with open(full_path, "r") as handle:
                try:
                    self._configs[key] = json.load(handle)
                except json.JSONDecodeError as exc:
                    raise ValueError(
                        f"Invalid JSON in config file '{full_path}': {exc}"
                    ) from exc

    def get(self, config_key: str, section: str, key: str, default: Any = None) -> Any:
        """Return a nested config value, or ``default`` when the key is absent."""
        return self._configs \
            .get(config_key, {}) \
            .get(section, {}) \
            .get(key, default)

    def get_section(self, config_key: str, section: str) -> Dict[str, Any]:
        """Return an entire config section as a dict (empty when absent)."""
        return self._configs \
            .get(config_key, {}) \
            .get(section, {})
