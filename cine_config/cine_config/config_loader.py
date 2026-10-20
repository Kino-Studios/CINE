from typing import Any

from cine_config import load

CONFIG_FILES = {
    'camera': 'camera/camera-params',
    'costs': 'cost_functions/cost-params',
    'light': 'light_avoidance/orientation-params',
    'mlp': 'mlp/mlp-params',
    'topics': 'topics/topic-params',
    'defaults': 'trajectory_constraints/defaults',
    'focus': 'trajectory_constraints/focus',
    'params': 'user_params/params',
    'waypoints': 'waypoint_generation/waypoint-params',
    'sensing': 'sensing/sensory-params',
}


class ConfigLoader:
    """Process-wide cache of every CINE JSON config file, keyed by short name."""

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            instance = super().__new__(cls)
            instance._configs = {key: load(path) for key, path in CONFIG_FILES.items()}
            cls._instance = instance

        return cls._instance

    def get(self, config_key: str, section: str, key: str, default: Any = None) -> Any:
        return self._configs.get(config_key, {}).get(section, {}).get(key, default)

    def get_section(self, config_key: str, section: str) -> dict[str, Any]:
        return self._configs.get(config_key, {}).get(section, {})
