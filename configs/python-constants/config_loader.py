import json
import os
from typing import Any, Dict

try:
    from .paths import repo_root_from
except (ImportError, ValueError):
    from paths import repo_root_from

class ConfigLoader:
    _instance = None
    _configs: Dict[str, Any] = {}

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ConfigLoader, cls).__new__(cls)
            cls._instance._load_all_configs()

        return cls._instance

    def _load_all_configs(self):
        root = repo_root_from(__file__)
        config_dir = os.path.join(root, "configs", "json_constants")
        
        config_files = {
            "camera": "camera/camera-params.json",
            "costs": "cost_functions/cost-params.json",
            "light": "light_avoidance/orientation-params.json",
            "topics": "topics/topic-params.json",
            "defaults": "trajectory_constraints/defaults.json",
            "focus": "trajectory_constraints/focus.json",
            "params": "user_params/params.json",
            "waypoints": "waypoint_generation/waypoint-params.json",
            "sensing": "sensing/sensory-params.json",
        }

        for key, relative_path in config_files.items():
            full_path = os.path.join(config_dir, relative_path)

            if os.path.exists(full_path):
                with open(full_path, 'r') as f:
                    try:
                        self._configs[key] = json.load(f)
                    except json.JSONDecodeError:
                        print(f"error decoding JSON from {full_path}")
                        self._configs[key] = {}
            else:
                print(f"config file not found: {full_path}")
                self._configs[key] = {}

    def get(self, config_key: str, section: str, key: str, default: Any = None) -> Any:
        return self._configs \
            .get(config_key, {}) \
            .get(section, {}) \
            .get(key, default)

    def get_section(self, config_key: str, section: str) -> Dict[str, Any]:
        return self._configs \
            .get(config_key, {}) \
            .get(section, {})
