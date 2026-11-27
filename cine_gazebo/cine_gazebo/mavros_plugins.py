"""Builds the MAVROS plugin deny list the simulator starts MAVROS with; free of ROS imports."""

import yaml

# guided_target sends the planner a goal millions of metres away, which exhausts its memory.
SIMULATOR_DENIED_PLUGINS = ('guided_target',)


def mavros_plugin_denylist(shipped_pluginlists_yaml: str) -> list[str]:
    # A later parameter replaces the shipped list, so the shipped entries are carried over.
    shipped = yaml.safe_load(shipped_pluginlists_yaml)['/**']['ros__parameters']['plugin_denylist']

    return shipped + [plugin for plugin in SIMULATOR_DENIED_PLUGINS if plugin not in shipped]
