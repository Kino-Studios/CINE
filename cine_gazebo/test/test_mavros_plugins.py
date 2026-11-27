from cine_gazebo.mavros_plugins import mavros_plugin_denylist

SHIPPED_PLUGINLISTS = """\
/**:
  ros__parameters:
    plugin_denylist:
      - actuator_control
      - ftp
      - wheel_odometry
"""


def test_denylist_keeps_every_plugin_mavros_already_denies():
    denylist = mavros_plugin_denylist(SHIPPED_PLUGINLISTS)

    assert {'actuator_control', 'ftp', 'wheel_odometry'} <= set(denylist)


def test_denylist_drops_guided_target_which_publishes_goals_to_the_planner():
    assert 'guided_target' in mavros_plugin_denylist(SHIPPED_PLUGINLISTS)


def test_denylist_names_guided_target_once_when_mavros_already_denies_it():
    shipped = SHIPPED_PLUGINLISTS + '      - guided_target\n'

    assert mavros_plugin_denylist(shipped).count('guided_target') == 1
