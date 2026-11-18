import pytest
from cine_gazebo.flight_starter_logic import (
    FlightStarter,
    Observation,
    StarterConfig,
    StarterState,
)


def _observation(**overrides):
    fields = {
        'time_s': 0.0,
        'connected': True,
        'mode': 'STABILIZE',
        'armed': False,
        'has_local_position': True,
        'altitude_m': 0.0,
        'autopilot_velocity': (0.0, 0.0, 0.0),
        'vio_velocity': None,
        'vio_age_s': None,
        'position_error_m': 0.0,
        'ekf_source_set': 1,
        'target_index': 0,
    }
    fields.update(overrides)

    return Observation(**fields)


def _sim_config(**overrides):
    return StarterConfig(nav_source='sim', **overrides)


def _vins_config(**overrides):
    fields = {'excitation_pattern': ((1.0, 0.0, 2.5, 0.0),)}
    fields.update(overrides)

    return StarterConfig(nav_source='vins', **fields)


def _starter_just_arrived_at_altitude(config, time_s=100.0):
    starter = FlightStarter(config)
    starter.step(_observation(time_s=time_s))
    starter.step(_observation(time_s=time_s + 1, mode='GUIDED'))
    starter.step(_observation(time_s=time_s + 2, mode='GUIDED', armed=True))

    return starter


def _vins_starter_exciting(config=None, time_s=100.0):
    starter = _starter_just_arrived_at_altitude(config or _vins_config(), time_s)
    starter.step(_observation(time_s=time_s + 3, mode='GUIDED', armed=True, altitude_m=2.4))

    return starter


def _flying(time_s, **overrides):
    fields = {'time_s': time_s, 'mode': 'GUIDED', 'armed': True, 'altitude_m': 2.5}
    fields.update(overrides)

    return _observation(**fields)


def _agreeing(time_s, **overrides):
    fields = {
        'time_s': time_s,
        'mode': 'GUIDED',
        'armed': True,
        'altitude_m': 2.5,
        'vio_velocity': (0.1, 0.0, 0.0),
        'vio_age_s': 0.05,
        'position_error_m': 2.0,
    }
    fields.update(overrides)

    return _observation(**fields)


def test_waits_without_arming_until_mavros_connects():
    starter = FlightStarter(_sim_config())

    actions = [starter.step(_observation(time_s=0.0, connected=False))]
    actions.append(starter.step(_observation(time_s=10.0, connected=False)))

    assert [action.kind for action in actions] == ['none', 'none']
    assert starter.state is StarterState.WAITING_FOR_LINK


def test_waits_for_ekf_position_before_guided():
    starter = FlightStarter(_sim_config())

    action = starter.step(_observation(has_local_position=False))

    assert action.kind == 'none'
    assert starter.state is StarterState.WAITING_FOR_POSITION


def test_gives_up_when_link_never_arrives():
    starter = FlightStarter(_sim_config())
    starter.step(_observation(time_s=0.0, connected=False))

    action = starter.step(_observation(time_s=121.0, connected=False))

    assert starter.state is StarterState.FAILED
    assert 'MAVROS' in action.message


def test_gives_up_when_ekf_position_never_arrives():
    starter = FlightStarter(_sim_config())
    starter.step(_observation(time_s=0.0, has_local_position=False))

    action = starter.step(_observation(time_s=121.0, has_local_position=False))

    assert starter.state is StarterState.FAILED
    assert 'local position' in action.message


def test_sim_mode_sequence_reaches_trigger():
    starter = FlightStarter(_sim_config())

    actions = [
        starter.step(_observation(time_s=0.0)),
        starter.step(_observation(time_s=1.0, mode='GUIDED')),
        starter.step(_observation(time_s=2.0, mode='GUIDED', armed=True)),
        starter.step(_observation(time_s=3.0, mode='GUIDED', armed=True, altitude_m=2.4)),
    ]

    assert [action.kind for action in actions] == [
        'set_mode',
        'arm',
        'takeoff',
        'trigger',
    ]
    assert actions[0].mode == 'GUIDED'
    assert actions[2].altitude_m == 2.5
    assert starter.state is StarterState.DONE


def test_takeoff_timeout_lands_the_armed_vehicle():
    starter = _starter_just_arrived_at_altitude(_sim_config(), time_s=0.0)

    action = starter.step(_flying(33.0, altitude_m=0.0))

    assert action.kind == 'land'
    assert starter.state is StarterState.LANDING_ON_GPS
    assert 'takeoff' in action.message


def test_vins_mode_switches_only_after_steady_agreement():
    starter = _vins_starter_exciting()
    starter.step(_agreeing(200.0))

    early_action = starter.step(_agreeing(204.9))
    ready_action = starter.step(_agreeing(205.0))

    assert early_action.kind != 'switch_source'
    assert ready_action.kind == 'switch_source'
    assert ready_action.source_set == 2


def test_stale_vio_resets_readiness():
    starter = _vins_starter_exciting()
    starter.step(_agreeing(200.0))
    starter.step(_agreeing(204.0))
    starter.step(_agreeing(204.5, vio_age_s=0.5))

    kinds_after_reset = [
        starter.step(_agreeing(205.0)).kind,
        starter.step(_agreeing(209.0)).kind,
    ]

    assert 'switch_source' not in kinds_after_reset


def test_disagreeing_vio_velocity_resets_readiness():
    starter = _vins_starter_exciting()
    starter.step(_agreeing(200.0))
    starter.step(_agreeing(204.0))
    starter.step(_agreeing(204.5, vio_velocity=(0.5, 0.0, 0.0)))

    kinds_after_reset = [
        starter.step(_agreeing(205.0)).kind,
        starter.step(_agreeing(209.0)).kind,
    ]

    assert 'switch_source' not in kinds_after_reset


def test_excitation_advances_to_next_setpoint_when_reached():
    pattern = ((1.0, 0.0, 2.5, 0.0), (0.0, 1.0, 2.5, 1.5))
    starter = _vins_starter_exciting(_vins_config(excitation_pattern=pattern))

    action = starter.step(_flying(110.0, position_error_m=0.2, target_index=0))

    assert action.kind == 'setpoint'
    assert action.setpoint == (0.0, 1.0, 2.5, 1.5)


def test_vins_never_ready_lands_on_gps():
    starter = _vins_starter_exciting()
    starter.step(_flying(110.0, position_error_m=0.1))
    starter.step(_flying(111.0, position_error_m=0.1))

    land_action = starter.step(_flying(112.0, position_error_m=0.1))
    starter.step(_observation(time_s=120.0, armed=False))

    assert land_action.kind == 'land'
    assert starter.state is StarterState.FAILED
    assert 'VINS did not initialise' in starter.failure_message


def test_drift_after_switch_lands():
    starter = _vins_starter_exciting()
    starter.step(_agreeing(200.0))
    starter.step(_agreeing(205.0))

    action = starter.step(_agreeing(206.0, position_error_m=0.8, ekf_source_set=2))

    assert action.kind == 'land'
    assert starter.state is StarterState.LANDING_ON_GPS


def test_steady_hold_after_switch_triggers_mission():
    starter = _vins_starter_exciting()
    starter.step(_agreeing(200.0))
    starter.step(_agreeing(205.0))
    starter.step(_agreeing(206.0, position_error_m=0.1, ekf_source_set=2))

    action = starter.step(_agreeing(211.0, position_error_m=0.1, ekf_source_set=2))

    assert action.kind == 'trigger'
    assert starter.state is StarterState.DONE


def test_rejects_unknown_nav_source():
    with pytest.raises(ValueError, match='nav_source'):
        FlightStarter(StarterConfig(nav_source='gps'))


def _starter_landing_after_drift():
    starter = _vins_starter_exciting()
    starter.step(_agreeing(200.0))
    starter.step(_agreeing(205.0))
    starter.step(_agreeing(206.0, position_error_m=0.8, ekf_source_set=2))

    return starter


def test_refused_guided_mode_fails_naming_the_step():
    starter = FlightStarter(_sim_config())
    starter.step(_observation(time_s=0.0))

    action = starter.step(_observation(time_s=11.0))

    assert starter.state is StarterState.FAILED
    assert 'GUIDED mode refused' in action.message


def test_refused_arm_fails_naming_the_step():
    starter = FlightStarter(_sim_config())
    starter.step(_observation(time_s=0.0, mode='GUIDED'))

    action = starter.step(_observation(time_s=11.0, mode='GUIDED'))

    assert starter.state is StarterState.FAILED
    assert 'arming refused' in action.message


def test_takeoff_is_re_sent_while_vehicle_is_still_on_the_ground():
    starter = _starter_just_arrived_at_altitude(_sim_config())

    action = starter.step(_flying(103.0, altitude_m=0.1))

    assert action.kind == 'takeoff'


def test_takeoff_is_not_re_sent_once_vehicle_is_climbing():
    starter = _starter_just_arrived_at_altitude(_sim_config())

    action = starter.step(_flying(103.0, altitude_m=0.5))

    assert action.kind == 'none'


def test_switch_source_is_re_sent_until_ekf_confirms_external_vision():
    starter = _vins_starter_exciting()
    starter.step(_agreeing(200.0))
    starter.step(_agreeing(205.0))

    action = starter.step(_agreeing(206.0, ekf_source_set=1))

    assert action.kind == 'switch_source'
    assert starter.state is StarterState.SWITCHING_SOURCE


def test_unconfirmed_source_switch_lands_on_gps():
    starter = _vins_starter_exciting()
    starter.step(_agreeing(200.0))
    starter.step(_agreeing(205.0))

    action = starter.step(_agreeing(216.0, ekf_source_set=1))

    assert action.kind == 'land'
    assert action.source_set == 1
    assert 'EKF source switch refused' in action.message


def test_hold_lands_when_ekf_falls_back_from_external_vision():
    starter = _vins_starter_exciting()
    starter.step(_agreeing(200.0))
    starter.step(_agreeing(205.0))
    starter.step(_agreeing(206.0, position_error_m=0.1, ekf_source_set=2))

    action = starter.step(_agreeing(207.0, position_error_m=0.1, ekf_source_set=1))

    assert action.kind == 'land'
    assert starter.state is StarterState.LANDING_ON_GPS


def test_land_is_re_sent_until_autopilot_enters_land_mode():
    starter = _starter_landing_after_drift()

    action = starter.step(_flying(207.0, mode='GUIDED'))

    assert action.kind == 'land'


def test_land_is_not_re_sent_once_autopilot_is_in_land_mode():
    starter = _starter_landing_after_drift()

    action = starter.step(_flying(207.0, mode='LAND'))

    assert action.kind == 'none'


def test_landing_that_never_disarms_fails():
    starter = _starter_landing_after_drift()

    action = starter.step(_flying(267.0, mode='LAND'))

    assert starter.state is StarterState.FAILED
    assert 'landing did not complete' in action.message


def test_link_loss_after_takeoff_fails_without_commands():
    starter = _vins_starter_exciting()

    action = starter.step(_flying(110.0, connected=False))

    assert starter.state is StarterState.FAILED
    assert action.kind == 'none'
    assert 'MAVROS link lost' in action.message


def test_unexpected_disarm_after_takeoff_fails_without_commands():
    starter = _vins_starter_exciting()

    action = starter.step(_flying(110.0, armed=False))

    assert starter.state is StarterState.FAILED
    assert action.kind == 'none'
    assert 'disarmed unexpectedly' in action.message


def test_autopilot_mode_change_after_takeoff_fails_without_commands():
    starter = _vins_starter_exciting()

    action = starter.step(_flying(110.0, mode='RTL'))

    assert starter.state is StarterState.FAILED
    assert action.kind == 'none'
    assert 'mode changed to RTL by the autopilot' in action.message


def test_observation_of_stale_target_does_not_advance_excitation():
    pattern = ((1.0, 0.0, 2.5, 0.0), (0.0, 1.0, 2.5, 1.5))
    starter = _vins_starter_exciting(_vins_config(excitation_pattern=pattern))
    starter.step(_flying(110.0, position_error_m=0.1, target_index=0))

    action = starter.step(_flying(111.0, position_error_m=0.1, target_index=0))

    assert action.setpoint == (0.0, 1.0, 2.5, 1.5)


MOVING = (1.2, 0.0, 0.0)


def _starter_braking_for_switch():
    starter = _vins_starter_exciting()
    starter.step(_agreeing(200.0))
    starter.step(_agreeing(205.0, autopilot_velocity=MOVING, vio_velocity=MOVING))

    return starter


def test_moving_vehicle_is_told_to_hold_its_position_before_the_source_switch():
    starter = _vins_starter_exciting()
    starter.step(_agreeing(200.0))

    action = starter.step(_agreeing(205.0, autopilot_velocity=MOVING, vio_velocity=MOVING))

    assert action.kind == 'hold_position'
    assert starter.state is StarterState.SETTLING


def test_source_switch_waits_while_overshoot_has_the_vehicle_far_from_the_hold_point():
    starter = _starter_braking_for_switch()

    action = starter.step(
        _agreeing(
            206.0, autopilot_velocity=(0.05, 0.0, 0.0), target_index=None, position_error_m=0.9
        )
    )

    assert action.kind == 'none'
    assert starter.state is StarterState.SETTLING


def test_source_switch_follows_once_the_vehicle_rests_at_the_hold_point():
    starter = _starter_braking_for_switch()

    action = starter.step(_agreeing(207.0, target_index=None, position_error_m=0.1))

    assert action.kind == 'switch_source'
    assert starter.state is StarterState.SWITCHING_SOURCE


def test_vehicle_that_never_settles_lands_with_a_message_saying_so():
    starter = _starter_braking_for_switch()

    action = starter.step(
        _agreeing(221.0, autopilot_velocity=MOVING, target_index=None, position_error_m=0.9)
    )

    assert action.kind == 'land'
    assert 'did not settle' in action.message


def test_hold_after_braking_is_confirmed_by_the_full_confirm_window():
    starter = _starter_braking_for_switch()
    starter.step(_agreeing(207.0, target_index=None, position_error_m=0.1))
    starter.step(_agreeing(208.0, target_index=None, position_error_m=0.1, ekf_source_set=2))

    early = starter.step(
        _agreeing(212.0, target_index=None, position_error_m=0.1, ekf_source_set=2)
    )
    confirmed = starter.step(
        _agreeing(213.0, target_index=None, position_error_m=0.1, ekf_source_set=2)
    )

    assert early.kind == 'none'
    assert confirmed.kind == 'trigger'


def test_position_jump_at_the_switch_after_braking_still_fails_the_hold():
    starter = _starter_braking_for_switch()
    starter.step(_agreeing(207.0, target_index=None, position_error_m=0.1))
    starter.step(_agreeing(208.0, target_index=None, position_error_m=0.1, ekf_source_set=2))

    action = starter.step(
        _agreeing(209.0, target_index=None, position_error_m=2.5, ekf_source_set=2)
    )

    assert action.kind == 'land'
    assert starter.state is StarterState.LANDING_ON_GPS
