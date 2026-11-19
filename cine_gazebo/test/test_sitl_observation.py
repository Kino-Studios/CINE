import math
from pathlib import Path

import pytest
from cine_gazebo.flight_starter_logic import DEFAULT_EXCITATION_PATTERN
from cine_gazebo.sitl_observation import (
    CommandGate,
    LinkState,
    OdometrySample,
    SensorStatus,
    autopilot_velocity_in_local,
    build_observation,
    land_request,
    load_excitation_pattern,
    vio_velocity_in_local,
)

SHIPPED_PATTERN_FILE = Path(__file__).resolve().parents[1] / 'config' / 'excitation_pattern.yaml'
# MAV_SYS_STATUS_PREARM_CHECK in MAVLink's SYS_STATUS sensor bitmasks.
PREARM_CHECK_BIT = 0x10000000
FACING_NORTH = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
FACING_EAST = (0.0, 0.0, 0.0, 1.0)
PATTERN = ((1.5, 0.0, 2.5, 0.0), (1.5, 1.5, 2.5, 1.57))


def test_gate_refuses_a_call_while_the_same_kind_is_pending():
    gate = CommandGate()
    gate.sent('arm', now_s=10.0)

    assert not gate.may_send('arm', now_s=12.0)


def test_gate_refuses_a_second_call_of_the_same_kind_within_one_second():
    gate = CommandGate()
    gate.sent('arm', now_s=10.0)
    gate.answered('arm')

    assert not gate.may_send('arm', now_s=10.5)


def test_gate_allows_the_same_kind_again_one_second_after_an_answer():
    gate = CommandGate()
    gate.sent('arm', now_s=10.0)
    gate.answered('arm')

    assert gate.may_send('arm', now_s=11.0)


def test_gate_does_not_block_a_different_kind():
    gate = CommandGate()
    gate.sent('arm', now_s=10.0)

    assert gate.may_send('set_mode', now_s=10.1)


def test_gate_releases_a_call_mavros_never_answers_after_five_seconds():
    gate = CommandGate()
    gate.sent('arm', now_s=10.0)

    assert gate.may_send('arm', now_s=15.0)


def test_shipped_excitation_pattern_matches_the_state_machine_default():
    assert load_excitation_pattern(SHIPPED_PATTERN_FILE) == DEFAULT_EXCITATION_PATTERN


def test_excitation_pattern_with_repeated_points_is_rejected(tmp_path):
    pattern_file = tmp_path / 'pattern.yaml'
    pattern_file.write_text('excitation_pattern:\n  - [1, 0, 2, 0]\n  - [1, 0, 2, 0]\n')

    with pytest.raises(ValueError, match='distinct'):
        load_excitation_pattern(pattern_file)


def test_excitation_point_without_yaw_is_rejected(tmp_path):
    pattern_file = tmp_path / 'pattern.yaml'
    pattern_file.write_text('excitation_pattern:\n  - [1, 0, 2]\n')

    with pytest.raises(ValueError, match='yaw'):
        load_excitation_pattern(pattern_file)


def test_forward_body_velocity_facing_north_becomes_north_velocity():
    autopilot = OdometrySample(
        position=(0.0, 0.0, 0.0),
        orientation=FACING_NORTH,
        linear_velocity=(1.0, 0.0, 0.0),
        received_s=0.0,
    )

    assert autopilot_velocity_in_local(autopilot) == pytest.approx((0.0, 1.0, 0.0))


def test_vio_velocity_is_turned_by_the_heading_offset_between_estimators():
    autopilot = OdometrySample(
        position=(0.0, 0.0, 0.0),
        orientation=FACING_NORTH,
        linear_velocity=(0.0, 0.0, 0.0),
        received_s=0.0,
    )
    # VINS believes the same vehicle faces its own x axis, so its world is turned 90 degrees.
    vio = OdometrySample(
        position=(0.0, 0.0, 0.0),
        orientation=FACING_EAST,
        linear_velocity=(1.0, 0.0, 0.5),
        received_s=0.0,
    )

    assert vio_velocity_in_local(autopilot, vio) == pytest.approx((0.0, 1.0, 0.5))


def test_error_is_measured_against_the_setpoint_whose_index_is_reported():
    autopilot = OdometrySample(
        position=(1.5, 1.5, 2.5),
        orientation=FACING_EAST,
        linear_velocity=(0.0, 0.0, 0.0),
        received_s=5.0,
    )

    observation = build_observation(
        now_s=5.0,
        link=LinkState(connected=True, mode='GUIDED', armed=True),
        autopilot=autopilot,
        vio=None,
        ekf_source_set=1,
        excitation_pattern=PATTERN,
        target_index=1,
        hold_setpoint=None,
        sensor_status=None,
    )

    assert (observation.target_index, observation.position_error_m) == (1, 0.0)


def test_observation_before_any_message_carries_the_boot_source_set():
    observation = build_observation(
        now_s=0.0,
        link=None,
        autopilot=None,
        vio=None,
        ekf_source_set=1,
        excitation_pattern=PATTERN,
        target_index=None,
        hold_setpoint=None,
        sensor_status=None,
    )

    assert observation.ekf_source_set == 1


def test_observation_refuses_an_unknown_source_set():
    with pytest.raises(ValueError, match='EKF source set'):
        build_observation(
            now_s=0.0,
            link=None,
            autopilot=None,
            vio=None,
            ekf_source_set=None,
            excitation_pattern=PATTERN,
            target_index=None,
            hold_setpoint=None,
            sensor_status=None,
        )


def _observation_with_sensor_status(sensor_status):
    return build_observation(
        now_s=0.0,
        link=None,
        autopilot=None,
        vio=None,
        ekf_source_set=1,
        excitation_pattern=PATTERN,
        target_index=None,
        hold_setpoint=None,
        sensor_status=sensor_status,
    )


def test_prearm_checks_have_not_passed_before_the_first_status_message():
    observation = _observation_with_sensor_status(None)

    assert observation.prearm_checks_passed is False


def test_prearm_checks_pass_when_the_autopilot_reports_the_prearm_bit_healthy():
    status = SensorStatus(sensors_enabled=PREARM_CHECK_BIT, sensors_health=PREARM_CHECK_BIT)

    observation = _observation_with_sensor_status(status)

    assert observation.prearm_checks_passed is True


def test_prearm_checks_have_not_passed_while_the_enabled_prearm_bit_is_unhealthy():
    status = SensorStatus(sensors_enabled=PREARM_CHECK_BIT, sensors_health=0)

    observation = _observation_with_sensor_status(status)

    assert observation.prearm_checks_passed is False


def test_prearm_checks_count_as_passed_when_the_autopilot_has_them_disabled():
    status = SensorStatus(sensors_enabled=0, sensors_health=0)

    observation = _observation_with_sensor_status(status)

    assert observation.prearm_checks_passed is True


def test_landing_on_gps_asks_for_land_mode_immediately():
    assert land_request(active_source_set=1, now_s=20.0, switch_requested_s=None) == 'land_mode'


def test_landing_on_external_vision_switches_to_gps_first():
    assert land_request(active_source_set=2, now_s=20.0, switch_requested_s=None) == (
        'switch_source'
    )


def test_landing_keeps_switching_while_the_switch_is_under_five_seconds_old():
    assert land_request(active_source_set=2, now_s=24.9, switch_requested_s=20.0) == (
        'switch_source'
    )


def test_landing_lands_anyway_five_seconds_after_an_unaccepted_switch():
    assert land_request(active_source_set=2, now_s=25.0, switch_requested_s=20.0) == 'land_mode'
