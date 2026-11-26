from pathlib import Path

PARAMETER_FILE = Path(__file__).resolve().parents[1] / 'config' / 'ardupilot' / 'vins_position.parm'


def _parameters():
    return _parameters_of(PARAMETER_FILE)


def _parameters_of(path):
    parameters = {}

    for line in path.read_text().splitlines():
        stripped = line.strip()
        if stripped and not stripped.startswith('#'):
            name, value = stripped.split()
            parameters[name] = float(value)

    return parameters


def test_vins_source_set_takes_horizontal_position_from_external_navigation():
    assert _parameters()['EK3_SRC2_POSXY'] == 6


def test_vins_source_set_takes_no_horizontal_velocity_because_none_is_forwarded():
    assert _parameters()['EK3_SRC2_VELXY'] == 0


def test_vins_source_set_takes_no_vertical_velocity_because_none_is_forwarded():
    assert _parameters()['EK3_SRC2_VELZ'] == 0


def test_neither_simulator_parameter_set_fixes_the_raw_imu_rate():
    # The imu_rate_requester node sets it from the JSON constants, as on the drone.
    sim_file = PARAMETER_FILE.with_name('sim_position.parm')
    declared = {
        path.name: 'MAV1_RAW_SENS' in _parameters_of(path) for path in (sim_file, PARAMETER_FILE)
    }

    assert declared == {'sim_position.parm': False, 'vins_position.parm': False}
