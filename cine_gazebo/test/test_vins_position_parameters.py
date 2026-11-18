from pathlib import Path

PARAMETER_FILE = Path(__file__).resolve().parents[1] / 'config' / 'ardupilot' / 'vins_position.parm'


def _parameters():
    parameters = {}
    for line in PARAMETER_FILE.read_text().splitlines():
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
