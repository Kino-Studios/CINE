import pytest
from cine_eval.settings import load_gate_thresholds, load_scoring_settings


def test_scoring_settings_read_the_committed_values():
    settings = load_scoring_settings()

    assert settings.clock_hz == 50
    assert settings.focus_tolerance_m == 0.1
    assert settings.airframe_radius_m == 0.25


def test_gate_thresholds_carry_the_provisional_flag():
    thresholds = load_gate_thresholds()

    assert thresholds.provisional is True
    assert thresholds.min_real_time_factor == 0.9


def test_scoring_override_replaces_only_the_named_attribute():
    settings = load_scoring_settings({'focus_tolerance_m': 0.0})

    assert settings.focus_tolerance_m == 0.0
    assert settings.clock_hz == 50


def test_scoring_override_rejects_unknown_attribute():
    with pytest.raises(ValueError, match='no_such_setting'):
        load_scoring_settings({'no_such_setting': 1})


def test_as_dict_returns_every_attribute_by_name():
    assert load_scoring_settings().as_dict()['sparc_padding_level'] == 4
    assert load_gate_thresholds().as_dict()['provisional'] is True


def test_scoring_settings_read_the_largest_pose_gap_bridged_by_interpolation():
    assert load_scoring_settings().max_pose_gap_s == 0.2
