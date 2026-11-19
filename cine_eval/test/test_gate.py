import pytest
from cine_eval.channels.result import OK, ChannelResult, unchecked
from cine_eval.gate import judge
from cine_eval.score import RunScore
from cine_eval.settings import load_gate_thresholds, load_scoring_settings

THRESHOLDS = load_gate_thresholds()

HEALTHY_CLEARANCE = {
    'collision_free': 1.0,
    'min_static_clearance_m': 2.0,
    'min_subject_clearance_m': 3.0,
    'min_ground_clearance_m': 1.0,
    'min_clearance_m': 1.0,
}


def _ok(name, summary):
    return ChannelResult(name=name, status=OK, reason='', summary=summary)


def _score(
    clearance=None,
    mission=None,
    motion=None,
    focus=None,
    following=None,
    freshness=None,
):
    return RunScore(
        run_name='run',
        channels={
            'clearance': clearance or _ok('clearance', HEALTHY_CLEARANCE),
            'mission': mission or _ok('mission', {'completed': 1.0, 'aborted': 0.0}),
            'motion': motion or _ok('motion', {'peak_speed_mps': 0.3}),
            'focus': focus or _ok('focus', {'held_share': 0.9}),
            'following': following or _ok('following', {'max_setpoint_jump_m': 0.1}),
            'freshness': freshness or _ok('freshness', {'max_age_s': 0.05}),
        },
        settings=load_scoring_settings().as_dict(),
        provenance={},
    )


def _clearance(**changes):
    return _ok('clearance', {**HEALTHY_CLEARANCE, **changes})


def _clearance_without(*keys):
    return _ok(
        'clearance',
        {key: value for key, value in HEALTHY_CLEARANCE.items() if key not in keys},
    )


def _judge(score, run_metadata=None):
    metadata = {'real_time_factor': 1.0} if run_metadata is None else run_metadata

    return judge(score, metadata, THRESHOLDS)


def test_healthy_run_is_pass():
    assert _judge(_score()).label == 'PASS'


def test_any_failure_makes_the_label_review():
    verdict = _judge(_score(mission=_ok('mission', {'completed': 0.0})))

    assert verdict.label == 'REVIEW'


def test_unchecked_channel_puts_its_rules_in_unchecked_not_failures():
    verdict = _judge(_score(freshness=unchecked('freshness', 'no stream')))

    assert 'data_age' in verdict.unchecked
    assert verdict.failures == ()


def test_unchecked_clearance_labels_run_review_not_pass():
    verdict = _judge(_score(clearance=unchecked('clearance', 'no drone_true')))

    assert verdict.failures == ()
    assert 'no_contact' in verdict.unchecked
    assert verdict.label == 'REVIEW'


def test_missing_subject_class_puts_subject_contact_check_in_unchecked():
    verdict = _judge(_score(clearance=_clearance_without('min_subject_clearance_m')))

    assert 'no_contact:subject' in verdict.unchecked
    assert 'min_clearance:subject' in verdict.unchecked
    assert verdict.label == 'REVIEW'


def test_missing_static_class_puts_static_contact_check_in_unchecked():
    verdict = _judge(_score(clearance=_clearance_without('min_static_clearance_m')))

    assert 'no_contact:static' in verdict.unchecked
    assert 'min_clearance:static' in verdict.unchecked
    assert verdict.label == 'REVIEW'


def test_contact_in_a_present_class_still_fails_when_another_class_is_missing():
    verdict = _judge(
        _score(
            clearance=_ok(
                'clearance',
                {'collision_free': 0.0, 'min_ground_clearance_m': -0.05, 'first_contact_s': 2.0},
            ),
        ),
    )

    assert verdict.failures[0] == (
        'no_contact: contact with ground (min -0.05 m < 0 m); first contact at 2.0 s'
    )


def test_contact_fails_the_gate():
    verdict = _judge(_score(clearance=_clearance(collision_free=0.0)))

    assert verdict.failures == ('no_contact: contact reported',)


def test_contact_failure_names_the_subject_class():
    verdict = _judge(
        _score(
            clearance=_clearance(
                collision_free=0.0,
                min_subject_clearance_m=-0.12,
                first_contact_s=12.3,
            ),
        ),
    )

    assert verdict.failures[0] == (
        'no_contact: contact with subject (min -0.12 m < 0 m); first contact at 12.3 s'
    )


def test_contact_failure_names_the_ground_class():
    verdict = _judge(_score(clearance=_clearance(collision_free=0.0, min_ground_clearance_m=-0.05)))

    assert verdict.failures[0] == 'no_contact: contact with ground (min -0.05 m < 0 m)'


def test_contact_failure_names_every_contacting_class():
    verdict = _judge(
        _score(
            clearance=_clearance(
                collision_free=0.0,
                min_static_clearance_m=-0.01,
                min_subject_clearance_m=-0.4,
                first_contact_s=5.0,
            ),
        ),
    )

    assert verdict.failures[0] == (
        'no_contact: contact with static (min -0.01 m < 0 m), subject (min -0.40 m < 0 m); '
        'first contact at 5.0 s'
    )


def test_slow_real_time_factor_is_review():
    verdict = _judge(_score(), {'real_time_factor': 0.8})

    assert verdict.failures == ('real_time_factor: 0.80, below 0.90',)


def test_missing_real_time_factor_is_unchecked():
    verdict = _judge(_score(), {})

    assert 'real_time_factor' in verdict.unchecked
    assert verdict.failures == ()


def test_too_close_to_the_subject_fails_min_subject_clearance():
    verdict = _judge(_score(clearance=_clearance(min_subject_clearance_m=0.2)))

    assert verdict.failures == ('min_subject_clearance: subject clearance 0.20 m, below 0.50 m',)


def test_speed_above_the_filming_cap_fails():
    verdict = _judge(_score(motion=_ok('motion', {'peak_speed_mps': 50.0})))

    assert verdict.failures[0].startswith('speed_cap_while_filming: peak speed 50.00 m/s')


def test_rule_with_absent_summary_key_is_unchecked_not_failed():
    verdict = _judge(_score(mission=_ok('mission', {'duration_s': 30.0})))

    assert 'mission_completed' in verdict.unchecked
    assert verdict.failures == ()


def test_partial_mission_completion_fails_and_names_value_and_threshold():
    verdict = _judge(_score(mission=_ok('mission', {'completed': 0.5})))

    assert verdict.failures == ('mission_completed: completed 0.50, below 1.00',)


def test_unknown_channel_status_is_rejected():
    score = _score(freshness=ChannelResult(name='freshness', status='OK', reason='', summary={}))

    with pytest.raises(ValueError, match="unknown status 'OK'"):
        _judge(score)


def test_provisional_flag_is_copied_from_the_thresholds():
    assert _judge(_score()).provisional == THRESHOLDS.provisional
