import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from cine_eval.analysis import compare, condition_key, load_plan
from cine_eval.channels.clearance import score_clearance
from cine_eval.channels.result import OK, ChannelResult, unchecked
from cine_eval.gate import judge
from cine_eval.record import DRONE_TRUE, FlightRecord, Stream
from cine_eval.scene import Box
from cine_eval.score import RunScore, read_score, write_score
from cine_eval.settings import load_gate_thresholds, load_scoring_settings

PLAN_PATH = Path(__file__).resolve().parents[2] / 'eval' / 'analysis_plan.toml'
PLAN = load_plan(PLAN_PATH)


def _run_metadata(seed):
    return {
        'scenario': 'orbit',
        'world_file': 'cine_gazebo/worlds/cine_field.sdf.template',
        'waypoints_sha1': 'a1b2c3',
        'wind': 0.0,
        'seed': seed,
    }


def _write_run(
    root,
    name,
    seed,
    held_share=0.5,
    completed=1.0,
    aborted=0.0,
    collision_free=1.0,
    clearance_checked=True,
    focus_checked=True,
    secondary_shift=0.0,
    filming_s=30.0,
    run_metadata=None,
    clearance=None,
):
    """A scored run directory with a minimal ``record.json``; seed None means no ``run.json``.

    ``completed`` None leaves arrival out of the mission summary, as when it was not measured;
    ``filming_s`` None leaves filming time out of the focus and motion summaries;
    ``run_metadata`` replaces the default run metadata for ``seed``; ``clearance`` replaces
    the clearance channel result.
    """

    directory = root / name
    filming = {} if filming_s is None else {'filming_s': filming_s}
    clearance = clearance or (
        ChannelResult(
            name='clearance',
            status=OK,
            reason='',
            summary={'collision_free': collision_free, 'min_clearance_m': 1.0 + secondary_shift},
        )
        if clearance_checked
        else unchecked('clearance', 'missing streams: scene')
    )
    focus = (
        ChannelResult(
            name='focus',
            status=OK,
            reason='',
            summary={
                'held_share': held_share,
                'worst_deviation_m': 0.5 + secondary_shift,
                **filming,
            },
        )
        if focus_checked
        else unchecked('focus', 'no filming time')
    )
    channels = {
        'mission': ChannelResult(
            name='mission',
            status=OK,
            reason='',
            summary={
                key: value
                for key, value in (
                    ('completed', completed),
                    ('aborted', aborted),
                    ('duration_s', 30.0),
                )
                if value is not None
            },
        ),
        'clearance': clearance,
        'focus': focus,
        'motion': ChannelResult(
            name='motion',
            status=OK,
            reason='',
            summary={
                'log_dimensionless_jerk': -10.0 + secondary_shift,
                'sparc': -3.0 + secondary_shift,
                'rms_jerk_mps3': 2.0 + secondary_shift,
                **filming,
            },
        ),
    }
    write_score(RunScore(run_name=name, channels=channels, settings={}, provenance={}), directory)

    if run_metadata is not None:
        metadata = {'run': run_metadata}
    elif seed is None:
        metadata = {}
    else:
        metadata = {'run': _run_metadata(seed)}

    (directory / 'record.json').write_text(json.dumps({'streams': {}, 'metadata': metadata}))

    return directory


def _write_arm(root, arm, held_shares, secondary_shifts=None):
    """One run per held share, seeds 1, 2, ... in order, named ``<arm>_<seed>``."""

    shifts = secondary_shifts if secondary_shifts is not None else [0.0] * len(held_shares)

    return [
        _write_run(root, f'{arm}_{seed}', seed, held_share=held_share, secondary_shift=shift)
        for seed, (held_share, shift) in enumerate(zip(held_shares, shifts), start=1)
    ]


def _view(metric_comparisons, metric, view):
    return next(
        comparison
        for comparison in metric_comparisons
        if comparison.metric == metric and comparison.view == view
    )


def test_load_plan_records_the_sha1_of_the_file_bytes():
    assert PLAN.digest == hashlib.sha1(PLAN_PATH.read_bytes()).hexdigest()


def test_load_plan_reads_the_primary_metric():
    assert PLAN.primary.metric == 'focus.held_share'


def test_condition_key_is_built_from_the_run_metadata():
    assert condition_key(_run_metadata(4), 'run_a') == condition_key(_run_metadata(4), 'run_b')


def test_condition_key_without_run_metadata_never_pairs():
    assert condition_key({}, 'run_a') == ('unpaired', 'run_a')


def test_condition_key_with_a_missing_field_never_pairs():
    metadata = _run_metadata(4)
    del metadata['wind']

    assert condition_key(metadata, 'run_a') == ('unpaired', 'run_a')


def test_condition_key_pairs_integer_wind_with_float_wind():
    integer_wind = _run_metadata(4) | {'wind': 0}
    float_wind = _run_metadata(4) | {'wind': 0.0}

    assert condition_key(integer_wind, 'run_a') == condition_key(float_wind, 'run_b')


def test_condition_key_keeps_string_seed_apart_from_integer_seed():
    assert condition_key(_run_metadata('1'), 'run_a') != condition_key(_run_metadata(1), 'run_b')


def test_conditions_whose_joined_labels_match_stay_separate_pairs(tmp_path):
    split_scenario = _run_metadata(1) | {'scenario': 'orbit|x', 'world_file': 'w'}
    split_world = _run_metadata(1) | {'scenario': 'orbit', 'world_file': 'x|w'}
    baseline = [
        _write_run(tmp_path, 'baseline_1', 1, held_share=0.5, run_metadata=split_scenario),
        _write_run(tmp_path, 'baseline_2', 1, held_share=0.5, run_metadata=split_world),
    ]
    cine = [
        _write_run(tmp_path, 'cine_1', 1, held_share=0.6, run_metadata=split_scenario),
        _write_run(tmp_path, 'cine_2', 1, held_share=0.7, run_metadata=split_world),
    ]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert len(worst.differences) == 2


def test_condition_key_keeps_boolean_seed_apart_from_its_string():
    assert condition_key(_run_metadata(True), 'run_a') != condition_key(
        _run_metadata('true'),
        'run_b',
    )


def test_condition_key_keeps_null_wind_apart_from_its_string():
    null_wind = _run_metadata(4) | {'wind': None}
    string_wind = _run_metadata(4) | {'wind': 'null'}

    assert condition_key(null_wind, 'run_a') != condition_key(string_wind, 'run_b')


def test_unpaired_runs_with_the_same_name_in_both_arms_are_not_paired(tmp_path):
    baseline = [_write_run(tmp_path / 'baseline', 'flight_07', None)]
    cine = [_write_run(tmp_path / 'cine', 'flight_07', None)]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    assert comparison.pairs == 0
    assert comparison.unpaired_runs == {'baseline': ('flight_07',), 'cine': ('flight_07',)}


def test_record_that_is_null_leaves_the_run_unpaired(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5])
    cine = [_write_run(tmp_path, 'cine_1', 1)]
    (cine[0] / 'record.json').write_text('null')

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    assert comparison.unpaired_runs['cine'] == ('cine_1',)


def test_record_with_list_metadata_leaves_the_run_unpaired(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5])
    cine = [_write_run(tmp_path, 'cine_1', 1)]
    (cine[0] / 'record.json').write_text(json.dumps({'streams': {}, 'metadata': ['run']}))

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    assert comparison.unpaired_runs['cine'] == ('cine_1',)


def test_record_with_null_metadata_leaves_the_run_unpaired(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5])
    cine = [_write_run(tmp_path, 'cine_1', 1)]
    (cine[0] / 'record.json').write_text(json.dumps({'streams': {}, 'metadata': None}))

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    assert comparison.unpaired_runs['cine'] == ('cine_1',)


def test_runs_pair_by_condition_key_and_unpaired_are_listed(tmp_path):
    baseline = [
        _write_run(tmp_path, 'baseline_1', 1),
        _write_run(tmp_path, 'baseline_2', 2),
        _write_run(tmp_path, 'baseline_loose', None),
    ]
    cine = [
        _write_run(tmp_path, 'cine_1', 1),
        _write_run(tmp_path, 'cine_2', 2),
        _write_run(tmp_path, 'cine_3', 3),
    ]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    assert comparison.pairs == 2
    assert comparison.unpaired_runs == {'baseline': ('baseline_loose',), 'cine': ('cine_3',)}


def test_repeats_of_one_condition_collapse_to_their_median(tmp_path):
    baseline = [
        _write_run(tmp_path, 'baseline_1a', 1, held_share=0.2),
        _write_run(tmp_path, 'baseline_1b', 1, held_share=0.3),
        _write_run(tmp_path, 'baseline_1c', 1, held_share=0.7),
    ]
    cine = [_write_run(tmp_path, 'cine_1', 1, held_share=0.5)]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    primary = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert primary.hodges_lehmann == pytest.approx(0.2)
    assert list(comparison.repeats['baseline'].values()) == [3]


def test_failed_run_takes_worst_value_and_both_views_are_reported(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = [
        _write_run(tmp_path, 'cine_1', 1, held_share=0.9, completed=0.0),
        _write_run(tmp_path, 'cine_2', 2, held_share=0.6),
        _write_run(tmp_path, 'cine_3', 3, held_share=0.6),
    ]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    removed = _view(comparison.primary, 'focus.held_share', 'failed pairs removed')
    seed_1 = '["orbit", "cine_gazebo/worlds/cine_field.sdf.template", "a1b2c3", 0.0, 1.0]'
    assert worst.differences[seed_1] == -0.5
    assert removed.pairs == 2
    assert removed.removed_failed_runs == ('cine_1',)


def test_aborted_run_counts_as_failed(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5])
    cine = [_write_run(tmp_path, 'cine_1', 1, held_share=0.9, aborted=1.0)]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.hodges_lehmann == -0.5


def test_aborted_run_with_unmeasured_arrival_counts_as_failed_mission(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5])
    cine = [_write_run(tmp_path, 'cine_1', 1, held_share=0.9, completed=None, aborted=1.0)]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.arms[1].run_values['cine_1'] == 0.0
    cine_mission = comparison.mission_success['cine']
    assert (cine_mission.successes, cine_mission.trials) == (0, 1)


def test_collided_run_counts_as_failed_and_lowers_collision_free_rate(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = [
        _write_run(tmp_path, 'cine_1', 1, held_share=0.9, collision_free=0.0),
        _write_run(tmp_path, 'cine_2', 2, held_share=0.6),
        _write_run(tmp_path, 'cine_3', 3, held_share=0.6),
    ]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.arms[1].run_values['cine_1'] == 0.0
    cine_collision_free = comparison.collision_free['cine']
    assert (cine_collision_free.successes, cine_collision_free.trials) == (2, 3)
    assert comparison.collided_only == {'baseline': 0, 'cine': 1}


def test_unchecked_clearance_is_left_out_of_the_collision_free_rate(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = [
        _write_run(tmp_path, 'cine_1', 1, clearance_checked=False),
        _write_run(tmp_path, 'cine_2', 2),
        _write_run(tmp_path, 'cine_3', 3),
    ]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    assert comparison.collision_free['cine'].trials == 2
    assert comparison.collision_free['cine'].unchecked_runs == ('cine_1',)


def _clearance_of_a_clean_hover_without_the_subject_stream():
    time = np.arange(0.0, 10.0 + 1e-9, 0.02)
    position = np.tile([2.0, 0.0, 1.0], (len(time), 1))
    drone = Stream(
        name=DRONE_TRUE,
        frame_id='cine_field',
        time=time,
        receive_time=time.copy(),
        fields={'position': position},
    )
    wall = Box('wall', np.array([5.0, 0.0, 1.0]), np.array([2.0, 2.0, 2.0]), 0.0)

    return score_clearance(
        FlightRecord(streams={DRONE_TRUE: drone}),
        load_scoring_settings(),
        [wall],
        'cine_field',
    )


def test_run_without_the_subject_checked_is_a_gate_review_and_not_counted_collision_free(
    tmp_path,
):
    clearance = _clearance_of_a_clean_hover_without_the_subject_stream()
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = [
        _write_run(tmp_path, 'cine_1', 1, clearance=clearance),
        _write_run(tmp_path, 'cine_2', 2),
        _write_run(tmp_path, 'cine_3', 3),
    ]

    verdict = judge(read_score(cine[0]), {}, load_gate_thresholds())
    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    assert 'no_contact:subject' in verdict.unchecked
    assert verdict.label == 'REVIEW'
    assert comparison.collision_free['cine'].successes == 2
    assert comparison.collision_free['cine'].trials == 2
    assert comparison.collision_partially_checked['cine'] == ('cine_1',)


def test_unchecked_clearance_channel_is_not_counted_as_partially_checked(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = [
        _write_run(tmp_path, 'cine_1', 1, clearance_checked=False),
        _write_run(tmp_path, 'cine_2', 2),
        _write_run(tmp_path, 'cine_3', 3),
    ]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    assert comparison.collision_partially_checked['cine'] == ()


def test_markdown_counts_the_partially_checked_runs_of_each_arm(tmp_path):
    clearance = _clearance_of_a_clean_hover_without_the_subject_stream()
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = [
        _write_run(tmp_path, 'cine_1', 1, clearance=clearance),
        _write_run(tmp_path, 'cine_2', 2),
        _write_run(tmp_path, 'cine_3', 3),
    ]

    markdown = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest).to_markdown()

    assert 'cine: collision-free 2/2' in markdown
    assert 'partially checked: 1 (cine_1)' in markdown


def test_score_file_holding_a_nan_collision_free_counts_it_as_unknown(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = _write_arm(tmp_path, 'cine', [0.6, 0.6, 0.6])
    score_file = cine[0] / 'score.json'
    score_file.write_text(
        score_file.read_text().replace('"collision_free": 1.0', '"collision_free": NaN'),
    )

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    assert comparison.collision_free['cine'].trials == 2
    assert comparison.collision_free['cine'].unchecked_runs == ('cine_1',)


def test_unchecked_metric_in_a_completed_run_is_excluded_not_worst_valued(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = [
        _write_run(tmp_path, 'cine_1', 1, focus_checked=False),
        _write_run(tmp_path, 'cine_2', 2),
        _write_run(tmp_path, 'cine_3', 3),
    ]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.pairs == 2
    assert worst.excluded_runs == ('cine_1',)


def test_non_failed_run_with_short_filming_is_excluded_from_focus_metric(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = [
        _write_run(tmp_path, 'cine_1', 1, held_share=0.9, filming_s=4.0),
        _write_run(tmp_path, 'cine_2', 2),
        _write_run(tmp_path, 'cine_3', 3),
    ]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.pairs == 2
    assert worst.short_filming_runs == ('cine_1',)
    assert worst.excluded_runs == ()


def test_short_filming_also_excludes_the_run_from_motion_metrics(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = [
        _write_run(tmp_path, 'cine_1', 1, filming_s=4.0),
        _write_run(tmp_path, 'cine_2', 2),
        _write_run(tmp_path, 'cine_3', 3),
    ]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    sparc = _view(comparison.secondary, 'motion.sparc', 'failures as worst value')
    clearance = _view(comparison.secondary, 'clearance.min_clearance_m', 'failures as worst value')
    assert sparc.short_filming_runs == ('cine_1',)
    assert clearance.pairs == 3


def test_failed_run_with_short_filming_still_takes_worst_value(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = [
        _write_run(tmp_path, 'cine_1', 1, held_share=0.9, completed=0.0, filming_s=1.0),
        _write_run(tmp_path, 'cine_2', 2),
        _write_run(tmp_path, 'cine_3', 3),
    ]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.pairs == 3
    assert worst.arms[1].run_values['cine_1'] == 0.0


def test_run_without_filming_time_is_excluded_as_unchecked(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5, 0.5])
    cine = [
        _write_run(tmp_path, 'cine_1', 1, filming_s=None),
        _write_run(tmp_path, 'cine_2', 2),
        _write_run(tmp_path, 'cine_3', 3),
    ]

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.excluded_runs == ('cine_1',)


def test_five_pairs_report_insufficient_pairs_without_p_value(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.1, 0.1, 0.1, 0.1, 0.1])
    cine = _write_arm(tmp_path, 'cine', [0.2, 0.3, 0.4, 0.5, 0.6])

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.p_value is None
    assert worst.label == 'insufficient nonzero pairs (n=5 of 5) for α=0.05'


def test_zero_differences_do_not_count_toward_minimum_pairs(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    cine = _write_arm(tmp_path, 'cine', [0.1, 0.1, 0.2, 0.3, 0.4, 0.5])

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.p_value is None
    assert worst.label == 'insufficient nonzero pairs (n=4 of 6) for α=0.05'


def test_differences_equal_up_to_float_error_count_as_equal(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.1])
    cine = _write_arm(tmp_path, 'cine', [0.6, 0.2])

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    seed_1 = '["orbit", "cine_gazebo/worlds/cine_field.sdf.template", "a1b2c3", 0.0, 1.0]'
    seed_2 = '["orbit", "cine_gazebo/worlds/cine_field.sdf.template", "a1b2c3", 0.0, 2.0]'
    assert worst.differences[seed_1] == worst.differences[seed_2]


def test_eight_consistent_pairs_report_significant_primary(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    cine = _write_arm(tmp_path, 'cine', [0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5])

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.p_value == pytest.approx(2 / 256)
    assert worst.label == 'cine better'


def test_lower_is_better_metric_names_the_lower_arm_better(tmp_path):
    baseline = _write_arm(
        tmp_path,
        'baseline',
        [0.5] * 8,
        [0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1],
    )
    cine = _write_arm(tmp_path, 'cine', [0.5] * 8)

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    jerk = _view(comparison.secondary, 'motion.rms_jerk_mps3', 'failures as worst value')
    assert jerk.hodges_lehmann < 0
    assert jerk.label == 'cine better'


def test_secondary_p_values_are_holm_adjusted(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5] * 8)
    cine = _write_arm(tmp_path, 'cine', [0.5] * 8, [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08])

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    sparc = _view(comparison.secondary, 'motion.sparc', 'failures as worst value')
    assert sparc.p_value == pytest.approx(2 / 256)
    assert sparc.p_adjusted == pytest.approx(5 * 2 / 256)


def test_all_zero_differences_give_no_p_value_and_stay_out_of_holm(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5] * 8)
    cine = _write_arm(tmp_path, 'cine', [0.5] * 8)

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    sparc = _view(comparison.secondary, 'motion.sparc', 'failures as worst value')
    assert worst.p_value is None
    assert worst.label == 'insufficient nonzero pairs (n=0 of 8) for α=0.05'
    assert worst.hodges_lehmann == 0.0
    assert sparc.p_adjusted is None
    assert '- failures as worst value: Holm across m=0 tested metrics' in comparison.to_markdown()


def test_changed_plan_labels_comparison_exploratory(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5])
    cine = _write_arm(tmp_path, 'cine', [0.6])

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, 'a different digest')

    assert comparison.plan_status == 'exploratory'


def test_unknown_committed_plan_labels_comparison_exploratory(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5])
    cine = _write_arm(tmp_path, 'cine', [0.6])

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, None)

    assert comparison.plan_status == 'exploratory'


def test_matching_committed_plan_labels_comparison_preregistered(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5])
    cine = _write_arm(tmp_path, 'cine', [0.6])

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    assert comparison.plan_status == 'preregistered'


def test_difference_below_noise_floor_is_not_resolved(tmp_path):
    baseline = _write_arm(tmp_path / 'scored', 'baseline', [0.5] * 8)
    cine = _write_arm(
        tmp_path / 'scored',
        'cine',
        [0.501, 0.502, 0.503, 0.504, 0.505, 0.506, 0.507, 0.508],
    )
    repeatability = {
        'baseline': [
            _write_run(tmp_path / 'repeats', 'repeat_a', 1, held_share=0.5),
            _write_run(tmp_path / 'repeats', 'repeat_b', 1, held_share=0.6),
            _write_run(tmp_path / 'repeats', 'repeat_c', 1, held_share=0.7),
        ],
    }

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, repeatability, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.noise_floor == pytest.approx(0.1)
    assert worst.label == 'not resolved'


def test_noise_floor_does_not_pool_across_arms(tmp_path):
    baseline = _write_arm(tmp_path / 'scored', 'baseline', [0.5] * 8)
    cine = _write_arm(tmp_path / 'scored', 'cine', [0.8] * 8)
    repeatability = {
        'baseline': [
            _write_run(tmp_path / 'repeats', 'baseline_a', 1, held_share=0.50),
            _write_run(tmp_path / 'repeats', 'baseline_b', 1, held_share=0.51),
            _write_run(tmp_path / 'repeats', 'baseline_c', 1, held_share=0.52),
        ],
        'cine': [
            _write_run(tmp_path / 'repeats', 'cine_a', 1, held_share=0.80),
            _write_run(tmp_path / 'repeats', 'cine_b', 1, held_share=0.81),
            _write_run(tmp_path / 'repeats', 'cine_c', 1, held_share=0.82),
        ],
    }

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, repeatability, PLAN.digest)

    worst = _view(comparison.primary, 'focus.held_share', 'failures as worst value')
    assert worst.noise_floor == pytest.approx(0.01)


def test_repeatability_for_an_arm_not_compared_is_rejected(tmp_path):
    baseline = _write_arm(tmp_path / 'scored', 'baseline', [0.5])
    cine = _write_arm(tmp_path / 'scored', 'cine', [0.6])
    repeatability = {'other': [_write_run(tmp_path / 'repeats', 'other_a', 1)]}

    with pytest.raises(ValueError, match='other'):
        compare({'baseline': baseline, 'cine': cine}, PLAN, repeatability, PLAN.digest)


def test_to_json_is_strict_json(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.4])
    cine = _write_arm(tmp_path, 'cine', [0.6, 0.7])

    comparison = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest)

    strict = json.dumps(comparison.to_json(), allow_nan=False)
    assert json.loads(strict)['plan_digest'] == PLAN.digest


def test_markdown_lists_rates_pairs_primary_secondary_then_plan(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.4])
    cine = _write_arm(tmp_path, 'cine', [0.6, 0.7])

    markdown = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest).to_markdown()

    assert (
        markdown.index('## Mission success and collision-free rates')
        < markdown.index('## Pairs')
        < markdown.index('## Primary metric')
        < markdown.index('## Secondary metrics')
        < markdown.index('## Analysis plan')
    )


def test_markdown_names_which_arm_is_a_and_which_is_b(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5])
    cine = _write_arm(tmp_path, 'cine', [0.6])

    markdown = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest).to_markdown()

    assert 'difference = B − A = cine − baseline' in markdown


def test_markdown_states_the_holm_family_size_per_view(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5] * 8)
    cine = _write_arm(tmp_path, 'cine', [0.5] * 8, [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08])

    markdown = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest).to_markdown()

    assert '- failures as worst value: Holm across m=5 tested metrics' in markdown


def test_markdown_prints_negative_zero_as_zero(tmp_path):
    baseline = [_write_run(tmp_path, 'baseline_1', 1, held_share=0.0)]
    cine = [_write_run(tmp_path, 'cine_1', 1, held_share=-0.0)]

    markdown = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest).to_markdown()

    assert '  - cine: n=1, mean 0, median 0, sd —, range [0, 0]; runs: cine_1=0' in markdown


def test_markdown_says_what_the_noise_floor_leaves_out(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5])
    cine = _write_arm(tmp_path, 'cine', [0.6])

    markdown = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest).to_markdown()

    assert (
        'The noise floor leaves out failed runs, runs filmed under min_filming_s and runs '
        'missing the metric'
    ) in markdown


def test_markdown_lists_short_filming_runs_with_count_and_names(tmp_path):
    baseline = _write_arm(tmp_path, 'baseline', [0.5, 0.5])
    cine = [_write_run(tmp_path, 'cine_1', 1, filming_s=4.0), _write_run(tmp_path, 'cine_2', 2)]

    markdown = compare({'baseline': baseline, 'cine': cine}, PLAN, None, PLAN.digest).to_markdown()

    assert 'filmed under min_filming_s=5: 1 (cine_1)' in markdown
