import csv
import json
import os
import shutil
from pathlib import Path

import numpy as np
import pytest
from cine_eval.channels.result import OK, ChannelResult
from cine_eval.record import DRONE_TRUE, FlightRecord, RecordError, Stream, save
from cine_eval.score import RunScore, write_score
from cine_eval.settings import load_scoring_settings
from cine_eval.standings import (
    BEGIN_MARKER,
    END_MARKER,
    build_standings,
    replace_generated_section,
)

FIXTURE = Path(__file__).parent / 'fixtures' / 'short_flight_record'
PLAN = Path(__file__).resolve().parents[2] / 'eval' / 'analysis_plan.toml'
COMMAND = 'cine eval standings'
CLEAN_RUN = {'git': {'CINE': {'commit': 'abc1234', 'dirty': False}}, 'image_id': 'sha256:img'}
ONE_RUN_ROW = 'arm = "cine"\ncondition = "c"\nruns = ["a1"]\ncomparable = true\nnote = ""'
THREE_RUN_ROW = ONE_RUN_ROW.replace('["a1"]', '["a1", "a2", "a3"]')


@pytest.fixture(autouse=True)
def _clean_scorer(monkeypatch):
    monkeypatch.setenv('CINE_EVAL_SCORER_COMMIT', 'scorer9')
    monkeypatch.setenv('CINE_EVAL_SCORER_DIRTY', '0')


def _write_run(
    runs_root: Path,
    name: str,
    run: dict | None,
    held_share: float = 0.5,
    bag_sha1: str = 'f' * 40,
    legacy: bool = False,
    completed: float = 1.0,
    filming_s: float = 30.0,
    settings: dict | None = None,
    scorer_commit: str = 'scorer9',
    scorer_dirty: str = '0',
) -> None:
    """A scored run; ``settings`` None stores the current scoring settings."""

    directory = runs_root / name
    metadata: dict = {'bag_sha1': bag_sha1}

    if run is not None:
        metadata['run'] = run

    if legacy:
        metadata['legacy'] = True

    stream = Stream(
        name=DRONE_TRUE,
        frame_id='cine_field',
        time=np.array([0.0, 1.0]),
        receive_time=np.array([0.0, 1.0]),
        fields={'position': np.zeros((2, 3))},
    )
    save(FlightRecord(streams={DRONE_TRUE: stream}, metadata=metadata), directory)
    channels = {
        'focus': ChannelResult(
            name='focus',
            status=OK,
            reason='',
            summary={'held_share': held_share, 'worst_deviation_m': 0.4, 'filming_s': filming_s},
        ),
        'motion': ChannelResult(
            name='motion',
            status=OK,
            reason='',
            summary={'peak_speed_mps': 1.2, 'filming_s': filming_s},
        ),
        'mission': ChannelResult(
            name='mission',
            status=OK,
            reason='',
            summary={'completed': completed, 'aborted': 0.0},
        ),
    }
    provenance = {
        'scorer_commit': scorer_commit,
        'scorer_dirty': scorer_dirty,
        'record_sha1': 'r' * 40,
        'bag_sha1': bag_sha1,
    }
    stored_settings = settings if settings is not None else load_scoring_settings().as_dict()
    score = RunScore(
        run_name=name,
        channels=channels,
        settings=stored_settings,
        provenance=provenance,
    )
    write_score(score, directory)


def _focus_status_after_build(tmp_path: Path) -> str:
    _build(tmp_path, ONE_RUN_ROW)
    stored = json.loads((tmp_path / 'runs' / 'a1' / 'score.json').read_text())

    return stored['channels']['focus']['status']


def _build(tmp_path: Path, row_body: str) -> dict:
    manifest = tmp_path / 'standings.toml'
    manifest.write_text(f'[[row]]\n{row_body}\n')
    results_doc = tmp_path / 'results.md'
    results_doc.write_text(f'intro\n{BEGIN_MARKER}\nold\n{END_MARKER}\noutro\n')
    outputs = tmp_path / 'outputs'
    build_standings(manifest, tmp_path / 'runs', outputs, results_doc, PLAN, COMMAND)

    return {
        'outputs': outputs,
        'doc': results_doc,
        'manifest': json.loads((outputs / 'manifest.json').read_text()),
    }


def _csv_rows(outputs: Path) -> list[dict]:
    with (outputs / 'results.csv').open(newline='') as handle:
        return list(csv.DictReader(handle))


def test_replace_generated_section_keeps_text_outside_markers():
    document = f'before\n{BEGIN_MARKER}\nold table\n{END_MARKER}\nafter\n'

    replaced = replace_generated_section(document, 'new table')

    assert replaced == f'before\n{BEGIN_MARKER}\nnew table\n{END_MARKER}\nafter\n'


def test_missing_markers_raise():
    with pytest.raises(ValueError, match='marker'):
        replace_generated_section('no markers here', 'table')


def test_markers_out_of_order_raise():
    with pytest.raises(ValueError, match='order'):
        replace_generated_section(f'{END_MARKER}\n{BEGIN_MARKER}\n', 'table')


def test_dirty_flight_forces_row_not_comparable(tmp_path):
    dirty = {'git': {'CINE': {'commit': 'abc1234', 'dirty': True}}, 'image_id': 'sha256:img'}
    _write_run(tmp_path / 'runs', 'a1', dirty)

    built = _build(tmp_path, ONE_RUN_ROW)

    assert {row['comparable'] for row in _csv_rows(built['outputs'])} == {'false'}


def test_clean_flight_keeps_row_comparable(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN)

    built = _build(tmp_path, ONE_RUN_ROW)

    assert {row['comparable'] for row in _csv_rows(built['outputs'])} == {'true'}


def test_run_without_run_json_forces_row_not_comparable(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', None)

    built = _build(tmp_path, ONE_RUN_ROW)

    assert {row['comparable'] for row in _csv_rows(built['outputs'])} == {'false'}


def test_legacy_run_forces_row_not_comparable(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN, legacy=True)

    built = _build(tmp_path, ONE_RUN_ROW)

    assert {row['comparable'] for row in _csv_rows(built['outputs'])} == {'false'}


def test_forced_not_comparable_row_appends_to_the_note(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', None)

    built = _build(tmp_path, ONE_RUN_ROW.replace('note = ""', 'note = "mine"'))

    note = _csv_rows(built['outputs'])[0]['note']
    assert note.startswith('mine')
    assert 'run.json' in note


def test_manifest_records_bag_digests_and_command(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN, bag_sha1='1' * 40)
    _write_run(tmp_path / 'runs', 'a2', CLEAN_RUN, bag_sha1='2' * 40)

    built = _build(tmp_path, ONE_RUN_ROW.replace('["a1"]', '["a1", "a2"]'))

    entry = built['manifest']['rows'][0]
    assert entry['bag_sha1s'] == ['1' * 40, '2' * 40]
    assert entry['command'] == COMMAND
    assert entry['flight_commits'] == ['abc1234']
    assert entry['image_id'] == ['sha256:img']
    assert entry['scorer_commit'] == ['scorer9']


def test_results_csv_summarises_each_metric_over_the_runs(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN, held_share=0.25)
    _write_run(tmp_path / 'runs', 'a2', CLEAN_RUN, held_share=0.75)

    built = _build(tmp_path, ONE_RUN_ROW.replace('["a1"]', '["a1", "a2"]'))

    held = [row for row in _csv_rows(built['outputs']) if row['metric'] == 'focus.held_share']
    assert [(row['n'], row['median'], row['mean']) for row in held] == [('2', '0.5', '0.5')]


def test_oracle_perception_is_labelled_in_the_table(tmp_path):
    oracle = {**CLEAN_RUN, 'perception_source': 'oracle'}
    _write_run(tmp_path / 'runs', 'a1', oracle)

    built = _build(tmp_path, ONE_RUN_ROW)

    assert 'oracle perception' in built['doc'].read_text()


def test_detector_perception_is_not_labelled_oracle(tmp_path):
    detector = {**CLEAN_RUN, 'perception_source': 'detector'}
    _write_run(tmp_path / 'runs', 'a1', detector)

    built = _build(tmp_path, ONE_RUN_ROW)

    assert 'oracle perception' not in built['doc'].read_text()


def test_missing_score_is_computed_from_the_record(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'runs' / 'a1')

    _build(tmp_path, ONE_RUN_ROW)

    assert (tmp_path / 'runs' / 'a1' / 'score.json').exists()


def test_duplicate_markers_are_rejected():
    document = f'{BEGIN_MARKER}\na\n{END_MARKER}\n{BEGIN_MARKER}\nb\n{END_MARKER}\n'

    with pytest.raises(ValueError, match='exactly once'):
        replace_generated_section(document, 'table')


def test_nested_markers_are_rejected():
    document = f'{BEGIN_MARKER}\n{BEGIN_MARKER}\na\n{END_MARKER}\n'

    with pytest.raises(ValueError, match='exactly once'):
        replace_generated_section(document, 'table')


def test_score_older_than_record_is_rescored(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN, held_share=0.5)
    run_directory = tmp_path / 'runs' / 'a1'
    score_time = (run_directory / 'score.json').stat().st_mtime
    os.utime(run_directory / 'record.npz', (score_time + 100, score_time + 100))

    _build(tmp_path, ONE_RUN_ROW)

    rescored = json.loads((run_directory / 'score.json').read_text())
    assert rescored['channels']['focus']['status'] == 'unchecked'


def test_run_missing_its_arrays_file_is_named_in_the_error(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN)
    (tmp_path / 'runs' / 'a1' / 'record.npz').unlink()

    with pytest.raises(RecordError, match='a1'):
        _build(tmp_path, ONE_RUN_ROW)


def test_run_with_unknown_dirty_flag_forces_row_not_comparable(tmp_path):
    unknown = {'git': {'CINE': {'commit': 'abc1234'}}, 'image_id': 'sha256:img'}
    _write_run(tmp_path / 'runs', 'a1', unknown)

    built = _build(tmp_path, ONE_RUN_ROW)

    row = _csv_rows(built['outputs'])[0]
    assert row['comparable'] == 'false'
    assert 'dirty flag' in row['note']


def test_row_without_runs_is_forced_not_comparable(tmp_path):
    built = _build(tmp_path, ONE_RUN_ROW.replace('["a1"]', '[]'))

    row = _csv_rows(built['outputs'])[0]
    assert row['comparable'] == 'false'
    assert 'no runs' in row['note']


def test_table_note_newlines_collapse_to_single_spaces(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN)

    built = _build(tmp_path, ONE_RUN_ROW.replace('note = ""', 'note = "one\\n\\n  two"'))

    assert '| one two |' in built['doc'].read_text()


def test_failed_run_takes_the_plan_worst_value_in_the_row_median(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN, held_share=0.5)
    _write_run(tmp_path / 'runs', 'a2', CLEAN_RUN, held_share=0.7)
    _write_run(tmp_path / 'runs', 'a3', CLEAN_RUN, held_share=0.9, completed=0.0)

    built = _build(tmp_path, THREE_RUN_ROW)

    held = [row for row in _csv_rows(built['outputs']) if row['metric'] == 'focus.held_share']
    assert [(row['n'], row['median']) for row in held] == [('3', '0.5')]


def test_peak_speed_outside_the_plan_leaves_failed_runs_out(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN)
    _write_run(tmp_path / 'runs', 'a2', CLEAN_RUN)
    _write_run(tmp_path / 'runs', 'a3', CLEAN_RUN, completed=0.0)

    built = _build(tmp_path, THREE_RUN_ROW)

    speed = [row for row in _csv_rows(built['outputs']) if row['metric'] == 'motion.peak_speed_mps']
    assert [row['n'] for row in speed] == ['2']


def test_run_filmed_under_the_plan_minimum_is_left_out_of_focus_and_motion(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN, held_share=0.5)
    _write_run(tmp_path / 'runs', 'a2', CLEAN_RUN, held_share=0.7)
    _write_run(tmp_path / 'runs', 'a3', CLEAN_RUN, held_share=0.9, filming_s=2.0)

    built = _build(tmp_path, THREE_RUN_ROW)

    held = [row for row in _csv_rows(built['outputs']) if row['metric'] == 'focus.held_share']
    assert [(row['n'], row['median']) for row in held] == [('2', '0.6')]


def test_results_csv_counts_the_failed_runs_of_each_row(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN)
    _write_run(tmp_path / 'runs', 'a2', CLEAN_RUN)
    _write_run(tmp_path / 'runs', 'a3', CLEAN_RUN, completed=0.0)

    built = _build(tmp_path, THREE_RUN_ROW)

    assert {row['failed_runs'] for row in _csv_rows(built['outputs'])} == {'1'}


def test_table_headers_say_median_and_cells_give_the_run_count(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN)
    _write_run(tmp_path / 'runs', 'a2', CLEAN_RUN)
    _write_run(tmp_path / 'runs', 'a3', CLEAN_RUN, completed=0.0)

    document = _build(tmp_path, THREE_RUN_ROW)['doc'].read_text()

    assert 'Time within focus tolerance (median)' in document
    assert '| Failed runs |' in document
    assert '1.2 m/s (n=2)' in document


def test_score_from_other_scoring_settings_is_rescored(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN, settings={'clock_hz': 10})

    assert _focus_status_after_build(tmp_path) == 'unchecked'


def test_score_from_another_scorer_commit_is_rescored(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN, scorer_commit='older1')

    assert _focus_status_after_build(tmp_path) == 'unchecked'


def test_score_from_a_dirty_scorer_is_rescored(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN, scorer_dirty='1')

    assert _focus_status_after_build(tmp_path) == 'unchecked'


def test_score_with_an_unknown_scorer_dirty_flag_is_rescored(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN, scorer_dirty='unknown')

    assert _focus_status_after_build(tmp_path) == 'unchecked'


def test_current_scorer_with_an_unknown_dirty_flag_rescores(tmp_path, monkeypatch):
    monkeypatch.delenv('CINE_EVAL_SCORER_DIRTY')
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN)

    assert _focus_status_after_build(tmp_path) == 'unchecked'


def test_score_matching_the_current_scorer_and_settings_is_kept(tmp_path):
    _write_run(tmp_path / 'runs', 'a1', CLEAN_RUN)

    assert _focus_status_after_build(tmp_path) == 'ok'
