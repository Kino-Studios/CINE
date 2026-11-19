import json
import shutil
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from cine_eval.record import load
from cine_eval.score import CHANNELS, read_score, score_run, write_score
from cine_eval.settings import load_scoring_settings

FIXTURE = Path(__file__).parent / 'fixtures' / 'short_flight_record'


def test_score_json_copies_the_settings_used(tmp_path):
    settings = replace(load_scoring_settings(), airframe_radius_m=0.42)

    write_score(score_run(FIXTURE, settings), tmp_path)

    written = json.loads((tmp_path / 'score.json').read_text())
    assert written['settings']['airframe_radius_m'] == 0.42


def test_every_channel_appears_in_score_json(tmp_path):
    write_score(score_run(FIXTURE), tmp_path)

    written = json.loads((tmp_path / 'score.json').read_text())
    assert tuple(written['channels']) == tuple(sorted(CHANNELS))


def test_metrics_are_stored_under_channel_and_series_names(tmp_path):
    write_score(score_run(FIXTURE), tmp_path)

    with np.load(tmp_path / 'metrics.npz') as metrics:
        assert 'clearance/clearance_m' in metrics.files


def test_read_score_returns_what_was_written(tmp_path):
    original = score_run(FIXTURE)
    write_score(original, tmp_path)

    restored = read_score(tmp_path)

    assert restored.channels['clearance'].summary == original.channels['clearance'].summary
    assert restored.run_name == original.run_name


def test_provenance_hashes_the_record_arrays(tmp_path):
    score = score_run(FIXTURE)

    assert len(score.provenance['record_sha1']) == 40
    assert score.provenance['bag_sha1'] == load(FIXTURE).metadata['bag_sha1']


def test_read_score_rejects_a_status_that_is_neither_ok_nor_unchecked(tmp_path):
    write_score(score_run(FIXTURE), tmp_path)
    document = json.loads((tmp_path / 'score.json').read_text())
    document['channels']['clearance']['status'] = 'OK'
    (tmp_path / 'score.json').write_text(json.dumps(document))

    with pytest.raises(ValueError, match="unknown status 'OK'"):
        read_score(tmp_path)


def test_fixture_scores_match_the_committed_expectation():
    expected = json.loads((FIXTURE / 'expected_score.json').read_text())

    channels = score_run(FIXTURE).channels

    assert tuple(sorted(channels)) == tuple(sorted(expected))
    assert channels['clearance'].status == expected['clearance']['status']
    assert channels['clearance'].reason == expected['clearance']['reason']
    assert channels['clearance'].summary == pytest.approx(
        expected['clearance']['summary'],
        rel=1e-9,
    )
    assert _snapshot(channels['estimate']) == expected['estimate']
    assert _snapshot(channels['focus']) == expected['focus']
    assert _snapshot(channels['following']) == expected['following']
    assert channels['freshness'].status == expected['freshness']['status']
    assert channels['freshness'].reason == expected['freshness']['reason']
    assert channels['freshness'].summary == pytest.approx(
        expected['freshness']['summary'],
        rel=1e-9,
    )
    assert _snapshot(channels['mission']) == expected['mission']
    assert _snapshot(channels['motion']) == expected['motion']


def _snapshot(result):
    return {'status': result.status, 'reason': result.reason, 'summary': result.summary}


def _record_with_bag_tail(tmp_path: Path) -> Path:
    run = tmp_path / 'run'
    shutil.copytree(FIXTURE, run)
    index = json.loads((run / 'record.json').read_text())
    index['metadata'].update(
        reindex_warnings=['[WARN] chunk 3 truncated'],
        last_message_s=31.5,
        bag_start_s=26.02,
        bag_duration_s=6.0,
    )
    (run / 'record.json').write_text(json.dumps(index))

    return run


def test_provenance_carries_how_the_bag_ended(tmp_path):
    score = score_run(_record_with_bag_tail(tmp_path))

    assert score.provenance['bag'] == {
        'reindex_warnings': ['[WARN] chunk 3 truncated'],
        'last_message_s': 31.5,
        'bag_start_s': 26.02,
        'bag_duration_s': 6.0,
    }


def test_provenance_marks_the_bag_ending_unknown_for_a_record_extracted_before_it_was_kept():
    score = score_run(FIXTURE)

    assert score.provenance['bag']['last_message_s'] == 'unknown'
