import hashlib
import json
import shutil
import sys
import types
from pathlib import Path

import pytest
from cine_eval.__main__ import _shown, main
from cine_eval.channels.result import OK, ChannelResult
from cine_eval.record import load
from cine_eval.score import RunScore, score_run, write_score
from cine_eval.settings import load_scoring_settings

FIXTURE = Path(__file__).parent / 'fixtures' / 'short_flight_record'
PLAN = Path(__file__).resolve().parents[2] / 'eval' / 'analysis_plan.toml'


class FakeBagError(Exception):
    pass


def _copy_fixture(destination: Path) -> Path:
    shutil.copytree(FIXTURE, destination)

    return destination


def _scored_run(destination: Path) -> Path:
    run_directory = _copy_fixture(destination)
    write_score(score_run(run_directory), run_directory)

    return run_directory


def _run_with_contact(destination: Path) -> Path:
    run_directory = _copy_fixture(destination)
    clearance = ChannelResult(
        name='clearance',
        status=OK,
        reason='',
        summary={'collision_free': 0.0, 'min_static_clearance_m': -0.2, 'first_contact_s': 3.0},
    )
    score = RunScore(
        run_name='contact',
        channels={'clearance': clearance},
        settings=load_scoring_settings().as_dict(),
        provenance={},
    )
    write_score(score, run_directory)

    return run_directory


def _fake_extract_module(extract_function) -> types.ModuleType:
    module = types.ModuleType('cine_eval.extract')
    module.BagError = FakeBagError
    module.extract = extract_function

    return module


def test_main_gate_exits_1_on_review(tmp_path):
    run_directory = _run_with_contact(tmp_path / 'run')

    assert main(['gate', str(run_directory)]) == 1


def test_main_gate_json_prints_one_object_with_label_and_failures(tmp_path, capsys):
    run_directory = _run_with_contact(tmp_path / 'run')

    main(['gate', str(run_directory), '--json'])

    printed = json.loads(capsys.readouterr().out)
    assert printed['label'] == 'REVIEW'
    assert printed['failures'][0].startswith('no_contact: contact with static')


def test_main_gate_on_unscored_run_exits_4(tmp_path):
    run_directory = _copy_fixture(tmp_path / 'run')

    assert main(['gate', str(run_directory)]) == 4


def test_main_extract_exits_4_on_unreadable_bag(tmp_path, monkeypatch):
    def refuse(bag_directory):
        raise FakeBagError(f'{bag_directory}: no .mcap files')

    monkeypatch.setitem(sys.modules, 'cine_eval.extract', _fake_extract_module(refuse))

    assert main(['extract', str(tmp_path / 'bag'), '--out', str(tmp_path / 'out')]) == 4


def test_main_extract_exits_3_when_rosbag2_py_is_missing(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, 'cine_eval.extract', None)

    assert main(['extract', str(tmp_path / 'bag'), '--out', str(tmp_path / 'out')]) == 3


def test_main_extract_saves_the_record_under_the_given_directory(tmp_path, monkeypatch):
    record = load(FIXTURE)
    monkeypatch.setitem(
        sys.modules,
        'cine_eval.extract',
        _fake_extract_module(lambda bag_directory: record),
    )

    exit_code = main(['extract', str(tmp_path / 'bag'), '--out', str(tmp_path / 'out')])

    assert exit_code == 0
    assert (tmp_path / 'out' / 'record.json').is_file()


def test_main_score_writes_score_json_next_to_the_record(tmp_path):
    run_directory = _copy_fixture(tmp_path / 'run')

    exit_code = main(['score', str(run_directory)])

    assert exit_code == 0
    assert (run_directory / 'score.json').is_file()


def test_main_compare_writes_json_and_markdown_named_after_the_comparison(tmp_path):
    run_a = _scored_run(tmp_path / 'a1')
    run_b = _scored_run(tmp_path / 'b1')
    out_directory = tmp_path / 'compare'

    exit_code = main(
        [
            'compare',
            '--arm',
            f'baseline={run_a}',
            '--arm',
            f'treatment={run_b}',
            '--plan',
            str(PLAN),
            '--name',
            'first',
            '--out-dir',
            str(out_directory),
        ],
    )

    assert exit_code == 0
    assert json.loads((out_directory / 'first.json').read_text())['arm_a'] == 'baseline'
    assert (out_directory / 'first.md').is_file()


def test_main_compare_marks_the_plan_exploratory_without_a_committed_digest(
    tmp_path,
    monkeypatch,
):
    monkeypatch.delenv('CINE_EVAL_PLAN_COMMITTED_DIGEST', raising=False)
    run_a = _scored_run(tmp_path / 'a1')
    run_b = _scored_run(tmp_path / 'b1')

    main(
        [
            'compare',
            '--arm',
            f'baseline={run_a}',
            '--arm',
            f'treatment={run_b}',
            '--plan',
            str(PLAN),
            '--name',
            'first',
            '--out-dir',
            str(tmp_path),
        ],
    )

    assert json.loads((tmp_path / 'first.json').read_text())['plan_status'] == 'exploratory'


def test_main_compare_rejects_repeatability_for_an_unknown_arm(tmp_path, capsys):
    run_a = _scored_run(tmp_path / 'a1')
    run_b = _scored_run(tmp_path / 'b1')

    exit_code = main(
        [
            'compare',
            '--arm',
            f'baseline={run_a}',
            '--arm',
            f'treatment={run_b}',
            '--repeatability',
            f'stranger={run_a}',
            '--plan',
            str(PLAN),
            '--name',
            'first',
            '--out-dir',
            str(tmp_path),
        ],
    )

    assert exit_code == 2
    assert 'stranger' in capsys.readouterr().err


def test_main_compare_with_one_arm_exits_2(tmp_path):
    run_a = _scored_run(tmp_path / 'a1')

    exit_code = main(
        [
            'compare',
            '--arm',
            f'baseline={run_a}',
            '--plan',
            str(PLAN),
            '--name',
            'first',
            '--out-dir',
            str(tmp_path),
        ],
    )

    assert exit_code == 2


def _two_arm_arguments(tmp_path, name='first', out_directory=None):
    run_a = _scored_run(tmp_path / 'a1')
    run_b = _scored_run(tmp_path / 'b1')

    return [
        'compare',
        '--arm',
        f'baseline={run_a}',
        '--arm',
        f'treatment={run_b}',
        '--plan',
        str(PLAN),
        '--name',
        name,
        '--out-dir',
        str(out_directory if out_directory is not None else tmp_path / 'compare'),
    ]


def _blocked_directory(tmp_path):
    """A path no directory can be made under, because its parent is a regular file."""

    parent = tmp_path / 'regular_file'
    parent.write_text('not a directory')

    return parent / 'below'


def test_main_compare_marks_the_plan_preregistered_when_the_committed_digest_matches(
    tmp_path,
    monkeypatch,
):
    monkeypatch.setenv(
        'CINE_EVAL_PLAN_COMMITTED_DIGEST',
        hashlib.sha1(PLAN.read_bytes()).hexdigest(),
    )

    main(_two_arm_arguments(tmp_path))

    written = json.loads((tmp_path / 'compare' / 'first.json').read_text())
    assert written['plan_status'] == 'preregistered'


@pytest.mark.parametrize('name', ['a/b', 'a\\b', '..', 'x..y', ''])
def test_main_compare_rejects_a_name_that_is_not_a_plain_file_name(tmp_path, name, capsys):
    exit_code = main(_two_arm_arguments(tmp_path, name=name))

    assert exit_code == 2
    assert '--name' in capsys.readouterr().err


def test_main_extract_exits_4_when_the_record_cannot_be_written(tmp_path, monkeypatch, capsys):
    record = load(FIXTURE)
    monkeypatch.setitem(
        sys.modules,
        'cine_eval.extract',
        _fake_extract_module(lambda bag_directory: record),
    )

    exit_code = main(['extract', str(tmp_path / 'bag'), '--out', str(_blocked_directory(tmp_path))])

    assert exit_code == 4
    assert 'only eval/ and docs/ are writable' in capsys.readouterr().err


def test_main_score_exits_4_when_score_json_cannot_be_written(tmp_path, capsys):
    run_directory = _copy_fixture(tmp_path / 'run')
    (run_directory / 'score.json').mkdir()

    exit_code = main(['score', str(run_directory)])

    assert exit_code == 4
    assert 'only eval/ and docs/ are writable' in capsys.readouterr().err


def test_main_score_exits_4_on_a_malformed_record(tmp_path):
    run_directory = _copy_fixture(tmp_path / 'run')
    (run_directory / 'record.json').write_text('{}')

    assert main(['score', str(run_directory)]) == 4


def test_main_compare_exits_4_when_the_output_directory_cannot_be_made(tmp_path, capsys):
    arguments = _two_arm_arguments(tmp_path, out_directory=_blocked_directory(tmp_path))

    exit_code = main(arguments)

    assert exit_code == 4
    assert 'only eval/ and docs/ are writable' in capsys.readouterr().err


def test_main_compare_exits_4_when_the_comparison_holds_a_nan(tmp_path, monkeypatch):
    nan_comparison = types.SimpleNamespace(
        to_json=lambda: {'difference': float('nan')},
        to_markdown=lambda: '',
        plan_status='exploratory',
    )
    monkeypatch.setattr('cine_eval.analysis.compare', lambda *arguments: nan_comparison)

    assert main(_two_arm_arguments(tmp_path)) == 4


def test_main_reports_a_path_under_the_repository_relative_to_it():
    root = Path(__file__).resolve().parents[2]

    assert _shown(root / 'eval' / 'runs' / 'flight') == 'eval/runs/flight'


def test_main_score_failure_names_the_run_directory_relative_to_the_repository(capsys):
    root = Path(__file__).resolve().parents[2]

    exit_code = main(['score', str(root / 'eval' / 'runs' / 'no_such_run')])

    assert exit_code == 4
    assert 'cannot score eval/runs/no_such_run:' in capsys.readouterr().err


def _run_naming_world_file(destination: Path, world_file: Path) -> Path:
    run_directory = _copy_fixture(destination)
    record_file = run_directory / 'record.json'
    document = json.loads(record_file.read_text())
    document['metadata']['run'] = {'world_file': str(world_file)}
    record_file.write_text(json.dumps(document))

    return run_directory


def test_main_score_exits_4_naming_the_world_file_when_it_is_malformed(tmp_path, capsys):
    world_file = tmp_path / 'broken.sdf'
    world_file.write_text('<sdf version="1.9"><world name="w">')
    run_directory = _run_naming_world_file(tmp_path / 'run', world_file)

    exit_code = main(['score', str(run_directory)])

    assert exit_code == 4
    assert 'world file' in capsys.readouterr().err


def test_main_score_exits_4_naming_the_world_file_when_it_is_missing(tmp_path, capsys):
    run_directory = _run_naming_world_file(tmp_path / 'run', tmp_path / 'absent.sdf')

    exit_code = main(['score', str(run_directory)])

    assert exit_code == 4
    assert 'absent.sdf' in capsys.readouterr().err
