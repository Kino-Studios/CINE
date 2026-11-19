import json
import os

import pytest
from cine_cli.common import CapturedOutput, Environment
from cine_cli.main import app
from typer.testing import CliRunner

from conftest import RecordingRunner

IMAGE_INSPECT_PREFIX = ('docker', 'image', 'inspect')
COMMIT = 'a' * 40
SCORE_FLIGHT = ['eval', 'score', 'eval/runs/flight']
TWO_ARMS = ['--arm', 'base=eval/runs/a1', '--arm', 'cine=eval/runs/b1']


@pytest.fixture
def bags_directory(tmp_path):
    directory = tmp_path / 'bags'
    (directory / 'sim_bag').mkdir(parents=True)

    return directory


@pytest.fixture
def invoke_eval(repository_root, bags_directory, monkeypatch):
    monkeypatch.chdir(repository_root)

    def invoke_command(
        *arguments,
        runner=None,
        installed=('docker', 'git'),
        bags_variable=None,
    ):
        runner = runner or RecordingRunner()
        environment = Environment(
            repository_root=repository_root,
            runner=runner,
            which=lambda name: f'/usr/bin/{name}' if name in installed else None,
            variables={'CINE_BAGS_DIR': bags_variable or str(bags_directory)},
        )

        return CliRunner().invoke(app, list(arguments), obj=environment)

    return invoke_command


def test_eval_score_builds_docker_command_with_repo_read_only_and_eval_writable(
    invoke_eval,
    repository_root,
):
    runner = RecordingRunner()

    result = invoke_eval(*SCORE_FLIGHT, runner=runner)

    command = runner.commands[0][0]
    assert result.exit_code == 0
    assert f'{repository_root}:/cine:ro' in command
    assert f'{repository_root}/eval:/cine/eval' in command
    assert f'{repository_root}/docs:/cine/docs' in command
    assert command[-3:] == ['cine-eval', 'score', '/cine/eval/runs/flight']


def test_eval_passes_scorer_commit_and_dirty_flag(invoke_eval):
    runner = RecordingRunner(
        captures={
            ('git', 'rev-parse'): CapturedOutput(returncode=0, stdout=f'{COMMIT}\n'),
            ('git', 'status'): CapturedOutput(returncode=0, stdout=' M cine_cli/cine_cli/sim.py\n'),
        },
    )

    invoke_eval(*SCORE_FLIGHT, runner=runner)

    command = runner.commands[0][0]
    assert f'CINE_EVAL_SCORER_COMMIT={COMMIT}' in command
    assert 'CINE_EVAL_SCORER_DIRTY=1' in command


def test_eval_reports_a_clean_working_tree_as_not_dirty(invoke_eval):
    runner = RecordingRunner(
        captures={
            ('git', 'rev-parse'): CapturedOutput(returncode=0, stdout=f'{COMMIT}\n'),
            ('git', 'status'): CapturedOutput(returncode=0, stdout=''),
        },
    )

    invoke_eval(*SCORE_FLIGHT, runner=runner)

    assert 'CINE_EVAL_SCORER_DIRTY=0' in runner.commands[0][0]


def test_eval_without_git_still_runs_and_says_the_commit_will_be_unknown(invoke_eval):
    runner = RecordingRunner()

    result = invoke_eval(*SCORE_FLIGHT, runner=runner, installed=('docker',))

    assert result.exit_code == 0
    assert 'recorded as unknown' in result.stderr
    assert runner.captured_commands == [list(IMAGE_INSPECT_PREFIX) + ['cine-gazebo-sim:dev']]
    assert not any('SCORER' in part for part in runner.commands[0][0])


def test_eval_dry_run_without_git_does_not_try_to_run_git(invoke_eval):
    runner = RecordingRunner()

    result = invoke_eval(*SCORE_FLIGHT, '--dry-run', runner=runner, installed=())

    assert result.exit_code == 0
    assert runner.captured_commands == []


def test_eval_outside_a_git_repository_omits_the_scorer_flags_and_says_so(invoke_eval):
    runner = RecordingRunner(
        captures={
            ('git', 'rev-parse'): CapturedOutput(returncode=128, stdout=''),
            ('git', 'status'): CapturedOutput(returncode=128, stdout=''),
        },
    )

    result = invoke_eval(*SCORE_FLIGHT, runner=runner)

    assert 'recorded as unknown' in result.stderr
    assert not any('SCORER' in part for part in runner.commands[0][0])


@pytest.mark.skipif(not hasattr(os, 'getuid'), reason='no user ids on this platform')
def test_eval_runs_the_image_as_the_calling_user(invoke_eval):
    runner = RecordingRunner()

    invoke_eval(*SCORE_FLIGHT, runner=runner)

    command = runner.commands[0][0]
    assert command[command.index('--user') + 1] == f'{os.getuid()}:{os.getgid()}'


def test_eval_extract_maps_a_bag_under_the_bags_directory_into_the_image(
    invoke_eval,
    bags_directory,
):
    runner = RecordingRunner()

    invoke_eval('eval', 'extract', str(bags_directory / 'sim_bag'), runner=runner)

    command = runner.commands[0][0]
    assert f'{bags_directory}:/bags:ro' in command
    assert command[-3:] == ['cine-eval', 'extract', '/bags/sim_bag']


def test_eval_mounts_the_bags_directory_with_symbolic_links_and_dots_resolved(
    invoke_eval,
    bags_directory,
):
    runner = RecordingRunner()
    roundabout = f'{bags_directory}/sim_bag/..'

    invoke_eval(
        'eval',
        'extract',
        str(bags_directory / 'sim_bag'),
        runner=runner,
        bags_variable=roundabout,
    )

    assert f'{bags_directory.resolve()}:/bags:ro' in runner.commands[0][0]


def test_eval_extract_with_a_missing_bags_directory_exits_2_and_names_the_variable(
    invoke_eval,
    tmp_path,
):
    runner = RecordingRunner()
    missing = tmp_path / 'no-bags-here'

    result = invoke_eval(
        'eval',
        'extract',
        str(missing / 'sim_bag'),
        runner=runner,
        bags_variable=str(missing),
    )

    assert result.exit_code == 2
    assert 'CINE_BAGS_DIR' in result.stderr
    assert runner.commands == []


def test_eval_rejects_bag_outside_bags_dir(invoke_eval, tmp_path):
    runner = RecordingRunner()

    result = invoke_eval('eval', 'extract', str(tmp_path / 'elsewhere'), runner=runner)

    assert result.exit_code == 2
    assert 'outside the bags directory' in result.stderr
    assert runner.commands == []


def test_eval_extract_maps_an_absolute_out_directory_under_the_repository(
    invoke_eval,
    bags_directory,
    repository_root,
):
    runner = RecordingRunner()

    invoke_eval(
        'eval',
        'extract',
        str(bags_directory / 'sim_bag'),
        '--out',
        str(repository_root / 'eval' / 'runs' / 'flight'),
        runner=runner,
    )

    assert runner.commands[0][0][-2:] == ['--out', '/cine/eval/runs/flight']


def test_eval_without_image_exits_3(invoke_eval):
    runner = RecordingRunner(
        captures={IMAGE_INSPECT_PREFIX: CapturedOutput(returncode=1, stdout='')},
    )

    result = invoke_eval(*SCORE_FLIGHT, runner=runner)

    assert result.exit_code == 3
    assert 'docker compose -f docker/gazebo-sim/compose.yaml build' in result.stderr
    assert runner.commands == []


def test_eval_without_docker_exits_3(invoke_eval):
    runner = RecordingRunner()

    result = invoke_eval(*SCORE_FLIGHT, runner=runner, installed=())

    assert result.exit_code == 3
    assert 'docker not found' in result.stderr
    assert runner.commands == []


def test_eval_returns_the_image_exit_code(invoke_eval):
    result = invoke_eval('eval', 'gate', 'eval/runs/flight', runner=RecordingRunner(exit_code=1))

    assert result.exit_code == 1


def test_eval_dry_run_prints_the_command_and_runs_nothing(invoke_eval):
    runner = RecordingRunner()

    result = invoke_eval('eval', 'gate', 'eval/runs/flight', '--dry-run', runner=runner)

    assert result.exit_code == 0
    assert 'python -m cine_eval' in result.stdout
    assert runner.commands == []


def test_eval_json_is_passed_to_the_image(invoke_eval):
    runner = RecordingRunner()

    invoke_eval('eval', 'gate', 'eval/runs/flight', '--json', runner=runner)

    assert runner.commands[0][0][-2:] == ['/cine/eval/runs/flight', '--json']


def test_eval_dry_run_json_prints_one_object(invoke_eval):
    result = invoke_eval('eval', 'gate', 'eval/runs/flight', '--dry-run', '--json')

    assert json.loads(result.stdout)['command'][-1] == '--json'


def test_eval_maps_an_absolute_run_directory_under_the_repository(invoke_eval, repository_root):
    runner = RecordingRunner()

    invoke_eval('eval', 'score', str(repository_root / 'eval' / 'runs' / 'flight'), runner=runner)

    assert runner.commands[0][0][-1] == '/cine/eval/runs/flight'


def test_eval_reads_a_relative_run_directory_from_the_current_directory(
    invoke_eval,
    repository_root,
    monkeypatch,
):
    runner = RecordingRunner()
    monkeypatch.chdir(repository_root / 'eval')

    invoke_eval('eval', 'score', 'runs/flight', runner=runner)

    assert runner.commands[0][0][-1] == '/cine/eval/runs/flight'


def test_eval_rejects_a_relative_run_directory_that_climbs_out_of_the_repository(
    invoke_eval,
):
    runner = RecordingRunner()

    result = invoke_eval('eval', 'score', '../elsewhere', runner=runner)

    assert result.exit_code == 2
    assert 'outside the repository' in result.stderr
    assert runner.commands == []


def test_eval_rejects_an_absolute_run_directory_outside_the_repository(invoke_eval, tmp_path):
    runner = RecordingRunner()

    result = invoke_eval('eval', 'score', str(tmp_path), runner=runner)

    assert result.exit_code == 2
    assert 'outside the repository' in result.stderr
    assert runner.commands == []


def test_eval_compare_forwards_arms_and_repeatability_by_arm(invoke_eval):
    runner = RecordingRunner()

    invoke_eval(
        'eval',
        'compare',
        '--arm',
        'base=eval/runs/a1,eval/runs/a2',
        '--arm',
        'cine=eval/runs/b1,eval/runs/b2',
        '--repeatability',
        'base=eval/runs/r1,eval/runs/r2',
        '--name',
        'first',
        runner=runner,
    )

    assert runner.commands[0][0][-12:] == [
        'cine-eval',
        'compare',
        '--arm',
        'base=/cine/eval/runs/a1,/cine/eval/runs/a2',
        '--arm',
        'cine=/cine/eval/runs/b1,/cine/eval/runs/b2',
        '--repeatability',
        'base=/cine/eval/runs/r1,/cine/eval/runs/r2',
        '--plan',
        '/cine/eval/analysis_plan.toml',
        '--name',
        'first',
    ]


def test_eval_compare_rejects_repeatability_for_an_unknown_arm(invoke_eval):
    runner = RecordingRunner()

    result = invoke_eval(
        'eval',
        'compare',
        *TWO_ARMS,
        '--repeatability',
        'stranger=eval/runs/r1',
        '--name',
        'first',
        runner=runner,
    )

    assert result.exit_code == 2
    assert 'stranger' in result.stderr
    assert runner.commands == []


@pytest.mark.parametrize('name', ['a/b', 'a\\b', '..', 'x..y'])
def test_eval_compare_rejects_a_name_that_is_not_a_plain_file_name(invoke_eval, name):
    runner = RecordingRunner()

    result = invoke_eval('eval', 'compare', *TWO_ARMS, '--name', name, runner=runner)

    assert result.exit_code == 2
    assert '--name' in result.stderr
    assert runner.commands == []


def test_eval_compare_passes_the_committed_plan_digest(invoke_eval):
    runner = RecordingRunner(
        captures={('git', 'show'): CapturedOutput(returncode=0, stdout='alpha = 0.05\n')},
    )

    invoke_eval('eval', 'compare', *TWO_ARMS, '--name', 'first', runner=runner)

    assert (
        'CINE_EVAL_PLAN_COMMITTED_DIGEST=e520238dd9d131d0f7534cef2e837624b0b22ae7'
        in runner.commands[0][0]
    )


def test_eval_compare_omits_the_digest_and_says_exploratory_when_the_plan_is_not_committed(
    invoke_eval,
):
    runner = RecordingRunner(
        captures={('git', 'show'): CapturedOutput(returncode=128, stdout='')},
    )

    result = invoke_eval('eval', 'compare', *TWO_ARMS, '--name', 'first', runner=runner)

    assert 'exploratory' in result.stderr
    assert not any('PLAN_COMMITTED_DIGEST' in part for part in runner.commands[0][0])


def test_eval_compare_without_git_says_exploratory(invoke_eval):
    runner = RecordingRunner()

    result = invoke_eval(
        'eval',
        'compare',
        *TWO_ARMS,
        '--name',
        'first',
        runner=runner,
        installed=('docker',),
    )

    assert result.exit_code == 0
    assert 'exploratory' in result.stderr
