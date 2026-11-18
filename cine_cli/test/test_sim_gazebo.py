import json

import pytest
from cine_cli.common import CapturedOutput, Environment
from cine_cli.main import app
from typer.testing import CliRunner

from conftest import RecordingRunner

IMAGE_INSPECT_PREFIX = ('docker', 'image', 'inspect')


def make_environment(repository_root, runner, installed=('docker', 'nvidia-smi')):
    return Environment(
        repository_root=repository_root,
        runner=runner,
        which=lambda name: f'/usr/bin/{name}' if name in installed else None,
        variables={},
    )


@pytest.fixture
def invoke_gazebo(repository_root):
    def invoke_command(*arguments, runner=None, installed=('docker', 'nvidia-smi')):
        runner = runner or RecordingRunner()
        environment = make_environment(repository_root, runner, installed)

        return CliRunner().invoke(app, ['sim', 'gazebo', *arguments], obj=environment)

    return invoke_command


def test_gazebo_runs_compose_with_chosen_nav_source(invoke_gazebo):
    runner = RecordingRunner()

    result = invoke_gazebo('--nav-source', 'vins', runner=runner)

    assert result.exit_code == 0
    assert 'NAV_SOURCE=vins' in runner.commands[0][0]


def test_gazebo_runs_the_compose_service_with_defaults(invoke_gazebo, repository_root):
    runner = RecordingRunner()

    invoke_gazebo(runner=runner)

    assert runner.commands[0][0] == [
        'docker',
        'compose',
        '-f',
        str(repository_root / 'docker/gazebo-sim/compose.yaml'),
        'run',
        '--rm',
        '-e',
        'NAV_SOURCE=sim',
        '-e',
        'HEADLESS=false',
        '-e',
        'WIND=calm',
        '-e',
        'MISSION=true',
        '-e',
        'CHECK=false',
        'gazebo-sim',
    ]


def test_gazebo_flags_map_to_environment_values(invoke_gazebo):
    runner = RecordingRunner()

    invoke_gazebo('--headless', '--wind', 'gusty', '--no-mission', '--check', runner=runner)

    command = runner.commands[0][0]
    assert 'HEADLESS=true' in command
    assert 'WIND=gusty' in command
    assert 'MISSION=false' in command
    assert 'CHECK=true' in command


def test_gazebo_check_returns_container_exit_code(invoke_gazebo):
    result = invoke_gazebo('--check', runner=RecordingRunner(exit_code=1))

    assert result.exit_code == 1


def test_gazebo_without_docker_exits_3(invoke_gazebo):
    runner = RecordingRunner()

    result = invoke_gazebo(runner=runner, installed=('nvidia-smi',))

    assert result.exit_code == 3
    assert 'docker not found' in result.stderr
    assert runner.commands == []


def test_gazebo_without_image_exits_3_and_names_build_command(invoke_gazebo):
    runner = RecordingRunner(
        captures={IMAGE_INSPECT_PREFIX: CapturedOutput(returncode=1, stdout='')},
    )

    result = invoke_gazebo(runner=runner)

    assert result.exit_code == 3
    assert 'docker compose -f docker/gazebo-sim/compose.yaml build' in result.stderr
    assert runner.commands == []


def test_gazebo_without_gpu_warns_and_runs(invoke_gazebo):
    runner = RecordingRunner()

    result = invoke_gazebo(runner=runner, installed=('docker',))

    assert result.exit_code == 0
    assert 'nvidia-smi not found' in result.stderr
    assert len(runner.commands) == 1


def test_gazebo_json_prints_one_object(invoke_gazebo):
    result = invoke_gazebo('--check', '--nav-source', 'vins', '--json')

    assert json.loads(result.stdout) == {
        'command': 'sim gazebo',
        'exit_code': 0,
        'nav_source': 'vins',
        'check': True,
    }
