from dataclasses import dataclass, field
from pathlib import Path

import pytest
from cine_cli.common import CapturedOutput, Environment
from cine_cli.main import app
from typer.testing import CliRunner

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


@dataclass
class RecordingRunner:
    exit_code: int = 0
    captures: dict[tuple[str, ...], CapturedOutput] = field(default_factory=dict)
    commands: list[tuple[list[str], Path]] = field(default_factory=list)
    stdout_redirections: list[bool] = field(default_factory=list)

    def run(self, command: list[str], cwd: Path, stdout_to_stderr: bool = False) -> int:
        self.commands.append((command, cwd))
        self.stdout_redirections.append(stdout_to_stderr)

        return self.exit_code

    def capture(self, command: list[str], cwd: Path, timeout_seconds: float) -> CapturedOutput:
        for prefix, output in self.captures.items():
            if tuple(command[: len(prefix)]) == prefix:
                return output

        return CapturedOutput(returncode=0, stdout='')


@pytest.fixture
def repository_root() -> Path:
    return REPOSITORY_ROOT


@pytest.fixture
def runner() -> RecordingRunner:
    return RecordingRunner()


@pytest.fixture
def environment(runner, repository_root) -> Environment:
    return Environment(
        repository_root=repository_root,
        runner=runner,
        which=lambda name: f'/opt/ros/jazzy/bin/{name}' if name == 'ros2' else None,
        variables={},
    )


@pytest.fixture
def invoke(environment):
    def invoke_command(*arguments, environment=environment):
        return CliRunner().invoke(app, list(arguments), obj=environment)

    return invoke_command
