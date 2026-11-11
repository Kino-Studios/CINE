import json
import os
import shlex
import subprocess
import sys
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Annotated, Protocol

import typer

INTERRUPTED_EXIT_CODE = 130
TIMEOUT_EXIT_CODE = 124
ROS2_MISSING_MESSAGE = (
    'ros2 not found: source /opt/ros/jazzy/setup.bash and <workspace>/install/setup.bash first'
)
ROOT_MISSING_MESSAGE = (
    'repository root not found: the tool was installed without --editable. '
    'Reinstall with `pipx install --editable ./cine_cli` from the repository, '
    'or set CINE_ROOT.'
)


class ExitCode(IntEnum):
    OK = 0
    FAILED = 1
    USAGE = 2
    ENVIRONMENT = 3


@dataclass(frozen=True)
class CapturedOutput:
    returncode: int
    stdout: str


class Runner(Protocol):
    def run(self, command: list[str], cwd: Path, stdout_to_stderr: bool = False) -> int: ...

    def capture(self, command: list[str], cwd: Path, timeout_seconds: float) -> CapturedOutput: ...


class SubprocessRunner:
    def run(self, command: list[str], cwd: Path, stdout_to_stderr: bool = False) -> int:
        process = subprocess.Popen(
            command, cwd=cwd, stdout=sys.stderr if stdout_to_stderr else None
        )
        try:
            return process.wait()
        except KeyboardInterrupt:
            # Ctrl-C already reached the child's process group; let it shut down gracefully.
            try:
                process.wait()
            except KeyboardInterrupt:
                process.kill()

            return INTERRUPTED_EXIT_CODE

    def capture(self, command: list[str], cwd: Path, timeout_seconds: float) -> CapturedOutput:
        try:
            completed = subprocess.run(
                command,
                cwd=cwd,
                env={**os.environ, 'PYTHONUNBUFFERED': '1'},
                capture_output=True,
                text=True,
                timeout=timeout_seconds,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            partial = exc.stdout.decode() if isinstance(exc.stdout, bytes) else exc.stdout

            return CapturedOutput(returncode=TIMEOUT_EXIT_CODE, stdout=partial or '')

        return CapturedOutput(returncode=completed.returncode, stdout=completed.stdout)


@dataclass(frozen=True)
class Environment:
    repository_root: Path | None
    runner: Runner
    which: Callable[[str], str | None]
    variables: Mapping[str, str]


def default_environment() -> Environment:
    import shutil

    variables = dict(os.environ)
    repository_root = None
    if 'CINE_ROOT' in variables:
        repository_root = Path(variables['CINE_ROOT'])
    else:
        candidate = Path(__file__).resolve().parents[2]
        if (candidate / 'cine_bringup').is_dir():
            repository_root = candidate

    return Environment(
        repository_root=repository_root,
        runner=SubprocessRunner(),
        which=shutil.which,
        variables=variables,
    )


def _fail_environment(message: str) -> typer.Exit:
    typer.echo(message, err=True)

    return typer.Exit(ExitCode.ENVIRONMENT)


def require_root(environment: Environment) -> Path:
    if environment.repository_root is None:
        raise _fail_environment(ROOT_MISSING_MESSAGE)

    return environment.repository_root


def require_ros2(environment: Environment) -> None:
    if environment.which('ros2') is None:
        raise _fail_environment(ROS2_MISSING_MESSAGE)


def execute(environment: Environment, command: list[str], dry_run: bool, as_json: bool) -> None:
    root = require_root(environment)
    if dry_run:
        if as_json:
            typer.echo(json.dumps({'command': command, 'cwd': str(root)}))
        else:
            typer.echo(shlex.join(command))

        return
    exit_code = environment.runner.run(command, root, stdout_to_stderr=as_json)
    if as_json:
        typer.echo(json.dumps({'command': command, 'exit_code': exit_code}))
    if exit_code != 0:
        raise typer.Exit(exit_code)


JsonFlag = Annotated[
    bool, typer.Option('--json', help='Print one JSON object on stdout instead of text.')
]
DryRunFlag = Annotated[
    bool, typer.Option('--dry-run', '-n', help='Print the command and start nothing.')
]


def get_environment(context: typer.Context) -> Environment:
    return context.obj
