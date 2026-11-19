import hashlib
import json
import os
import shlex
from pathlib import Path
from typing import Annotated

import typer

from cine_cli.common import (
    DryRunFlag,
    Environment,
    ExitCode,
    JsonFlag,
    get_environment,
    require_docker_image,
    require_root,
)

EVAL_IMAGE = 'cine-gazebo-sim:dev'
EVAL_BUILD_COMMAND = 'docker compose -f docker/gazebo-sim/compose.yaml build'
GIT_TIMEOUT_SECONDS = 30
CONTAINER_ROOT = '/cine'
CONTAINER_BAGS = '/bags'
DEFAULT_PLAN = 'eval/analysis_plan.toml'
IMAGE_SCRIPT = (
    'source /opt/ros/jazzy/setup.bash && exec /opt/cine-venv/bin/python -m cine_eval "$@"'
)

app = typer.Typer(
    no_args_is_help=True,
    help=(
        'Score recorded flights inside the cine-gazebo-sim:dev image. Relative paths are '
        'read from the repository root.'
    ),
)


def _usage_error(message: str) -> typer.Exit:
    typer.echo(message, err=True)

    return typer.Exit(ExitCode.USAGE)


def _container_path(root: Path, text: str) -> str:
    """Maps a host path under the repository to its place in the image.

    A relative path is read from the current directory, as a shell would; a path that does not
    end up under the repository, ``..`` escapes included, is a usage error.
    """

    path = Path(text).expanduser()
    absolute = path if path.is_absolute() else Path.cwd() / path

    try:
        relative = absolute.resolve().relative_to(root.resolve())
    except ValueError:
        raise _usage_error(
            f'{text} is outside the repository {root}; the image cannot see it',
        ) from None

    if relative == Path('.'):
        return CONTAINER_ROOT

    return f'{CONTAINER_ROOT}/{relative.as_posix()}'


def _bags_directory(environment: Environment) -> Path:
    configured = environment.variables.get('CINE_BAGS_DIR')

    return (Path(configured) if configured else Path.home() / 'cine-bags').expanduser().resolve()


def _container_bag_path(environment: Environment, text: str) -> str:
    bags_directory = _bags_directory(environment)
    if not bags_directory.is_dir():
        raise _usage_error(
            f'bags directory {bags_directory} does not exist; set CINE_BAGS_DIR to where '
            'the bags are recorded',
        )

    path = Path(text).expanduser()
    absolute = (path if path.is_absolute() else Path.cwd() / path).resolve()

    try:
        relative = absolute.relative_to(bags_directory)
    except ValueError:
        raise _usage_error(
            f'{text} is outside the bags directory {bags_directory}; '
            'set CINE_BAGS_DIR or move the bag there',
        ) from None

    return f'{CONTAINER_BAGS}/{relative.as_posix()}'


def _named_directories(root: Path, option: str, assignments: list[str]) -> list[str]:
    """Checks NAME=DIR[,DIR...] and maps each directory into the image."""

    translated = []

    for assignment in assignments:
        name, separator, directories = assignment.partition('=')
        parts = [part for part in directories.split(',') if part]
        if not separator or not name or not parts:
            raise _usage_error(f'{option} {assignment!r} is not NAME=DIR[,DIR...]')

        mapped = ','.join(_container_path(root, part) for part in parts)
        translated += [option, f'{name}={mapped}']

    return translated


def _scorer_flags(environment: Environment, root: Path) -> list[str]:
    note = 'git is unavailable here: the scorer commit and dirty flag will be recorded as unknown'

    if environment.which('git') is None:
        typer.echo(note, err=True)

        return []

    commit = environment.runner.capture(['git', 'rev-parse', 'HEAD'], root, GIT_TIMEOUT_SECONDS)
    status = environment.runner.capture(['git', 'status', '--porcelain'], root, GIT_TIMEOUT_SECONDS)
    flags = []

    if commit.returncode == 0 and commit.stdout.strip():
        flags += ['-e', f'CINE_EVAL_SCORER_COMMIT={commit.stdout.strip()}']

    if status.returncode == 0:
        flags += ['-e', f'CINE_EVAL_SCORER_DIRTY={1 if status.stdout.strip() else 0}']

    if len(flags) != 4:
        typer.echo(note.replace('is unavailable here', 'failed here'), err=True)

        return []

    return flags


def _committed_plan_flags(environment: Environment, root: Path, plan: str) -> list[str]:
    """The SHA-1 of the plan as committed at HEAD, for the image, which has no git.

    Without it the comparison is labelled exploratory. Hashing the text git prints equals
    hashing the file only while the committed file uses LF line endings.
    """

    note = 'the plan is not committed at HEAD or git failed: the comparison will be labelled exploratory'
    path = Path(plan).expanduser()
    absolute = path if path.is_absolute() else Path.cwd() / path
    shown = None

    if environment.which('git') is not None:
        shown = environment.runner.capture(
            ['git', 'show', f'HEAD:{absolute.resolve().relative_to(root.resolve()).as_posix()}'],
            root,
            GIT_TIMEOUT_SECONDS,
        )

    if shown is None or shown.returncode != 0:
        typer.echo(note, err=True)

        return []

    digest = hashlib.sha1(shown.stdout.encode('utf-8')).hexdigest()

    return ['-e', f'CINE_EVAL_PLAN_COMMITTED_DIGEST={digest}']


def _user_flags() -> list[str]:
    if not hasattr(os, 'getuid'):
        return []

    return ['--user', f'{os.getuid()}:{os.getgid()}']


def _image_command(
    environment: Environment,
    root: Path,
    arguments: list[str],
    extra_flags: list[str],
    with_bags: bool,
) -> list[str]:
    mounts = [
        '-v',
        f'{root}:{CONTAINER_ROOT}:ro',
        '-v',
        f'{root}/eval:{CONTAINER_ROOT}/eval',
        '-v',
        f'{root}/docs:{CONTAINER_ROOT}/docs',
    ]

    if with_bags:
        mounts += ['-v', f'{_bags_directory(environment)}:{CONTAINER_BAGS}:ro']

    return [
        'docker',
        'run',
        '--rm',
        *_user_flags(),
        *_scorer_flags(environment, root),
        *extra_flags,
        '-e',
        f'CINE_CONFIG_ROOT={CONTAINER_ROOT}/cine_config/config',
        '-e',
        f'PYTHONPATH={CONTAINER_ROOT}/cine_eval:{CONTAINER_ROOT}/cine_config',
        '-e',
        'PYTHONDONTWRITEBYTECODE=1',
        *mounts,
        '-w',
        CONTAINER_ROOT,
        '--entrypoint',
        'bash',
        EVAL_IMAGE,
        '-c',
        IMAGE_SCRIPT,
        'cine-eval',
        *arguments,
    ]


def _run_in_image(
    environment: Environment,
    arguments: list[str],
    dry_run: bool,
    as_json: bool,
    extra_flags: list[str] | None = None,
    with_bags: bool = False,
) -> None:
    root = require_root(environment)

    if not dry_run:
        require_docker_image(environment, root, EVAL_IMAGE, EVAL_BUILD_COMMAND)

    command = _image_command(
        environment,
        root,
        [*arguments, *(['--json'] if as_json else [])],
        extra_flags or [],
        with_bags,
    )

    if dry_run:
        typer.echo(
            json.dumps({'command': command, 'cwd': str(root)}) if as_json else shlex.join(command),
        )

        return

    exit_code = environment.runner.run(command, root)
    if exit_code != 0:
        raise typer.Exit(exit_code)


@app.command()
def extract(
    context: typer.Context,
    bag_dir: Annotated[
        str,
        typer.Argument(help='Recorded bag directory under the bags directory.'),
    ],
    out: Annotated[
        str | None,
        typer.Option(help='Record directory; default eval/runs/<bag name>.'),
    ] = None,
    dry_run: DryRunFlag = False,
    as_json: JsonFlag = False,
) -> None:
    """Read a recorded bag into a flight record."""

    environment = get_environment(context)
    root = require_root(environment)
    arguments = ['extract', _container_bag_path(environment, bag_dir)]

    if out is not None:
        arguments += ['--out', _container_path(root, out)]

    _run_in_image(environment, arguments, dry_run, as_json, with_bags=True)


@app.command()
def score(
    context: typer.Context,
    run_dir: Annotated[str, typer.Argument(help='Run directory under eval/runs.')],
    dry_run: DryRunFlag = False,
    as_json: JsonFlag = False,
) -> None:
    """Score a flight record on every channel."""

    environment = get_environment(context)
    root = require_root(environment)

    _run_in_image(
        environment,
        ['score', _container_path(root, run_dir)],
        dry_run,
        as_json,
    )


@app.command()
def gate(
    context: typer.Context,
    run_dir: Annotated[str, typer.Argument(help='Run directory under eval/runs.')],
    dry_run: DryRunFlag = False,
    as_json: JsonFlag = False,
) -> None:
    """Judge a scored run: PASS, or REVIEW (exit 1)."""

    environment = get_environment(context)
    root = require_root(environment)

    _run_in_image(environment, ['gate', _container_path(root, run_dir)], dry_run, as_json)


@app.command()
def compare(
    context: typer.Context,
    arm: Annotated[
        list[str],
        typer.Option('--arm', help='NAME=DIR[,DIR...]; give exactly two.'),
    ],
    name: Annotated[str, typer.Option(help='Name of the output files.')],
    repeatability: Annotated[
        list[str] | None,
        typer.Option(help='NAME=DIR[,DIR...]: repeat flights of the arm NAME; repeatable.'),
    ] = None,
    plan: Annotated[str, typer.Option(help='Analysis plan file.')] = DEFAULT_PLAN,
    out_dir: Annotated[
        str | None,
        typer.Option(help='Output directory; default eval/runs/compare.'),
    ] = None,
    dry_run: DryRunFlag = False,
    as_json: JsonFlag = False,
) -> None:
    """Compare two arms of scored runs against the preregistered analysis plan."""

    environment = get_environment(context)
    root = require_root(environment)
    arm_options = _named_directories(root, '--arm', arm)
    repeat_options = _named_directories(root, '--repeatability', repeatability or [])
    arm_names = {assignment.partition('=')[0] for assignment in arm}
    unknown = sorted(
        {assignment.partition('=')[0] for assignment in repeatability or []} - arm_names,
    )

    if len(arm) != 2:
        raise _usage_error(f'compare needs exactly two --arm options, got {len(arm)}')

    if not name or '/' in name or '\\' in name or '..' in name:
        raise _usage_error(f'--name {name!r} must be a plain file name')

    if unknown:
        raise _usage_error(f'--repeatability names {", ".join(unknown)}, which no --arm names')

    arguments = [
        'compare',
        *arm_options,
        *repeat_options,
        '--plan',
        _container_path(root, plan),
        '--name',
        name,
    ]

    if out_dir is not None:
        arguments += ['--out-dir', _container_path(root, out_dir)]

    _run_in_image(
        environment,
        arguments,
        dry_run,
        as_json,
        extra_flags=_committed_plan_flags(environment, root, plan),
    )


@app.command()
def standings(
    context: typer.Context,
    manifest: Annotated[str, typer.Option(help='Standings manifest file.')] = 'eval/standings.toml',
    dry_run: DryRunFlag = False,
    as_json: JsonFlag = False,
) -> None:
    """Rebuild the results table, results.csv and manifest.json."""

    environment = get_environment(context)
    root = require_root(environment)

    _run_in_image(
        environment,
        ['standings', '--manifest', _container_path(root, manifest)],
        dry_run,
        as_json,
    )
