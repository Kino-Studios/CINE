"""Fly the CINE Gazebo simulator on Modal and download what it produced.

    python scripts/cloud/run_sim_modal.py --mode probe              # estimate only
    python scripts/cloud/run_sim_modal.py --mode probe --confirm-spend
    python scripts/cloud/run_sim_modal.py --fetch <run_id>          # download again

The container image is docker/gazebo-sim/Dockerfile plus three cloud-only lines (Xvfb,
a `python` on PATH, no entrypoint), so the cloud run builds, fetches weights and
launches exactly as `docker compose up` does. The three repositories are uploaded
from the sibling checkouts next to CINE. Every run is saved on the `cine-sim-runs`
volume first and downloaded afterwards, so a dropped connection loses nothing.
"""

import atexit
import json
import secrets
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import modal
from modal.file_pattern_matcher import FilePatternMatcher
from sim_cli import (
    local_path_for,
    parse_arguments,
    plan_from_arguments,
    plan_with_git_state,
    repository_root,
)
from sim_plan import (
    MODAL_MAX_TIMEOUT_SECONDS,
    SimRunPlan,
    cloud_dockerfile,
    estimate_cost,
    executable_paths,
    format_estimate,
    is_safe_run_id,
    restore_modes_command,
    summary_lines,
    time_budget,
    upload_ignore_patterns,
    wind_profile_names,
)

REMOTE_SOURCE_ROOT = '/workspace/src'
# Inside the container Modal runs this file from /root, where the uploaded checkout
# stands in for the local one.
REPO_ROOT = repository_root(Path(__file__).resolve()) or Path(REMOTE_SOURCE_ROOT) / 'CINE'
DOCKER_CONTEXT = REPO_ROOT / 'docker' / 'gazebo-sim'
WIND_PROFILES = REPO_ROOT / 'cine_gazebo' / 'config' / 'wind_profiles.yaml'
SOURCE_REPOSITORIES = ('CINE', 'CINE-Sensing', 'CINE-Interfaces')
DOWNLOAD_ROOT = REPO_ROOT / 'eval' / 'runs' / 'modal'
APP_NAME = 'cine-gazebo-sim'
RUNS_VOLUME = modal.Volume.from_name('cine-sim-runs', create_if_missing=True)
WEIGHTS_VOLUME = modal.Volume.from_name('cine-sim-weights', create_if_missing=True)
RUNS_MOUNT = '/runs'
WEIGHTS_MOUNT = '/weights'
MIB_PER_GIB = 1024
# How long the local client waits for the result before it stops waiting; the run itself
# keeps going on Modal and is fetched later with --fetch.
RESULT_POLL_SECONDS = 60


def _source_directory(name: str) -> Path:
    return REPO_ROOT if name == 'CINE' else REPO_ROOT.parent / name


def _git_executables(directory: Path) -> list[str]:
    """Inside the container the checkout has no .git, and the image is already built."""

    if not modal.is_local():
        return []

    listing = subprocess.run(
        ['git', 'ls-files', '-s', '-z'],
        cwd=directory,
        capture_output=True,
        text=True,
        check=True,
    )

    return executable_paths(listing.stdout)


def _derived_dockerfile() -> str:
    """Writes the cloud Dockerfile to a temporary file, never into the repo.

    Only the local client builds the image; inside the container this module is imported
    again, and there the path is never read.
    """

    if not modal.is_local():
        return str(DOCKER_CONTEXT / 'Dockerfile')

    handle = tempfile.NamedTemporaryFile('w', suffix='.Dockerfile', delete=False, encoding='utf-8')
    with handle:
        handle.write(cloud_dockerfile((DOCKER_CONTEXT / 'Dockerfile').read_text(encoding='utf-8')))

    atexit.register(Path(handle.name).unlink, missing_ok=True)

    return handle.name


def _image() -> modal.Image:
    image = modal.Image.from_dockerfile(
        _derived_dockerfile(),
        context_dir=DOCKER_CONTEXT,
        ignore=['__pycache__', 'test_*.py', 'compose.yaml'],
    )
    ignore = FilePatternMatcher(*upload_ignore_patterns())

    # Copied rather than mounted, so the layer after it can give back the executable bits
    # that a Windows checkout cannot carry; ROS skips a node script that lacks one.
    for name in SOURCE_REPOSITORIES:
        remote_directory = f'{REMOTE_SOURCE_ROOT}/{name}'
        image = image.add_local_dir(
            _source_directory(name),
            remote_directory,
            ignore=ignore,
            copy=True,
        )
        executables = _git_executables(_source_directory(name))
        if executables:
            image = image.run_commands(restore_modes_command(remote_directory, executables))

    return image.add_local_python_source('sim_plan', 'sim_container', 'sim_cli')


app = modal.App(APP_NAME)


@app.function(
    image=_image(),
    volumes={RUNS_MOUNT: RUNS_VOLUME, WEIGHTS_MOUNT: WEIGHTS_VOLUME},
    timeout=MODAL_MAX_TIMEOUT_SECONDS,
)
def fly_simulator(plan_fields: dict) -> dict:
    from sim_container import ContainerPaths, run_container

    plan = SimRunPlan(**plan_fields)
    paths = ContainerPaths(
        entrypoint='/usr/local/bin/entrypoint.sh',
        work=Path('/tmp/cine-run'),
        bags=Path('/bags'),
        runs=Path(RUNS_MOUNT),
        repo_root=Path(REMOTE_SOURCE_ROOT) / 'CINE',
        colcon_base=Path('/workspace/colcon'),
        ros_setup=Path('/opt/ros/jazzy/setup.bash'),
        eval_python='/opt/cine-venv/bin/python',
        weights=Path(WEIGHTS_MOUNT),
        xvfb=shutil.which('Xvfb'),
    )

    def commit() -> None:
        RUNS_VOLUME.commit()
        WEIGHTS_VOLUME.commit()

    return run_container(plan, paths, commit=commit)


def download_run(run_id: str) -> Path | None:
    """Copies runs/<run_id> from the volume into eval/runs/modal/<run_id>."""

    destination = DOWNLOAD_ROOT / run_id

    try:
        entries = RUNS_VOLUME.listdir(run_id, recursive=True)
    except (FileNotFoundError, modal.exception.NotFoundError):
        print(f'No run {run_id} on the cine-sim-runs volume.')

        return None

    count = 0
    skipped = 0

    for entry in entries:
        if entry.type != modal.volume.FileEntryType.FILE:
            continue

        local_path = local_path_for(entry.path, run_id, destination)
        if local_path is None:
            print(f'skipped {entry.path}: outside the run directory')
            skipped += 1

            continue

        try:
            local_path.parent.mkdir(parents=True, exist_ok=True)

            with local_path.open('wb') as handle:
                for chunk in RUNS_VOLUME.read_file(entry.path):
                    handle.write(chunk)
        # Broad on purpose: a network failure after modal's own retries is not an
        # OSError, and one file lost must not abort the rest of the download.
        except Exception as exception:
            print(f'skipped {entry.path}: {exception}')
            skipped += 1

            # A half-written file would read as a complete one.
            if local_path.is_file():
                local_path.unlink(missing_ok=True)

            continue

        count += 1

    print(f'Downloaded {count} files to {destination}, skipped {skipped}')

    return destination


def print_summary(destination: Path | None) -> int:
    result_path = None if destination is None else destination / 'result.json'
    if result_path is None or not result_path.exists():
        print('The run left no result.json; see its launch.log if one was downloaded.')

        return 1

    result = json.loads(result_path.read_text(encoding='utf-8'))
    print('\n'.join(summary_lines(result)))

    return 0 if result.get('status') in ('ok', 'check_passed', 'probe_done') else 1


def _wait_for(call: modal.FunctionCall) -> dict | None:
    while True:
        try:
            return call.get(timeout=RESULT_POLL_SECONDS)
        except TimeoutError:
            print(f'still running ({time.strftime("%H:%M:%S")})', flush=True)


def use_utf8_output() -> None:
    # Windows Python writes redirected output in cp1252, and Modal's progress lines carry
    # characters cp1252 lacks, so a logged run would crash mid-launch without this.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, 'reconfigure'):
            stream.reconfigure(encoding='utf-8', errors='backslashreplace')


def main(argv: list[str] | None = None) -> int:
    use_utf8_output()
    arguments = parse_arguments(argv)
    if arguments.fetch is not None:
        if not is_safe_run_id(arguments.fetch):
            print(f'run id {arguments.fetch!r} may hold only letters, digits, ".", "_" and "-"')

            return 2

        return print_summary(download_run(arguments.fetch))

    wind_names = wind_profile_names(WIND_PROFILES.read_text(encoding='utf-8'))
    plan, problems = plan_from_arguments(
        arguments,
        wind_names,
        datetime.now(),
        secrets.token_hex(2),
    )
    plan = plan_with_git_state(plan, REPO_ROOT)

    if problems:
        print('The plan is not valid:\n  ' + '\n  '.join(problems))

        return 2

    missing = [
        str(_source_directory(name))
        for name in SOURCE_REPOSITORIES
        if not _source_directory(name).is_dir()
    ]
    if missing:
        print('Missing source checkouts, expected next to CINE:\n  ' + '\n  '.join(missing))

        return 2

    print(format_estimate(plan, estimate_cost(plan), launching=arguments.confirm_spend))

    if not arguments.confirm_spend:
        return 0

    budget = time_budget(plan)
    configured = fly_simulator.with_options(
        cpu=(plan.cpu_cores, plan.cpu_cores),
        memory=(int(plan.memory_gib * MIB_PER_GIB), int(plan.memory_gib * MIB_PER_GIB)),
        gpu=plan.gpu,
        timeout=budget.function_timeout_seconds,
    )

    with modal.enable_output(), app.run(detach=True):
        call = configured.spawn(asdict(plan))
        print(f'Launched {plan.run_id} as call {call.object_id}.')
        print(
            f'If this terminal disconnects the run continues; fetch it with --fetch {plan.run_id}.',
        )

        try:
            _wait_for(call)
        except modal.exception.FunctionTimeoutError:
            print(
                f'The run hit its {budget.function_timeout_seconds} s timeout; fetching what it saved.',
            )
        except KeyboardInterrupt:
            print(
                f'Stopped waiting. The run continues on Modal; fetch it with --fetch {plan.run_id}.',
            )

            return 130
        except Exception as exception:
            print(
                f'The remote call failed ({type(exception).__name__}: {exception}); fetching what it saved.',
            )

    return print_summary(download_run(plan.run_id))


if __name__ == '__main__':
    sys.exit(main())
