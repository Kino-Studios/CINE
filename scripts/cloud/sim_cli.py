"""The command line of the Modal runner: its arguments, the plan they make, and download paths.

Kept apart from run_sim_modal.py so it can be tested without Modal installed.
"""

import argparse
import subprocess
from dataclasses import replace
from datetime import datetime
from pathlib import Path, PurePath, PurePosixPath

from sim_plan import (
    GPU_USD_PER_SECOND,
    MODES,
    NAV_SOURCES,
    SimRunPlan,
    git_state_from_output,
    make_run_id,
    validate_plan,
)

# A checkout on a cloud-synced folder can stall git while files are fetched.
GIT_TIMEOUT_SECONDS = 30

DESCRIPTION = """Fly the CINE Gazebo simulator on Modal's CPUs and download what it produced.

Without --confirm-spend this only prints the plan and its cost estimate.
Modes: 'probe' checks the container (display, DDS, gz transport, rendering, weights)
without building; 'check' builds and runs the smoke flight; 'mission' builds, flies
for --flight-seconds, records a bag and scores it with cine_eval."""


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=DESCRIPTION,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--mode', default='mission', choices=MODES)
    parser.add_argument(
        '--cpu',
        type=float,
        default=8.0,
        help='physical cores reserved (default 8)',
    )
    parser.add_argument(
        '--memory-gib',
        type=float,
        default=16.0,
        help='memory reserved in GiB (default 16)',
    )
    parser.add_argument(
        '--gpu',
        default=None,
        choices=sorted(GPU_USD_PER_SECOND),
        help='default: none',
    )
    parser.add_argument(
        '--flight-seconds',
        type=int,
        default=600,
        help='mission flight time (default 600)',
    )
    parser.add_argument('--nav-source', default='sim', choices=NAV_SOURCES)
    parser.add_argument(
        '--wind',
        default='calm',
        help='a profile in cine_gazebo/config/wind_profiles.yaml',
    )
    parser.add_argument(
        '--confirm-spend',
        action='store_true',
        help='launch the run and pay for it',
    )
    parser.add_argument(
        '--fetch',
        metavar='RUN_ID',
        help='download a finished run from the cine-sim-runs volume instead of launching one',
    )
    arguments = parser.parse_args(argv)
    if arguments.fetch is not None and arguments.confirm_spend:
        parser.error(
            '--fetch downloads an existing run; it cannot be combined with --confirm-spend',
        )

    return arguments


def plan_from_arguments(
    arguments: argparse.Namespace,
    wind_names: list[str],
    now: datetime,
    token: str,
) -> tuple[SimRunPlan, list[str]]:
    plan = SimRunPlan(
        mode=arguments.mode,
        cpu_cores=arguments.cpu,
        memory_gib=arguments.memory_gib,
        gpu=arguments.gpu,
        flight_seconds=arguments.flight_seconds,
        nav_source=arguments.nav_source,
        wind=arguments.wind,
        run_id=make_run_id(now, arguments.mode, arguments.nav_source, arguments.wind, token),
    )

    return plan, validate_plan(plan, wind_names)


def git_output(arguments: list[str], repository: PurePath) -> str | None:
    """What `git <arguments>` prints in the repository, or None when the call fails."""

    try:
        return subprocess.run(
            ['git', *arguments],
            cwd=repository,
            capture_output=True,
            text=True,
            check=True,
            timeout=GIT_TIMEOUT_SECONDS,
        ).stdout
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None


def plan_with_git_state(plan: SimRunPlan, repository: PurePath) -> SimRunPlan:
    """The plan carrying the checkout's commit and dirty flag, read on this host."""

    commit, dirty = git_state_from_output(
        git_output(['rev-parse', 'HEAD'], repository),
        git_output(['status', '--porcelain'], repository),
    )

    return replace(plan, git_commit=commit, git_dirty=dirty)


def repository_root(script_path: PurePath) -> PurePath | None:
    """The CINE checkout holding scripts/cloud/<script>, or None where Modal copied the
    script on its own (to /root inside the container)."""

    folder = script_path.parent
    if folder.name != 'cloud' or folder.parent.name != 'scripts':
        return None

    return folder.parent.parent


def _is_plain_part(part: str) -> bool:
    # Windows reads a backslash as a separator and a colon as a drive or stream.
    return part not in ('..', '.', '') and '\\' not in part and ':' not in part


def local_path_for(volume_path: str, run_id: str, destination: Path) -> Path | None:
    """Where a file listed on the volume is saved, or None when it is not inside the run."""

    parts = PurePosixPath(volume_path.lstrip('/')).parts
    if len(parts) < 2 or parts[0] != run_id or not all(map(_is_plain_part, parts)):
        return None

    local_path = destination.joinpath(*parts[1:])
    if not local_path.resolve().is_relative_to(destination.resolve()):
        return None

    return local_path
