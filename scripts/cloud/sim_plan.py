"""Plans a Gazebo simulator run on Modal: checks, sizing, cost, and reading its output.

Imports nothing from Modal or ROS, so bare pytest covers it on any machine. The runner
(run_sim_modal.py) and the in-container driver (sim_container.py) call into it.
"""

import math
import re
import shlex
from dataclasses import dataclass
from datetime import datetime
from typing import NamedTuple

MODES = ('mission', 'check', 'probe')
NAV_SOURCES = ('sim', 'vins')

# Modal list prices, read from https://modal.com/pricing on 2026-11-20. One "core" is a
# physical core, two vCPUs. Billing is the larger of request and use, so the runner sets
# the limit equal to the request and the ceiling below holds.
PRICES_READ_ON = '2026-11-20'
CPU_CORE_USD_PER_SECOND = 0.0000131
MEMORY_GIB_USD_PER_SECOND = 0.00000222
GPU_USD_PER_SECOND = {
    'T4': 0.000164,
    'L4': 0.000222,
    'L40S': 0.000542,
    'A100-40GB': 0.000583,
    'A100-80GB': 0.000694,
    'H100': 0.001097,
}

CPU_CORES_RANGE = (2.0, 64.0)
MEMORY_GIB_RANGE = (8.0, 256.0)
FLIGHT_SECONDS_RANGE = (60, 14400)

# A C++ compile job of the planner (Eigen-heavy) peaks near 2 GiB; 2 GiB stay free for the
# rest of the container. More than three packages at once gains little, because the
# workspace's dependency chain serialises most of the build.
GIB_PER_COMPILE_JOB = 2.0
RESERVED_BUILD_GIB = 2.0
MAX_PARALLEL_PACKAGES = 3

# Typical durations are assumed, not measured on Modal; the allowances bound each stage.
TYPICAL_SECONDS = {
    'build': 1200,
    'weights': 300,
    'launch': 180,
    'check_flight': 600,
    'shutdown': 120,
    'scoring': 300,
    'probe': 300,
}
ALLOWANCE_SECONDS = {
    'build': 3600,
    'weights': 900,
    'launch': 600,
    'check_flight': 1500,
    'shutdown': 120,
    'scoring': 900,
    'probe': 900,
}
# The driver stops itself this long before Modal's timeout, so results are always saved.
TIMEOUT_MARGIN_SECONDS = 300
MODAL_MAX_TIMEOUT_SECONDS = 86400

LOOPBACK = '127.0.0.1'
# Unicast discovery over loopback: the whole stack shares one container, and multicast
# inside Modal's sandbox is unverified. Each participant takes its own index, and the
# stack runs about twenty processes, so the default cap of nine indices is raised.
CYCLONEDDS_LOOPBACK_CONFIG = (
    '<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="127.0.0.1"/>'
    '</Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery>'
    '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120'
    '</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery>'
    '</Domain></CycloneDDS>'
)
XVFB_DISPLAY = ':99'

UPLOAD_EXCLUDES = (
    '**/.git/**',
    '**/.venv/**',
    '**/venv/**',
    '**/__pycache__/**',
    '**/.pytest_cache/**',
    '**/.ruff_cache/**',
    '**/.superpowers/**',
    '**/build/**',
    '**/install/**',
    '**/log/**',
    '**/eval/runs/**',
    '**/datasets/**',
    '**/*.mcap',
    '**/*.db3',
    '**/.deps/**',
    # Licensed airframe meshes that may not be redistributed (gitignored for that reason).
    '**/meshes/restricted_*',
)

CLOUD_DOCKERFILE_LINES = (
    '',
    '# Appended by scripts/cloud/sim_plan.py for Modal: Xvfb gives Gazebo a display to render',
    '# its camera through Mesa, Modal needs `python` on PATH, and Modal runs its own command.',
    'RUN set -eux; \\',
    '    apt-get update; \\',
    '    apt-get install -y --no-install-recommends xvfb; \\',
    '    rm -rf /var/lib/apt/lists/*; \\',
    '    ln -sf /usr/bin/python3 /usr/local/bin/python',
    'ENTRYPOINT []',
)

DOCKERFILE_USER = re.compile(r'^USER\s+(\S+)\s*$')
# Modal builds every step as root and ignores USER, and its builder sets no_new_privs, so
# sudo cannot work there. A USER section is therefore built as root: sudo becomes a
# pass-through (env runs its arguments), git trusts the non-root-owned checkout, and
# ArduPilot's installer, which refuses root, has that one check switched off. The lines
# avoid "$" and backslashes so they read the same under any Dockerfile parser.
ENTER_ROOT_BUILT_SECTION = (
    "RUN ln -sf /usr/bin/env /usr/local/sbin/sudo; git config --system --add safe.directory '*'"
)
LEAVE_ROOT_BUILT_SECTION = (
    'RUN rm -f /usr/local/sbin/sudo; git config --system --unset-all safe.directory'
)
INSTALLER_ROOT_CHECK_PATCH = (
    'if [ -f Tools/environment_install/install-prereqs-ubuntu.sh ]; then '
    "sed -i 's/^if [[] .EUID == 0 []]; then/if false; then/' "
    'Tools/environment_install/install-prereqs-ubuntu.sh; fi'
)

SAFE_RUN_ID = re.compile(r'[A-Za-z0-9._-]+')
TOP_LEVEL_KEY = re.compile(r'^([A-Za-z0-9_]+):', re.MULTILINE)
PROCESS_DIED = re.compile(r'\[ERROR\] \[(.+?)-\d+\]: process has died \[pid \d+, exit code (-?\d+)')
REAL_TIME_FACTOR = re.compile(r'real_time_factor:\s*([0-9.eE+-]+)')
STATS_MESSAGE_START = re.compile(r'^\s*sim_time\s*\{', re.MULTILINE)
STATS_SIM_TIME = re.compile(r'\bsim_time\s*\{([^}]*)\}')
STATS_REAL_TIME = re.compile(r'\breal_time\s*\{([^}]*)\}')
STATS_SECONDS = re.compile(r'\bsec:\s*(-?\d+)')
STATS_NANOSECONDS = re.compile(r'\bnsec:\s*(-?\d+)')
NANOSECONDS_PER_SECOND = 1e9
MEMORY_AVAILABLE = re.compile(r'^MemAvailable:\s+(\d+) kB', re.MULTILINE)
# A process ended by the SIGINT the driver sends at the end of a flight is not a crash.
SIGINT_EXIT_CODES = (-2, 130)
KIB_PER_GIB = 1024**2
BYTES_PER_GIB = 1024**3


@dataclass(frozen=True)
class SimRunPlan:
    mode: str
    cpu_cores: float
    memory_gib: float
    gpu: str | None
    flight_seconds: int
    nav_source: str
    wind: str
    run_id: str
    # The checkout's state on the host, where git exists; the simulator image has none.
    git_commit: str | None = None
    git_dirty: bool | None = None


@dataclass(frozen=True)
class BuildParallelism:
    package_workers: int
    make_jobs: int


@dataclass(frozen=True)
class TimeBudget:
    typical_seconds: int
    function_timeout_seconds: int
    internal_deadline_seconds: int


@dataclass(frozen=True)
class CostEstimate:
    usd_per_hour: float
    typical_seconds: int
    typical_usd: float
    ceiling_seconds: int
    ceiling_usd: float


@dataclass(frozen=True)
class Crash:
    name: str
    exit_code: int


@dataclass(frozen=True)
class OutcomeFacts:
    mode: str
    launch_started: bool
    # None while the stack was still running when the driver stopped it.
    stack_exit_code: int | None
    # 'flight_time', 'memory' or 'deadline' when the driver stopped the stack, else None.
    stopped_by: str | None
    bag_complete: bool
    crash_count: int


def _outside(name: str, value: float, bounds: tuple[float, float]) -> list[str]:
    low, high = bounds
    if low <= value <= high:
        return []

    return [f'{name} {value} is outside {low:g} to {high:g}']


def validate_plan(plan: SimRunPlan, wind_names: list[str]) -> list[str]:
    problems = []

    if plan.mode not in MODES:
        problems.append(f"mode '{plan.mode}' is not one of {', '.join(MODES)}")

    if plan.nav_source not in NAV_SOURCES:
        problems.append(f"nav_source '{plan.nav_source}' is not one of {', '.join(NAV_SOURCES)}")

    if plan.wind not in wind_names:
        problems.append(
            f"wind '{plan.wind}' is not in wind_profiles.yaml ({', '.join(wind_names)})",
        )

    if plan.gpu is not None and plan.gpu not in GPU_USD_PER_SECOND:
        problems.append(f"gpu '{plan.gpu}' is not one of {', '.join(GPU_USD_PER_SECOND)}")

    problems += _outside('cpu_cores', plan.cpu_cores, CPU_CORES_RANGE)
    problems += _outside('memory_gib', plan.memory_gib, MEMORY_GIB_RANGE)

    if plan.mode == 'mission':
        problems += _outside('flight_seconds', plan.flight_seconds, FLIGHT_SECONDS_RANGE)

    if not is_safe_run_id(plan.run_id):
        problems.append(f"run_id '{plan.run_id}' may hold only letters, digits, '.', '_' and '-'")

    return problems


def wind_profile_names(yaml_text: str) -> list[str]:
    return TOP_LEVEL_KEY.findall(yaml_text)


def make_run_id(now: datetime, mode: str, nav_source: str, wind: str, token: str) -> str:
    return f'{now:%Y-%m-%d_%H-%M-%S}_{mode}_{nav_source}_{wind}_{token}'


def is_safe_run_id(text: str) -> bool:
    return bool(SAFE_RUN_ID.fullmatch(text)) and set(text) != {'.'}


def build_parallelism(cpu_cores: float, memory_gib: float) -> BuildParallelism:
    virtual_cpus = math.floor(cpu_cores * 2)
    jobs_memory_allows = math.floor((memory_gib - RESERVED_BUILD_GIB) / GIB_PER_COMPILE_JOB)
    total_jobs = max(1, min(virtual_cpus, jobs_memory_allows))
    package_workers = min(MAX_PARALLEL_PACKAGES, total_jobs)

    return BuildParallelism(package_workers, max(1, total_jobs // package_workers))


def colcon_defaults_yaml(parallelism: BuildParallelism) -> str:
    return f'build:\n  parallel-workers: {parallelism.package_workers}\n'


def _build_user_sections_as_root(original: str) -> str:
    in_user_section = False
    lines = []

    for line in original.splitlines():
        user_match = DOCKERFILE_USER.match(line)
        if user_match:
            entering = user_match.group(1) not in ('root', '0')
            if entering and not in_user_section:
                lines.append(ENTER_ROOT_BUILT_SECTION)
            elif in_user_section and not entering:
                lines.append(LEAVE_ROOT_BUILT_SECTION)

            in_user_section = entering
        elif in_user_section and line.startswith('RUN ') and not line.startswith('RUN ['):
            lines.append(f'RUN {INSTALLER_ROOT_CHECK_PATCH}; {line[len("RUN ") :]}')
        else:
            lines.append(line)

    return '\n'.join(lines) + '\n'


def cloud_dockerfile(original: str) -> str:
    return _build_user_sections_as_root(original) + '\n'.join(CLOUD_DOCKERFILE_LINES) + '\n'


def upload_ignore_patterns() -> list[str]:
    return list(UPLOAD_EXCLUDES)


def executable_paths(ls_files_output: str) -> list[str]:
    """The files `git ls-files -s -z` lists with mode 100755.

    A checkout on Windows has no executable bit, so the upload loses it; git's index is
    the one place that still records which files ROS must be able to run.
    """

    paths = []

    for entry in ls_files_output.split('\0'):
        metadata, _, path = entry.partition('\t')
        if path and metadata.split(' ')[0] == '100755':
            paths.append(path)

    return paths


def restore_modes_command(directory: str, paths: list[str]) -> str:
    """A shell command that marks each listed file under `directory` executable.

    A listed file the upload ignored is skipped rather than failing the image build.
    """

    quoted = ' '.join(shlex.quote(path) for path in paths)

    return (
        f'cd {shlex.quote(directory)} && for path in {quoted}; do '
        'if [ -e "$path" ]; then chmod +x "$path"; fi; done'
    )


def git_state_from_output(
    head_output: str | None,
    status_output: str | None,
) -> tuple[str | None, bool | None]:
    """The commit and dirty flag from `git rev-parse HEAD` and `git status --porcelain`.

    None stands for a call that failed: no commit, or a tree whose state is not known.
    """

    commit = (head_output or '').strip()
    if not commit:
        return None, None

    if status_output is None:
        return commit, None

    return commit, bool(status_output.strip())


def stack_environment(
    plan: SimRunPlan,
    parallelism: BuildParallelism,
    colcon_defaults_path: str,
) -> dict[str, str]:
    is_check = plan.mode == 'check'
    git_state = {}

    if plan.git_commit is not None:
        git_state['CINE_GIT_COMMIT'] = plan.git_commit

    if plan.git_dirty is not None:
        git_state['CINE_GIT_DIRTY'] = '1' if plan.git_dirty else '0'

    return {
        **git_state,
        'HEADLESS': 'true',
        'NAV_SOURCE': plan.nav_source,
        'WIND': plan.wind,
        'MISSION': 'false' if is_check else 'true',
        'CHECK': 'true' if is_check else 'false',
        'DISPLAY': XVFB_DISPLAY,
        'MAKEFLAGS': f'-j{parallelism.make_jobs}',
        'COLCON_DEFAULTS_FILE': colcon_defaults_path,
        'CYCLONEDDS_URI': CYCLONEDDS_LOOPBACK_CONFIG,
        'GZ_IP': LOOPBACK,
        'PYTHONUNBUFFERED': '1',
    }


def _stage_seconds(plan: SimRunPlan, durations: dict[str, int]) -> int:
    if plan.mode == 'probe':
        return durations['weights'] + durations['probe']

    stages = ('build', 'weights', 'launch', 'shutdown')

    # Only a mission records a bag worth scoring; the smoke check reports pass or fail itself.
    if plan.mode == 'check':
        return sum(durations[stage] for stage in stages) + durations['check_flight']

    return sum(durations[stage] for stage in (*stages, 'scoring')) + plan.flight_seconds


def time_budget(plan: SimRunPlan) -> TimeBudget:
    internal_deadline = min(
        _stage_seconds(plan, ALLOWANCE_SECONDS),
        MODAL_MAX_TIMEOUT_SECONDS - TIMEOUT_MARGIN_SECONDS,
    )

    return TimeBudget(
        typical_seconds=_stage_seconds(plan, TYPICAL_SECONDS),
        function_timeout_seconds=internal_deadline + TIMEOUT_MARGIN_SECONDS,
        internal_deadline_seconds=internal_deadline,
    )


def estimate_cost(plan: SimRunPlan) -> CostEstimate:
    usd_per_second = (
        plan.cpu_cores * CPU_CORE_USD_PER_SECOND + plan.memory_gib * MEMORY_GIB_USD_PER_SECOND
    )

    if plan.gpu is not None:
        usd_per_second += GPU_USD_PER_SECOND[plan.gpu]

    budget = time_budget(plan)

    return CostEstimate(
        usd_per_hour=usd_per_second * 3600,
        typical_seconds=budget.typical_seconds,
        typical_usd=usd_per_second * budget.typical_seconds,
        ceiling_seconds=budget.function_timeout_seconds,
        ceiling_usd=usd_per_second * budget.function_timeout_seconds,
    )


def format_estimate(plan: SimRunPlan, estimate: CostEstimate, launching: bool) -> str:
    gpu = plan.gpu or 'none'
    lines = [
        f'Run {plan.run_id}: mode {plan.mode}, nav_source {plan.nav_source}, wind {plan.wind}',
        f'  resources: {plan.cpu_cores:g} cores, {plan.memory_gib:g} GiB, gpu {gpu}',
        f'  rate: ${estimate.usd_per_hour:.3f}/hour (Modal list prices read {PRICES_READ_ON})',
        f'  typical: {estimate.typical_seconds / 60:.0f} min, ${estimate.typical_usd:.2f} '
        '(assumed stage durations, not measured on Modal)',
        f'  ceiling: {estimate.ceiling_seconds / 60:.0f} min, ${estimate.ceiling_usd:.2f} '
        '(the function timeout, per attempt; a preempted call restarts with a fresh one)',
        '  not included: the one-time image build, which the pricing page does not price',
    ]

    if not launching:
        lines.append('Nothing was launched. Add --confirm-spend to run it.')

    return '\n'.join(lines)


def parse_crashes(log_text: str) -> list[Crash]:
    crashes = []

    for match in PROCESS_DIED.finditer(log_text):
        exit_code = int(match.group(2))
        if exit_code not in SIGINT_EXIT_CODES:
            crashes.append(Crash(match.group(1), exit_code))

    return crashes


def parse_real_time_factors(text: str) -> list[float]:
    return [float(value) for value in REAL_TIME_FACTOR.findall(text)]


@dataclass(frozen=True)
class StatsSample:
    """One `/stats` message: cumulative simulated and wall seconds, and Gazebo's one-step factor."""

    sim_seconds: float
    real_seconds: float
    step_real_time_factor: float | None


def _stats_seconds(block: str) -> float:
    # Protobuf text format leaves out a zero field, so a missing sec or nsec means 0.
    seconds = STATS_SECONDS.search(block)
    nanoseconds = STATS_NANOSECONDS.search(block)

    return (int(seconds.group(1)) if seconds else 0) + (
        int(nanoseconds.group(1)) / NANOSECONDS_PER_SECOND if nanoseconds else 0
    )


def parse_stats_samples(text: str) -> list[StatsSample]:
    starts = [match.start() for match in STATS_MESSAGE_START.finditer(text)]
    samples = []

    for start, end in zip(starts, starts[1:] + [len(text)]):
        message = text[start:end]
        sim_time = STATS_SIM_TIME.search(message)
        real_time = STATS_REAL_TIME.search(message)
        if sim_time is None or real_time is None:
            continue

        factor = REAL_TIME_FACTOR.search(message)
        samples.append(
            StatsSample(
                _stats_seconds(sim_time.group(1)),
                _stats_seconds(real_time.group(1)),
                float(factor.group(1)) if factor else None,
            ),
        )

    return samples


def interval_real_time_factors(samples: list[StatsSample]) -> list[float]:
    """Sim seconds gained per wall second between neighbouring samples.

    Both times come from the messages, so the sampler's own delay cannot skew them. A sample
    that timed out is absent, so the interval after it spans the stall.
    """

    return [
        (later.sim_seconds - earlier.sim_seconds) / (later.real_seconds - earlier.real_seconds)
        for earlier, later in zip(samples, samples[1:])
        if later.real_seconds > earlier.real_seconds
    ]


class StatsReading(NamedTuple):
    step_factors: list[float]
    sample: StatsSample | None
    row_fields: dict


def read_stats_output(output: str) -> StatsReading:
    """Decide what one sampler output contributes.

    Only the first message counts as the sample, the one the samples.jsonl row records, so the
    intervals are the gaps between consecutive successful rows. An empty or timed-out output
    gives no sample.
    """

    step_factors = parse_real_time_factors(output)
    samples = parse_stats_samples(output)
    sample = samples[0] if samples else None
    if sample is not None:
        row_step_factor = sample.step_real_time_factor
    else:
        row_step_factor = step_factors[0] if step_factors else None

    return StatsReading(
        step_factors,
        sample,
        {
            'real_time_factor': row_step_factor,
            'sim_seconds': sample.sim_seconds if sample else None,
            'real_seconds': sample.real_seconds if sample else None,
        },
    )


def record_stats_reading(samples: list[StatsSample], reading: StatsReading) -> None:
    if reading.sample is not None:
        samples.append(reading.sample)


def summarize_intervals(samples: list[StatsSample]) -> dict | None:
    factors = interval_real_time_factors(samples)
    if not factors:
        return None

    simulated = samples[-1].sim_seconds - samples[0].sim_seconds
    elapsed = samples[-1].real_seconds - samples[0].real_seconds
    if elapsed <= 0:
        return None

    return {
        'samples': len(factors),
        'mean': round(simulated / elapsed, 3),
        'min': min(factors),
        'max': max(factors),
    }


def memory_available_gib(meminfo_text: str) -> float | None:
    match = MEMORY_AVAILABLE.search(meminfo_text)
    if match is None:
        return None

    return int(match.group(1)) / KIB_PER_GIB


def cgroup_memory_headroom_gib(
    current_text: str,
    max_text: str,
    stat_text: str = '',
) -> float | None:
    """Limit minus usage, with inactive file cache counted as free.

    memory.current includes page cache, which the kernel reclaims before it kills
    anything; a bag being written fills that cache, and counting it as used would stop
    a healthy flight.
    """

    current = current_text.strip()
    limit = max_text.strip()
    if not current.isdigit() or not limit.isdigit():
        return None

    reclaimable = 0

    for line in stat_text.splitlines():
        name, _, value = line.partition(' ')
        if name == 'inactive_file' and value.strip().isdigit():
            reclaimable = int(value)

    return (int(limit) - int(current) + reclaimable) / BYTES_PER_GIB


def bounded_log(text: str, limit_chars: int) -> str:
    if len(text) <= limit_chars:
        return text

    half = limit_chars // 2
    elided = len(text) - 2 * half

    return f'{text[:half]}\n... {elided} characters elided ...\n{text[-half:]}'


def classify_outcome(facts: OutcomeFacts) -> str:
    if facts.stopped_by == 'memory':
        return 'memory_stop'

    if facts.stopped_by == 'deadline':
        return 'deadline_stop'

    if not facts.launch_started:
        return 'failed_before_launch'

    if facts.mode == 'check':
        return 'check_passed' if facts.stack_exit_code == 0 else 'check_failed'

    if facts.stack_exit_code is not None and facts.stopped_by is None:
        return 'stack_exited'

    if not facts.bag_complete:
        return 'no_bag'

    if facts.crash_count:
        return 'process_died'

    return 'ok'


def summarize_factors(values: list[float]) -> dict | None:
    if not values:
        return None

    return {
        'samples': len(values),
        'mean': round(sum(values) / len(values), 3),
        'min': min(values),
        'max': max(values),
    }


def _minutes(seconds: float | None) -> str:
    return 'n/a' if seconds is None else f'{seconds / 60:.1f} min'


def _probe_lines(steps: list[dict]) -> list[str]:
    lines = ['probe steps:']

    for step in steps:
        verdict = 'pass' if step['exit_code'] == 0 else f'FAIL (exit {step["exit_code"]})'
        lines.append(f'  {step["name"]}: {verdict}, {step["seconds"]:.0f} s')

    return lines


def summary_lines(result: dict) -> list[str]:
    timings = result.get('timings', {})
    lines = [
        f'run {result["run_id"]} ({result["mode"]}): {result["status"]}',
        f'  total {_minutes(timings.get("total_seconds"))}, cost ${result.get("cost", {}).get("elapsed_usd", 0):.2f}',
    ]

    if result['mode'] == 'probe':
        return (
            lines
            + ['  ' + line for line in _probe_lines(result.get('probe', []))]
            + [f'  warning: {warning}' for warning in result.get('warnings', [])]
        )

    lines.append(
        f'  build and launch {_minutes(timings.get("launch_seconds"))}, flight {_minutes(timings.get("flight_seconds"))}',
    )
    factors = result.get('real_time_factor')
    step_factors = result.get('step_real_time_factor')

    if factors is None:
        lines.append('  real-time factor: not measured')
    elif 'step_real_time_factor' not in result:
        lines.append(
            '  real-time factor (older run, average of single-step readings): '
            f'mean {factors["mean"]:.2f}, '
            f'min {factors["min"]:.2f}, max {factors["max"]:.2f} over {factors["samples"]} samples',
        )
    else:
        lines.append(
            f'  real-time factor (sim time over wall time): mean {factors["mean"]:.2f}, '
            f'min {factors["min"]:.2f}, max {factors["max"]:.2f} '
            f'over {factors["samples"]} intervals',
        )

    if step_factors is not None:
        lines.append(
            '  per-step real-time factor (single 1 ms steps, not a speed): '
            f'mean {step_factors["mean"]:.2f}, '
            f'min {step_factors["min"]:.2f}, max {step_factors["max"]:.2f} '
            f'over {step_factors["samples"]} readings',
        )

    lowest = result.get('memory', {}).get('lowest_headroom_gib')
    lines.append(
        '  lowest memory headroom: ' + ('not measured' if lowest is None else f'{lowest:.1f} GiB'),
    )
    crashes = result.get('crashes', [])
    if crashes:
        names = ', '.join(f'{crash["name"]} (exit {crash["exit_code"]})' for crash in crashes)
        lines.append(f'  processes that died during the flight: {names}')

    bag = result.get('bag')
    if bag is None:
        lines.append('  bag: none')
    else:
        state = 'complete' if bag['complete'] else 'INCOMPLETE'
        lines.append(
            f'  bag: {bag["name"]} ({state}{", reindexed" if bag.get("reindexed") else ""})',
        )

    lines.append(f'  gate: {result.get("gate") or "not scored"}')

    return lines + [f'  warning: {warning}' for warning in result.get('warnings', [])]
