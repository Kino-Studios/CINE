"""The judge: a paired, preregistered comparison of two arms of scored flights.

Rules (spec "Judge" and rigour rules 1-4):

- Runs pair by condition key; repeats of one condition within one arm collapse to their median.
- A failed run (mission not completed, aborted, or collided) takes each metric's ``worst_value``;
  every metric is also compared with failed runs removed.
- A metric that is unchecked or missing in a run that did not fail is never given the worst value:
  that run is left out of that metric and listed by name.
- A focus or motion metric from a run that did not fail is left out, and listed apart, when the
  channel filmed for less than ``min_filming_s``; with no filming time recorded it counts as
  unchecked. A failed run takes the worst value whatever its filming time.
- Differences are rounded to 12 decimals, so float error cannot make a zero nonzero or break a tie.
  Only nonzero differences count toward the fewest pairs that can reach alpha.
- The noise floor groups repeats by arm and condition, so a gap between arms never widens it.
- Every difference is arm B minus arm A, the arms in the order given; ``higher_is_better`` only
  decides which arm the verdict calls better.
"""

import hashlib
import json
import tomllib
from collections import defaultdict
from collections.abc import Callable
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

from cine_eval.channels.result import OK
from cine_eval.score import RunScore, read_score
from cine_eval.statistics import (
    hodges_lehmann,
    holm_adjust,
    minimum_pairs_for_alpha,
    paired_bootstrap_interval,
    wilcoxon_exact_p,
    wilson_interval,
)

RECORD_FILE = 'record.json'
CONDITION_FIELDS = ('scenario', 'world_file', 'waypoints_sha1', 'wind', 'seed')
UNPAIRED = 'unpaired'
SUPPORTED_TEST = 'wilcoxon_exact'
VIEW_WORST = 'failures as worst value'
VIEW_REMOVED = 'failed pairs removed'
VIEWS = (VIEW_WORST, VIEW_REMOVED)
PREREGISTERED = 'preregistered'
EXPLORATORY = 'exploratory'
BOOTSTRAP_RESAMPLES = 10000
BOOTSTRAP_SEED = 0
BOOTSTRAP_LEVEL = 0.95
FILMING_CHANNELS = ('focus', 'motion')
FILMING_MISSING = 'filming time missing'
FILMING_SHORT = 'filming time short'
DIFFERENCE_DECIMALS = 12


@dataclass(frozen=True)
class MetricSpec:
    metric: str
    higher_is_better: bool
    worst_value: float


@dataclass(frozen=True)
class AnalysisPlan:
    primary: MetricSpec
    secondary: tuple[MetricSpec, ...]
    alpha: float
    test: str
    min_filming_s: float
    digest: str


@dataclass(frozen=True)
class Rate:
    successes: int
    trials: int
    interval: tuple[float, float] | None
    unchecked_runs: tuple[str, ...]


@dataclass(frozen=True)
class ArmSummary:
    """One arm's per-run values of one metric in one view, before repeats are collapsed."""

    arm: str
    run_values: dict[str, float]
    mean: float | None
    median: float | None
    standard_deviation: float | None
    minimum: float | None
    maximum: float | None


@dataclass(frozen=True)
class MetricComparison:
    metric: str
    higher_is_better: bool
    view: str
    pairs: int
    nonzero_pairs: int
    differences: dict[str, float]
    excluded_runs: tuple[str, ...]
    short_filming_runs: tuple[str, ...]
    removed_failed_runs: tuple[str, ...]
    hodges_lehmann: float | None
    interval: tuple[float, float] | None
    p_value: float | None
    p_adjusted: float | None
    noise_floor: float | None
    resolved: bool | None
    label: str
    arms: tuple[ArmSummary, ...]


@dataclass(frozen=True)
class Comparison:
    arm_a: str
    arm_b: str
    mission_success: dict[str, Rate]
    collision_free: dict[str, Rate]
    collision_partially_checked: dict[str, tuple[str, ...]]
    collided_only: dict[str, int]
    failed_runs: dict[str, dict[str, str]]
    pairs: int
    unpaired_runs: dict[str, tuple[str, ...]]
    repeats: dict[str, dict[str, int]]
    primary: tuple[MetricComparison, ...]
    secondary: tuple[MetricComparison, ...]
    alpha: float
    test: str
    minimum_pairs: int
    min_filming_s: float
    bootstrap_resamples: int
    bootstrap_seed: int
    plan_digest: str
    committed_digest: str | None
    plan_status: str

    def to_json(self) -> dict:
        return asdict(self)

    def to_markdown(self) -> str:
        return _markdown(self)


@dataclass(frozen=True)
class _Run:
    name: str
    condition: tuple
    score: RunScore
    failure_reasons: tuple[str, ...]

    @property
    def failed(self) -> bool:
        return bool(self.failure_reasons)

    def summary(self, channel_name: str) -> dict[str, float]:
        return _checked_summary(self.score, channel_name)


def load_plan(path: Path) -> AnalysisPlan:
    content = Path(path).read_bytes()
    document = tomllib.loads(content.decode('utf-8'))
    if document['test'] != SUPPORTED_TEST:
        raise ValueError(
            f'analysis plan names test {document["test"]!r}; only {SUPPORTED_TEST!r} exists',
        )

    return AnalysisPlan(
        primary=_metric_spec(document['primary']),
        secondary=tuple(_metric_spec(entry) for entry in document.get('secondary', [])),
        alpha=float(document['alpha']),
        test=document['test'],
        min_filming_s=float(document['min_filming_s']),
        digest=hashlib.sha1(content).hexdigest(),
    )


def _metric_spec(entry: dict) -> MetricSpec:
    return MetricSpec(
        metric=entry['metric'],
        higher_is_better=bool(entry['higher_is_better']),
        worst_value=float(entry['worst_value']),
    )


def condition_key(run_metadata: dict | None, run_name: str) -> tuple:
    """``(scenario, world_file, waypoints_sha1, wind, seed)`` from the run's ``run.json`` contents.

    A run without ``run.json``, or whose ``run.json`` is not an object or lacks any of those
    fields, gets ``("unpaired", run_name)``, which ``compare`` never pairs: guessing a missing field
    could pair different conditions. Each field is tagged with its JSON type, so values of
    different types never pair (seed true and seed "true", wind null and wind "null"). Numbers
    become floats, so wind 0 pairs with wind 0.0.
    """

    if not isinstance(run_metadata, dict) or any(
        field not in run_metadata for field in CONDITION_FIELDS
    ):
        return (UNPAIRED, run_name)

    return tuple(_canonical(run_metadata[field]) for field in CONDITION_FIELDS)


def _is_unpaired(condition: tuple) -> bool:
    return len(condition) == 2 and condition[0] == UNPAIRED


def _canonical(value: object) -> tuple[str, str | float | bool | None]:
    """``(json_type, value)``; lists and objects are held as JSON text so the key is hashable."""

    if value is None:
        return ('null', None)

    if isinstance(value, bool):
        return ('bool', value)

    if isinstance(value, (int, float)):
        return ('number', float(value))

    if isinstance(value, str):
        return ('string', value)

    json_type = 'list' if isinstance(value, list) else 'object'

    return (json_type, json.dumps(value, sort_keys=True))


def _plain(field: object) -> object:
    """A condition field as its JSON value, for writing the condition out."""

    if not isinstance(field, tuple):
        return field

    json_type, value = field

    return json.loads(value) if json_type in ('list', 'object') else value


def _condition_id(condition: tuple) -> str:
    """The condition as JSON: unlike a joined label, two different conditions never share it.

    JSON keeps the type of each value (``true``, ``"true"``, ``null``, ``"null"`` differ).
    """

    return json.dumps([_plain(field) for field in condition])


def _condition_label(condition_id: str) -> str:
    """A readable label for display only; it can coincide for different conditions."""

    return '|'.join(str(field) for field in json.loads(condition_id))


def compare(
    arms: dict[str, list[Path]],
    plan: AnalysisPlan,
    repeatability: dict[str, list[Path]] | None = None,
    plan_path_committed_digest: str | None = None,
) -> Comparison:
    """Compare arm B against arm A, the arms in the order ``arms`` gives them.

    ``repeatability`` maps an arm name to repeat flights of that arm; it sets each metric's
    noise floor. None means no floor is reported.
    """

    if len(arms) != 2:
        raise ValueError(f'compare needs exactly two arms, got {len(arms)}')

    unknown_arms = sorted(set(repeatability or {}) - set(arms))
    if unknown_arms:
        raise ValueError(f'repeatability names arms that are not compared: {unknown_arms}')

    arm_a, arm_b = arms
    runs = {
        arm: [_load_run(directory) for directory in directories]
        for arm, directories in arms.items()
    }
    repeat_runs = {
        arm: [_load_run(directory) for directory in directories]
        for arm, directories in (repeatability or {}).items()
    }
    conditions = {arm: {run.condition for run in arm_runs} for arm, arm_runs in runs.items()}
    common = sorted(
        (
            condition
            for condition in conditions[arm_a] & conditions[arm_b]
            if not _is_unpaired(condition)
        ),
        key=_condition_id,
    )
    minimum_pairs = minimum_pairs_for_alpha(plan.alpha)
    noise_floors = {
        spec.metric: (
            _noise_floor(repeat_runs, spec, plan.min_filming_s)
            if repeatability is not None
            else None
        )
        for spec in (plan.primary, *plan.secondary)
    }

    def compare_metric(spec: MetricSpec, view: str) -> MetricComparison:
        return _compare_metric(
            runs,
            (arm_a, arm_b),
            common,
            spec,
            view,
            noise_floors[spec.metric],
            minimum_pairs,
            plan.min_filming_s,
        )

    primary = tuple(
        _with_label(compare_metric(plan.primary, view), plan, arm_a, arm_b, adjusted=False)
        for view in VIEWS
    )
    secondary = []

    for view in VIEWS:
        unlabelled = [compare_metric(spec, view) for spec in plan.secondary]
        secondary.extend(
            _with_label(comparison, plan, arm_a, arm_b, adjusted=True)
            for comparison in _holm(unlabelled)
        )

    return Comparison(
        arm_a=arm_a,
        arm_b=arm_b,
        mission_success={
            arm: _rate(arm_runs, 'mission', _mission_success) for arm, arm_runs in runs.items()
        },
        collision_free={
            arm: _rate(arm_runs, 'clearance', _collision_free) for arm, arm_runs in runs.items()
        },
        collision_partially_checked={
            arm: _partially_checked(arm_runs) for arm, arm_runs in runs.items()
        },
        collided_only=_collided_only(runs, arm_a, arm_b, common),
        failed_runs={
            arm: {run.name: ', '.join(run.failure_reasons) for run in arm_runs if run.failed}
            for arm, arm_runs in runs.items()
        },
        pairs=len(common),
        unpaired_runs={
            arm: tuple(run.name for run in arm_runs if run.condition not in common)
            for arm, arm_runs in runs.items()
        },
        repeats={arm: _repeats(arm_runs) for arm, arm_runs in runs.items()},
        primary=primary,
        secondary=tuple(secondary),
        alpha=plan.alpha,
        test=plan.test,
        minimum_pairs=minimum_pairs,
        min_filming_s=plan.min_filming_s,
        bootstrap_resamples=BOOTSTRAP_RESAMPLES,
        bootstrap_seed=BOOTSTRAP_SEED,
        plan_digest=plan.digest,
        committed_digest=plan_path_committed_digest,
        plan_status=PREREGISTERED if plan_path_committed_digest == plan.digest else EXPLORATORY,
    )


def _load_run(directory: Path) -> _Run:
    score = read_score(directory)
    record_path = Path(directory) / RECORD_FILE
    run_metadata = (
        _run_metadata(json.loads(record_path.read_text())) if record_path.exists() else None
    )

    return _Run(
        name=score.run_name,
        condition=condition_key(run_metadata, score.run_name),
        score=score,
        failure_reasons=failure_reasons(score),
    )


def _run_metadata(record: object) -> dict | None:
    """``record["metadata"]["run"]``, or None when any level is missing or not an object."""

    metadata = record.get('metadata') if isinstance(record, dict) else None
    run_metadata = metadata.get('run') if isinstance(metadata, dict) else None

    return run_metadata if isinstance(run_metadata, dict) else None


def _checked_summary(score: RunScore, channel_name: str) -> dict[str, float]:
    """The channel's summary, or an empty dict when the channel is absent or unchecked."""

    channel = score.channels.get(channel_name)

    return channel.summary if channel is not None and channel.status == OK else {}


def failure_reasons(score: RunScore) -> tuple[str, ...]:
    """Why the run counts as failed: mission not completed, aborted, or collided."""

    mission = _checked_summary(score, 'mission')
    clearance = _checked_summary(score, 'clearance')
    reasons = []

    if mission.get('completed') == 0.0:
        reasons.append('mission not completed')

    if mission.get('aborted') == 1.0:
        reasons.append('aborted')

    if clearance.get('collision_free') == 0.0:
        reasons.append('collided')

    return tuple(reasons)


def _mission_success(summary: dict[str, float]) -> bool | None:
    if summary.get('aborted') == 1.0:
        return False

    if 'completed' not in summary or 'aborted' not in summary:
        return None

    return summary['completed'] == 1.0 and summary['aborted'] == 0.0


def _collision_free(summary: dict[str, float]) -> bool | None:
    """True or False only when clearance checked every contact class over the whole flight.

    A missing or non-finite ``collision_free`` means some class went unchecked: unknown.
    """

    value = summary.get('collision_free')
    if value is None or not np.isfinite(value):
        return None

    return value == 1.0


def _partially_checked(runs: list[_Run]) -> tuple[str, ...]:
    """Runs whose clearance channel ran but could not vouch for every contact class."""

    return tuple(
        run.name
        for run in runs
        if run.score.channels.get('clearance') is not None
        and run.score.channels['clearance'].status == OK
        and _collision_free(run.summary('clearance')) is None
    )


def _rate(
    runs: list[_Run],
    channel_name: str,
    outcome: Callable[[dict[str, float]], bool | None],
) -> Rate:
    """Share of checked runs that succeeded; unchecked runs are named, never counted."""

    outcomes = {run.name: outcome(run.summary(channel_name)) for run in runs}
    checked = [succeeded for succeeded in outcomes.values() if succeeded is not None]
    successes = sum(checked)

    return Rate(
        successes=successes,
        trials=len(checked),
        interval=wilson_interval(successes, len(checked)) if checked else None,
        unchecked_runs=tuple(name for name, succeeded in outcomes.items() if succeeded is None),
    )


def _collided_only(
    runs: dict[str, list[_Run]],
    arm_a: str,
    arm_b: str,
    common: list[tuple],
) -> dict[str, int]:
    """Paired conditions where one arm collided (any repeat) and the other was checked, clean."""

    def collision_state(arm: str, condition: tuple) -> str | None:
        outcomes = [
            _collision_free(run.summary('clearance'))
            for run in runs[arm]
            if run.condition == condition
        ]
        checked = [outcome for outcome in outcomes if outcome is not None]
        if not checked:
            return None

        return 'clean' if all(checked) else 'collided'

    states = [
        (collision_state(arm_a, condition), collision_state(arm_b, condition))
        for condition in common
    ]

    return {
        arm_a: states.count(('collided', 'clean')),
        arm_b: states.count(('clean', 'collided')),
    }


def _repeats(runs: list[_Run]) -> dict[str, int]:
    counts: dict[str, int] = defaultdict(int)

    for run in runs:
        counts[_condition_id(run.condition)] += 1

    return {condition_id: count for condition_id, count in counts.items() if count > 1}


def filming_problem(score: RunScore, metric: str, min_filming_s: float) -> str | None:
    """For a focus or motion metric: ``FILMING_MISSING`` or ``FILMING_SHORT``, else None.

    A missing filming time is not a short one: the metric is then treated as unchecked.
    """

    channel_name = metric.partition('.')[0]
    if channel_name not in FILMING_CHANNELS:
        return None

    filming_s = _checked_summary(score, channel_name).get('filming_s')
    if filming_s is None:
        return FILMING_MISSING

    return FILMING_SHORT if filming_s < min_filming_s else None


def run_value(
    score: RunScore,
    metric: str,
    min_filming_s: float,
    worst_value: float | None,
) -> float | None:
    """One run's value of ``metric`` under the judge's rules; None leaves the run out.

    A failed run takes ``worst_value`` (None leaves failed runs out); a focus or motion
    metric filmed for less than ``min_filming_s``, or with no filming time, is left out.
    """

    if failure_reasons(score):
        return worst_value

    if filming_problem(score, metric, min_filming_s) is not None:
        return None

    channel_name, _, key = metric.partition('.')

    return _checked_summary(score, channel_name).get(key)


def _filming_problem(run: _Run, spec: MetricSpec, min_filming_s: float) -> str | None:
    return filming_problem(run.score, spec.metric, min_filming_s)


def _view_value(run: _Run, spec: MetricSpec, view: str, min_filming_s: float) -> float | None:
    worst_value = spec.worst_value if view == VIEW_WORST else None

    return run_value(run.score, spec.metric, min_filming_s, worst_value)


def _compare_metric(
    runs: dict[str, list[_Run]],
    arm_names: tuple[str, str],
    common: list[tuple],
    spec: MetricSpec,
    view: str,
    noise_floor: float | None,
    minimum_pairs: int,
    min_filming_s: float,
) -> MetricComparison:
    values: dict[str, dict[tuple, list[float]]] = {arm: defaultdict(list) for arm in arm_names}
    excluded_runs = []
    short_filming_runs = []
    removed_failed_runs = []

    for arm in arm_names:
        for run in runs[arm]:
            value = _view_value(run, spec, view, min_filming_s)
            if value is not None:
                values[arm][run.condition].append(value)
            elif run.condition not in common:
                continue
            elif run.failed:
                removed_failed_runs.append(run.name)
            elif _filming_problem(run, spec, min_filming_s) == FILMING_SHORT:
                short_filming_runs.append(run.name)
            else:
                excluded_runs.append(run.name)

    arm_a, arm_b = arm_names
    paired = [
        condition for condition in common if values[arm_a][condition] and values[arm_b][condition]
    ]
    differences = {
        _condition_id(condition): float(
            np.round(
                np.median(values[arm_b][condition]) - np.median(values[arm_a][condition]),
                DIFFERENCE_DECIMALS,
            ),
        )
        for condition in paired
    }
    difference_array = np.array(list(differences.values()))
    has_pairs = difference_array.size > 0
    estimate = hodges_lehmann(difference_array) if has_pairs else None
    nonzero_pairs = int(np.count_nonzero(difference_array))
    testable = nonzero_pairs >= minimum_pairs

    return MetricComparison(
        metric=spec.metric,
        higher_is_better=spec.higher_is_better,
        view=view,
        pairs=len(paired),
        nonzero_pairs=nonzero_pairs,
        differences=differences,
        excluded_runs=tuple(excluded_runs),
        short_filming_runs=tuple(short_filming_runs),
        removed_failed_runs=tuple(removed_failed_runs),
        hodges_lehmann=estimate,
        interval=(
            paired_bootstrap_interval(
                difference_array,
                resamples=BOOTSTRAP_RESAMPLES,
                seed=BOOTSTRAP_SEED,
                level=BOOTSTRAP_LEVEL,
            )
            if has_pairs
            else None
        ),
        p_value=wilcoxon_exact_p(difference_array) if testable else None,
        p_adjusted=None,
        noise_floor=noise_floor,
        resolved=None if noise_floor is None or estimate is None else abs(estimate) >= noise_floor,
        label='',
        arms=tuple(_arm_summary(arm, runs[arm], spec, view, min_filming_s) for arm in arm_names),
    )


def _arm_summary(
    arm: str,
    runs: list[_Run],
    spec: MetricSpec,
    view: str,
    min_filming_s: float,
) -> ArmSummary:
    run_values = {
        run.name: value
        for run in runs
        if (value := _view_value(run, spec, view, min_filming_s)) is not None
    }
    array = np.array(list(run_values.values()))
    if array.size == 0:
        return ArmSummary(arm, run_values, None, None, None, None, None)

    return ArmSummary(
        arm=arm,
        run_values=run_values,
        mean=float(array.mean()),
        median=float(np.median(array)),
        standard_deviation=float(array.std(ddof=1)) if array.size > 1 else None,
        minimum=float(array.min()),
        maximum=float(array.max()),
    )


def _noise_floor(
    runs: dict[str, list[_Run]],
    spec: MetricSpec,
    min_filming_s: float,
) -> float | None:
    """Pooled median absolute deviation from each (arm, condition) group's median.

    Grouping by arm keeps the gap between arms out of the floor. Failed runs, runs missing the
    metric and runs filmed for under ``min_filming_s`` are left out, as in the comparison with
    failed pairs removed. Groups with a single run carry no spread and are skipped.
    """

    groups: dict[tuple[str, tuple], list[float]] = defaultdict(list)

    for arm, arm_runs in runs.items():
        for run in arm_runs:
            value = _view_value(run, spec, VIEW_REMOVED, min_filming_s)
            if value is not None:
                groups[(arm, run.condition)].append(value)

    deviations = [
        abs(value - np.median(group))
        for group in groups.values()
        if len(group) > 1
        for value in group
    ]

    return float(np.median(deviations)) if deviations else None


def _holm(comparisons: list[MetricComparison]) -> list[MetricComparison]:
    """Holm-adjust across the metrics that produced a p value; the others had no test to adjust."""

    tested = [
        index for index, comparison in enumerate(comparisons) if comparison.p_value is not None
    ]
    adjusted = holm_adjust([comparisons[index].p_value for index in tested])
    result = list(comparisons)

    for index, p_adjusted in zip(tested, adjusted):
        result[index] = replace(result[index], p_adjusted=p_adjusted)

    return result


def _with_label(
    comparison: MetricComparison,
    plan: AnalysisPlan,
    arm_a: str,
    arm_b: str,
    adjusted: bool,
) -> MetricComparison:
    p_for_decision = comparison.p_adjusted if adjusted else comparison.p_value

    label = _verdict(comparison, p_for_decision, plan.alpha, arm_a, arm_b)

    return replace(comparison, label=label)


def _verdict(
    comparison: MetricComparison,
    p_for_decision: float | None,
    alpha: float,
    arm_a: str,
    arm_b: str,
) -> str:
    if comparison.pairs == 0:
        return 'no pairs'

    if comparison.p_value is None:
        return (
            f'insufficient nonzero pairs (n={comparison.nonzero_pairs} of {comparison.pairs}) '
            f'for α={alpha:g}'
        )

    if comparison.resolved is False:
        return 'not resolved'

    if p_for_decision > alpha:
        return 'no significant difference'

    if comparison.hodges_lehmann == 0:
        return 'significant, but the median difference is zero'

    b_is_better = (comparison.hodges_lehmann > 0) == comparison.higher_is_better

    return f'{arm_b if b_is_better else arm_a} better'


def _number(value: float | None) -> str:
    if value is None:
        return '—'

    # Adding 0.0 turns -0.0 into 0.0, so a zero never prints as "-0".

    return f'{value + 0.0:.4g}'


def _interval_text(interval: tuple[float, float] | None) -> str:
    return '—' if interval is None else f'[{_number(interval[0])}, {_number(interval[1])}]'


def _names(names: tuple[str, ...] | list[str]) -> str:
    return ', '.join(names) if names else 'none'


def _rate_text(rate: Rate) -> str:
    if rate.trials == 0:
        return 'no checked runs'

    share = rate.successes / rate.trials

    return (
        f'{rate.successes}/{rate.trials} ({share:.0%}), 95% Wilson {_interval_text(rate.interval)}'
    )


def _markdown(comparison: Comparison) -> str:
    arm_a, arm_b = comparison.arm_a, comparison.arm_b
    lines = [
        f'# Comparison: {arm_b} against {arm_a}',
        '',
        f'Arm A = {arm_a}, arm B = {arm_b}; every difference = B − A = {arm_b} − {arm_a}.',
        '',
        '## Mission success and collision-free rates',
        '',
        'Rates count every run (repeats are not collapsed). An unchecked run is named and left',
        'out of the denominator; it is never counted as a success.',
        '',
    ]

    for arm in (arm_a, arm_b):
        mission = comparison.mission_success[arm]
        clearance = comparison.collision_free[arm]
        partial = comparison.collision_partially_checked[arm]
        lines += [
            f'- {arm}: mission success {_rate_text(mission)}; mission unchecked: '
            f'{len(mission.unchecked_runs)} ({_names(mission.unchecked_runs)})',
            f'- {arm}: collision-free {_rate_text(clearance)} over fully checked runs; '
            f'collision freedom unchecked: '
            f'{len(clearance.unchecked_runs)} ({_names(clearance.unchecked_runs)}), of which '
            f'partially checked: {len(partial)} ({_names(partial)})',
        ]

    lines += [
        f'- Paired conditions where only {arm_a} collided: {comparison.collided_only[arm_a]}; '
        f'only {arm_b} collided: {comparison.collided_only[arm_b]}',
    ]

    for arm in (arm_a, arm_b):
        failed = comparison.failed_runs[arm]
        failed_text = ', '.join(f'{name} ({reasons})' for name, reasons in failed.items()) or 'none'
        lines.append(
            f"- Failed runs in {arm} (take each metric's worst value): {len(failed)}: {failed_text}",
        )

    lines += ['', '## Pairs', '', f'- Paired conditions: {comparison.pairs}']

    for arm in (arm_a, arm_b):
        unpaired = comparison.unpaired_runs[arm]
        lines.append(f'- Unpaired runs in {arm}: {len(unpaired)}: {_names(unpaired)}')

    for arm in (arm_a, arm_b):
        repeats = comparison.repeats[arm]
        repeats_text = (
            '; '.join(
                f'{_condition_label(condition_id)} ×{count}'
                for condition_id, count in repeats.items()
            )
            or 'none'
        )
        lines.append(f'- Repeats collapsed to their median in {arm}: {repeats_text}')

    lines += ['', '## Primary metric (unadjusted p)', '']
    lines += _metric_lines(comparison.primary, comparison.min_filming_s, adjusted=False)
    lines += ['', '## Secondary metrics (Holm-adjusted p)', '']

    for view in VIEWS:
        tested = [
            entry
            for entry in comparison.secondary
            if entry.view == view and entry.p_value is not None
        ]
        lines.append(f'- {view}: Holm across m={len(tested)} tested metrics')

    lines.append('')
    lines += _metric_lines(comparison.secondary, comparison.min_filming_s, adjusted=True)
    committed = (
        comparison.committed_digest if comparison.committed_digest is not None else 'not given'
    )
    lines += [
        '',
        '## Analysis plan',
        '',
        f'- Status: {comparison.plan_status}',
        f'- Plan digest (SHA-1): {comparison.plan_digest}; committed plan digest: {committed}',
        f'- Test: {comparison.test} (two-sided), α={comparison.alpha:g}; '
        f'fewest pairs that can reach α: {comparison.minimum_pairs}',
        '- Effect size: Hodges–Lehmann median difference; 95% percentile interval from '
        f'{comparison.bootstrap_resamples} paired bootstrap resamples, '
        f'seed {comparison.bootstrap_seed}',
        f'- min_filming_s = {comparison.min_filming_s:g}: a run that did not fail and whose '
        'focus or motion channel filmed for less than this many seconds is left out of that '
        "channel's metrics and listed by name; a run with no recorded filming time counts "
        'as unchecked; a failed run takes the worst value whatever its filming time',
        '- The test runs only when at least that many pairs differ from zero (differences '
        'rounded to 12 decimals); otherwise no p value and no verdict are given, and the '
        'metric is left out of the Holm family',
        '- The noise floor is the median absolute deviation of repeat flights from the median '
        'of their arm and condition. The noise floor leaves out failed runs, runs filmed '
        'under min_filming_s and runs missing the metric',
        '',
    ]

    return '\n'.join(lines)


def _metric_lines(
    metrics: tuple[MetricComparison, ...],
    min_filming_s: float,
    adjusted: bool,
) -> list[str]:
    lines = []

    for metric in dict.fromkeys(entry.metric for entry in metrics):
        entries = [entry for entry in metrics if entry.metric == metric]
        direction = 'higher is better' if entries[0].higher_is_better else 'lower is better'
        p_header = 'p (raw) | p (Holm)' if adjusted else 'p'
        lines += [
            f'### {metric} ({direction})',
            '',
            '| view | pairs | nonzero pairs | B − A (Hodges–Lehmann) | '
            f'95% bootstrap interval | {p_header} | noise floor | verdict |',
            '|---|---|---|---|---|---|---|---|' + ('---|' if adjusted else ''),
        ]

        for entry in entries:
            p_text = (
                f'{_number(entry.p_value)} | {_number(entry.p_adjusted)}'
                if adjusted
                else _number(entry.p_value)
            )
            lines.append(
                f'| {entry.view} | {entry.pairs} | {entry.nonzero_pairs} | '
                f'{_number(entry.hodges_lehmann)} | '
                f'{_interval_text(entry.interval)} | {p_text} | {_number(entry.noise_floor)} | '
                f'{entry.label} |',
            )

        lines.append('')

        for entry in entries:
            lines.append(
                f'- {entry.view}: excluded (metric unchecked or missing): '
                f'{len(entry.excluded_runs)} ({_names(entry.excluded_runs)}); '
                f'filmed under min_filming_s={min_filming_s:g}: '
                f'{len(entry.short_filming_runs)} ({_names(entry.short_filming_runs)}); '
                f'failed runs removed: {len(entry.removed_failed_runs)} '
                f'({_names(entry.removed_failed_runs)})',
            )

            for summary in entry.arms:
                run_text = ', '.join(
                    f'{name}={_number(value)}' for name, value in summary.run_values.items()
                )
                lines.append(
                    f'  - {summary.arm}: n={len(summary.run_values)}, '
                    f'mean {_number(summary.mean)}, median {_number(summary.median)}, '
                    f'sd {_number(summary.standard_deviation)}, '
                    f'range [{_number(summary.minimum)}, {_number(summary.maximum)}]; '
                    f'runs: {run_text or "none"}',
                )

        lines.append('')

    return lines
