"""Standings: which runs make up each results row, with provenance, written out as a table."""

import csv
import json
import re
import tomllib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from cine_eval.analysis import failure_reasons, load_plan, run_value
from cine_eval.channels.result import OK
from cine_eval.record import RECORD_ARRAYS_FILE, RECORD_INDEX_FILE, RecordError
from cine_eval.score import (
    SCORE_FILE,
    RunScore,
    current_scorer_provenance,
    read_score,
    score_run,
    scorer_is_clean,
    write_score,
)
from cine_eval.settings import load_scoring_settings

BEGIN_MARKER = '<!-- cine-eval:begin -->'
END_MARKER = '<!-- cine-eval:end -->'
UNKNOWN = 'unknown'
ORACLE = 'oracle'
CSV_COLUMNS = (
    'row_id',
    'arm',
    'condition',
    'metric',
    'n',
    'failed_runs',
    'median',
    'mean',
    'std',
    'min',
    'max',
    'comparable',
    'perception',
    'note',
)


@dataclass(frozen=True)
class Metric:
    channel: str
    key: str
    heading: str
    scale: float = 1.0
    unit: str = ''

    @property
    def name(self) -> str:
        return f'{self.channel}.{self.key}'


METRICS = (
    Metric('motion', 'peak_speed_mps', 'Peak speed while filming', unit=' m/s'),
    Metric('focus', 'held_share', 'Time within focus tolerance', scale=100.0, unit='%'),
    Metric('focus', 'worst_deviation_m', 'Largest focus deviation', unit=' m'),
)


@dataclass(frozen=True)
class RunFacts:
    name: str
    metadata: dict
    summaries: dict[str, dict[str, float]]
    provenance: dict
    score: RunScore

    @property
    def failed(self) -> bool:
        return bool(failure_reasons(self.score))

    @property
    def run(self) -> dict | None:
        return self.metadata.get('run')

    @property
    def legacy(self) -> bool:
        return bool(self.metadata.get('legacy'))

    @property
    def dirty(self) -> bool:
        checkouts = (self.run or {}).get('git', {}).values()

        return any(entry.get('dirty') is True for entry in checkouts)

    @property
    def dirty_flag_unknown(self) -> bool:
        """True when run.json exists but does not say, as a boolean, whether the CINE tree was clean."""

        if self.run is None:
            return False

        checkout = self.run.get('git', {}).get('CINE') or {}

        return not isinstance(checkout.get('dirty'), bool)

    @property
    def flight_commit(self) -> str:
        return ((self.run or {}).get('git', {}).get('CINE') or {}).get('commit', UNKNOWN)

    @property
    def perception(self) -> str:
        return (self.run or {}).get('perception_source', UNKNOWN)


def replace_generated_section(document: str, table_markdown: str) -> str:
    """Swaps the text between the generated-section markers; everything else is kept."""

    for marker in (BEGIN_MARKER, END_MARKER):
        if document.count(marker) != 1:
            raise ValueError(f'document needs the marker {marker} exactly once')

    begin = document.find(BEGIN_MARKER)
    end = document.find(END_MARKER)
    if end < begin:
        raise ValueError('generated-section markers are in the wrong order: end before begin')

    return (
        f'{document[: begin + len(BEGIN_MARKER)]}\n{table_markdown.strip(chr(10))}\n'
        f'{document[end:]}'
    )


def build_standings(
    manifest_path: Path,
    runs_root: Path,
    outputs_root: Path,
    results_doc: Path,
    plan_path: Path,
    command: str,
) -> None:
    """Scores stale runs, then writes results.csv, manifest.json and the generated table.

    Each run's value follows the judge's rules in ``plan_path``: a failed run takes the
    plan's worst value, and a focus or motion metric needs ``min_filming_s`` of filming.
    """

    rows = tomllib.loads(Path(manifest_path).read_text(encoding='utf-8')).get('row', [])
    plan = load_plan(Path(plan_path))
    worst_values = {spec.metric: spec.worst_value for spec in (plan.primary, *plan.secondary)}
    current_settings = load_scoring_settings().as_dict()
    current_scorer = current_scorer_provenance()
    runs_root = Path(runs_root)
    outputs_root = Path(outputs_root)
    csv_lines: list[dict] = []
    manifest_rows: list[dict] = []
    table_rows: list[dict] = []

    for row in rows:
        runs = [
            _load_facts(runs_root, run_name, current_settings, current_scorer)
            for run_name in row['runs']
        ]
        comparable, note = _comparability(row, runs)
        row_id = _row_id(row['arm'], row['condition'])
        perception = _perception_label(runs)
        failed_runs = sum(run.failed for run in runs)
        statistics = {
            metric: _statistics(
                runs,
                metric,
                worst_values.get(metric.name),
                plan.min_filming_s,
            )
            for metric in METRICS
        }
        for metric, stats in statistics.items():
            csv_lines.append(
                _csv_line(row_id, row, metric, stats, failed_runs, comparable, perception, note),
            )

        manifest_rows.append(_manifest_row(row_id, row, runs, comparable, perception, command))
        table_rows.append(
            {
                'row': row,
                'runs': len(runs),
                'failed_runs': failed_runs,
                'statistics': statistics,
                'comparable': comparable,
                'oracle': any(run.perception == ORACLE for run in runs),
                'note': note,
            },
        )

    updated_doc = replace_generated_section(
        Path(results_doc).read_text(encoding='utf-8'),
        _table_markdown(table_rows, command, worst_values, plan.min_filming_s),
    )
    outputs_root.mkdir(parents=True, exist_ok=True)

    with (outputs_root / 'results.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS, lineterminator='\n')
        writer.writeheader()
        writer.writerows(csv_lines)

    manifest = {'command': command, 'rows': manifest_rows}

    with (outputs_root / 'manifest.json').open('w', newline='\n', encoding='utf-8') as handle:
        handle.write(json.dumps(manifest, allow_nan=False, indent=2) + '\n')

    with Path(results_doc).open('w', newline='\n', encoding='utf-8') as handle:
        handle.write(updated_doc)


def _load_facts(
    runs_root: Path,
    run_name: str,
    current_settings: dict,
    current_scorer: dict[str, str],
) -> RunFacts:
    directory = runs_root / run_name
    if not (directory / RECORD_INDEX_FILE).exists():
        raise ValueError(f'run {run_name!r} has no {RECORD_INDEX_FILE} under {runs_root}')

    arrays_file = directory / RECORD_ARRAYS_FILE
    if not arrays_file.exists():
        raise RecordError(f'run {run_name!r} has {RECORD_INDEX_FILE} but no {RECORD_ARRAYS_FILE}')

    if _score_is_stale(directory, current_settings, current_scorer):
        write_score(score_run(directory), directory)

    score = read_score(directory)
    index = json.loads((directory / RECORD_INDEX_FILE).read_text(encoding='utf-8'))
    summaries = {
        name: result.summary for name, result in score.channels.items() if result.status == OK
    }

    return RunFacts(
        name=run_name,
        metadata=index['metadata'],
        summaries=summaries,
        provenance=score.provenance,
        score=score,
    )


def _score_is_stale(
    directory: Path,
    current_settings: dict,
    current_scorer: dict[str, str],
) -> bool:
    """True when score.json may not be what the current scorer would write for this record.

    A dirty or unknown checkout on either side might score differently under the same
    commit, so it always rescores.
    """

    score_file = directory / SCORE_FILE
    if not score_file.exists():
        return True

    if score_file.stat().st_mtime < (directory / RECORD_ARRAYS_FILE).stat().st_mtime:
        return True

    stored = read_score(directory)
    stored_commit = stored.provenance.get('scorer_commit', UNKNOWN)
    stored_dirty = stored.provenance.get('scorer_dirty', UNKNOWN)

    return (
        stored.settings != current_settings
        or stored_commit != current_scorer['scorer_commit']
        or not scorer_is_clean(stored_dirty)
        or not scorer_is_clean(current_scorer['scorer_dirty'])
    )


def _comparability(row: dict, runs: list[RunFacts]) -> tuple[bool, str]:
    reasons = []

    if not runs:
        reasons.append('the row lists no runs')

    if any(run.dirty_flag_unknown for run in runs):
        reasons.append('a run.json has a missing or unknown dirty flag, so it counts as dirty')

    if any(run.dirty for run in runs):
        reasons.append('a flight ran on a working tree with uncommitted changes')

    if any(run.run is None for run in runs):
        reasons.append('a run has no run.json')

    if any(run.legacy for run in runs):
        reasons.append('a run is legacy data')

    note = row.get('note', '')

    if not reasons:
        return bool(row['comparable']), note

    forced = f'forced not comparable: {"; ".join(reasons)}'

    return False, f'{note}; {forced}' if note else forced


def _statistics(
    runs: list[RunFacts],
    metric: Metric,
    worst_value: float | None,
    min_filming_s: float,
) -> dict[str, float] | None:
    """Statistics over the runs' values; ``worst_value`` None leaves failed runs out."""

    values = [
        value
        for run in runs
        if (value := run_value(run.score, metric.name, min_filming_s, worst_value)) is not None
    ]
    if not values:
        return None

    array = np.array(values, dtype=float)

    return {
        'n': float(len(array)),
        'median': float(np.median(array)),
        'mean': float(array.mean()),
        'std': float(array.std(ddof=1)) if len(array) > 1 else 0.0,
        'min': float(array.min()),
        'max': float(array.max()),
    }


def _csv_line(
    row_id: str,
    row: dict,
    metric: Metric,
    stats: dict[str, float] | None,
    failed_runs: int,
    comparable: bool,
    perception: str,
    note: str,
) -> dict:
    line = {
        'row_id': row_id,
        'arm': row['arm'],
        'condition': row['condition'],
        'metric': metric.name,
        'n': 0,
        'failed_runs': failed_runs,
        'median': '',
        'mean': '',
        'std': '',
        'min': '',
        'max': '',
        'comparable': str(comparable).lower(),
        'perception': perception,
        'note': note,
    }

    if stats is not None:
        line['n'] = int(stats['n'])

        for key in ('median', 'mean', 'std', 'min', 'max'):
            line[key] = format(stats[key], '.6g')

    return line


def _manifest_row(
    row_id: str,
    row: dict,
    runs: list[RunFacts],
    comparable: bool,
    perception: str,
    command: str,
) -> dict:
    return {
        'row_id': row_id,
        'arm': row['arm'],
        'condition': row['condition'],
        'runs': [run.name for run in runs],
        'comparable': comparable,
        'perception': perception,
        'bag_sha1s': [run.provenance.get('bag_sha1', UNKNOWN) for run in runs],
        'flight_commits': _distinct(run.flight_commit for run in runs),
        'flight_dirty': any(run.dirty for run in runs),
        'image_id': _distinct((run.run or {}).get('image_id', UNKNOWN) for run in runs),
        'scorer_commit': _distinct(run.provenance.get('scorer_commit', UNKNOWN) for run in runs),
        'scorer_dirty': _distinct(run.provenance.get('scorer_dirty', UNKNOWN) for run in runs),
        'command': command,
    }


def _perception_label(runs: list[RunFacts]) -> str:
    sources = _distinct(run.perception for run in runs)
    if len(sources) == 1:
        return sources[0]

    return 'mixed' if sources else UNKNOWN


def _distinct(values) -> list:
    seen: list = []

    for value in values:
        if value not in seen:
            seen.append(value)

    return seen


def _table_markdown(
    table_rows: list[dict],
    command: str,
    worst_values: dict[str, float],
    min_filming_s: float,
) -> str:
    headings = ['Arm', 'Condition', 'Runs', 'Failed runs']
    headings += [f'{metric.heading} (median)' for metric in METRICS]
    headings += ['Comparable', 'Note']
    worst_valued = [metric.heading for metric in METRICS if metric.name in worst_values]
    failed_left_out = [metric.heading for metric in METRICS if metric.name not in worst_values]
    lines = [
        f'Generated by `{command}` from `eval/standings.toml`; edit the manifest, not this table.',
        '',
        '| ' + ' | '.join(headings) + ' |',
        '| ' + ' | '.join('---' for _ in headings) + ' |',
    ]

    for entry in table_rows:
        cells = [
            entry['row']['arm'],
            entry['row']['condition'],
            str(entry['runs']),
            str(entry['failed_runs']),
        ]

        for metric in METRICS:
            stats = entry['statistics'][metric]
            cells.append(
                '-'
                if stats is None
                else f'{stats["median"] * metric.scale:.3g}{metric.unit} (n={int(stats["n"])})',
            )

        note = ' '.join(entry['note'].split())

        if entry['oracle']:
            note = f'oracle perception; {note}' if note else 'oracle perception'

        cells += ['yes' if entry['comparable'] else 'no', note]
        lines.append('| ' + ' | '.join(cell.replace('|', '/') for cell in cells) + ' |')

    rules = (
        f'n counts the runs behind each median. Focus and motion values need at least '
        f'{min_filming_s:g} s of filming (`eval/analysis_plan.toml`).'
    )

    if worst_valued:
        rules += (
            f" A failed run (mission not completed, aborted, or collided) takes the plan's "
            f'worst value in: {", ".join(worst_valued)}.'
        )

    if failed_left_out:
        rules += f' Not in the plan, so failed runs are left out: {", ".join(failed_left_out)}.'

    lines += ['', rules]

    return '\n'.join(lines)


def _row_id(arm: str, condition: str) -> str:
    return re.sub(r'[^a-z0-9]+', '-', f'{arm} {condition}'.lower()).strip('-')
