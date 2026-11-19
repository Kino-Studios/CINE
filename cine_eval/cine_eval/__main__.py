"""Command line of the evaluation harness: ``python -m cine_eval <subcommand>``.

Exit codes: 0 done, 1 gate verdict REVIEW, 2 bad arguments, 3 the ROS bag reader is missing
(run inside cine-gazebo-sim:dev), 4 a bag, record or score cannot be read.
"""

import argparse
import importlib
import json
import os
import sys
from pathlib import Path

from cine_eval.record import RecordError, load, save
from cine_eval.scene import WorldError, repo_root

EXIT_REVIEW = 1
EXIT_USAGE = 2
EXIT_ENVIRONMENT = 3
EXIT_UNREADABLE = 4
COMMITTED_DIGEST_VARIABLE = 'CINE_EVAL_PLAN_COMMITTED_DIGEST'


def _emit(as_json: bool, payload: dict, text: str) -> None:
    print(json.dumps(payload, allow_nan=False) if as_json else text)


def _shown(path: Path) -> str:
    """A path as the user would type it: relative to the repository when it lies under it."""

    try:
        return Path(path).resolve().relative_to(repo_root().resolve()).as_posix()
    except ValueError:
        return str(path)


def _write_failure(path: Path, exception: Exception) -> int:
    return _fail(
        f'cannot write {_shown(path)}: {exception}; only eval/ and docs/ are writable here',
        EXIT_UNREADABLE,
    )


def _fail(message: str, exit_code: int) -> int:
    print(message, file=sys.stderr)

    return exit_code


def _directories(text: str) -> list[Path]:
    return [Path(part) for part in text.split(',') if part]


def _named_directories(option: str, assignments: list[str]) -> dict[str, list[Path]]:
    """Parses NAME=DIR[,DIR...] options into a mapping; repeating a name is an error."""

    parsed: dict[str, list[Path]] = {}

    for assignment in assignments:
        name, separator, directories = assignment.partition('=')
        if not separator or not name or not _directories(directories):
            raise ValueError(f'{option} {assignment!r} is not NAME=DIR[,DIR...]')

        if name in parsed:
            raise ValueError(f'{option} names {name!r} twice')

        parsed[name] = _directories(directories)

    return parsed


def _run_extract(arguments: argparse.Namespace) -> int:
    try:
        extract_module = importlib.import_module('cine_eval.extract')
    except ImportError as exception:
        return _fail(
            f'bag reader unavailable ({exception}): run inside the cine-gazebo-sim:dev image',
            EXIT_ENVIRONMENT,
        )

    bag_directory = Path(arguments.bag_dir)
    out_directory = (
        Path(arguments.out)
        if arguments.out is not None
        else repo_root() / 'eval' / 'runs' / bag_directory.name
    )

    try:
        record = extract_module.extract(bag_directory)
    except (extract_module.BagError, RecordError) as exception:
        return _fail(f'cannot extract {bag_directory}: {exception}', EXIT_UNREADABLE)

    try:
        save(record, out_directory)
    except OSError as exception:
        return _write_failure(out_directory, exception)

    streams = sorted(record.streams)
    _emit(
        arguments.json,
        {'run_directory': _shown(out_directory), 'streams': streams},
        f'record written to {_shown(out_directory)}\nstreams: {", ".join(streams)}',
    )

    return 0


def _run_score(arguments: argparse.Namespace) -> int:
    from cine_eval.score import score_run, write_score

    run_directory = Path(arguments.run_dir)

    try:
        score = score_run(run_directory)
    except WorldError as exception:
        return _fail(f'cannot score {_shown(run_directory)}: {exception}', EXIT_UNREADABLE)
    except (OSError, RecordError, KeyError, ValueError) as exception:
        return _fail(f'cannot score {_shown(run_directory)}: {exception!r}', EXIT_UNREADABLE)

    try:
        write_score(score, run_directory)
    except OSError as exception:
        return _write_failure(run_directory, exception)

    channels = {
        name: {'status': result.status, 'reason': result.reason}
        for name, result in score.channels.items()
    }
    lines = [f'score written to {_shown(run_directory / "score.json")}']
    lines += [
        f'  {name}: {entry["status"]}' + (f' ({entry["reason"]})' if entry['reason'] else '')
        for name, entry in sorted(channels.items())
    ]
    _emit(arguments.json, {'run': score.run_name, 'channels': channels}, '\n'.join(lines))

    return 0


def _run_gate(arguments: argparse.Namespace) -> int:
    from cine_eval.gate import judge
    from cine_eval.score import read_score
    from cine_eval.settings import load_gate_thresholds

    run_directory = Path(arguments.run_dir)

    try:
        score = read_score(run_directory)
        run_metadata = load(run_directory).metadata.get('run', {})
    except (OSError, RecordError, KeyError, ValueError) as exception:
        return _fail(
            f'cannot read the scored run {_shown(run_directory)} (run `score` first): {exception}',
            EXIT_UNREADABLE,
        )

    verdict = judge(score, run_metadata, load_gate_thresholds())
    lines = [
        f'gate: {verdict.label}' + (' (provisional thresholds)' if verdict.provisional else ''),
    ]
    lines += [f'  failed: {failure}' for failure in verdict.failures]
    lines += [f'  unchecked: {rule}' for rule in verdict.unchecked]
    _emit(
        arguments.json,
        {
            'label': verdict.label,
            'failures': list(verdict.failures),
            'unchecked': list(verdict.unchecked),
            'provisional': verdict.provisional,
        },
        '\n'.join(lines),
    )

    return EXIT_REVIEW if verdict.label == 'REVIEW' else 0


def _run_compare(arguments: argparse.Namespace) -> int:
    from cine_eval.analysis import compare, load_plan

    try:
        arms = _named_directories('--arm', arguments.arm)
        repeatability = _named_directories('--repeatability', arguments.repeatability)
    except ValueError as exception:
        return _fail(str(exception), EXIT_USAGE)

    if len(arms) != 2:
        return _fail(f'compare needs exactly two --arm options, got {len(arms)}', EXIT_USAGE)

    name = arguments.name
    if not name or '/' in name or '\\' in name or '..' in name:
        return _fail(f'--name {name!r} must be a plain file name', EXIT_USAGE)

    unknown = sorted(set(repeatability) - set(arms))
    if unknown:
        return _fail(
            f'--repeatability names {", ".join(unknown)}, which no --arm names',
            EXIT_USAGE,
        )

    try:
        comparison = compare(
            arms,
            load_plan(Path(arguments.plan)),
            repeatability or None,
            os.environ.get(COMMITTED_DIGEST_VARIABLE),
        )
    except (OSError, RecordError, KeyError, ValueError) as exception:
        return _fail(f'cannot compare: {exception}', EXIT_UNREADABLE)

    out_directory = (
        Path(arguments.out_dir)
        if arguments.out_dir is not None
        else repo_root() / 'eval' / 'runs' / 'compare'
    )
    json_path = out_directory / f'{name}.json'
    markdown_path = out_directory / f'{name}.md'

    try:
        json_text = json.dumps(comparison.to_json(), allow_nan=False, indent=2) + '\n'
        out_directory.mkdir(parents=True, exist_ok=True)

        with json_path.open('w', newline='\n', encoding='utf-8') as handle:
            handle.write(json_text)

        with markdown_path.open('w', newline='\n', encoding='utf-8') as handle:
            handle.write(comparison.to_markdown())
    except ValueError as exception:
        return _fail(f'cannot write the comparison: {exception}', EXIT_UNREADABLE)
    except OSError as exception:
        return _write_failure(out_directory, exception)

    _emit(
        arguments.json,
        {
            'json': _shown(json_path),
            'markdown': _shown(markdown_path),
            'plan_status': comparison.plan_status,
        },
        f'comparison written to {_shown(json_path)} and {_shown(markdown_path)} ({comparison.plan_status})',
    )

    return 0


def _run_standings(arguments: argparse.Namespace) -> int:
    from cine_eval.standings import build_standings

    root = repo_root()

    try:
        build_standings(
            Path(arguments.manifest),
            root / 'eval' / 'runs',
            root / 'eval' / 'outputs',
            root / 'docs' / 'results.md',
            root / 'eval' / 'analysis_plan.toml',
            'cine eval standings',
        )
    except (OSError, RecordError, KeyError, ValueError) as exception:
        return _fail(f'cannot build standings: {exception}', EXIT_UNREADABLE)

    _emit(
        arguments.json,
        {'outputs': 'eval/outputs', 'results': 'docs/results.md'},
        'standings written to eval/outputs and docs/results.md',
    )

    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog='cine_eval', description=__doc__.splitlines()[0])
    subcommands = parser.add_subparsers(dest='subcommand', required=True)

    def add(name: str, handler, help_text: str) -> argparse.ArgumentParser:
        subparser = subcommands.add_parser(name, help=help_text)
        subparser.add_argument('--json', action='store_true', help='print one JSON object')
        subparser.set_defaults(handler=handler)

        return subparser

    extract = add('extract', _run_extract, 'read a recorded bag into a flight record')
    extract.add_argument('bag_dir')
    extract.add_argument('--out', help='record directory; default eval/runs/<bag name>')

    score = add('score', _run_score, 'score a flight record on every channel')
    score.add_argument('run_dir')

    gate = add('gate', _run_gate, 'judge a scored run: PASS, or REVIEW (exit 1)')
    gate.add_argument('run_dir')

    compare = add('compare', _run_compare, 'paired comparison of two arms of scored runs')
    compare.add_argument('--arm', action='append', default=[], metavar='NAME=DIR[,DIR...]')
    compare.add_argument(
        '--repeatability',
        action='append',
        default=[],
        metavar='NAME=DIR[,DIR...]',
    )
    compare.add_argument('--plan', default='eval/analysis_plan.toml')
    compare.add_argument('--name', required=True)
    compare.add_argument('--out-dir', help='default eval/runs/compare')

    standings = add('standings', _run_standings, 'rebuild the results table and its files')
    standings.add_argument('--manifest', default='eval/standings.toml')

    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = _build_parser().parse_args(argv)

    return arguments.handler(arguments)


if __name__ == '__main__':
    sys.exit(main())
