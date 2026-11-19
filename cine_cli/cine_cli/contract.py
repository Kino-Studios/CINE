import json
from dataclasses import asdict, dataclass
from enum import StrEnum
from typing import Annotated

import typer

from cine_cli.common import (
    ExitCode,
    JsonFlag,
    get_environment,
    require_root,
    require_ros2,
)

CHECKER_MISSING_MESSAGE = (
    'cine_interfaces is not installed: pipx inject cine-cli ../CINE-Interfaces/cine_interfaces'
)
RATE_TOOL = 'tools/measure_topic_rates.py'
GRAPH_QUERY_TIMEOUT_SECONDS = 30.0
# Importing rclpy and discovering every publisher takes a while before the window starts.
RATE_TOOL_STARTUP_SECONDS = 60.0

app = typer.Typer(
    no_args_is_help=True,
    help='Check this repository against the CINE-Interfaces contract.',
)


@app.command()
def check(context: typer.Context, as_json: JsonFlag = False) -> None:
    """Fail when a shared topic here drifts from the CINE-Interfaces contract."""

    root = require_root(get_environment(context))

    try:
        from cine_interfaces import check as checker
    except ImportError as import_failure:
        typer.echo(CHECKER_MISSING_MESSAGE, err=True)

        raise typer.Exit(ExitCode.ENVIRONMENT) from import_failure

    exit_code = checker.main(['CINE', str(root)] + (['--json'] if as_json else []))
    if exit_code != 0:
        raise typer.Exit(exit_code)


class Mode(StrEnum):
    SIM = 'sim'
    HARDWARE = 'hardware'


ModeOption = Annotated[
    Mode,
    typer.Option('--mode', help='Which side publishes the drone topics: the simulator or the Pi.'),
]
SecondsOption = Annotated[
    float,
    typer.Option('--seconds', min=0.5, help='How long to count messages on each topic.'),
]


@dataclass(frozen=True)
class ExchangeFinding:
    severity: str
    stream_key: str | None
    message: str


def _import_contract_checker():
    try:
        from cine_interfaces import contract, live_check
    except ImportError as import_failure:
        typer.echo(CHECKER_MISSING_MESSAGE, err=True)

        raise typer.Exit(ExitCode.ENVIRONMENT) from import_failure

    return contract, live_check


def _published_topics(contract, mode: str) -> dict[str, list]:
    """Map each topic the contract gives a publisher in `mode` to the streams carried on it."""

    streams_by_topic = {}

    for stream in contract.streams:
        if mode not in stream.modes:
            continue

        for publisher in stream.publishers:
            streams_by_topic.setdefault(publisher.topic, [])

            if stream not in streams_by_topic[publisher.topic]:
                streams_by_topic[publisher.topic].append(stream)

    return streams_by_topic


def _measure_rates(environment, root, streams_by_topic, window_seconds: float) -> dict:
    command = [
        'python3',
        RATE_TOOL,
        '--seconds',
        str(window_seconds),
        *(f'{topic}={streams[0].message_type}' for topic, streams in streams_by_topic.items()),
    ]
    output = environment.runner.capture(
        command,
        root,
        timeout_seconds=window_seconds + RATE_TOOL_STARTUP_SECONDS,
    )

    try:
        if output.returncode != 0:
            raise ValueError(f'exit code {output.returncode}')

        return json.loads(output.stdout)['topics']
    except (ValueError, KeyError) as exc:
        typer.echo(
            f'rate measurement failed ({exc}); {RATE_TOOL} needs rclpy, so source ROS 2 first',
            err=True,
        )

        raise typer.Exit(ExitCode.ENVIRONMENT) from exc


def _silent_topic_findings(streams_by_topic, rates, window_seconds: float) -> list:
    findings = []

    for topic, streams in streams_by_topic.items():
        if rates.get(topic, {}).get('messages', 0) > 0:
            continue

        expected_silence = all(stream.accepted_mismatch for stream in streams)
        findings.append(
            ExchangeFinding(
                'info' if expected_silence else 'error',
                streams[0].key,
                f'{topic}: no messages in {window_seconds:g} s',
            ),
        )

    return findings


def _print_exchange_report(findings, rates, window_seconds: float) -> None:
    for finding in findings:
        typer.echo(f'{finding.severity.upper()} [{finding.stream_key or "-"}] {finding.message}')

    typer.echo(f'message rates over {window_seconds:g} s:')

    for topic, measurement in sorted(rates.items()):
        rate = measurement['rate_hz']
        rate_text = f'{rate:.1f} Hz' if rate is not None else '-'
        typer.echo(f'  {topic}  {measurement["messages"]} messages  {rate_text}')

    error_count = sum(1 for finding in findings if finding.severity == 'error')
    typer.echo(f'{error_count} errors, {len(findings) - error_count} info')


@app.command()
def exchange(
    context: typer.Context,
    mode: ModeOption = Mode.SIM,
    seconds: SecondsOption = 5.0,
    as_json: JsonFlag = False,
) -> None:
    """Check the running ROS 2 graph against the contract and measure each topic's rate.

    Fails when a contract node or topic is missing from the graph, or when a topic the
    contract gives a publisher carries no messages during the window.
    """

    environment = get_environment(context)
    root = require_root(environment)
    require_ros2(environment)
    contract_module, live_check = _import_contract_checker()
    contract = contract_module.load()

    def topic_info(topic: str) -> str | None:
        output = environment.runner.capture(
            ['ros2', 'topic', 'info', '--verbose', topic],
            root,
            timeout_seconds=GRAPH_QUERY_TIMEOUT_SECONDS,
        )

        return output.stdout if output.returncode == 0 else None

    graph_findings = [
        ExchangeFinding(finding.severity, finding.stream_key, finding.message)
        for finding in live_check.check_live(contract, topic_info=topic_info, mode=mode.value)
    ]
    streams_by_topic = _published_topics(contract, mode.value)
    rates = _measure_rates(environment, root, streams_by_topic, seconds)
    findings = graph_findings + _silent_topic_findings(streams_by_topic, rates, seconds)
    error_count = sum(1 for finding in findings if finding.severity == 'error')

    if as_json:
        report = {
            'ok': error_count == 0,
            'mode': mode.value,
            'window_seconds': seconds,
            'errors': error_count,
            'findings': [asdict(finding) for finding in findings],
            'topics': rates,
        }
        typer.echo(json.dumps(report, indent=2))
    else:
        _print_exchange_report(findings, rates, seconds)

    if error_count:
        raise typer.Exit(ExitCode.FAILED)
