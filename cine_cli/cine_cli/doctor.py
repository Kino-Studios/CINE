import json
from dataclasses import asdict, dataclass
from pathlib import Path

import typer

from cine_cli.common import (
    ROS2_MISSING_MESSAGE,
    ExitCode,
    JsonFlag,
    get_environment,
)

WORKSPACE_TIMEOUT_SECONDS = 10


@dataclass(frozen=True)
class Check:
    name: str
    passed: bool
    detail: str


def _format_check(check: Check) -> str:
    status = 'PASS' if check.passed else 'FAIL'

    return f'{status}  {check.name}  {check.detail}'


def doctor(context: typer.Context, as_json: JsonFlag = False) -> None:
    """Check that ROS 2 is sourced and the CINE workspace is built."""
    environment = get_environment(context)
    ros2_path = environment.which('ros2')
    checks: list[Check] = []
    if ros2_path is None:
        checks.append(Check('ros2', False, ROS2_MISSING_MESSAGE))
        exit_code = ExitCode.ENVIRONMENT
    else:
        checks.append(Check('ros2', True, ros2_path))
        domain_id = environment.variables.get('ROS_DOMAIN_ID')
        checks.append(Check('ROS_DOMAIN_ID', True, domain_id or 'unset (0)'))
        prefix = environment.runner.capture(
            ['ros2', 'pkg', 'prefix', 'cine_bringup'], Path.cwd(), WORKSPACE_TIMEOUT_SECONDS
        )
        built = prefix.returncode == 0
        checks.append(
            Check(
                'workspace built',
                built,
                prefix.stdout.strip() if built else 'cine_bringup not found: build and source it',
            )
        )
        exit_code = ExitCode.OK if all(check.passed for check in checks) else ExitCode.FAILED
    if as_json:
        typer.echo(json.dumps({'checks': [asdict(check) for check in checks]}))
    else:
        for check in checks:
            typer.echo(_format_check(check))

    raise typer.Exit(exit_code)
