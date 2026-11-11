from enum import StrEnum
from typing import Annotated

import typer

from cine_cli.common import (
    DryRunFlag,
    JsonFlag,
    execute,
    get_environment,
    require_root,
    require_ros2,
)

app = typer.Typer(no_args_is_help=True, help='Run the CINE simulation.')


class SimulationMode(StrEnum):
    BASELINE = 'baseline'
    CONSTRAINED = 'constrained'


def launch_arguments(assignments: list[str]) -> list[str]:
    arguments = []
    for assignment in assignments:
        name, separator, value = assignment.partition('=')
        if not separator or not name or not value:
            raise typer.BadParameter(
                f"'{assignment}' is not NAME=VALUE with a non-empty value", param_hint='--set'
            )
        arguments.append(f'{name}:={value}')

    return arguments


@app.command()
def start(
    context: typer.Context,
    mode: Annotated[
        SimulationMode | None, typer.Argument(help='Simulation mode; omit for default.')
    ] = None,
    set_: Annotated[
        list[str] | None,
        typer.Option('--set', help='Launch argument as NAME=VALUE; repeat for several.'),
    ] = None,
    dry_run: DryRunFlag = False,
    as_json: JsonFlag = False,
) -> None:
    """Start the simulation through tools/start_simulation.sh."""
    environment = get_environment(context)
    require_root(environment)
    arguments = launch_arguments(set_ or [])
    if not dry_run:
        require_ros2(environment)
    command = ['bash', 'tools/start_simulation.sh']
    if mode is not None:
        command.append(mode.value)
    elif arguments:
        # The script reads its first argument as the mode; an empty one means no mode.
        command.append('')
    execute(environment, command + arguments, dry_run, as_json)
