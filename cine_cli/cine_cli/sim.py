import json
from enum import StrEnum
from typing import Annotated

import typer

from cine_cli.common import (
    DryRunFlag,
    JsonFlag,
    execute,
    get_environment,
    require_docker_image,
    require_root,
    require_ros2,
)

GAZEBO_IMAGE = 'cine-gazebo-sim:dev'
GAZEBO_COMPOSE_FILE = 'docker/gazebo-sim/compose.yaml'
GAZEBO_BUILD_COMMAND = f'docker compose -f {GAZEBO_COMPOSE_FILE} build'

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
                f"'{assignment}' is not NAME=VALUE with a non-empty value",
                param_hint='--set',
            )

        arguments.append(f'{name}:={value}')

    return arguments


@app.command()
def start(
    context: typer.Context,
    mode: Annotated[
        SimulationMode | None,
        typer.Argument(help='Simulation mode; omit for default.'),
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


class NavSource(StrEnum):
    SIM = 'sim'
    VINS = 'vins'


class Wind(StrEnum):
    CALM = 'calm'
    GUSTY = 'gusty'


@app.command()
def gazebo(
    context: typer.Context,
    nav_source: Annotated[
        NavSource,
        typer.Option(help='Where navigation state comes from.'),
    ] = NavSource.SIM,
    headless: Annotated[bool, typer.Option(help='Run Gazebo without a window.')] = False,
    wind: Annotated[Wind, typer.Option(help='Wind disturbance profile.')] = Wind.CALM,
    no_mission: Annotated[
        bool,
        typer.Option('--no-mission', help='Start the world without flying the mission.'),
    ] = False,
    check: Annotated[
        bool,
        typer.Option('--check', help='Run the pass/fail check and return its exit code.'),
    ] = False,
    as_json: JsonFlag = False,
) -> None:
    """Start the Gazebo simulator in its container (needs Docker and, ideally, an NVIDIA GPU)."""

    environment = get_environment(context)
    root = require_root(environment)

    require_docker_image(environment, root, GAZEBO_IMAGE, GAZEBO_BUILD_COMMAND)

    if environment.which('nvidia-smi') is None:
        typer.echo('warning: nvidia-smi not found; Gazebo will render without a GPU', err=True)

    settings = {
        'NAV_SOURCE': nav_source.value,
        'HEADLESS': str(headless).lower(),
        'WIND': wind.value,
        'MISSION': str(not no_mission).lower(),
        'CHECK': str(check).lower(),
    }
    command = ['docker', 'compose', '-f', str(root / GAZEBO_COMPOSE_FILE), 'run', '--rm']

    for name, value in settings.items():
        command += ['-e', f'{name}={value}']

    command.append('gazebo-sim')
    exit_code = environment.runner.run(command, root, stdout_to_stderr=as_json)

    if as_json:
        typer.echo(
            json.dumps(
                {
                    'command': 'sim gazebo',
                    'exit_code': exit_code,
                    'nav_source': nav_source.value,
                    'check': check,
                },
            ),
        )

    if exit_code != 0:
        raise typer.Exit(exit_code)
