from pathlib import Path
from typing import Annotated

import typer

from cine_cli.common import DryRunFlag, JsonFlag, execute, get_environment

app = typer.Typer(no_args_is_help=True, help='Summarise recorded trajectories.')

TrajectoryRootOption = Annotated[
    Path | None, typer.Option(help='Directory holding the trajectory datasets.')
]
DatasetsOption = Annotated[
    list[str] | None, typer.Option(help='Dataset to analyse; repeat for several.')
]
OutputDirOption = Annotated[Path | None, typer.Option(help='Directory for the output files.')]


def script_command(
    script: str,
    trajectory_root: Path | None,
    datasets: list[str] | None,
    output_dir: Path | None,
    extra_options: dict[str, Path | float | None] | None = None,
) -> list[str]:
    command = ['python3', f'tools/analysis/{script}']
    if trajectory_root is not None:
        command += ['--trajectory-root', str(trajectory_root)]
    if datasets:
        # The scripts declare --datasets with nargs='+', so a repeated flag would keep only the last.
        command += ['--datasets', *datasets]
    if output_dir is not None:
        command += ['--output-dir', str(output_dir)]
    for flag, value in (extra_options or {}).items():
        if value is not None:
            command += [flag, str(value)]

    return command


@app.command()
def stats(
    context: typer.Context,
    trajectory_root: TrajectoryRootOption = None,
    datasets: DatasetsOption = None,
    output_dir: OutputDirOption = None,
    dry_run: DryRunFlag = False,
    as_json: JsonFlag = False,
) -> None:
    """Print trajectory statistics with tools/analysis/trajectory_stats.py."""
    command = script_command('trajectory_stats.py', trajectory_root, datasets, output_dir)
    execute(get_environment(context), command, dry_run, as_json)


@app.command()
def mean(
    context: typer.Context,
    trajectory_root: TrajectoryRootOption = None,
    datasets: DatasetsOption = None,
    output_dir: OutputDirOption = None,
    time_step: Annotated[float | None, typer.Option(help='Resampling step in seconds.')] = None,
    combined_output: Annotated[
        Path | None, typer.Option(help='File for the combined mean trajectory.')
    ] = None,
    dry_run: DryRunFlag = False,
    as_json: JsonFlag = False,
) -> None:
    """Average trajectories with tools/analysis/trajectory_mean.py."""
    command = script_command(
        'trajectory_mean.py',
        trajectory_root,
        datasets,
        output_dir,
        {'--time-step': time_step, '--combined-output': combined_output},
    )
    execute(get_environment(context), command, dry_run, as_json)
