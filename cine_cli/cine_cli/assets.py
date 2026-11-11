from typing import Annotated

import typer

from cine_cli.common import DryRunFlag, JsonFlag, execute, get_environment

app = typer.Typer(no_args_is_help=True, help='Fetch the data files kept out of git.')


@app.command()
def download(
    context: typer.Context,
    only: Annotated[
        list[str] | None,
        typer.Option(
            help='Download only this asset; repeat for several. The script rejects unknown names.'
        ),
    ] = None,
    dry_run: DryRunFlag = False,
    as_json: JsonFlag = False,
) -> None:
    """Download the release assets with tools/download_assets.py."""
    command = ['python3', 'tools/download_assets.py']
    for name in only or []:
        command += ['--only', name]
    execute(get_environment(context), command, dry_run, as_json)
