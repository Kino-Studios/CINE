import typer

from cine_cli.common import ExitCode, JsonFlag, get_environment, require_root

CHECKER_MISSING_MESSAGE = (
    'cine_interfaces is not installed: pipx inject cine-cli ../CINE-Interfaces/cine_interfaces'
)

app = typer.Typer(
    no_args_is_help=True, help='Check this repository against the CINE-Interfaces contract.'
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
