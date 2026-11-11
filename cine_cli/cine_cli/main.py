import typer

from cine_cli import analyze, assets, collect, contract, doctor, sim
from cine_cli.common import default_environment

app = typer.Typer(no_args_is_help=True, help='Run the CINE simulation and data-collection tools.')
app.add_typer(sim.app, name='sim')
app.add_typer(analyze.app, name='analyze')
app.add_typer(contract.app, name='contract')
app.add_typer(assets.app, name='assets')
app.command('collect')(collect.collect)
app.command('doctor')(doctor.doctor)


@app.callback()
def main(context: typer.Context) -> None:
    if context.obj is None:
        context.obj = default_environment()
