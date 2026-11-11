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


class CollectionMode(StrEnum):
    BASELINE = 'baseline'
    CONSTRAINED = 'constrained'


def collect(
    context: typer.Context,
    mode: Annotated[CollectionMode, typer.Argument(help='Which data-collection run to start.')],
    dry_run: DryRunFlag = False,
    as_json: JsonFlag = False,
) -> None:
    """Collect training data by running the matching tools/collect_*_data.sh script."""
    environment = get_environment(context)
    require_root(environment)
    if not dry_run:
        require_ros2(environment)
    execute(environment, ['bash', f'tools/collect_{mode.value}_data.sh'], dry_run, as_json)
