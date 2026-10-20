"""Every relative import in cine_agent names a module that exists.

The nodes import rclpy and cine_msgs, so they cannot be imported here; reading the
import statements catches a moved module without a ROS install.
"""

import ast
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / 'cine_agent'


def _relative_imports():
    found = []
    for source_path in sorted(PACKAGE_ROOT.rglob('*.py')):
        tree = ast.parse(source_path.read_text(), filename=str(source_path))
        for statement in ast.walk(tree):
            if isinstance(statement, ast.ImportFrom) and statement.level and statement.module:
                found.append((source_path, statement.level, statement.module))

    return found


def _module_exists(source_path, level, module):
    base_directory = source_path.parent
    for _ in range(level - 1):
        base_directory = base_directory.parent
    target = base_directory.joinpath(*module.split('.'))

    return target.with_suffix('.py').is_file() or (target / '__init__.py').is_file()


@pytest.mark.parametrize(
    'source_path, level, module',
    _relative_imports(),
    ids=lambda value: (
        value.relative_to(PACKAGE_ROOT).as_posix() if isinstance(value, Path) else str(value)
    ),
)
def test_relative_import_names_an_existing_module(source_path, level, module):
    assert _module_exists(source_path, level, module)
