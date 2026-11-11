"""Fail when a cross-repository topic here drifts from the CINE-Interfaces contract."""

from pathlib import Path

import pytest

try:
    from cine_interfaces import check
except ImportError:
    check = None

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def test_repository_matches_interface_contract(capsys):
    if check is None:
        pytest.fail(
            'cine_interfaces is not installed: '
            'pip install -e ../CINE-Interfaces/cine_interfaces (see CONTRIBUTING.md)'
        )
    exit_code = check.main(['CINE', str(REPOSITORY_ROOT)])

    assert exit_code == 0, capsys.readouterr().out
