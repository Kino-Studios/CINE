import sys
from unittest.mock import MagicMock

import pytest

sys.modules.setdefault('rclpy', MagicMock())
sys.modules.setdefault('rclpy.node', MagicMock())

from cine_agent.data_collection.collection_launcher import _find_workspace_root  # noqa: E402


def test_workspace_root_is_the_tools_directory_beside_the_package(tmp_path):
    tools_directory = tmp_path / 'CINE' / 'tools'
    tools_directory.mkdir(parents=True)
    (tools_directory / 'start_simulation.sh').write_text('')
    module_path = (
        tmp_path / 'CINE' / 'cine_agent' / 'cine_agent' / 'data_collection' / 'launcher.py'
    )
    module_path.parent.mkdir(parents=True)
    module_path.write_text('')

    assert _find_workspace_root(module_path) == tools_directory


def test_workspace_root_lookup_fails_without_a_tools_directory(tmp_path):
    module_path = tmp_path / 'install' / 'launcher.py'
    module_path.parent.mkdir(parents=True)
    module_path.write_text('')

    with pytest.raises(RuntimeError, match='tools/'):
        _find_workspace_root(module_path)
