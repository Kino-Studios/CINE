import sys
from unittest.mock import MagicMock

import pytest

sys.modules.setdefault('rclpy', MagicMock())
sys.modules.setdefault('rclpy.node', MagicMock())

from cine_agent.data_collection.collection_launcher import (  # noqa: E402
    _ensure_display,
    _find_workspace_root,
)


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


def test_display_on_plain_linux_is_the_local_x_server(tmp_path):
    proc_version = tmp_path / 'version'
    proc_version.write_text('Linux version 6.8.0-45-generic (builder@ubuntu)\n')
    resolv_conf = tmp_path / 'resolv.conf'
    resolv_conf.write_text('nameserver 127.0.0.53\n')
    env = {}

    _ensure_display(env, proc_version, resolv_conf)

    assert env['DISPLAY'] == ':0'


def test_display_under_wsl_is_the_windows_host(tmp_path):
    proc_version = tmp_path / 'version'
    proc_version.write_text('Linux version 5.15.153.1-microsoft-standard-WSL2\n')
    resolv_conf = tmp_path / 'resolv.conf'
    resolv_conf.write_text('nameserver 172.29.32.1\n')
    env = {}

    _ensure_display(env, proc_version, resolv_conf)

    assert env['DISPLAY'] == '172.29.32.1:0'


def test_display_already_set_is_left_alone(tmp_path):
    proc_version = tmp_path / 'version'
    proc_version.write_text('Linux version 5.15.153.1-microsoft-standard-WSL2\n')
    resolv_conf = tmp_path / 'resolv.conf'
    resolv_conf.write_text('nameserver 172.29.32.1\n')
    env = {'DISPLAY': ':1'}

    _ensure_display(env, proc_version, resolv_conf)

    assert env['DISPLAY'] == ':1'
