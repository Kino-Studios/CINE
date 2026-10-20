import os
import subprocess
import sys
from pathlib import Path

import pytest
from cine_config.config_loader import ConfigLoader

PACKAGE_DIR = Path(__file__).resolve().parents[1]
INSTALLED_CONFIG_DIR = PACKAGE_DIR / 'config'


def _run_python(code, config_root, working_directory):
    environment = dict(os.environ, PYTHONPATH=str(PACKAGE_DIR), CINE_CONFIG_ROOT=str(config_root))

    return subprocess.run(
        [sys.executable, '-c', code],
        cwd=working_directory,
        env=environment,
        capture_output=True,
        text=True,
    )


def test_constants_import_does_not_require_rclpy(tmp_path):
    result = _run_python(
        "import sys; sys.modules['rclpy'] = None; sys.modules['rclpy.qos'] = None; "
        'from cine_config.constants import SimulationParams; print(SimulationParams.DRONE_ID)',
        INSTALLED_CONFIG_DIR,
        tmp_path,
    )
    assert result.stdout.strip() == '0', result.stderr


def test_config_loader_missing_file_names_the_file(tmp_path, monkeypatch):
    monkeypatch.setenv('CINE_CONFIG_ROOT', str(tmp_path))
    monkeypatch.setattr(ConfigLoader, '_instance', None)
    with pytest.raises(FileNotFoundError, match='camera-params.json'):
        ConfigLoader()


def test_get_config_prints_the_named_constant(tmp_path):
    result = _run_python(
        'import sys; sys.argv = ["get_config", "DRONE_ID"]; '
        'from cine_config.get_config import main; main()',
        INSTALLED_CONFIG_DIR,
        tmp_path,
    )
    assert result.stdout.strip() == '0', result.stderr


def test_get_config_prints_booleans_as_one_or_zero(tmp_path):
    result = _run_python(
        'import sys; sys.argv = ["get_config", "USE_CINEMATIC_BSPLINE"]; '
        'from cine_config.get_config import main; main()',
        INSTALLED_CONFIG_DIR,
        tmp_path,
    )
    assert result.stdout.strip() == '1', result.stderr


def test_get_config_unknown_name_exits_nonzero(tmp_path):
    result = _run_python(
        'import sys; sys.argv = ["get_config", "NOT_A_CONSTANT"]; '
        'from cine_config.get_config import main; main()',
        INSTALLED_CONFIG_DIR,
        tmp_path,
    )
    assert result.returncode == 1


def test_shipped_config_enables_simulation_for_the_cinematic_launch(tmp_path):
    result = _run_python(
        'from cine_config.constants import SimulationParams; print(SimulationParams.USE_SIMULATION)',
        INSTALLED_CONFIG_DIR,
        tmp_path,
    )
    assert result.stdout.strip() == 'True', result.stderr
