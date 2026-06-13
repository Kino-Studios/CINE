"""Tests for the JSON config loader and repo-relative path resolution.

Covers two behaviours hardened in Phase C2:

* The loader fails fast (``FileNotFoundError``) when a declared config file is
  missing, instead of silently substituting an empty mapping.
* ``mlp_weights_path`` is stored repo-relative in JSON and resolved against the
  repo root (``CINE_ROOT`` env var, else the directory containing ``VERSION``).
"""

import json
import os
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIGS_PYTHON = REPO_ROOT / "configs" / "python-constants"

if str(CONFIGS_PYTHON) not in sys.path:
    sys.path.insert(0, str(CONFIGS_PYTHON))

import config_loader as config_loader_module
from config_loader import ConfigLoader
from paths import repo_root_from, resolve_repo_path


def _reset_singleton() -> None:
    """Clear the ConfigLoader singleton so each test loads fresh state."""
    ConfigLoader._instance = None
    ConfigLoader._configs = {}


@pytest.fixture(autouse=True)
def reset_singleton_around_test():
    _reset_singleton()
    yield
    _reset_singleton()


def test_missing_config_file_raises_filenotfounderror(monkeypatch, tmp_path):
    """A declared-but-absent config file must raise, naming the absolute path."""
    fake_root = tmp_path
    (fake_root / "VERSION").write_text("0.0.0\n")

    monkeypatch.setattr(config_loader_module, "repo_root_from", lambda _file: str(fake_root))

    with pytest.raises(FileNotFoundError) as excinfo:
        ConfigLoader()

    expected_fragment = str(fake_root / "configs" / "json_constants")
    assert expected_fragment in str(excinfo.value)


def test_loader_reads_real_configs():
    """Against the real repo, every declared config loads without error."""
    loader = ConfigLoader()

    assert loader.get("mlp", "mlp", "use_mlp_correction") is True


def test_mlp_weights_path_is_relative_in_json():
    """The on-disk JSON value must not be an absolute path."""
    json_path = REPO_ROOT / "configs" / "json_constants" / "mlp" / "mlp-params.json"
    raw = json.loads(json_path.read_text())

    stored = raw["mlp"]["mlp_weights_path"]

    assert not os.path.isabs(stored), f"expected relative path, got {stored!r}"


def test_resolve_repo_path_uses_cine_root_env(monkeypatch, tmp_path):
    """When CINE_ROOT is set, relative values resolve under it."""
    monkeypatch.setenv("CINE_ROOT", str(tmp_path))

    resolved = resolve_repo_path("mlp/weights")

    assert resolved == str(tmp_path / "mlp" / "weights")


def test_resolve_repo_path_walks_to_version_sentinel(monkeypatch):
    """Without CINE_ROOT, resolution walks up to the VERSION sentinel."""
    monkeypatch.delenv("CINE_ROOT", raising=False)

    resolved = resolve_repo_path("mlp/weights")

    assert resolved == str(REPO_ROOT / "mlp" / "weights")
    assert (REPO_ROOT / "VERSION").is_file()


def test_resolve_repo_path_passes_through_absolute(monkeypatch, tmp_path):
    """An already-absolute value is returned unchanged."""
    monkeypatch.delenv("CINE_ROOT", raising=False)
    absolute = str(tmp_path / "somewhere")

    assert resolve_repo_path(absolute) == absolute


def test_repo_root_from_finds_version_root():
    """repo_root_from resolves the CINE checkout containing VERSION."""
    root = Path(repo_root_from(str(CONFIGS_PYTHON / "config_loader.py")))

    assert (root / "VERSION").is_file()


def test_resolve_repo_path_traversal_raises(monkeypatch, tmp_path):
    """Traversal value that escapes repo root must raise ValueError."""
    monkeypatch.setenv("CINE_ROOT", str(tmp_path))

    with pytest.raises(ValueError, match="escapes repo root"):
        resolve_repo_path("../../../etc/passwd")


def test_resolve_repo_path_traversal_dotdot_raises(monkeypatch, tmp_path):
    """Multiple traversal segments must also raise ValueError."""
    monkeypatch.setenv("CINE_ROOT", str(tmp_path))

    with pytest.raises(ValueError, match="escapes repo root"):
        resolve_repo_path("mlp/../../../../../../tmp/evil")


def test_resolve_repo_path_normal_relative_still_resolves(monkeypatch, tmp_path):
    """Normal relative value resolves to path under repo root (dir need not exist)."""
    monkeypatch.setenv("CINE_ROOT", str(tmp_path))

    result = resolve_repo_path("mlp/weights")

    assert result == str(tmp_path / "mlp" / "weights")
