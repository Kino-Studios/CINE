"""Repo-root and path-resolution helpers for the JSON config layer.

This module is the low-level path utility used by ``config_loader.ConfigLoader``
to locate the CINE checkout and to turn repo-relative config values (such as
``mlp_weights_path``) into absolute paths. No absolute paths are ever hardcoded
here: the repo root is discovered at runtime from the ``CINE_ROOT`` environment
variable when set, otherwise by walking up to the directory containing the
``VERSION`` sentinel file.
"""

import os

VERSION_SENTINEL = "VERSION"


def repo_root_from(file_path: str, max_levels_up: int = 12) -> str:
    """Return the CINE repo root by walking upward from ``file_path``.

    The walk stops at the first ancestor that looks like the repo root: one
    containing the ``VERSION`` sentinel, a ``configs`` directory, or the ROS 2
    workspace directory. Falls back to the starting directory if none match.
    """
    cur = os.path.abspath(os.path.dirname(file_path))

    for _ in range(int(max_levels_up)):
        if os.path.isfile(os.path.join(cur, VERSION_SENTINEL)):
            return cur

        if os.path.isdir(os.path.join(cur, "configs")):
            return cur

        if os.path.isdir(os.path.join(cur, "trajectory-optimization_ws")):
            return cur

        parent = os.path.dirname(cur)

        if parent == cur:
            break

        cur = parent

    return os.path.abspath(os.path.dirname(file_path))


def workspace_root_from(file_path: str, levels_up: int = 6) -> str:
    """Return the workspace root, allowing a deeper upward walk than the repo."""
    return repo_root_from(file_path, max_levels_up=max(1, int(levels_up) + 6))


def cine_root() -> str:
    """Resolve the CINE repo root.

    Prefers the ``CINE_ROOT`` environment variable when set; otherwise walks up
    from this file to the directory containing the ``VERSION`` sentinel.
    """
    env_root = os.environ.get("CINE_ROOT")

    if env_root:
        return os.path.abspath(env_root)

    return repo_root_from(__file__)


def resolve_repo_path(value: str) -> str:
    """Resolve a possibly-relative config value against the CINE repo root.

    Absolute paths and empty values are returned unchanged so that callers can
    pass through user-supplied overrides or sentinel empties without surprise.
    Absolute paths are trusted unconditionally because configs are local and
    version-controlled; only relative paths are containment-checked.

    Raises ValueError when a relative value resolves outside the repo root
    (e.g. via ``..`` traversal).
    """
    if not value:
        return value

    if os.path.isabs(value):
        return value

    resolved = os.path.realpath(os.path.join(cine_root(), value))
    root = os.path.realpath(cine_root())

    if not resolved.startswith(root + os.sep):
        raise ValueError(f"config path escapes repo root: {value!r}")

    return resolved
