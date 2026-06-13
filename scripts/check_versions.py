"""
check_versions.py — CI version consistency guard.

Reads the root VERSION file and asserts that every setup.py and package.xml
in the repository declares the same version string. Exits non-zero and prints
a mismatch report to stderr when inconsistencies are found.

Usage:
    python scripts/check_versions.py
"""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import NamedTuple

# Directories that contain third-party or generated manifests, not ours.
SKIP_DIRS = {
    "venv",
    "install",
    ".git",
    "ego-planner-swarm",
}


class VersionEntry(NamedTuple):
    file: Path
    declared: str


def _repo_root() -> Path:
    """Return the repository root (directory that contains VERSION)."""
    candidate = Path(__file__).resolve().parent
    while candidate != candidate.parent:
        if (candidate / "VERSION").is_file():
            return candidate
        candidate = candidate.parent
    raise FileNotFoundError("VERSION file not found walking up from script location")


def _is_skipped(path: Path, root: Path) -> bool:
    parts = path.relative_to(root).parts
    return bool(set(parts) & SKIP_DIRS)


def _parse_setup_py_version(path: Path) -> str | None:
    """Extract version string from a setup() call in setup.py."""
    text = path.read_text(encoding="utf-8")
    match = re.search(r"""version\s*=\s*['"]([^'"]+)['"]""", text)
    return match.group(1) if match else None


def _parse_package_xml_version(path: Path) -> str | None:
    """Extract <version> text from a ROS package.xml."""
    try:
        tree = ET.parse(path)
        elem = tree.getroot().find("version")
        return elem.text.strip() if elem is not None and elem.text else None
    except ET.ParseError:
        return None


def collect_versions(root: Path) -> list[VersionEntry]:
    entries: list[VersionEntry] = []

    for setup_py in root.rglob("setup.py"):
        if _is_skipped(setup_py, root):
            continue
        version = _parse_setup_py_version(setup_py)
        if version is not None:
            entries.append(VersionEntry(file=setup_py.relative_to(root), declared=version))

    for pkg_xml in root.rglob("package.xml"):
        if _is_skipped(pkg_xml, root):
            continue
        version = _parse_package_xml_version(pkg_xml)
        if version is not None:
            entries.append(VersionEntry(file=pkg_xml.relative_to(root), declared=version))

    return entries


def main() -> int:
    root = _repo_root()
    expected = (root / "VERSION").read_text(encoding="utf-8").strip()

    entries = collect_versions(root)
    if not entries:
        print("check_versions: no setup.py or package.xml found", file=sys.stderr)
        return 1

    mismatches = [e for e in entries if e.declared != expected]

    if mismatches:
        print(
            f"check_versions: FAIL — expected {expected!r}, found mismatches:",
            file=sys.stderr,
        )
        for entry in mismatches:
            print(f"  {entry.file}: {entry.declared!r}", file=sys.stderr)
        return 1

    print(f"check_versions: OK — all {len(entries)} files declare {expected!r}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
