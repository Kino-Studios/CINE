import argparse
import hashlib
import json
import sys
import urllib.request
from pathlib import Path

_SCRIPTS_DIR = Path(__file__).resolve().parent
_PYTHON_CONSTANTS = _SCRIPTS_DIR.parent / "configs" / "python-constants"

if str(_PYTHON_CONSTANTS) not in sys.path:
    sys.path.insert(0, str(_PYTHON_CONSTANTS))

from paths import cine_root  # noqa: E402

REPO = "fishnos/CINE"
WEIGHTS_FILES = ["mlp_correction.onnx", "mlp_correction.onnx.data", "scaler.json"]

API_LATEST = f"https://api.github.com/repos/{REPO}/releases/latest"
API_BY_TAG = "https://api.github.com/repos/{repo}/releases/tags/{tag}"


def _fetch_json(url: str) -> dict:
    req = urllib.request.Request(url, headers={"Accept": "application/vnd.github+json"})
    with urllib.request.urlopen(req) as resp:
        return json.loads(resp.read())


def _sha256_of(path: Path) -> str:
    """Return lowercase hex SHA-256 digest of file at ``path``."""
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)

    return digest.hexdigest()


def _download(url: str, dest: Path, expected_digest: str | None) -> None:
    """Download ``url`` to ``dest`` then verify SHA-256 when digest is known.

    ``expected_digest`` format: ``sha256:<hex>`` (GitHub release-asset API).
    Missing digest prints a warning; mismatch deletes the file and raises.
    """
    print(f"  downloading {dest.name} ...", end=" ", flush=True)
    urllib.request.urlretrieve(url, dest)
    size_kb = dest.stat().st_size // 1024
    print(f"{size_kb} KB")

    if not expected_digest:
        print(f"  WARNING: no digest for {dest.name} — integrity could not be verified")

        return

    prefix = "sha256:"
    if not expected_digest.startswith(prefix):
        print(
            f"  WARNING: unrecognised digest format {expected_digest!r} for {dest.name} — skipping"
        )

        return

    expected_hex = expected_digest[len(prefix) :]
    actual_hex = _sha256_of(dest)

    if actual_hex != expected_hex:
        dest.unlink(missing_ok=True)

        raise ValueError(
            f"SHA-256 mismatch for {dest.name}: expected {expected_hex}, got {actual_hex}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Download MLP weights from GitHub release")
    parser.add_argument("--tag", default=None, help="Release tag (default: latest)")
    args = parser.parse_args()

    root = Path(cine_root())
    weights_dir = root / "mlp" / "weights"
    weights_dir.mkdir(parents=True, exist_ok=True)

    if args.tag:
        url = API_BY_TAG.format(repo=REPO, tag=args.tag)
    else:
        url = API_LATEST

    print(f"Fetching release info from {REPO} ...")
    try:
        release = _fetch_json(url)
    except urllib.error.HTTPError as e:
        print(f"Error: could not fetch release ({e})")
        sys.exit(1)

    tag = release["tag_name"]
    print(f"Release: {tag}")

    assets = {
        a["name"]: {
            "url": a["browser_download_url"],
            "digest": a.get("digest"),
        }
        for a in release.get("assets", [])
    }

    missing = [f for f in WEIGHTS_FILES if f not in assets]
    if missing:
        print(f"Error: release '{tag}' is missing assets: {missing}")
        print(f"Available: {list(assets.keys())}")
        sys.exit(1)

    for filename in WEIGHTS_FILES:
        asset = assets[filename]
        _download(asset["url"], weights_dir / filename, asset["digest"])

    print(f"\nWeights saved to: {weights_dir}")

    config_path = root / "configs" / "json_constants" / "mlp" / "mlp-params.json"
    if config_path.exists():
        with open(config_path) as f:
            config = json.load(f)

        # Store repo-relative so the config stays portable across checkouts;
        # the loader resolves it against the repo root at runtime.
        try:
            rel_path = weights_dir.relative_to(root).as_posix()
        except ValueError as exc:
            raise ValueError(
                f"weights_dir {weights_dir} is not under repo root {root}; "
                "cannot write a portable relative path to config"
            ) from exc

        config["mlp"]["mlp_weights_path"] = rel_path

        with open(config_path, "w") as f:
            json.dump(config, f, indent=2)
            f.write("\n")
        print(f"Updated mlp_weights_path in {config_path.relative_to(root)}")


if __name__ == "__main__":
    main()
