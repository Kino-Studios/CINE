import argparse
import json
import os
import sys
import urllib.request
from pathlib import Path

REPO = "fishnos/CINE"
WEIGHTS_FILES = ["mlp_correction.onnx", "mlp_correction.onnx.data", "scaler.json"]

API_LATEST = f"https://api.github.com/repos/{REPO}/releases/latest"
API_BY_TAG  = "https://api.github.com/repos/{repo}/releases/tags/{tag}"


def _repo_root() -> Path:
    here = Path(__file__).resolve()
    for p in [here] + list(here.parents):
        if (p / "configs").is_dir():
            return p
    return here.parent


def _fetch_json(url: str) -> dict:
    req = urllib.request.Request(url, headers={"Accept": "application/vnd.github+json"})
    with urllib.request.urlopen(req) as resp:
        return json.loads(resp.read())


def _download(url: str, dest: Path) -> None:
    print(f"  downloading {dest.name} ...", end=" ", flush=True)
    urllib.request.urlretrieve(url, dest)
    size_kb = dest.stat().st_size // 1024
    print(f"{size_kb} KB")


def main() -> None:
    parser = argparse.ArgumentParser(description="Download MLP weights from GitHub release")
    parser.add_argument("--tag", default=None, help="Release tag (default: latest)")
    args = parser.parse_args()

    root = _repo_root()
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

    assets = {a["name"]: a["browser_download_url"] for a in release.get("assets", [])}

    missing = [f for f in WEIGHTS_FILES if f not in assets]
    if missing:
        print(f"Error: release '{tag}' is missing assets: {missing}")
        print(f"Available: {list(assets.keys())}")
        sys.exit(1)

    for filename in WEIGHTS_FILES:
        _download(assets[filename], weights_dir / filename)

    print(f"\nWeights saved to: {weights_dir}")

    config_path = root / "configs" / "json_constants" / "mlp" / "mlp-params.json"
    if config_path.exists():
        with open(config_path) as f:
            config = json.load(f)
        config["mlp"]["mlp_weights_path"] = str(weights_dir)
        with open(config_path, "w") as f:
            json.dump(config, f, indent=2)
            f.write("\n")
        print(f"Updated mlp_weights_path in {config_path.relative_to(root)}")


if __name__ == "__main__":
    main()
