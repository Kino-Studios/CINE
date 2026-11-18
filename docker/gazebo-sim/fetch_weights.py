"""Download the pinned detection and depth weights into the weights volume.

Run inside the gazebo-sim image (the entrypoint does this on every start):
    /opt/cine-venv/bin/python /opt/cine/fetch_weights.py --weights-dir /weights
Files already present with the pinned content are not downloaded again.
"""

import argparse
import hashlib
import shutil
import sys
import urllib.request
from pathlib import Path

YOLO_FILE_NAME = 'yolov8n.pt'
YOLO_URL = 'https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8n.pt'
YOLO_SHA256 = 'f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36'

# Hugging Face repository id to the commit it is pinned at. Each lands in
# <weights-dir>/<repository id>, a directory the depth node's model_id can name directly.
DEPTH_MODEL_REVISIONS = {
    'depth-anything/DA3METRIC-LARGE': '4010e39f3634a45bc60553321fb49fb760bd594e',
    'depth-anything/DA3-SMALL': 'e08cab65ca0ec38e7826075418411ab90cab4da3',
    'depth-anything/DA3-BASE': 'f4a6c9b3c95e41c82048423d3493a81ec3fa810e',
}

HASH_CHUNK_BYTES = 1 << 20


def verify_sha256(path, expected):
    digest = hashlib.sha256()

    with open(path, 'rb') as file_handle:
        for chunk in iter(lambda: file_handle.read(HASH_CHUNK_BYTES), b''):
            digest.update(chunk)

    return digest.hexdigest() == expected.lower()


def fetch_yolo(weights_dir):
    destination = weights_dir / YOLO_FILE_NAME
    if destination.exists() and verify_sha256(destination, YOLO_SHA256):
        print(f'{destination} present with the pinned hash')

        return

    # Download beside the destination and rename, so an interrupted download never
    # leaves a file at the final path.
    partial = destination.with_name(destination.name + '.partial')
    print(f'Downloading {YOLO_URL}')

    with urllib.request.urlopen(YOLO_URL) as response, open(partial, 'wb') as file_handle:
        shutil.copyfileobj(response, file_handle)

    if not verify_sha256(partial, YOLO_SHA256):
        partial.unlink()

        raise RuntimeError(f'{YOLO_URL} does not match sha256 {YOLO_SHA256}')

    partial.replace(destination)


def fetch_depth_models(weights_dir):
    from huggingface_hub import snapshot_download

    for repository_id, revision in DEPTH_MODEL_REVISIONS.items():
        # snapshot_download skips files already in local_dir at this revision.
        local_dir = snapshot_download(
            repo_id=repository_id,
            revision=revision,
            local_dir=weights_dir / repository_id,
        )
        print(f'{repository_id}@{revision} in {local_dir}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--weights-dir', type=Path, default=Path('/weights'))
    arguments = parser.parse_args(argv)

    arguments.weights_dir.mkdir(parents=True, exist_ok=True)
    fetch_yolo(arguments.weights_dir)
    fetch_depth_models(arguments.weights_dir)

    return 0


if __name__ == '__main__':
    sys.exit(main())
