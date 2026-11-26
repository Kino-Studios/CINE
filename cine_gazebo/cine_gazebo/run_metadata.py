"""Builds run.json, telling the scorer (cine_eval) what a run was; ROS-free so CI can test it."""

import hashlib
import json
import time
from pathlib import Path

RUN_FILE_NAME = 'run.json'
GIT_COMMIT_VARIABLE = 'CINE_GIT_COMMIT'
GIT_DIRTY_VARIABLE = 'CINE_GIT_DIRTY'
DEFAULT_WAIT_S = 0.5


def build_run_metadata(
    *,
    scenario: str,
    world: str,
    wind: str,
    seed: int | None,
    nav_source: str,
    waypoints,
    waypoints_sha1: str,
    focus_target_m: float,
    git: dict,
    launch_arguments: dict,
    config_sha1: str,
    perception_source: str | None = None,
    image_id: str | None = None,
) -> dict:
    """`perception_source` and `image_id` are left out of the file when not known, so the
    standings read them as unknown instead of as a null label."""

    metadata = {
        'scenario': scenario,
        # Repo-relative template, as the scorer resolves it; the rendered /tmp copy never leaves.
        'world_file': f'cine_gazebo/worlds/{world}.sdf.template',
        'waypoints': [[float(coordinate) for coordinate in point] for point in waypoints],
        # Gazebo reports true poses in a frame named after the world; the waypoints are metres
        # in it, the drone spawning at its origin (config/mission_waypoints.yaml).
        'waypoints_frame': world,
        'waypoints_sha1': waypoints_sha1,
        'wind': wind,
        'seed': seed,
        'nav_source': nav_source,
        'focus_target_m': float(focus_target_m),
        'git': git,
        'launch_arguments': dict(launch_arguments),
        'config_sha1': config_sha1,
    }

    if perception_source is not None:
        metadata['perception_source'] = perception_source

    if image_id is not None:
        metadata['image_id'] = image_id

    return metadata


def file_sha1(path: Path) -> str:
    return hashlib.sha1(Path(path).read_bytes()).hexdigest()


def directory_sha1(directory: Path) -> str:
    """One digest over every file under the directory: relative paths and contents."""

    directory = Path(directory)
    digest = hashlib.sha1()

    for path in sorted(entry for entry in directory.rglob('*') if entry.is_file()):
        digest.update(path.relative_to(directory).as_posix().encode('utf-8') + b'\0')
        digest.update(path.read_bytes() + b'\0')

    return digest.hexdigest()


def parse_seed(text: str) -> int | None:
    if text == '':
        return None

    try:
        return int(text)
    except ValueError:
        raise ValueError(f'seed must be an integer or empty, got {text!r}') from None


def git_state_from_environment(environment) -> dict:
    """Commit and dirty flag from CINE_GIT_COMMIT and CINE_GIT_DIRTY ("0"/"1"); no git in the image.

    A missing or unrecognised value is null, so the scorer reports unknown, never false changes."""

    commit = environment.get(GIT_COMMIT_VARIABLE) or None
    dirty = {'0': False, '1': True}.get(environment.get(GIT_DIRTY_VARIABLE))

    return {'CINE': {'commit': commit, 'dirty': dirty}}


def write_run_metadata_when_directory_exists(
    bag_directory: Path,
    metadata: dict,
    timeout_s: float,
    wait=lambda: time.sleep(DEFAULT_WAIT_S),
) -> Path:
    """Writes run.json into the bag directory once the recorder creates it; `ros2 bag record`
    refuses an existing output directory, so run.json cannot be written first."""

    bag_directory = Path(bag_directory)
    deadline = time.monotonic() + timeout_s

    while not bag_directory.is_dir():
        if time.monotonic() >= deadline:
            raise FileNotFoundError(
                f'bag directory {bag_directory} did not appear within {timeout_s} s',
            )

        wait()

    run_file = bag_directory / RUN_FILE_NAME
    run_file.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')

    return run_file
