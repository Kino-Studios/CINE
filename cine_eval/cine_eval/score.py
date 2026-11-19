"""Scores one flight record on every channel and stores the result next to it."""

import hashlib
import json
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from cine_eval.channels.clearance import score_clearance
from cine_eval.channels.estimate import score_estimate
from cine_eval.channels.focus import score_focus
from cine_eval.channels.following import score_following
from cine_eval.channels.freshness import score_freshness
from cine_eval.channels.mission import score_mission
from cine_eval.channels.motion import score_motion
from cine_eval.channels.result import OK, UNCHECKED, ChannelResult
from cine_eval.record import RECORD_ARRAYS_FILE, load
from cine_eval.scene import load_world, repo_root, resolve_world_path
from cine_eval.settings import ScoringSettings, load_scoring_settings

CHANNELS: tuple[str, ...] = (
    'focus',
    'motion',
    'clearance',
    'estimate',
    'following',
    'freshness',
    'mission',
)
SCORE_FILE = 'score.json'
METRICS_FILE = 'metrics.npz'
UNKNOWN = 'unknown'
# cine_cli/evaluate.py exports 1 or 0; with git missing it exports nothing (unknown).
CLEAN_DIRTY_FLAG = '0'
VALID_STATUSES = (OK, UNCHECKED)
# How the bag ended, as extract recorded it; a truncated tail shows as a last message
# well before bag_start_s + bag_duration_s, or as reindex warnings.
BAG_ENDING_KEYS = ('reindex_warnings', 'last_message_s', 'bag_start_s', 'bag_duration_s')


@dataclass(frozen=True)
class RunScore:
    run_name: str
    channels: dict[str, ChannelResult]
    settings: dict
    provenance: dict


def score_run(run_directory: Path, settings: ScoringSettings | None = None) -> RunScore:
    run_directory = Path(run_directory)
    settings = settings if settings is not None else load_scoring_settings()
    record = load(run_directory)
    world_path = resolve_world_path(record)
    root = repo_root()
    world_label = (
        world_path.relative_to(root).as_posix()
        if world_path.is_relative_to(root)
        else str(world_path)
    )
    world = load_world(world_path)
    channels = {
        'focus': score_focus(record, settings),
        'motion': score_motion(record, settings),
        'clearance': score_clearance(
            record,
            settings,
            world.obstacles,
            world.frame,
            world_label=world_label,
            skipped=world.skipped,
        ),
        'estimate': score_estimate(record, settings),
        'following': score_following(record, settings),
        'freshness': score_freshness(record, settings),
        'mission': score_mission(record, settings),
    }
    provenance = {
        **current_scorer_provenance(),
        'record_sha1': hashlib.sha1((run_directory / RECORD_ARRAYS_FILE).read_bytes()).hexdigest(),
        'bag_sha1': record.metadata.get('bag_sha1', UNKNOWN),
        'bag': {key: record.metadata.get(key, UNKNOWN) for key in BAG_ENDING_KEYS},
    }

    return RunScore(
        run_name=record.metadata.get('bag_name', run_directory.name),
        channels=channels,
        settings=settings.as_dict(),
        provenance=provenance,
    )


def current_scorer_provenance() -> dict[str, str]:
    """The scorer checkout this process runs from, as the host wrapper exported it."""

    return {
        'scorer_commit': os.environ.get('CINE_EVAL_SCORER_COMMIT', UNKNOWN),
        'scorer_dirty': os.environ.get('CINE_EVAL_SCORER_DIRTY', UNKNOWN),
    }


def scorer_is_clean(dirty_flag: str) -> bool:
    """Only an explicit clean flag counts; dirty and unknown both mean the scorer may differ."""

    return dirty_flag == CLEAN_DIRTY_FLAG


def validate_statuses(channels: dict[str, ChannelResult]) -> None:
    """A status other than ``ok`` or ``unchecked`` must never be read as a passing channel."""

    for name, result in channels.items():
        if result.status not in VALID_STATUSES:
            raise ValueError(f'channel {name!r} has unknown status {result.status!r}')


def write_score(score: RunScore, run_directory: Path) -> None:
    run_directory = Path(run_directory)
    run_directory.mkdir(parents=True, exist_ok=True)
    document = {
        'run': score.run_name,
        'channels': {
            name: {
                'status': result.status,
                'reason': result.reason,
                'summary': result.summary,
                'clock_hz': result.clock_hz,
            }
            for name, result in score.channels.items()
        },
        'settings': score.settings,
        'provenance': score.provenance,
    }
    (run_directory / SCORE_FILE).write_text(
        json.dumps(document, allow_nan=False, indent=2, sort_keys=True) + '\n',
    )
    series = {
        f'{name}/{series_name}': values
        for name, result in score.channels.items()
        for series_name, values in result.series.items()
    }
    np.savez(run_directory / METRICS_FILE, **series)


def read_score(run_directory: Path) -> RunScore:
    """The stored score; per-channel series stay in ``metrics.npz`` and are not loaded."""

    document = json.loads((Path(run_directory) / SCORE_FILE).read_text())
    channels = {
        name: ChannelResult(
            name=name,
            status=entry['status'],
            reason=entry['reason'],
            summary=entry['summary'],
            series={},
            clock_hz=entry['clock_hz'],
        )
        for name, entry in document['channels'].items()
    }

    validate_statuses(channels)

    return RunScore(
        run_name=document['run'],
        channels=channels,
        settings=document['settings'],
        provenance=document['provenance'],
    )
