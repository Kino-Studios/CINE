"""Turns a January 2026 distance log (CSV) into a flight record the scorer can read."""

import csv
from pathlib import Path

import numpy as np

from cine_eval.record import (
    DRONE_TRUE,
    FOCUS_TARGET,
    FRAMING_ACTIVE,
    SUBJECT_TRUE,
    FlightRecord,
    RecordError,
    Stream,
)

FRAME_ID = 'legacy_log'
DRONE_COLUMNS = ('drone_x', 'drone_y', 'drone_z')
SUBJECT_COLUMNS = ('object_x', 'object_y', 'object_z')
LOGGER_THRESHOLD_M_ESTIMATED = 0.15
REQUIRED_COLUMNS = ('time', *DRONE_COLUMNS, 'target_distance', 'within_tolerance')


def record_from_legacy_csv(csv_path: Path) -> FlightRecord:
    """Reads one ``distance_*`` log; times are simulation seconds, sorted and deduplicated."""

    csv_path = Path(csv_path)
    with csv_path.open(newline='') as handle:
        reader = csv.DictReader(handle)
        columns = reader.fieldnames or []
        rows = list(reader)

    if not all(column in columns for column in SUBJECT_COLUMNS):
        raise RecordError(f'{csv_path.name}: no subject position')

    missing = [column for column in REQUIRED_COLUMNS if column not in columns]
    if missing:
        raise RecordError(f'{csv_path.name}: missing columns {", ".join(missing)}')

    if not rows:
        raise RecordError(f'{csv_path.name}: no rows')

    def column(*names: str) -> np.ndarray:
        return np.array([[float(row[name]) for name in names] for row in rows], dtype=float)

    time = column('time')[:, 0]
    drone = column(*DRONE_COLUMNS)
    subject = column(*SUBJECT_COLUMNS)
    target = column('target_distance')[:, 0]
    within_tolerance = column('within_tolerance')[:, 0]
    order = np.argsort(time, kind='stable')
    _, first_of_each_stamp = np.unique(time[order], return_index=True)
    keep = order[first_of_each_stamp]
    time, drone, subject = time[keep], drone[keep], subject[keep]
    target, within_tolerance = target[keep], within_tolerance[keep]

    def stream(name: str, fields: dict[str, np.ndarray]) -> Stream:
        return Stream(
            name=name,
            frame_id=FRAME_ID,
            time=time,
            receive_time=time.copy(),
            fields=fields,
        )

    streams = {
        DRONE_TRUE: stream(DRONE_TRUE, {'position': drone}),
        SUBJECT_TRUE: stream(SUBJECT_TRUE, {'position': subject}),
        FOCUS_TARGET: stream(FOCUS_TARGET, {'distance_m': target}),
        # Assumed: the focus and motion channels need this stream, and the logger wrote rows only
        # while following the subject. Nothing in the CSV records the cue itself.
        FRAMING_ACTIVE: stream(FRAMING_ACTIVE, {'active': np.ones(len(time), dtype=bool)}),
    }
    metadata = {
        'legacy': True,
        'source_csv': csv_path.name,
        'logger_within_tolerance_share': float(within_tolerance.mean()),
        'logger_within_tolerance_threshold_m_estimated': LOGGER_THRESHOLD_M_ESTIMATED,
        'logger_within_tolerance_threshold_note': (
            'inferred from the data, not documented: across both arms the largest abs_error '
            'flagged within tolerance is about 0.148 m and the smallest unflagged is about 0.151 m'
        ),
        'framing_active_assumed': True,
    }

    return FlightRecord(streams=streams, metadata=metadata)
