"""The flight record: named time series extracted from one recorded flight."""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

DRONE_TRUE = 'drone_true'
SUBJECT_TRUE = 'subject_true'
DRONE_ESTIMATE = 'drone_estimate'
SUBJECT_ESTIMATE = 'subject_estimate'
DETECTIONS = 'detections'
SETPOINTS = 'setpoints'
PLANNED_PATH = 'planned_path'
FLOWN_PATH = 'flown_path'
FRAMING_ACTIVE = 'framing_active'
COSTS = 'costs'
FOCUS_TARGET = 'focus_target'

RECORD_ARRAYS_FILE = 'record.npz'
RECORD_INDEX_FILE = 'record.json'


class RecordError(ValueError):
    """A flight record breaks the contract."""


@dataclass(frozen=True)
class Stream:
    name: str
    frame_id: str
    time: np.ndarray
    receive_time: np.ndarray
    fields: dict[str, np.ndarray]
    stamp_source: str = 'header'


@dataclass(frozen=True)
class FlightRecord:
    streams: dict[str, Stream]
    metadata: dict = field(default_factory=dict)

    def has(self, *names: str) -> bool:
        return all(name in self.streams for name in names)


def validate(record: FlightRecord) -> None:
    for name, stream in record.streams.items():
        if '/' in name:
            raise RecordError(f'stream name {name!r} contains "/"')

        if stream.name != name:
            raise RecordError(f'stream keyed {name!r} is named {stream.name!r}')

        length = len(stream.time)
        if length == 0:
            raise RecordError(f'{name}: stream has no samples; an absent stream is left out')

        if len(stream.receive_time) != length:
            raise RecordError(
                f'{name}: receive_time length {len(stream.receive_time)} != time length {length}',
            )

        if np.any(np.diff(stream.time) < 0):
            raise RecordError(f'{name}: time decreases')

        for field_name, values in stream.fields.items():
            if '/' in field_name:
                raise RecordError(f'{name}: field name {field_name!r} contains "/"')

            if len(values) != length:
                raise RecordError(
                    f'{name}/{field_name}: length {len(values)} != time length {length}',
                )


def save(record: FlightRecord, directory: Path) -> None:
    validate(record)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    arrays: dict[str, np.ndarray] = {}
    index_streams: dict[str, dict] = {}

    for name, stream in record.streams.items():
        arrays[f'{name}/time'] = stream.time
        arrays[f'{name}/receive_time'] = stream.receive_time

        for field_name, values in stream.fields.items():
            arrays[f'{name}/{field_name}'] = values

        index_streams[name] = {
            'frame_id': stream.frame_id,
            'stamp_source': stream.stamp_source,
            'fields': list(stream.fields),
        }

    np.savez(directory / RECORD_ARRAYS_FILE, **arrays)
    index = {'streams': index_streams, 'metadata': record.metadata}
    (directory / RECORD_INDEX_FILE).write_text(json.dumps(index, allow_nan=False, indent=2))


def load(directory: Path) -> FlightRecord:
    directory = Path(directory)
    index = json.loads((directory / RECORD_INDEX_FILE).read_text())
    streams: dict[str, Stream] = {}

    with np.load(directory / RECORD_ARRAYS_FILE) as arrays:
        for name, entry in index['streams'].items():
            streams[name] = Stream(
                name=name,
                frame_id=entry['frame_id'],
                time=arrays[f'{name}/time'],
                receive_time=arrays[f'{name}/receive_time'],
                fields={
                    field_name: arrays[f'{name}/{field_name}'] for field_name in entry['fields']
                },
                stamp_source=entry['stamp_source'],
            )

    return FlightRecord(streams=streams, metadata=index['metadata'])
