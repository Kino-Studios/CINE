"""Recorded bag (MCAP) to flight record. The only module that imports rosbag2_py.

Runs inside cine-gazebo-sim:dev, where rosbag2_py, rclpy and rosidl_runtime_py exist.
"""

import contextlib
import hashlib
import json
import shutil
import subprocess
import tempfile
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from cine_eval.message_time import HEADER, RECEIVE, header_clock_is_foreign, message_time
from cine_eval.record import (
    DETECTIONS,
    DRONE_ESTIMATE,
    DRONE_TRUE,
    FOCUS_TARGET,
    FRAMING_ACTIVE,
    SETPOINTS,
    SUBJECT_ESTIMATE,
    SUBJECT_TRUE,
    FlightRecord,
    Stream,
    validate,
)

EXTRACTOR_VERSION = '1'
BAG_METADATA_FILE = 'metadata.yaml'
RUN_FILE = 'run.json'
MIXED_STAMP_SOURCE = 'mixed'

TOPIC_STREAMS: dict[str, str] = {
    '/gazebo/drone_pose': DRONE_TRUE,
    '/gazebo/subject_pose': SUBJECT_TRUE,
    '/mavros/local_position/odom': DRONE_ESTIMATE,
    '/cinematography/target_world': SUBJECT_ESTIMATE,
    '/object_detection/center_point': DETECTIONS,
    '/mavros/setpoint_position/local': SETPOINTS,
    '/cinematography/framing_active': FRAMING_ACTIVE,
    '/cinematography/anchor_focus_distance': FOCUS_TARGET,
}
# Used only when the stream's primary topic above yields no samples.
FALLBACK_TOPIC_STREAMS: dict[str, str] = {
    '/vio/odometry': DRONE_ESTIMATE,
}


class BagError(Exception):
    """The bag cannot be read into a flight record."""


def _xyz(vector) -> list[float]:
    return [vector.x, vector.y, vector.z]


def _xyzw(quaternion) -> list[float]:
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def _pose_stamped(message) -> dict:
    return {
        'position': _xyz(message.pose.position),
        'orientation': _xyzw(message.pose.orientation),
    }


def _odometry(message) -> dict:
    return {
        'position': _xyz(message.pose.pose.position),
        'orientation': _xyzw(message.pose.pose.orientation),
        'velocity': _xyz(message.twist.twist.linear),
    }


def _point_stamped(message) -> dict:
    return {'position': _xyz(message.point)}


def _point(message) -> dict:
    return {'position': _xyz(message)}


def _bool(message) -> dict:
    return {'active': bool(message.data)}


def _float32(message) -> dict:
    return {'value': float(message.data)}


CONVERTERS: dict[str, Callable[[object], dict]] = {
    'geometry_msgs/msg/PoseStamped': _pose_stamped,
    'nav_msgs/msg/Odometry': _odometry,
    'geometry_msgs/msg/PointStamped': _point_stamped,
    'geometry_msgs/msg/Point': _point,
    'std_msgs/msg/Bool': _bool,
    'std_msgs/msg/Float32': _float32,
}


@dataclass
class _TopicSamples:
    frame_id: str
    time: list[float] = field(default_factory=list)
    receive_time: list[float] = field(default_factory=list)
    stamp_sources: set[str] = field(default_factory=set)
    fields: dict[str, list] = field(default_factory=dict)


def _bag_sha1(bag_directory: Path) -> str:
    """SHA-1 over each .mcap file's SHA-1 digest, files taken in name order."""

    combined = hashlib.sha1()

    for path in sorted(bag_directory.glob('*.mcap'), key=lambda mcap_path: mcap_path.name):
        with path.open('rb') as file_handle:
            combined.update(hashlib.file_digest(file_handle, 'sha1').digest())

    return combined.hexdigest()


@contextlib.contextmanager
def _readable_bag(bag_directory: Path) -> Iterator[tuple[Path, bool, list[str]]]:
    """Yields (directory to read, whether it was reindexed, the reindex warnings).

    Never writes to bag_directory. A bag whose recorder was stopped uncleanly has no
    metadata.yaml; a reindexed copy reads.
    """

    if (bag_directory / BAG_METADATA_FILE).exists():
        yield bag_directory, False, []

        return

    with tempfile.TemporaryDirectory() as scratch:
        copy = Path(scratch) / bag_directory.name
        shutil.copytree(bag_directory, copy)

        try:
            result = subprocess.run(
                ['ros2', 'bag', 'reindex', str(copy), '-s', 'mcap'],
                capture_output=True,
                text=True,
                check=False,
            )
        except FileNotFoundError as exception:
            raise BagError(f'{bag_directory}: cannot run ros2 bag reindex: {exception}') from None

        if result.returncode != 0:
            raise BagError(
                f'{bag_directory}: ros2 bag reindex failed: {result.stderr.strip()[-500:]}',
            )

        yield copy, True, _reindex_warnings(result.stderr)


def _reindex_warnings(stderr: str) -> list[str]:
    """The warning and error lines of ``ros2 bag reindex``; a clean run logs only [INFO]."""

    return [line.strip() for line in stderr.splitlines() if '[WARN]' in line or '[ERROR]' in line]


def _header_stamp_ns(header) -> int | None:
    if header is None:
        return None

    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def _read_topics(
    bag_directory: Path,
    readable_directory: Path,
) -> tuple[dict[str, _TopicSamples], list[str], dict[str, float]]:
    """Returns the samples of every converted topic, the topics left unconverted, and the
    bag's declared span with the receive time of its last message.

    A bag that stops reading early without an error shows as a last message well before
    the declared end.
    """

    reader = rosbag2_py.SequentialReader()

    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(readable_directory), storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''),
        )
    except RuntimeError as exception:
        raise BagError(f'{bag_directory}: cannot open bag: {exception}') from None

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    wanted = TOPIC_STREAMS.keys() | FALLBACK_TOPIC_STREAMS.keys()
    converted_types = {
        topic: topic_type
        for topic, topic_type in topic_types.items()
        if topic in wanted and topic_type in CONVERTERS
    }
    message_classes = {
        topic: get_message(topic_type) for topic, topic_type in converted_types.items()
    }
    samples_by_topic: dict[str, _TopicSamples] = {}
    message_count = 0
    last_receive_ns = 0

    while reader.has_next():
        # rosbag2 and rclpy raise their own pybind exception types, not a shared base class.
        try:
            topic, data, receive_ns = reader.read_next()
        except Exception as exception:
            raise BagError(
                f'{bag_directory}: cannot read message {message_count + 1}: {exception}',
            ) from None

        message_count += 1
        last_receive_ns = max(last_receive_ns, receive_ns)

        if topic not in converted_types:
            continue

        try:
            message = deserialize_message(data, message_classes[topic])
        except Exception as exception:
            raise BagError(
                f'{bag_directory}: cannot deserialize message {message_count} on {topic}: '
                f'{exception}',
            ) from None

        header = getattr(message, 'header', None)
        samples = samples_by_topic.setdefault(
            topic,
            _TopicSamples(frame_id=header.frame_id if header is not None else ''),
        )
        seconds, stamp_source = message_time(_header_stamp_ns(header), receive_ns)
        samples.time.append(seconds)
        samples.receive_time.append(receive_ns / 1e9)
        samples.stamp_sources.add(stamp_source)

        for field_name, value in CONVERTERS[converted_types[topic]](message).items():
            samples.fields.setdefault(field_name, []).append(value)

    if message_count == 0:
        raise BagError(f'{bag_directory}: no messages')

    unconverted = sorted(topic for topic in topic_types if topic not in converted_types)
    bag_metadata = reader.get_metadata()
    span = {
        'last_message_s': last_receive_ns / 1e9,
        'bag_start_s': bag_metadata.starting_time.nanoseconds / 1e9,
        'bag_duration_s': bag_metadata.duration.nanoseconds / 1e9,
    }

    return samples_by_topic, unconverted, span


def _on_simulation_clock(samples: _TopicSamples) -> tuple[_TopicSamples, bool]:
    """Returns the samples timed on the bag's clock, and whether their header clock was foreign.

    The bag's receive times are simulation time; a header on another clock (for example the
    system clock) is replaced by the receive time.
    """

    if HEADER not in samples.stamp_sources:
        return samples, False

    if not header_clock_is_foreign(samples.time, samples.receive_time):
        return samples, False

    return replace(samples, time=list(samples.receive_time), stamp_sources={RECEIVE}), True


def _to_stream(name: str, samples: _TopicSamples) -> Stream:
    """Sorts by time (stable, so receive order breaks ties) and drops exact-duplicate stamps."""

    time = np.asarray(samples.time, dtype=float)
    order = np.argsort(time, kind='stable')
    sorted_time = time[order]
    keep = np.concatenate(([True], np.diff(sorted_time) != 0))
    kept = order[keep]

    if len(samples.stamp_sources) == 1:
        stamp_source = next(iter(samples.stamp_sources))
    else:
        stamp_source = MIXED_STAMP_SOURCE

    return Stream(
        name=name,
        frame_id=samples.frame_id,
        time=time[kept],
        receive_time=np.asarray(samples.receive_time, dtype=float)[kept],
        fields={
            field_name: np.asarray(values)[kept] for field_name, values in samples.fields.items()
        },
        stamp_source=stamp_source,
    )


def extract(bag_directory: Path) -> FlightRecord:
    """Reads one recorded flight into a validated flight record.

    A topic with no convertible messages yields no stream; topics in the bag that were not
    converted are listed in metadata['skipped_topics']. Topics whose header stamps were replaced
    by receive time are listed in metadata['foreign_clock_topics']. How the bag ended is kept in
    metadata['reindex_warnings'], ['last_message_s'], ['bag_start_s'] and ['bag_duration_s'].
    """

    bag_directory = Path(bag_directory)
    mcap_files = sorted(bag_directory.glob('*.mcap'))
    if not mcap_files:
        raise BagError(f'{bag_directory}: no .mcap files')

    if all(path.stat().st_size == 0 for path in mcap_files):
        raise BagError(f'{bag_directory}: no messages; every .mcap file is empty')

    with _readable_bag(bag_directory) as (readable_directory, reindexed, reindex_warnings):
        samples_by_topic, skipped_topics, span = _read_topics(bag_directory, readable_directory)

    streams: dict[str, Stream] = {}
    foreign_clock_topics: list[str] = []

    def add_stream(topic: str, name: str) -> None:
        samples, foreign_clock = _on_simulation_clock(samples_by_topic[topic])
        if foreign_clock:
            foreign_clock_topics.append(topic)

        streams[name] = _to_stream(name, samples)

    for topic, name in TOPIC_STREAMS.items():
        if topic in samples_by_topic:
            add_stream(topic, name)

    for topic, name in FALLBACK_TOPIC_STREAMS.items():
        if topic not in samples_by_topic:
            continue

        if name in streams:
            skipped_topics.append(topic)
        else:
            add_stream(topic, name)

    metadata: dict = {
        'bag_name': bag_directory.name,
        'bag_sha1': _bag_sha1(bag_directory),
        'extractor_version': EXTRACTOR_VERSION,
        'reindexed': reindexed,
        'reindex_warnings': reindex_warnings,
        **span,
    }
    run_file = bag_directory / RUN_FILE
    if run_file.exists():
        try:
            metadata['run'] = json.loads(run_file.read_text())
        except json.JSONDecodeError as exception:
            raise BagError(f'{run_file}: not valid JSON: {exception}') from None

    metadata['skipped_topics'] = sorted(skipped_topics)
    metadata['foreign_clock_topics'] = sorted(foreign_clock_topics)
    record = FlightRecord(streams=streams, metadata=metadata)
    validate(record)

    return record
