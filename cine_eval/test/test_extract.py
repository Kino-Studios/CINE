import json
import os
import shutil
from pathlib import Path

import numpy as np
import pytest

rosbag2_py = pytest.importorskip('rosbag2_py', reason='extraction runs inside cine-gazebo-sim:dev')

import cine_eval.extract as extract_module  # noqa: E402
from cine_eval.channels.focus import ANCHOR_DISTANCE_FIELD  # noqa: E402
from cine_eval.extract import BagError, _to_stream, _TopicSamples, extract  # noqa: E402
from cine_eval.record import (  # noqa: E402
    DRONE_ESTIMATE,
    DRONE_TRUE,
    FOCUS_TARGET,
    FRAMING_ACTIVE,
    SUBJECT_TRUE,
)
from std_msgs.msg import Float32  # noqa: E402

FIXTURE_BAG = Path(__file__).parent / 'fixtures' / 'short_flight'
RealSequentialReader = rosbag2_py.SequentialReader


def test_extract_fixture_has_truth_and_estimate_streams():
    record = extract(FIXTURE_BAG)

    assert record.has(DRONE_TRUE, SUBJECT_TRUE, DRONE_ESTIMATE)
    assert not record.has(FRAMING_ACTIVE)
    assert record.streams[DRONE_TRUE].frame_id != ''
    assert np.all(np.diff(record.streams[DRONE_TRUE].time) > 0)
    assert np.all(np.diff(record.streams[DRONE_ESTIMATE].time) > 0)


def test_extract_fixture_keeps_odometry_velocity_and_header_stamps():
    record = extract(FIXTURE_BAG)

    assert record.streams[DRONE_ESTIMATE].fields['velocity'].shape == (60, 3)
    assert record.streams[DRONE_TRUE].fields['orientation'].shape[1] == 4
    assert record.streams[DRONE_TRUE].stamp_source == 'header'


def test_extract_fixture_estimate_uses_receive_time_when_header_clock_is_foreign():
    record = extract(FIXTURE_BAG)
    estimate_time = record.streams[DRONE_ESTIMATE].time
    truth_time = record.streams[DRONE_TRUE].time

    assert record.streams[DRONE_ESTIMATE].stamp_source == 'receive'
    assert '/mavros/local_position/odom' in record.metadata['foreign_clock_topics']
    assert estimate_time[0] < truth_time[-1]
    assert truth_time[0] < estimate_time[-1]


def test_extract_fixture_keeps_header_time_for_streams_on_the_simulation_clock():
    record = extract(FIXTURE_BAG)

    assert '/gazebo/drone_pose' not in record.metadata['foreign_clock_topics']


def test_extract_records_provenance_and_skipped_topics():
    record = extract(FIXTURE_BAG)

    assert record.metadata['bag_name'] == 'short_flight'
    assert record.metadata['extractor_version'] == '1'
    assert record.metadata['reindexed'] is False
    assert len(record.metadata['bag_sha1']) == 40
    assert 'run' not in record.metadata
    assert record.metadata['skipped_topics'] == ['/imu_bridge/data_raw', '/mavros/imu/data_raw']


def test_extract_copies_run_json_into_metadata(tmp_path):
    bag = tmp_path / 'with_run'
    shutil.copytree(FIXTURE_BAG, bag)
    (bag / 'run.json').write_text(json.dumps({'seed': 7}))

    record = extract(bag)

    assert record.metadata['run'] == {'seed': 7}


def test_extract_lifts_the_run_frame_transforms_to_where_the_scorers_read_them(tmp_path):
    bag = tmp_path / 'with_transforms'
    shutil.copytree(FIXTURE_BAG, bag)
    identity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    (bag / 'run.json').write_text(json.dumps({'frame_transforms': {'map->cine_field': identity}}))

    record = extract(bag)

    assert record.metadata['frame_transforms'] == {'map->cine_field': identity}


def test_extract_unclosed_bag_reindexes_a_copy_and_leaves_original_untouched(tmp_path):
    bag = tmp_path / 'unclosed'
    shutil.copytree(FIXTURE_BAG, bag)
    (bag / 'metadata.yaml').unlink()
    listing_before = sorted(os.listdir(bag))

    record = extract(bag)

    assert record.metadata['reindexed'] is True
    assert sorted(os.listdir(bag)) == listing_before


def test_stream_is_sorted_by_time_and_keeps_the_first_received_of_duplicate_stamps():
    samples = _TopicSamples(
        frame_id='cine_field',
        time=[2.0, 1.0, 2.0],
        receive_time=[2.1, 2.2, 2.3],
        stamp_sources={'header'},
        fields={'position': [[2.0, 0.0, 0.0], [1.0, 0.0, 0.0], [9.0, 0.0, 0.0]]},
    )

    stream = _to_stream(DRONE_TRUE, samples)

    assert stream.time.tolist() == [1.0, 2.0]
    assert stream.receive_time.tolist() == [2.2, 2.1]
    assert stream.fields['position'].tolist() == [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]


def test_receive_stamped_stream_keeps_the_last_message_of_a_shared_clock_tick():
    # The generator publishes a stale focus distance and its replacement 3 ms apart, inside one
    # tick of the simulator clock; the replacement is the value that held afterwards.
    samples = _TopicSamples(
        frame_id='',
        time=[37.4, 37.4, 37.8],
        receive_time=[37.4, 37.4, 37.8],
        stamp_sources={'receive'},
        fields={'value': [6.161, 1.0, 1.0]},
    )

    stream = _to_stream('focus_target', samples)

    assert stream.fields['value'].tolist() == [1.0, 1.0]


def test_extract_zero_byte_bag_raises_bag_error(tmp_path):
    bag = tmp_path / 'empty'
    bag.mkdir()
    (bag / 'x_0.mcap').write_bytes(b'')

    with pytest.raises(BagError, match='no messages'):
        extract(bag)


def _bag_with_one_unreadable_pose(bag: Path) -> Path:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=0,
            name='/gazebo/drone_pose',
            type='geometry_msgs/msg/PoseStamped',
            serialization_format='cdr',
        ),
    )
    writer.write('/gazebo/drone_pose', b'not a pose', 1_000_000_000)
    del writer

    return bag


class _ReaderFailingOnRead:
    """The real reader, except that reading a message fails as a corrupt chunk would."""

    def __init__(self):
        self._reader = RealSequentialReader()

    def __getattr__(self, name):
        return getattr(self._reader, name)

    def read_next(self):
        raise RuntimeError('chunk CRC mismatch')


def test_extract_message_that_cannot_be_deserialized_raises_bag_error(tmp_path):
    bag = _bag_with_one_unreadable_pose(tmp_path / 'garbage')

    with pytest.raises(BagError, match='cannot deserialize message 1 on /gazebo/drone_pose'):
        extract(bag)


def test_extract_message_that_cannot_be_read_raises_bag_error(monkeypatch):
    monkeypatch.setattr(rosbag2_py, 'SequentialReader', _ReaderFailingOnRead)

    with pytest.raises(BagError, match='cannot read message 1: chunk CRC mismatch'):
        extract(FIXTURE_BAG)


def test_extract_records_the_declared_bag_span_and_the_last_message_time():
    metadata = extract(FIXTURE_BAG).metadata

    assert metadata['bag_start_s'] == 26.02
    assert metadata['bag_duration_s'] == 6.0
    assert metadata['last_message_s'] == 32.02


def test_extract_of_a_closed_bag_records_no_reindex_warnings():
    assert extract(FIXTURE_BAG).metadata['reindex_warnings'] == []


def test_extract_of_a_cleanly_reindexed_bag_records_no_reindex_warnings(tmp_path):
    bag = tmp_path / 'unclosed'
    shutil.copytree(FIXTURE_BAG, bag)
    (bag / 'metadata.yaml').unlink()

    assert extract(bag).metadata['reindex_warnings'] == []


def test_reindex_warnings_keep_only_warning_and_error_lines():
    stderr = '[INFO] [1] Beginning reindexing\n[WARN] [2] chunk 3 truncated\n[ERROR] [3] bad\n'

    assert extract_module._reindex_warnings(stderr) == [
        '[WARN] [2] chunk 3 truncated',
        '[ERROR] [3] bad',
    ]


def test_anchor_focus_distance_becomes_the_field_the_focus_channel_reads():
    assert extract_module.TOPIC_STREAMS['/cinematography/focus_target_distance'] == FOCUS_TARGET

    fields = extract_module.CONVERTERS['std_msgs/msg/Float32'](Float32(data=3.0))

    assert fields[ANCHOR_DISTANCE_FIELD] == pytest.approx(3.0)
