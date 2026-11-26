"""Cut a short window out of a recorded bag into a new MCAP bag, for the extraction tests.

Runs inside cine-gazebo-sim:dev. The source bag is never written: an unclosed bag (no
metadata.yaml) is copied to a temporary directory and reindexed there. With --record, the slice
is then extracted and saved as a flight record, so the committed record always matches the
committed slice.

    python cine_eval/test/fixtures/make_fixture.py /bags/<run> \
        cine_eval/test/fixtures/short_flight --start-s 26.0 --duration-s 6.0 \
        --record cine_eval/test/fixtures/short_flight_record
"""

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import rosbag2_py

NANOSECONDS_PER_SECOND = 1_000_000_000


def _open_reader(bag_directory: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )

    return reader


def _first_receive_ns(bag_directory: Path) -> int:
    reader = _open_reader(bag_directory)
    if not reader.has_next():
        raise SystemExit(f'{bag_directory}: no messages')

    _, _, receive_ns = reader.read_next()

    return receive_ns


def slice_bag(source: Path, output: Path, start_s: float, duration_s: float) -> int:
    """Copies messages in [first + start, first + start + duration] to a new bag and returns
    how many it copied."""

    first_ns = _first_receive_ns(source)
    window_start_ns = first_ns + round(start_s * NANOSECONDS_PER_SECOND)
    window_end_ns = window_start_ns + round(duration_s * NANOSECONDS_PER_SECOND)
    reader = _open_reader(source)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )

    for topic_metadata in reader.get_all_topics_and_types():
        writer.create_topic(topic_metadata)

    written = 0

    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        if window_start_ns <= receive_ns <= window_end_ns:
            writer.write(topic, data, receive_ns)
            written += 1

    writer.close()

    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('source', type=Path, help='recorded bag directory')
    parser.add_argument('output', type=Path, help='new bag directory; must not exist')
    parser.add_argument(
        '--start-s',
        type=float,
        default=0.0,
        help='window start after the first receive time',
    )
    parser.add_argument('--duration-s', type=float, default=6.0, help='window length')
    parser.add_argument('--record', type=Path, help='also extract the slice into this directory')
    arguments = parser.parse_args()
    if arguments.output.exists():
        parser.error(f'{arguments.output} exists; remove it first')

    with tempfile.TemporaryDirectory() as scratch:
        source = arguments.source
        if not (source / 'metadata.yaml').exists():
            source = Path(scratch) / arguments.source.name
            shutil.copytree(arguments.source, source)
            subprocess.run(['ros2', 'bag', 'reindex', str(source), '-s', 'mcap'], check=True)

        written = slice_bag(source, arguments.output, arguments.start_s, arguments.duration_s)

    print(f'wrote {written} messages to {arguments.output}')

    if arguments.record:
        from cine_eval.extract import extract
        from cine_eval.record import save

        save(extract(arguments.output), arguments.record)
        print(f'wrote flight record to {arguments.record}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
