from pathlib import Path

import numpy as np
import pytest
from cine_eval.legacy_csv import record_from_legacy_csv
from cine_eval.record import (
    DRONE_TRUE,
    FOCUS_TARGET,
    FRAMING_ACTIVE,
    SUBJECT_TRUE,
    RecordError,
    save,
)

HEADER = (
    'mode,time,drone_x,drone_y,drone_z,object_x,object_y,object_z,'
    'distance,target_distance,error,abs_error,within_tolerance'
)
THREE_ROWS = [
    'baseline,8.0,1.0,2.0,3.0,10.0,20.0,30.0,2.5,1.75,0.75,0.75,0',
    'baseline,8.5,1.5,2.5,3.5,10.0,20.0,30.0,2.0,1.75,0.25,0.25,0',
    'baseline,9.0,2.0,3.0,4.0,10.0,20.0,30.0,1.8,1.75,0.05,0.05,1',
]


def _write_csv(tmp_path: Path, rows: list[str], header: str = HEADER) -> Path:
    path = tmp_path / 'distance_baseline_20260106_000000.csv'
    path.write_text('\n'.join([header, *rows]) + '\n')

    return path


def test_legacy_rows_become_a_record_with_target_stream(tmp_path):
    record = record_from_legacy_csv(_write_csv(tmp_path, THREE_ROWS))

    assert record.streams[DRONE_TRUE].time.tolist() == [8.0, 8.5, 9.0]
    assert record.streams[DRONE_TRUE].fields['position'][1].tolist() == [1.5, 2.5, 3.5]
    assert record.streams[SUBJECT_TRUE].fields['position'][0].tolist() == [10.0, 20.0, 30.0]
    assert record.streams[FOCUS_TARGET].fields['distance_m'].tolist() == [1.75, 1.75, 1.75]
    assert record.streams[FRAMING_ACTIVE].fields['active'].tolist() == [True, True, True]


def test_legacy_logger_share_is_kept_for_the_record(tmp_path):
    path = _write_csv(tmp_path, THREE_ROWS)

    record = record_from_legacy_csv(path)

    assert record.metadata['legacy'] is True
    assert record.metadata['source_csv'] == path.name
    assert record.metadata['logger_within_tolerance_share'] == pytest.approx(1 / 3)


def test_file_without_subject_columns_is_rejected(tmp_path):
    path = _write_csv(
        tmp_path,
        ['0.007,0.0,0.0,0.0,0.0,0.0,0.0,0.0,False'],
        header='time,odom_x,odom_y,odom_z,odom_vx,odom_vy,odom_vz,odom_speed,cinematic_framing_active',
    )

    with pytest.raises(RecordError, match='no subject position'):
        record_from_legacy_csv(path)


def test_unsorted_and_repeated_stamps_are_sorted_and_deduplicated(tmp_path):
    rows = [THREE_ROWS[2], THREE_ROWS[0], THREE_ROWS[0], THREE_ROWS[1]]

    record = record_from_legacy_csv(_write_csv(tmp_path, rows))

    assert record.streams[DRONE_TRUE].time.tolist() == [8.0, 8.5, 9.0]


def test_converted_record_passes_the_record_contract(tmp_path):
    record = record_from_legacy_csv(_write_csv(tmp_path, THREE_ROWS))

    save(record, tmp_path / 'run')

    assert np.load(tmp_path / 'run' / 'record.npz')['drone_true/time'].tolist() == [8.0, 8.5, 9.0]


def test_legacy_record_states_estimated_logger_threshold(tmp_path):
    record = record_from_legacy_csv(_write_csv(tmp_path, THREE_ROWS))

    assert record.metadata['logger_within_tolerance_threshold_m_estimated'] == 0.15
    assert 'inferred' in record.metadata['logger_within_tolerance_threshold_note']


def test_legacy_record_marks_framing_active_as_assumed(tmp_path):
    record = record_from_legacy_csv(_write_csv(tmp_path, THREE_ROWS))

    assert record.metadata['framing_active_assumed'] is True
