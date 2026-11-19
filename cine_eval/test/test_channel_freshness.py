import numpy as np
import pytest
from cine_eval.channels.freshness import score_freshness
from cine_eval.record import FlightRecord, Stream
from cine_eval.settings import load_scoring_settings

TIME = np.arange(0.0, 5.0, 0.1)
SETTINGS = load_scoring_settings()


def _stream(name, delay_s, stamp_source='header'):
    return Stream(
        name=name,
        frame_id='map',
        time=TIME,
        receive_time=TIME + delay_s,
        fields={},
        stamp_source=stamp_source,
    )


def _record(*streams):
    return FlightRecord(streams={stream.name: stream for stream in streams})


def test_freshness_reports_age_between_stamp_and_receipt():
    result = score_freshness(_record(_stream('subject_estimate', 0.08)), SETTINGS)

    assert result.summary['subject_estimate.median_age_s'] == pytest.approx(0.08)
    assert result.summary['subject_estimate.max_age_s'] == pytest.approx(0.08)
    assert result.summary['max_age_s'] == pytest.approx(0.08)


def test_overall_max_age_is_the_worst_stream():
    record = _record(_stream('drone_true', 0.02), _stream('subject_estimate', 0.3))

    result = score_freshness(record, SETTINGS)

    assert result.summary['max_age_s'] == pytest.approx(0.3)


def test_freshness_skips_streams_stamped_on_receipt():
    record = _record(_stream('drone_true', 0.05), _stream('detections', 0.0, 'receive'))

    result = score_freshness(record, SETTINGS)

    assert 'detections.median_age_s' not in result.summary
    assert 'detections' in result.reason


def test_freshness_with_only_receipt_stamped_streams_is_unchecked():
    result = score_freshness(_record(_stream('detections', 0.0, 'receive')), SETTINGS)

    assert result.status == 'unchecked'
    assert 'detections' in result.reason
