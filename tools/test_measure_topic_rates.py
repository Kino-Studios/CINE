import measure_topic_rates
import pytest


def test_rate_counts_intervals_between_first_and_last_message():
    assert measure_topic_rates.rate_from_arrivals([10.0, 10.5, 11.0]) == pytest.approx(2.0)


def test_rate_is_unknown_from_a_single_message():
    assert measure_topic_rates.rate_from_arrivals([10.0]) is None


def test_rate_is_unknown_without_messages():
    assert measure_topic_rates.rate_from_arrivals([]) is None


def test_report_gives_each_topic_its_message_count_and_rate():
    report = measure_topic_rates.build_report({'/camera/image_raw': [0.0, 0.25, 0.5]}, 5.0)

    assert report['topics']['/camera/image_raw'] == {'messages': 3, 'rate_hz': 4.0}


def test_report_keeps_a_silent_topic_with_zero_messages():
    report = measure_topic_rates.build_report({'/depth/image_raw': []}, 5.0)

    assert report['topics']['/depth/image_raw'] == {'messages': 0, 'rate_hz': None}


def test_topic_arguments_pair_each_topic_with_its_message_type():
    topic_types = measure_topic_rates.parse_topic_types(['/camera/image_raw=sensor_msgs/msg/Image'])

    assert topic_types == {'/camera/image_raw': 'sensor_msgs/msg/Image'}


def test_topic_argument_without_a_message_type_is_rejected():
    with pytest.raises(ValueError, match='/camera/image_raw'):
        measure_topic_rates.parse_topic_types(['/camera/image_raw'])
