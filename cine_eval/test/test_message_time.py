from cine_eval.message_time import header_clock_is_foreign, message_time


def test_zero_stamp_falls_back_to_receive_time():
    assert message_time(0, 2_250_000_000) == (2.25, 'receive')


def test_missing_stamp_falls_back_to_receive_time():
    assert message_time(None, 2_250_000_000) == (2.25, 'receive')


def test_header_stamp_is_used_when_set():
    assert message_time(1_500_000_000, 1_600_000_000) == (1.5, 'header')


def test_header_clock_far_from_receive_time_is_foreign():
    assert header_clock_is_foreign([1.795e9 + 26.0, 1.795e9 + 27.0], [26.0, 27.0])


def test_header_clock_slightly_behind_receive_time_is_kept():
    assert not header_clock_is_foreign([25.98, 26.98], [26.0, 27.0])
