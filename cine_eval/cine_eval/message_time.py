"""Which clock a recorded message's time comes from."""

from collections.abc import Sequence

import numpy as np

HEADER = 'header'
RECEIVE = 'receive'
# Transport latency is under a second, so a gap this large means the header uses another clock.
FOREIGN_CLOCK_THRESHOLD_S = 60.0


def message_time(header_stamp_ns: int | None, receive_ns: int) -> tuple[float, str]:
    """Returns (seconds, source): the header stamp, or receive time when the header is unfilled (0)."""

    if header_stamp_ns:
        return header_stamp_ns / 1e9, HEADER

    return receive_ns / 1e9, RECEIVE


def header_clock_is_foreign(
    header_seconds: Sequence[float],
    receive_seconds: Sequence[float],
) -> bool:
    """Whether a stream's header stamps run on a clock other than the bag's simulation clock."""

    header = np.asarray(header_seconds, dtype=float)
    receive = np.asarray(receive_seconds, dtype=float)
    gaps = np.abs(header - receive)

    return bool(np.median(gaps) > FOREIGN_CLOCK_THRESHOLD_S)
