"""Shared clock, filming window and smoothed derivatives for the scoring channels."""

from dataclasses import dataclass

import numpy as np
from scipy.signal import savgol_filter

from cine_eval.record import FRAMING_ACTIVE, FlightRecord, Stream


def fixed_clock(start: float, end: float, clock_hz: float) -> np.ndarray:
    """Evenly spaced instants from ``start`` through ``end`` (inclusive when it fits)."""

    count = int(np.floor((end - start) * clock_hz + 1e-9)) + 1

    return start + np.arange(count) / clock_hz


@dataclass(frozen=True)
class SampleGap:
    """The longest stretch of a window with no source sample."""

    length_s: float
    start_s: float


def sample(
    stream: Stream,
    field: str,
    clock: np.ndarray,
    max_gap_s: float | None = None,
) -> np.ndarray:
    """Linear interpolation of each column; NaN outside the stream's span.

    With ``max_gap_s``, instants strictly between two source samples further apart than
    that are NaN too, so a dropout is never filled with a straight line.
    """

    values = np.asarray(stream.fields[field], dtype=float)
    if values.ndim == 1:
        sampled = np.interp(clock, stream.time, values, left=np.nan, right=np.nan)
    else:
        columns = [
            np.interp(clock, stream.time, values[:, column], left=np.nan, right=np.nan)
            for column in range(values.shape[1])
        ]
        sampled = np.stack(columns, axis=1)

    if max_gap_s is None:
        return sampled

    sampled[_inside_long_gap(stream.time, clock, max_gap_s)] = np.nan

    return sampled


def largest_gap(time: np.ndarray, start: float, end: float) -> SampleGap:
    """Longest wait between samples over ``[start, end]``, the window edges included.

    The samples just outside the window count, because interpolation bridges from them.
    """

    time = np.asarray(time, dtype=float)
    if len(time) == 0:
        return SampleGap(length_s=float(end - start), start_s=float(start))

    before_index = int(np.searchsorted(time, start, side='right')) - 1
    after_index = int(np.searchsorted(time, end, side='left'))
    points = time[max(before_index, 0) : min(after_index, len(time) - 1) + 1]

    if before_index < 0:
        points = np.concatenate([[start], points])

    if after_index >= len(time):
        points = np.concatenate([points, [end]])

    if len(points) < 2:
        return SampleGap(length_s=0.0, start_s=float(start))

    waits = np.diff(points)
    longest_index = int(np.argmax(waits))

    return SampleGap(length_s=float(waits[longest_index]), start_s=float(points[longest_index]))


def _inside_long_gap(time: np.ndarray, clock: np.ndarray, max_gap_s: float) -> np.ndarray:
    next_index = np.searchsorted(time, clock, side='right')
    between_samples = (next_index > 0) & (next_index < len(time))
    previous_time = time[np.clip(next_index - 1, 0, len(time) - 1)]
    next_time = time[np.clip(next_index, 0, len(time) - 1)]

    return (
        between_samples
        & (next_time - previous_time > max_gap_s)
        & (clock > previous_time)
        & (clock < next_time)
    )


def sample_hold(stream: Stream, field: str, clock: np.ndarray) -> np.ndarray:
    """Last value at or before each instant; False (bool fields) or NaN before the first sample."""

    values = np.asarray(stream.fields[field])
    last_index = np.searchsorted(stream.time, clock, side='right') - 1
    before_first = last_index < 0
    held = values[np.maximum(last_index, 0)]

    if values.dtype == bool:
        held[before_first] = False

        return held

    held = held.astype(float)
    held[before_first] = np.nan

    return held


def filming_mask(record: FlightRecord, clock: np.ndarray) -> np.ndarray | None:
    """True where the framing cue is active; None when the record has no cue."""

    if not record.has(FRAMING_ACTIVE):
        return None

    return sample_hold(record.streams[FRAMING_ACTIVE], 'active', clock)


def savgol_window_samples(
    sample_count: int,
    clock_hz: float,
    window_s: float,
    polyorder: int,
) -> int | None:
    """Odd filter length used for a segment of ``sample_count`` samples; None when it is too short."""

    window_length = int(round(window_s * clock_hz))
    if window_length % 2 == 0:
        window_length += 1

    smallest_window = polyorder + 2
    if smallest_window % 2 == 0:
        smallest_window += 1

    window_length = max(window_length, smallest_window)
    longest_window = sample_count - (1 - sample_count % 2)
    if longest_window < smallest_window:
        return None

    return min(window_length, longest_window)


def smoothed_derivatives(
    position: np.ndarray,
    clock_hz: float,
    window_s: float,
    polyorder: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Velocity, acceleration and jerk from a Savitzky-Golay fit of ``position``."""

    window_length = savgol_window_samples(len(position), clock_hz, window_s, polyorder)
    if window_length is None:
        empty = np.full(np.shape(position), np.nan)

        return empty, empty.copy(), empty.copy()

    delta = 1.0 / clock_hz
    derivatives = [
        savgol_filter(position, window_length, polyorder, deriv=order, delta=delta, axis=0)
        for order in (1, 2, 3)
    ]

    return derivatives[0], derivatives[1], derivatives[2]
