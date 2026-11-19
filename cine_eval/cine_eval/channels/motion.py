"""Motion channel: speed, acceleration, jerk and speed-independent smoothness while filming."""

import numpy as np
from scipy.integrate import trapezoid

from cine_eval.channels.result import OK, ChannelResult, unchecked
from cine_eval.record import DRONE_TRUE, FRAMING_ACTIVE, FlightRecord
from cine_eval.settings import ScoringSettings
from cine_eval.timeline import (
    filming_mask,
    fixed_clock,
    sample,
    savgol_window_samples,
    smoothed_derivatives,
)

NAME = 'motion'
REQUIRED_STREAMS = (DRONE_TRUE, FRAMING_ACTIVE)


def log_dimensionless_jerk(jerk: np.ndarray, clock_hz: float, path_length_m: float) -> float:
    """Speed-invariant smoothness of one movement; higher (less negative) is smoother."""

    duration_s = (len(jerk) - 1) / clock_hz
    squared_jerk = np.sum(jerk**2, axis=1)
    jerk_energy = trapezoid(squared_jerk, dx=1.0 / clock_hz)

    return -float(np.log(duration_s**5 / path_length_m**2 * jerk_energy))


def spectral_arc_length(
    speed: np.ndarray,
    clock_hz: float,
    cutoff_hz: float,
    amplitude_threshold: float,
    padding_level: int,
) -> float:
    """SPARC (Balasubramanian et al. 2015) of a speed profile; closer to 0 is smoother."""

    nfft = 2 ** (int(np.ceil(np.log2(len(speed)))) + padding_level)
    frequency = np.arange(nfft) * clock_hz / nfft
    magnitude = np.abs(np.fft.fft(speed, nfft))
    magnitude = magnitude / magnitude.max()
    within_cutoff = frequency <= cutoff_hz
    frequency, magnitude = frequency[within_cutoff], magnitude[within_cutoff]
    above = np.nonzero(magnitude >= amplitude_threshold)[0]
    frequency = frequency[above[0] : above[-1] + 1]
    magnitude = magnitude[above[0] : above[-1] + 1]
    normalised_step = np.diff(frequency) / (frequency[-1] - frequency[0])

    return -float(np.sum(np.sqrt(normalised_step**2 + np.diff(magnitude) ** 2)))


def score_motion(record: FlightRecord, settings: ScoringSettings) -> ChannelResult:
    missing = [name for name in REQUIRED_STREAMS if not record.has(name)]
    if missing:
        return unchecked(NAME, f'missing streams: {", ".join(missing)}')

    drone = record.streams[DRONE_TRUE]
    clock = fixed_clock(drone.time[0], drone.time[-1], settings.clock_hz)
    position = sample(drone, 'position', clock)
    filming = filming_mask(record, clock) & np.all(np.isfinite(position), axis=1)
    if not filming.any():
        return unchecked(NAME, 'no filming time')

    segments = _filming_segments(filming)
    derivatives = [
        smoothed_derivatives(
            position[start:stop],
            settings.clock_hz,
            settings.savgol_window_s,
            settings.savgol_polyorder,
        )
        for start, stop in segments
    ]
    usable = [
        (segment, smoothed)
        for segment, smoothed in zip(segments, derivatives)
        if np.all(np.isfinite(smoothed[0]))
    ]
    if not usable:
        return unchecked(NAME, 'filming segments too short for smoothing')

    speed = np.concatenate([np.linalg.norm(smoothed[0], axis=1) for _, smoothed in usable])
    acceleration = np.concatenate([np.linalg.norm(smoothed[1], axis=1) for _, smoothed in usable])
    jerk = np.concatenate([np.linalg.norm(smoothed[2], axis=1) for _, smoothed in usable])
    longest_segment, longest_derivatives = max(usable, key=lambda item: item[0][1] - item[0][0])
    longest_start, longest_stop = longest_segment
    longest_count = longest_stop - longest_start
    summary = {
        'peak_speed_mps': float(speed.max()),
        'mean_speed_mps': float(speed.mean()),
        'peak_acceleration_mps2': float(acceleration.max()),
        'rms_acceleration_mps2': float(np.sqrt(np.mean(acceleration**2))),
        'peak_jerk_mps3': float(jerk.max()),
        'rms_jerk_mps3': float(np.sqrt(np.mean(jerk**2))),
        'filming_s': float(filming.sum() / settings.clock_hz),
        'savgol_window_samples': float(
            savgol_window_samples(
                longest_count,
                settings.clock_hz,
                settings.savgol_window_s,
                settings.savgol_polyorder,
            ),
        ),
        'smoothness_segment_s': float(longest_count / settings.clock_hz),
    }
    reason = _add_smoothness(
        summary,
        position[longest_start:longest_stop],
        longest_derivatives,
        settings,
    )
    series = {
        'time': clock,
        'speed_mps': _spread(usable, speed, len(clock)),
        'acceleration_mps2': _spread(usable, acceleration, len(clock)),
        'jerk_mps3': _spread(usable, jerk, len(clock)),
        'filming': filming,
    }

    return ChannelResult(
        name=NAME,
        status=OK,
        reason=reason,
        summary=summary,
        series=series,
        clock_hz=settings.clock_hz,
    )


def _add_smoothness(
    summary: dict[str, float],
    segment_position: np.ndarray,
    segment_derivatives: tuple[np.ndarray, np.ndarray, np.ndarray],
    settings: ScoringSettings,
) -> str:
    path_length_m = float(np.sum(np.linalg.norm(np.diff(segment_position, axis=0), axis=1)))
    if path_length_m <= 0.0:
        return 'longest filming segment has no path length: smoothness measures omitted'

    velocity, _, jerk = segment_derivatives
    ldlj = log_dimensionless_jerk(jerk, settings.clock_hz, path_length_m)
    sparc = spectral_arc_length(
        np.linalg.norm(velocity, axis=1),
        settings.clock_hz,
        settings.sparc_cutoff_hz,
        settings.sparc_amplitude_threshold,
        settings.sparc_padding_level,
    )
    if not (np.isfinite(ldlj) and np.isfinite(sparc)):
        return 'smoothness measures not finite on the longest filming segment: omitted'

    summary['log_dimensionless_jerk'] = ldlj
    summary['sparc'] = sparc

    return ''


def _filming_segments(filming: np.ndarray) -> list[tuple[int, int]]:
    """Half-open index ranges of each contiguous run of True."""

    edges = np.diff(np.concatenate([[0], filming.astype(int), [0]]))
    starts = np.nonzero(edges == 1)[0]
    stops = np.nonzero(edges == -1)[0]

    return list(zip(starts.tolist(), stops.tolist()))


def _spread(usable: list, values: np.ndarray, length: int) -> np.ndarray:
    """Place per-segment values back on the full clock; NaN outside usable segments."""

    spread = np.full(length, np.nan)
    offset = 0

    for (start, stop), _ in usable:
        spread[start:stop] = values[offset : offset + stop - start]
        offset += stop - start

    return spread
