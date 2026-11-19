"""Freshness channel: how old each stream's data was when the scorer's bag received it."""

import numpy as np

from cine_eval.channels.result import OK, ChannelResult, unchecked
from cine_eval.record import FlightRecord
from cine_eval.settings import ScoringSettings

NAME = 'freshness'
HEADER = 'header'


def score_freshness(record: FlightRecord, settings: ScoringSettings) -> ChannelResult:
    header_streams = [s for s in record.streams.values() if s.stamp_source == HEADER]
    receipt_stamped = [s.name for s in record.streams.values() if s.stamp_source != HEADER]
    note = f'unchecked, stamped on receipt: {", ".join(receipt_stamped)}' if receipt_stamped else ''

    if not header_streams:
        return unchecked(NAME, '; '.join(['no header-stamped stream', note]).strip('; '))

    summary: dict[str, float] = {}
    worst_ages: list[float] = []

    for stream in header_streams:
        if len(stream.time) == 0:
            continue

        age_s = stream.receive_time - stream.time
        summary[f'{stream.name}.median_age_s'] = float(np.median(age_s))
        summary[f'{stream.name}.max_age_s'] = float(age_s.max())
        worst_ages.append(float(age_s.max()))

    if not worst_ages:
        return unchecked(NAME, 'header-stamped streams are empty')

    summary['max_age_s'] = max(worst_ages)

    return ChannelResult(
        name=NAME,
        status=OK,
        reason=note,
        summary=summary,
        series={},
        clock_hz=0.0,
    )
