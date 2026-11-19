"""The value every scoring channel returns."""

from dataclasses import dataclass, field

import numpy as np

OK = 'ok'
UNCHECKED = 'unchecked'


@dataclass(frozen=True)
class ChannelResult:
    name: str
    status: str
    reason: str
    summary: dict[str, float] = field(default_factory=dict)
    series: dict[str, np.ndarray] = field(default_factory=dict)
    clock_hz: float = 0.0


def unchecked(name: str, reason: str) -> ChannelResult:
    return ChannelResult(
        name=name,
        status=UNCHECKED,
        reason=reason,
        summary={},
        series={},
        clock_hz=0.0,
    )
