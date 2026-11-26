"""Small-sample statistics for paired comparisons of scored flights."""

import math

import numpy as np
from scipy import stats

WILSON_Z_95 = 1.959964
BOOTSTRAP_CHUNK = 500


def wilson_interval(successes: int, trials: int, z: float = WILSON_Z_95) -> tuple[float, float]:
    """Wilson score interval for a binomial rate."""

    if trials <= 0:
        raise ValueError('wilson_interval needs at least one trial')

    share = successes / trials
    denominator = 1 + z**2 / trials
    centre = (share + z**2 / (2 * trials)) / denominator
    half_width = z * math.sqrt(share * (1 - share) / trials + z**2 / (4 * trials**2)) / denominator

    return (max(0.0, centre - half_width), min(1.0, centre + half_width))


def wilcoxon_exact_p(differences: np.ndarray) -> float:
    """Two-sided exact Wilcoxon signed-rank p, zero differences dropped (``zero_method="wilcox"``).

    scipy 1.11.4's ``method="exact"`` mishandles zeros and silently uses the untied null on tied
    magnitudes, so scipy runs only when no magnitudes tie; otherwise the tied null is built.
    """

    nonzero = np.asarray(differences, dtype=float)
    nonzero = nonzero[nonzero != 0]
    if nonzero.size == 0:
        return 1.0

    magnitudes = np.abs(nonzero)
    if np.unique(magnitudes).size == magnitudes.size:
        return float(stats.wilcoxon(nonzero, method='exact').pvalue)

    return _tied_exact_p(nonzero)


def _tied_exact_p(nonzero: np.ndarray) -> float:
    # Doubled mid-ranks are integers, so the positive-rank-sum null builds by convolution.
    doubled_ranks = np.rint(2 * stats.rankdata(np.abs(nonzero))).astype(int)
    probabilities = np.array([1.0])

    for weight in doubled_ranks:
        shifted = np.zeros(probabilities.size + weight)
        shifted[: probabilities.size] += probabilities
        shifted[weight:] += probabilities
        probabilities = shifted / 2

    statistic = int(doubled_ranks[nonzero > 0].sum())
    lower_tail = probabilities[: statistic + 1].sum()
    upper_tail = probabilities[statistic:].sum()

    return float(min(1.0, 2 * min(lower_tail, upper_tail)))


def minimum_pairs_for_alpha(alpha: float) -> int:
    """Fewest pairs whose best possible exact two-sided p (all one sign) reaches ``alpha``."""

    if alpha <= 0:
        raise ValueError('alpha must be positive')

    pairs = 1
    while 2 / 2**pairs > alpha:
        pairs += 1

    return pairs


def hodges_lehmann(differences: np.ndarray) -> float:
    """Median of the Walsh averages ``(d_i + d_j) / 2`` for ``i <= j``."""

    values = np.asarray(differences, dtype=float)
    if values.size == 0:
        raise ValueError('hodges_lehmann needs at least one difference')

    rows, columns = np.triu_indices(values.size)

    return float(np.median((values[rows] + values[columns]) / 2))


def paired_bootstrap_interval(
    differences: np.ndarray,
    resamples: int = 10000,
    seed: int = 0,
    level: float = 0.95,
) -> tuple[float, float]:
    """Percentile interval of the Hodges-Lehmann estimate over pairs resampled with replacement."""

    values = np.asarray(differences, dtype=float)
    if values.size == 0:
        raise ValueError('paired_bootstrap_interval needs at least one difference')

    generator = np.random.default_rng(seed)
    resample_indices = generator.integers(0, values.size, size=(resamples, values.size))
    rows, columns = np.triu_indices(values.size)
    estimates = np.empty(resamples)

    for start in range(0, resamples, BOOTSTRAP_CHUNK):
        samples = values[resample_indices[start : start + BOOTSTRAP_CHUNK]]
        walsh_averages = (samples[:, rows] + samples[:, columns]) / 2
        estimates[start : start + BOOTSTRAP_CHUNK] = np.median(walsh_averages, axis=1)

    tail = (1 - level) / 2
    lower, upper = np.quantile(estimates, [tail, 1 - tail])

    return (float(lower), float(upper))


def holm_adjust(p_values: list[float]) -> list[float]:
    """Holm step-down adjusted p values, in the input order."""

    count = len(p_values)
    adjusted = [0.0] * count
    running_maximum = 0.0

    for rank, index in enumerate(np.argsort(p_values, kind='stable')):
        running_maximum = max(running_maximum, min(1.0, (count - rank) * p_values[index]))
        adjusted[index] = running_maximum

    return adjusted
