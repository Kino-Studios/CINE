import numpy as np
import pytest
from cine_eval.statistics import (
    hodges_lehmann,
    holm_adjust,
    minimum_pairs_for_alpha,
    paired_bootstrap_interval,
    wilcoxon_exact_p,
    wilson_interval,
)


def test_wilson_interval_for_8_of_10():
    lower, upper = wilson_interval(8, 10)

    assert lower == pytest.approx(0.4902, abs=5e-5)
    assert upper == pytest.approx(0.9433, abs=5e-5)


def test_wilson_interval_rejects_zero_trials():
    with pytest.raises(ValueError):
        wilson_interval(0, 0)


def test_minimum_pairs_for_alpha_005_is_6():
    assert minimum_pairs_for_alpha(0.05) == 6


def test_wilcoxon_exact_p_of_eight_same_sign_differences_is_2_over_256():
    differences = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])

    assert wilcoxon_exact_p(differences) == pytest.approx(2 / 256)


def test_wilcoxon_exact_p_drops_zero_differences_and_stays_exact():
    differences = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])

    assert wilcoxon_exact_p(differences) == pytest.approx(2 / 256)


def test_wilcoxon_exact_p_with_tied_magnitudes_uses_the_tied_rank_distribution():
    differences = np.array([1.0, 1.0, -1.0, 2.0, 2.0, 3.0, 3.0, 4.0])

    assert wilcoxon_exact_p(differences) == pytest.approx(8 / 256)


def test_wilcoxon_exact_p_of_all_zero_differences_is_one():
    assert wilcoxon_exact_p(np.array([0.0, 0.0, 0.0])) == 1.0


def test_hodges_lehmann_of_symmetric_differences_is_their_centre():
    assert hodges_lehmann(np.array([0.5, 1.5, 2.5, 3.5, 4.5])) == pytest.approx(2.5)


def test_hodges_lehmann_is_the_median_of_walsh_averages():
    # Walsh averages of [0, 1, 10]: 0, 0.5, 5, 1, 5.5, 10 -> median 3.0.
    assert hodges_lehmann(np.array([0.0, 1.0, 10.0])) == pytest.approx(3.0)


def test_bootstrap_interval_is_reproducible_with_the_same_seed():
    differences = np.array([0.1, -0.2, 0.4, 0.3, 0.05, 0.6, -0.1, 0.25])

    first = paired_bootstrap_interval(differences, resamples=2000, seed=7)
    second = paired_bootstrap_interval(differences, resamples=2000, seed=7)

    assert first == second


def test_bootstrap_interval_brackets_the_hodges_lehmann_estimate():
    differences = np.array([0.1, -0.2, 0.4, 0.3, 0.05, 0.6, -0.1, 0.25])

    lower, upper = paired_bootstrap_interval(differences, resamples=2000, seed=0)

    assert lower <= hodges_lehmann(differences) <= upper


def test_holm_adjust_matches_hand_example():
    assert holm_adjust([0.01, 0.04, 0.03]) == pytest.approx([0.03, 0.06, 0.06])


def test_holm_adjust_caps_at_one():
    assert holm_adjust([0.6, 0.9]) == pytest.approx([1.0, 1.0])
