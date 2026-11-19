"""Scoring and gate settings, read from the JSON files in ``cine_config``."""

from dataclasses import asdict, dataclass, replace

from cine_config.config_loader import ConfigLoader

from cine_config import load


@dataclass(frozen=True)
class ScoringSettings:
    clock_hz: float
    max_pose_gap_s: float
    focus_tolerance_m: float
    savgol_window_s: float
    savgol_polyorder: int
    sparc_cutoff_hz: float
    sparc_amplitude_threshold: float
    sparc_padding_level: int
    airframe_radius_m: float
    airframe_half_height_m: float
    subject_radius_m: float
    subject_height_m: float
    airborne_height_m: float
    detection_stale_s: float

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class GateThresholds:
    provisional: bool
    min_clearance_m: float
    min_subject_clearance_m: float
    min_focus_held_share: float
    speed_cap_margin: float
    max_setpoint_jump_m: float
    max_data_age_s: float
    min_real_time_factor: float

    def as_dict(self) -> dict:
        return asdict(self)


def load_scoring_settings(overrides: dict | None = None) -> ScoringSettings:
    """Reads the committed scoring values; ``overrides`` replaces named attributes."""

    loader = ConfigLoader()
    resampling = loader.get_section('eval_scoring', 'resampling')
    focus = loader.get_section('eval_scoring', 'focus')
    motion = loader.get_section('eval_scoring', 'motion')
    clearance = loader.get_section('eval_scoring', 'clearance')
    estimate = loader.get_section('eval_scoring', 'estimate')
    settings = ScoringSettings(
        clock_hz=resampling['clock_hz'],
        max_pose_gap_s=resampling['max_pose_gap_s'],
        focus_tolerance_m=focus['tolerance_m'],
        savgol_window_s=motion['savgol_window_s'],
        savgol_polyorder=motion['savgol_polyorder'],
        sparc_cutoff_hz=motion['sparc_cutoff_hz'],
        sparc_amplitude_threshold=motion['sparc_amplitude_threshold'],
        sparc_padding_level=motion['sparc_padding_level'],
        airframe_radius_m=clearance['airframe_radius_m'],
        airframe_half_height_m=clearance['airframe_half_height_m'],
        subject_radius_m=clearance['subject_radius_m'],
        subject_height_m=clearance['subject_height_m'],
        airborne_height_m=clearance['airborne_height_m'],
        detection_stale_s=estimate['detection_stale_s'],
    )

    if not overrides:
        return settings

    unknown = sorted(set(overrides) - set(settings.as_dict()))
    if unknown:
        raise ValueError(f'unknown scoring settings: {", ".join(unknown)}')

    return replace(settings, **overrides)


def load_gate_thresholds() -> GateThresholds:
    gate = load('eval/gate')

    return GateThresholds(provisional=gate['provisional'], **gate['thresholds'])
