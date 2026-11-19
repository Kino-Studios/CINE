# Evaluation Harness (Plan 1 of 3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `cine_eval`, the package that turns a Gazebo bag into a flight record, scores it,
applies pass/fail rules, compares arms, builds the results table and draws the flight, all run
through `cine eval` / `cine viz` inside the simulator image.

**Architecture:** A pipeline of stages that each read only the previous stage's files:
bag → `record.npz` → `score.json` → verdict / comparison / standings, with the viewer reading
`record.npz` and `score.json`. Only `extract.py` imports ROS; everything else is numpy/scipy and
tested by bare `pytest`. The host CLI only builds a `docker run` command.

**Tech Stack:** Python 3.12, numpy 1.26.4, scipy 1.11.4, matplotlib (3.6.3 in the image, 3.10.6 in
WSL; use only APIs present in both), `rosbag2_py` (ROS Jazzy, image only), typer 0.29 (host CLI),
`tomllib` (stdlib).

**Spec:** `docs/superpowers/specs/2026-11-18-eval-harness-design.md`

**Plans 2 and 3 (not this plan):** Plan 2 covers the recording and flight-stack changes (missing
topics, `run.json`, the filming-cue node, arm pass-through, repeated runs, clean recorder
shutdown), plus a Gazebo contact sensor on the drone model so physics-reported contacts can be
cross-checked against the geometric clearance check here. Plan 3 covers the scenario generator
(obstacle layouts, subject paths and missions varied per scenario, each with its own world file)
and its held-out split. This plan reads the fields
those plans will write and reports a channel or rule as unchecked when a field is absent.

## Global Constraints

- No new dependencies. Everything above is already installed (checked 2026-11-18: image venv
  `/opt/cine-venv/bin/python` has rosbag2_py, numpy, scipy, matplotlib, pytest 7.4.4; WSL `python3`
  has numpy, scipy, matplotlib, pytest; no ffmpeg binary in the image).
- No commits, no pushes (owner's standing rule). Each task ends with a checkpoint, not a commit.
- After every task: `python3 tools/format.py <changed .py files>` and `python3 -m py_compile <each>`.
  LF line endings.
- Constants live as JSON under `cine_config/config/`, read through `cine_config` (AGENTS.md).
- Names spelled out (`position`, not `pos`); no banner or "what" comments.
- Quaternions stay `(x, y, z, w)` everywhere. Time in seconds of simulation time. SI units.
- A stream absent from the bag is absent from the record, never empty-filled; channels that need it
  return `status="unchecked"` with a reason.
- Exit codes: 0 pass, 1 review/failure, 3 environment not ready, 4 unreadable input (2 is typer's
  usage error).
- Focus: 3-D drone–subject distance, tolerance 0.1 m, target = anchor focus distance stream if
  present, else `focus_target_m` from `run.json`, else unchecked.
- Channel clock: 50 Hz, recorded in each channel's output.
- Gate real-time-factor floor 0.9 (provisional). All gate thresholds provisional, owner's call.
- Bare pytest from repo root (WSL): `cd /mnt/c/Users/vtsyp/OneDrive/Desktop/Robotics/CINE && python3 -m pytest cine_eval/test -q`
- Image tests (ROS): `docker run --rm -e PYTHONDONTWRITEBYTECODE=1 -v "$PWD":/cine:ro -w /cine --entrypoint bash cine-gazebo-sim:dev -c 'source /opt/ros/jazzy/setup.bash && /opt/cine-venv/bin/python -m pytest cine_eval/test -q'`

## Review Focus

1. **Empty or zero-byte bag** (two exist in `~/cine-bags`): `extract` exits 4 with
   "bag has no messages: <path>", never a traceback. Test in Task 8.
2. **Filming cue never on, or on for zero time:** `focus` and `motion` return unchecked with reason
   "no filming time", never NaN or a division error. Tests in Tasks 4 and 5.
3. **Out-of-order or duplicate timestamps in a stream:** extraction sorts by time; `validate()`
   rejects a stream whose time decreases. Test in Task 1.
4. **Unstamped messages (header stamp 0, or `std_msgs/Bool` with no header):** time falls back to
   the bag receive time and `record.json` marks that stream `stamp_source: "receive"`; `freshness`
   reports that stream unchecked. Test in Task 8 (pure function) and Task 7.
5. **Too few pairs for the test to ever reach significance** (exact Wilcoxon, ≤5 pairs at
   α = 0.05 two-sided): the comparison prints "insufficient pairs (n=K) for α=0.05", no p-value
   presented as meaningful. Test in Task 11.

---

### Task 1: Package skeleton and flight-record contract

**Files:**

- Create: `cine_eval/COLCON_IGNORE` (empty), `cine_eval/cine_eval/__init__.py`, `cine_eval/cine_eval/record.py`
- Modify: `pyproject.toml` (append `'cine_eval/test'` to `testpaths`, `'cine_eval'` to `pythonpath`)
- Test: `cine_eval/test/test_record.py`

**Interfaces:**

- Produces:
  - `@dataclass(frozen=True) class Stream: name: str; frame_id: str; time: np.ndarray; receive_time: np.ndarray; fields: dict[str, np.ndarray]; stamp_source: str = "header"`
  - `@dataclass(frozen=True) class FlightRecord: streams: dict[str, Stream]; metadata: dict`
  - `FlightRecord.has(*names: str) -> bool`
  - `validate(record: FlightRecord) -> None` raises `RecordError(ValueError)`
  - `save(record: FlightRecord, directory: Path) -> None` writes `record.npz` (keys `"<stream>/time"`, `"<stream>/receive_time"`, `"<stream>/<field>"`) and `record.json` (`{"streams": {name: {"frame_id", "stamp_source", "fields": [...]}}, "metadata": {...}}`)
  - `load(directory: Path) -> FlightRecord`
  - Stream names (constants in `record.py`): `DRONE_TRUE, SUBJECT_TRUE, DRONE_ESTIMATE, SUBJECT_ESTIMATE, DETECTIONS, SETPOINTS, PLANNED_PATH, FLOWN_PATH, FRAMING_ACTIVE, COSTS, FOCUS_TARGET`

- [ ] **Step 1: Write the failing tests**

```python
def test_save_then_load_returns_identical_streams(tmp_path): ...
    # build a record with DRONE_TRUE (position (N,3), orientation (N,4)) and FRAMING_ACTIVE (active (N,) bool)
    # assert np.array_equal on every field, time, receive_time; frame_id and stamp_source preserved

def test_validate_rejects_decreasing_time():
    # time [0.0, 0.2, 0.1] -> pytest.raises(RecordError, match="time decreases")

def test_validate_rejects_field_length_mismatch():
    # time len 3, position shape (2,3) -> RecordError match "length"

def test_absent_stream_is_not_in_record(tmp_path):
    # save a record without SUBJECT_ESTIMATE, load it, assert not record.has(SUBJECT_ESTIMATE)
```

- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_record.py -q` → FAIL (`ModuleNotFoundError: cine_eval`)
- [ ] **Step 3: Implement `record.py`** with the interfaces above. `save` calls `validate` first.
- [ ] **Step 4: Run** the same command → 4 passed
- [ ] **Step 5: Checkpoint** (format, py_compile; no commit)

### Task 2: Scoring and gate settings in `cine_config`

**Files:**

- Create: `cine_config/config/eval/scoring.json`, `cine_config/config/eval/gate.json`, `cine_eval/cine_eval/settings.py`
- Modify: `cine_config/cine_config/config_loader.py` (`CONFIG_FILES` += `'eval_scoring': 'eval/scoring'`, `'eval_gate': 'eval/gate'`)
- Test: `cine_eval/test/test_settings.py`, add one case to `cine_config/test/test_loader.py`

**Interfaces:**

- Produces: `@dataclass(frozen=True) class ScoringSettings` and `GateThresholds`; `load_scoring_settings() -> ScoringSettings`; `load_gate_thresholds() -> GateThresholds`; both have `as_dict() -> dict` for copying into outputs.

`scoring.json` (exact values):

```json
{
  "resampling": { "clock_hz": 50 },
  "focus": { "tolerance_m": 0.1 },
  "motion": {
    "savgol_window_s": 0.3,
    "savgol_polyorder": 3,
    "sparc_cutoff_hz": 10.0,
    "sparc_amplitude_threshold": 0.05,
    "sparc_padding_level": 4
  },
  "clearance": {
    "airframe_radius_m": 0.25,
    "airframe_half_height_m": 0.1,
    "subject_radius_m": 0.3,
    "subject_height_m": 1.8,
    "airborne_height_m": 0.5
  },
  "estimate": { "detection_stale_s": 0.5 }
}
```

`gate.json` (all provisional; the top-level `"provisional": true` is printed by `gate`):

```json
{
  "provisional": true,
  "thresholds": {
    "min_clearance_m": 0.3,
    "min_subject_clearance_m": 0.5,
    "min_focus_held_share": 0.5,
    "speed_cap_margin": 1.1,
    "max_setpoint_jump_m": 1.0,
    "max_data_age_s": 0.2,
    "min_real_time_factor": 0.9
  }
}
```

`speed_cap_margin` multiplies `focus_max_velocity_mps` (`focus` config, 0.45 m/s).

- [ ] **Step 1: Write failing tests:** `test_scoring_settings_read_the_committed_values` (assert `clock_hz == 50`, `focus_tolerance_m == 0.1`, `airframe_radius_m == 0.25`); `test_gate_thresholds_carry_the_provisional_flag` (assert `provisional is True`, `min_real_time_factor == 0.9`); loader case `test_loader_exposes_eval_scoring`.
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_settings.py cine_config/test/test_loader.py -q` → FAIL
- [ ] **Step 3: Implement** the JSON files, loader entries and `settings.py`.
- [ ] **Step 4: Run** → pass
- [ ] **Step 5: Checkpoint**

### Task 3: Shared clock, filming window and smoothed derivatives

**Files:**

- Create: `cine_eval/cine_eval/timeline.py`
- Test: `cine_eval/test/test_timeline.py`

**Interfaces:**

- Consumes: `Stream`, `FlightRecord` (Task 1)
- Produces:
  - `fixed_clock(start: float, end: float, clock_hz: float) -> np.ndarray`
  - `sample(stream: Stream, field: str, clock: np.ndarray) -> np.ndarray` (linear interpolation per column; values outside the stream's span are NaN)
  - `sample_hold(stream: Stream, field: str, clock: np.ndarray) -> np.ndarray` (last value at or before each instant; before the first sample → False/NaN)
  - `filming_mask(record: FlightRecord, clock: np.ndarray) -> np.ndarray | None` (None when `FRAMING_ACTIVE` absent)
  - `smoothed_derivatives(position: np.ndarray, clock_hz: float, window_s: float, polyorder: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]` (velocity, acceleration, jerk via `scipy.signal.savgol_filter(..., deriv=k, delta=1/clock_hz, axis=0)`; window length = odd number of samples nearest `window_s * clock_hz`, at least `polyorder + 2`)

- [ ] **Step 1: Failing tests:**
  - `test_sample_hold_keeps_last_cue_value` (cue True at 1.0 s, False at 3.0 s; clock 0..4 s at 1 Hz → `[False, True, True, False, False]`)
  - `test_sample_outside_span_is_nan`
  - `test_smoothed_derivatives_recover_constant_acceleration` (x = 0.5·2·t² at 50 Hz; interior acceleration ≈ 2.0 within 1e-6, jerk ≈ 0 within 1e-6)
  - `test_filming_mask_is_none_without_cue`
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_timeline.py -q` → FAIL
- [ ] **Step 3: Implement** `timeline.py`.
- [ ] **Step 4: Run** → pass
- [ ] **Step 5: Checkpoint**

### Task 4: Channel result type and the `focus` channel

**Files:**

- Create: `cine_eval/cine_eval/channels/__init__.py`, `cine_eval/cine_eval/channels/result.py`, `cine_eval/cine_eval/channels/focus.py`
- Test: `cine_eval/test/test_channel_focus.py`

**Interfaces:**

- Consumes: Tasks 1–3
- Produces:
  - `@dataclass(frozen=True) class ChannelResult: name: str; status: str  # "ok" | "unchecked"; reason: str; summary: dict[str, float]; series: dict[str, np.ndarray]; clock_hz: float`
  - `unchecked(name: str, reason: str) -> ChannelResult`
  - `score_focus(record: FlightRecord, settings: ScoringSettings) -> ChannelResult`; summary keys `held_share`, `worst_deviation_m`, `filming_s`, `target_m`; series `time`, `distance_m`, `error_m`, `filming`. `summary["target_source"]` is not a float, so the target's source goes in `reason` when status is ok (`"target from anchor stream"` / `"target from run.json"`).

Target resolution order: `FOCUS_TARGET` stream (field `distance_m`, sample-hold) → `record.metadata["run"]["focus_target_m"]` → unchecked `"no focus target: neither anchor stream nor run.json"`. Needs `DRONE_TRUE`, `SUBJECT_TRUE`, `FRAMING_ACTIVE`, else unchecked naming the missing streams.

- [ ] **Step 1: Failing tests:**
  - `test_drone_circling_at_target_holds_focus_all_filming_time` (circle radius 1.5 m around a fixed subject at equal height, target 1.5 in run metadata, cue on throughout → `held_share == 1.0`, `worst_deviation_m < 1e-9`)
  - `test_drone_at_twice_the_target_never_holds_focus` (`held_share == 0.0`, `worst_deviation_m == pytest.approx(1.5)`)
  - `test_only_filming_time_counts` (cue on for the half where distance is on target → `held_share == 1.0`, `filming_s == pytest.approx(5.0, abs=0.02)`)
  - `test_focus_without_cue_is_unchecked` (reason contains `"framing_active"`)
  - `test_focus_with_cue_never_on_is_unchecked_no_filming_time` (Review Focus 2: reason `"no filming time"`, summary empty)
  - `test_focus_without_any_target_is_unchecked`
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_channel_focus.py -q` → FAIL
- [ ] **Step 3: Implement** `result.py` and `focus.py`.
- [ ] **Step 4: Run** → pass
- [ ] **Step 5: Checkpoint**

### Task 5: The `motion` channel (raw and speed-invariant smoothness)

**Files:**

- Create: `cine_eval/cine_eval/channels/motion.py`
- Test: `cine_eval/test/test_channel_motion.py`

**Interfaces:**

- Consumes: Tasks 1–4
- Produces: `score_motion(record, settings) -> ChannelResult`; `log_dimensionless_jerk(jerk: np.ndarray, clock_hz: float, path_length_m: float) -> float`; `spectral_arc_length(speed: np.ndarray, clock_hz: float, cutoff_hz: float, amplitude_threshold: float, padding_level: int) -> float`. Summary keys: `peak_speed_mps`, `mean_speed_mps`, `peak_acceleration_mps2`, `rms_acceleration_mps2`, `peak_jerk_mps3`, `rms_jerk_mps3`, `log_dimensionless_jerk`, `sparc`, `filming_s`, `savgol_window_samples`.

Computed over filming time only; each contiguous filming segment is differentiated separately and the
summaries combine samples from all segments (LDLJ and SPARC on the longest segment, with its length
recorded as `smoothness_segment_s`).

LDLJ, higher is smoother: `-ln( (T**5 / L**2) * ∫|jerk|² dt )`, T = segment duration, L = path length.
SPARC, closer to 0 is smoother (Balasubramanian et al. 2015); not determined by the signature, so:

```python
nfft = 2 ** (int(np.ceil(np.log2(len(speed)))) + padding_level)
frequency = np.arange(nfft) * clock_hz / nfft
magnitude = np.abs(np.fft.fft(speed, nfft))
magnitude = magnitude / magnitude.max()
within_cutoff = frequency <= cutoff_hz
frequency, magnitude = frequency[within_cutoff], magnitude[within_cutoff]
above = np.nonzero(magnitude >= amplitude_threshold)[0]
frequency, magnitude = frequency[above[0]:above[-1] + 1], magnitude[above[0]:above[-1] + 1]
normalised_step = np.diff(frequency) / (frequency[-1] - frequency[0])
return -float(np.sum(np.sqrt(normalised_step ** 2 + np.diff(magnitude) ** 2)))
```

- [ ] **Step 1: Failing tests:**
  - `test_constant_speed_line_has_zero_acceleration_and_jerk` (1 m/s straight line, cue on → peak acceleration and jerk < 1e-6)
  - `test_speed_invariant_measures_do_not_change_when_motion_is_time_scaled` (the same minimum-jerk point-to-point path flown over 4 s and over 8 s: `log_dimensionless_jerk` equal within 0.05, `sparc` equal within 0.05; raw `peak_jerk_mps3` differs by ≥ 8×)
  - `test_minimum_jerk_profile_is_smoother_than_a_jerky_one` (add a 3 Hz 0.05 m wobble → lower LDLJ and more negative SPARC than the clean profile)
  - `test_motion_with_cue_never_on_is_unchecked_no_filming_time`
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_channel_motion.py -q` → FAIL
- [ ] **Step 3: Implement** `motion.py`.
- [ ] **Step 4: Run** → pass
- [ ] **Step 5: Checkpoint**

### Task 6: Scene from the world file and the `clearance` channel

**Files:**

- Create: `cine_eval/cine_eval/scene.py`, `cine_eval/cine_eval/channels/clearance.py`
- Test: `cine_eval/test/test_scene.py`, `cine_eval/test/test_channel_clearance.py`

**Interfaces:**

- Produces:
  - `@dataclass(frozen=True) class Box: name: str; centre: np.ndarray; size: np.ndarray; yaw: float`
  - `@dataclass(frozen=True) class Cylinder: name: str; centre: np.ndarray; radius: float; length: float`
  - `load_scene(world_path: Path) -> list[Box | Cylinder]` (static models only; collision geometry; model `<pose>` x y z roll pitch yaw with roll/pitch ignored; honours `degrees="true"`; the `{{wind}}` placeholder text is harmless to the XML parser)
  - `distance_to_obstacle(points: np.ndarray, obstacle) -> np.ndarray` (signed: negative inside)
  - `score_clearance(record, settings, scene) -> ChannelResult`, over the whole flight (not only filming). Three obstacle classes, each a clearance = surface distance minus the airframe size:
    - static: every `Box`/`Cylinder` from the scene, minus `airframe_radius_m`;
    - subject: an upright cylinder of `subject_radius_m` from z = 0 to `subject_height_m`, centred on `SUBJECT_TRUE`'s x, y at each instant (sampled onto the drone's clock), minus `airframe_radius_m`; skipped with a note in `reason` when `SUBJECT_TRUE` is absent;
    - ground: drone z minus `airframe_half_height_m`, only inside the airborne window (first to last sample with z > `airborne_height_m`), so standing on the pad is not contact.
  - Summary: `min_clearance_m` (minimum over all three), `min_static_clearance_m`, `min_subject_clearance_m`, `min_ground_clearance_m`, `contact_frames` (samples with any clearance < 0), `collision_free` (1.0 when `contact_frames == 0`, else 0.0), `first_contact_s` (NaN when none). `reason` names the nearest obstacle and the world file used. Series `time`, `clearance_m`, `static_clearance_m`, `subject_clearance_m`, `ground_clearance_m`.
  - Plane and mesh collision geometry is skipped and listed in `reason` (the template's only non-box/cylinder model is the included `runway`, which is ground).
- World path: `record.metadata["run"]["world_file"]` when present, else `cine_gazebo/worlds/cine_field.sdf.template` (the only world today), resolved from the repo root; the path used is in `reason`.

- [ ] **Step 1: Failing tests:**
  - `test_load_scene_reads_the_five_static_obstacles` (names `hedge_east`, `hedge_north`, `pillar_north_west`, `pillar_north_east`, `pillar_far_north`; `pillar_north_west` radius 0.8, centre (-8, 9, 3); `hedge_east` size (1.5, 6, 3))
  - `test_distance_to_box_and_cylinder_matches_hand_geometry` (point 2 m east of a box face → 2.0; point inside a cylinder → negative)
  - `test_drone_passing_through_a_box_scores_contact` (`contact_frames > 0`, `min_clearance_m < 0`, `collision_free == 0.0`)
  - `test_clearance_subtracts_the_airframe_radius`
  - `test_drone_flying_through_the_walking_subject_is_a_collision` (subject moves along x, drone crosses its path at the same instant at 1.0 m height → `min_subject_clearance_m < 0`, `collision_free == 0.0`; same drone path 2 s later, when the subject has moved 2.4 m on → `collision_free == 1.0`)
  - `test_standing_on_the_pad_is_not_ground_contact` (z = 0 for 2 s, climb to 3 m, land → `min_ground_clearance_m > 0` over the airborne window)
  - `test_dipping_below_the_airborne_height_mid_flight_into_the_ground_is_contact` (z drops to 0.05 m between two segments above 3 m → contact; note the window is first-to-last above `airborne_height_m`, so a mid-flight dip stays inside it)
  - `test_clearance_without_subject_stream_still_scores_static_and_ground` (status ok, `reason` notes subject skipped)
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_scene.py cine_eval/test/test_channel_clearance.py -q` → FAIL
- [ ] **Step 3: Implement** `scene.py` (`xml.etree.ElementTree`) and `clearance.py`.
- [ ] **Step 4: Run** → pass
- [ ] **Step 5: Checkpoint**

### Task 7: The `estimate` and `freshness` channels

**Files:**

- Create: `cine_eval/cine_eval/channels/estimate.py`, `cine_eval/cine_eval/channels/freshness.py`
- Test: `cine_eval/test/test_channel_estimate.py`, `cine_eval/test/test_channel_freshness.py`

**Interfaces:**

- Produces:
  - `score_estimate(record, settings) -> ChannelResult`: needs `SUBJECT_ESTIMATE`, `SUBJECT_TRUE`; summary `median_error_m`, `p95_error_m`, `max_error_m`; with `DETECTIONS` and the cue also `stale_detection_share` (filming time with no detection newer than `detection_stale_s`). Refuses (unchecked) when the two streams' `frame_id`s differ and `record.metadata` has no `frame_transforms` entry for the pair — the spec's no-mixed-frames rule.
  - `score_freshness(record, settings) -> ChannelResult`: for each stream with `stamp_source == "header"`, age = `receive_time - time`; summary `"<stream>.median_age_s"`, `"<stream>.max_age_s"`, plus `max_age_s` over all; streams with `stamp_source == "receive"` listed in `reason` as unchecked.

- [ ] **Step 1: Failing tests:**
  - `test_estimate_offset_by_constant_vector_reports_that_error` (estimate = truth + (0.3, 0.4, 0) → median 0.5)
  - `test_estimate_in_a_different_frame_without_transform_is_unchecked` (reason contains both frame ids)
  - `test_stale_detection_share_counts_gaps_longer_than_the_bound`
  - `test_freshness_reports_age_between_stamp_and_receipt` (receive = stamp + 0.08 → median 0.08)
  - `test_freshness_skips_streams_stamped_on_receipt` (Review Focus 4)
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_channel_estimate.py cine_eval/test/test_channel_freshness.py -q` → FAIL
- [ ] **Step 3: Implement** both modules.
- [ ] **Step 4: Run** → pass
- [ ] **Step 5: Checkpoint**

### Task 8: Bag extraction (image only) and the committed fixture

**Files:**

- Create: `cine_eval/cine_eval/extract.py`, `cine_eval/cine_eval/message_time.py`, `cine_eval/test/fixtures/make_fixture.py`, `cine_eval/test/fixtures/short_flight/` (MCAP slice + `metadata.yaml`, target < 400 KB), `cine_eval/test/fixtures/short_flight_record/` (`record.npz` + `record.json` produced by `extract` from that slice)
- Test: `cine_eval/test/test_message_time.py` (bare pytest), `cine_eval/test/test_extract.py` (`pytest.importorskip("rosbag2_py", reason="extraction runs inside cine-gazebo-sim:dev")`)

**Interfaces:**

- Produces:
  - `message_time(header_stamp_ns: int | None, receive_ns: int) -> tuple[float, str]` → `(seconds, "header" | "receive")`; `None` or 0 → receive.
  - `TOPIC_STREAMS: dict[str, str]` mapping: `/gazebo/drone_pose`→`DRONE_TRUE`, `/gazebo/subject_pose`→`SUBJECT_TRUE`, `/mavros/local_position/odom`→`DRONE_ESTIMATE`, `/cinematography/target_world`→`SUBJECT_ESTIMATE`, `/object_detection/center_point`→`DETECTIONS`, `/mavros/setpoint_position/local`→`SETPOINTS`, `/cinematography/framing_active`→`FRAMING_ACTIVE`. `/vio/odometry` goes to `DRONE_ESTIMATE` only when the mavros topic is absent. Plan 2 adds the costs, anchor-distance and B-spline topics.
  - Converters keyed by message type name: `geometry_msgs/msg/PoseStamped` (position, orientation), `nav_msgs/msg/Odometry` (position, orientation, velocity), `geometry_msgs/msg/PointStamped` (position), `geometry_msgs/msg/Point` (position), `std_msgs/msg/Bool` (active), `std_msgs/msg/Float32` (value). Topics with other types are skipped and listed in `record.json` `metadata.skipped_topics`.
  - `extract(bag_directory: Path) -> FlightRecord`; metadata: `bag_name`, `bag_sha1` (SHA-1 over each `.mcap` file's SHA-1, files sorted by name), `extractor_version` (string constant `"1"`), `reindexed: bool`, `run` (contents of `bag_directory/run.json` if present, else absent), `skipped_topics`.
  - `class BagError(Exception)` for unreadable input.
- Unclosed bag: when `metadata.yaml` is missing, copy the directory to a `tempfile.TemporaryDirectory`, run `ros2 bag reindex <copy> -s mcap`, read the copy; never write to the original.
- Every stream sorted by time with exact-duplicate stamps dropped before `validate`.

- [ ] **Step 1: Failing bare tests:** `test_zero_stamp_falls_back_to_receive_time`, `test_header_stamp_is_used_when_set` (`message_time(1_500_000_000, 1_600_000_000) == (1.5, "header")`).
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_message_time.py -q` → FAIL; implement `message_time.py`; run → pass.
- [ ] **Step 3: Write `make_fixture.py`:** reads a source bag with `rosbag2_py.SequentialReader`, writes messages with receive time ≤ first + 6.0 s to a new MCAP bag with `rosbag2_py.SequentialWriter` (topics and types preserved). Run it in the image against `~/cine-bags/2026-11-18_23-51-45_sim_cine_field_calm` (reindex a copy first) → `cine_eval/test/fixtures/short_flight/`. Expected: `ros2 bag info` shows `/gazebo/drone_pose`, `/gazebo/subject_pose`, `/mavros/local_position/odom`, about 6 s.
- [ ] **Step 4: Failing image tests:**
  - `test_extract_fixture_has_truth_and_estimate_streams` (`DRONE_TRUE`, `SUBJECT_TRUE`, `DRONE_ESTIMATE` present; `FRAMING_ACTIVE` absent; `DRONE_TRUE.frame_id` non-empty; time non-decreasing)
  - `test_extract_unclosed_bag_reindexes_a_copy_and_leaves_original_untouched` (copy fixture to tmp, delete its `metadata.yaml`, extract → `reindexed is True`, original directory listing unchanged)
  - `test_extract_zero_byte_bag_raises_bag_error` (Review Focus 1: tmp dir with an empty `x_0.mcap` → `BagError` match `"no messages"`)
- [ ] **Step 5: Run the image test command** → FAIL; implement `extract.py`; run → pass. Then extract the fixture to `short_flight_record/` in the image.
- [ ] **Step 6: Checkpoint**

### Task 9: Scoring a run, and the regression check on the fixture

**Files:**

- Create: `cine_eval/cine_eval/channels/following.py`, `cine_eval/cine_eval/channels/mission.py`, `cine_eval/cine_eval/score.py`, `cine_eval/test/fixtures/short_flight_record/expected_score.json`
- Test: `cine_eval/test/test_channel_following.py`, `cine_eval/test/test_channel_mission.py`, `cine_eval/test/test_score.py`

**Interfaces:**

- Consumes: all channels, `load`, settings
- Produces:
  - `score_following(record, settings) -> ChannelResult`: needs `DRONE_TRUE`, `SETPOINTS`; summary `median_tracking_error_m`, `max_tracking_error_m`, `max_setpoint_jump_m` (largest step between consecutive setpoints).
  - `score_mission(record, settings) -> ChannelResult`: needs `DRONE_TRUE` and `record.metadata["run"]["waypoints"]` (list of [x, y, z]); summary `completed` (1.0 if the drone came within 0.5 m of the final waypoint), `duration_s`, `aborted` (from `run["outcome"] == "aborted"`, else 0.0).
  - `CHANNELS: tuple[str, ...] = ("focus", "motion", "clearance", "estimate", "following", "freshness", "mission")`
  - `@dataclass(frozen=True) class RunScore: run_name: str; channels: dict[str, ChannelResult]; settings: dict; provenance: dict`
  - `score_run(run_directory: Path) -> RunScore`; `write_score(score: RunScore, run_directory: Path) -> None` writes `score.json` (`{"run", "channels": {name: {"status","reason","summary","clock_hz"}}, "settings", "provenance"}`) and `metrics.npz` (`"<channel>/<series>"`); `read_score(run_directory: Path) -> RunScore` (series not loaded).
  - Provenance: `scorer_commit` and `scorer_dirty` from env `CINE_EVAL_SCORER_COMMIT` / `CINE_EVAL_SCORER_DIRTY` (absent → `"unknown"`), `record_sha1` (of `record.npz`), `bag_sha1` from record metadata.

- [ ] **Step 1: Failing tests:** `test_setpoint_jump_is_the_largest_consecutive_step`, `test_mission_completed_when_final_waypoint_reached`, `test_mission_without_waypoints_is_unchecked`, `test_score_json_copies_the_settings_used`, and the regression test `test_fixture_scores_match_the_committed_expectation` (score `short_flight_record`, compare statuses exactly and every summary float to `expected_score.json` with `pytest.approx(rel=1e-9)`).
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_channel_following.py cine_eval/test/test_channel_mission.py cine_eval/test/test_score.py -q` → FAIL
- [ ] **Step 3: Implement** the two channels and `score.py`.
- [ ] **Step 4: Generate `expected_score.json` once** with `write_score` on the fixture and read it by hand before committing it to the test: expected statuses are `clearance: ok` (static, subject and ground all scored, since the fixture has `SUBJECT_TRUE`), all others `unchecked` (the fixture has no cue, no setpoints, no subject estimate, no `run.json`). Plan 2 regenerates the fixture from a full mission bag.
- [ ] **Step 5: Run** → pass. Confirm the regression test can fail: change `airframe_radius_m` in a copy of the settings passed to the scorer → `min_clearance_m` mismatch; restore.
- [ ] **Step 6: Checkpoint**

### Task 10: Pass/fail gate

**Files:**

- Create: `cine_eval/cine_eval/gate.py`
- Test: `cine_eval/test/test_gate.py`

**Interfaces:**

- Consumes: `RunScore`, `GateThresholds`, `FOCUS_MAX_VELOCITY_MPS` from `cine_config.constants.CinematicConstraints`
- Produces: `@dataclass(frozen=True) class GateVerdict: failures: tuple[str, ...]; unchecked: tuple[str, ...]; provisional: bool` with property `label -> "PASS" | "REVIEW"`; `judge(score: RunScore, run_metadata: dict, thresholds: GateThresholds) -> GateVerdict`.
- Rules (each names its channel; channel unchecked → rule goes in `unchecked`):
  `no_contact` (clearance.collision_free == 1.0; failure message names the class: static, subject or ground), `min_clearance` (static and ground ≥ `min_clearance_m`), `min_subject_clearance` (≥ `min_subject_clearance_m`), `mission_completed`,
  `speed_cap_while_filming` (motion.peak_speed_mps ≤ `speed_cap_margin` × focus max velocity),
  `focus_held` (≥ `min_focus_held_share`), `setpoint_jump` (≤ `max_setpoint_jump_m`),
  `data_age` (freshness.max_age_s ≤ `max_data_age_s`),
  `real_time_factor` (`run_metadata["real_time_factor"]` ≥ `min_real_time_factor`; absent → unchecked).

- [ ] **Step 1: Failing tests:** `test_label_is_pass_only_without_failures`, `test_unchecked_channel_puts_its_rules_in_unchecked_not_failures`, `test_contact_fails_the_gate`, `test_slow_real_time_factor_is_review` (0.8 → `"real_time_factor"` in failures), `test_missing_real_time_factor_is_unchecked`.
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_gate.py -q` → FAIL
- [ ] **Step 3: Implement** `gate.py`.
- [ ] **Step 4: Run** → pass
- [ ] **Step 5: Checkpoint**

### Task 11: The judge (paired comparison, analysis plan, noise floor)

**Files:**

- Create: `cine_eval/cine_eval/statistics.py`, `cine_eval/cine_eval/analysis.py`, `eval/analysis_plan.toml`
- Test: `cine_eval/test/test_statistics.py`, `cine_eval/test/test_analysis.py`

**Interfaces:**

- Produces in `statistics.py`:
  - `wilson_interval(successes: int, trials: int, z: float = 1.959964) -> tuple[float, float]`
  - `wilcoxon_exact_p(differences: np.ndarray) -> float` (`scipy.stats.wilcoxon(differences, method="exact")`; zero differences dropped, as scipy's default `zero_method="wilcox"`)
  - `minimum_pairs_for_alpha(alpha: float) -> int` (smallest n where the exact two-sided p for all-same-sign differences is ≤ alpha: `2 / 2**n <= alpha` → 6 for 0.05)
  - `hodges_lehmann(differences: np.ndarray) -> float` (median of Walsh averages `(d_i + d_j)/2, i ≤ j`)
  - `paired_bootstrap_interval(differences: np.ndarray, resamples: int = 10000, seed: int = 0, level: float = 0.95) -> tuple[float, float]` (percentile interval of the resampled Hodges–Lehmann estimate)
  - `holm_adjust(p_values: list[float]) -> list[float]`
- Produces in `analysis.py`:
  - `@dataclass(frozen=True) class AnalysisPlan: primary: MetricSpec; secondary: tuple[MetricSpec, ...]; alpha: float; test: str; min_filming_s: float; digest: str` with `MetricSpec(metric: str, higher_is_better: bool, worst_value: float)`; `load_plan(path: Path) -> AnalysisPlan` (`tomllib`; digest = SHA-1 of the file bytes).
  - `condition_key(run_metadata: dict, run_name: str) -> tuple[str, ...]`: `(scenario, world_file, waypoints_sha1, wind, seed)` from `run`; runs without `run.json` get `("unpaired", run_name)` so they never pair.
  - `compare(arms: dict[str, list[Path]], plan: AnalysisPlan, repeatability: list[Path] | None, plan_path_committed_digest: str | None) -> Comparison`; `Comparison.to_json() -> dict`; `Comparison.to_markdown() -> str`.
- Rules the judge applies (spec "Judge" and rigour rules 1–4):
  - Repeats of one condition within one arm collapse to their median; the count is reported.
  - Pairs = conditions present in both arms; unpaired runs listed by name.
  - Per arm per metric: each run's value, mean, median, standard deviation, min, max. Rates (`mission.completed`) also get a Wilson interval.
  - Mission success rate and collision-free rate (`clearance.collision_free`) reported first, each with a Wilson interval per arm, plus the paired count of conditions where one arm collided and the other did not.
  - A run whose `mission.completed == 0`, `mission.aborted == 1` or `clearance.collision_free == 0` is a failed run and takes each metric's `worst_value`; the comparison is printed twice, "failures as worst value" and "failed pairs removed".
  - Primary metric: exact Wilcoxon p, Hodges–Lehmann difference with bootstrap interval. Secondary: same, p values Holm-adjusted.
  - n < `minimum_pairs_for_alpha(alpha)` → `"insufficient pairs (n=K) for α=0.05"` and no p value (Review Focus 5).
  - `plan.digest` recorded; label `"exploratory"` when `plan_path_committed_digest` is given and differs, or is `None` (cannot tell the plan was committed first).
  - With `repeatability`: per metric, the median absolute deviation across repeats of the same arm and condition is printed as `noise_floor`; a Hodges–Lehmann difference smaller in magnitude than it is labelled `"not resolved"`.

`eval/analysis_plan.toml`:

```toml
alpha = 0.05
test = "wilcoxon_exact"
min_filming_s = 5.0

[primary]
metric = "focus.held_share"
higher_is_better = true
worst_value = 0.0

[[secondary]]
metric = "motion.log_dimensionless_jerk"
higher_is_better = true
worst_value = -50.0

[[secondary]]
metric = "motion.sparc"
higher_is_better = true
worst_value = -20.0

[[secondary]]
metric = "motion.rms_jerk_mps3"
higher_is_better = false
worst_value = 1000.0

[[secondary]]
metric = "focus.worst_deviation_m"
higher_is_better = false
worst_value = 100.0

[[secondary]]
metric = "clearance.min_clearance_m"
higher_is_better = true
worst_value = -1.0
```

The `worst_value`s are provisional; choosing them is the owner's call, and they must be settled before the first scored comparison.

- [ ] **Step 1: Failing statistics tests:** `test_wilson_interval_for_8_of_10` (≈ (0.4902, 0.9433) to 4 decimals), `test_minimum_pairs_for_alpha_005_is_6`, `test_hodges_lehmann_of_symmetric_differences_is_their_centre`, `test_bootstrap_interval_is_reproducible_with_the_same_seed`, `test_holm_adjust_matches_hand_example` (`[0.01, 0.04, 0.03] → [0.03, 0.06, 0.06]`).
- [ ] **Step 2: Failing analysis tests** (scores built as `score.json` files in `tmp_path`):
  - `test_runs_pair_by_condition_key_and_unpaired_are_listed`
  - `test_failed_run_takes_worst_value_and_both_views_are_reported`
  - `test_collided_run_counts_as_failed_and_lowers_collision_free_rate`
  - `test_five_pairs_report_insufficient_pairs_without_p_value`
  - `test_eight_consistent_pairs_report_significant_primary` (p = 2/256 ≈ 0.0078)
  - `test_changed_plan_labels_comparison_exploratory`
  - `test_difference_below_noise_floor_is_not_resolved`
- [ ] **Step 3: Run** `python3 -m pytest cine_eval/test/test_statistics.py cine_eval/test/test_analysis.py -q` → FAIL
- [ ] **Step 4: Implement** `statistics.py`, `analysis.py`, `eval/analysis_plan.toml`.
- [ ] **Step 5: Run** → pass
- [ ] **Step 6: Checkpoint**

### Task 12: Standings, provenance and the generated part of `docs/results.md`

**Files:**

- Create: `cine_eval/cine_eval/standings.py`, `eval/standings.toml` (empty row list with a header comment), `eval/outputs/README.md`
- Modify: `.gitignore` (add `eval/runs/`), `docs/results.md` (wrap the table in `<!-- cine-eval:begin -->` / `<!-- cine-eval:end -->`; move the drift-correction, 10 cm, light-avoidance and bench rows under a heading "Not produced by `cine eval`" with their last-measured dates)
- Test: `cine_eval/test/test_standings.py`

**Interfaces:**

- Produces: `build_standings(manifest_path: Path, runs_root: Path, outputs_root: Path, results_doc: Path, command: str) -> None`; `replace_generated_section(document: str, table_markdown: str) -> str` (raises `ValueError` when markers are missing or out of order).
- `eval/standings.toml` rows: `[[row]] arm, condition, runs = [...], comparable = bool, note = "..."`.
- `results.csv` columns: `row_id,arm,condition,metric,n,median,mean,std,min,max,comparable,perception,note`.
- `manifest.json` per row: `bag_sha1s`, `flight_commits` (from `run.json` `git.CINE.commit`), `flight_dirty`, `image_id`, `scorer_commit`, `scorer_dirty`, `command`.
- A row is forced `comparable: false` (note appended) when any run has `flight_dirty` true, has no `run.json`, or is legacy. `perception` = `run["perception_source"]` (`"detector"`, `"oracle"`, or `"unknown"`); `"oracle"` rows show "oracle perception" in the doc table.
- Rows whose runs' `score.json` is older than the run's `record.npz` are rescored first.

- [ ] **Step 1: Failing tests:** `test_replace_generated_section_keeps_text_outside_markers`, `test_missing_markers_raise`, `test_dirty_flight_forces_row_not_comparable`, `test_manifest_records_bag_digests_and_command`, `test_oracle_perception_is_labelled_in_the_table`.
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_standings.py -q` → FAIL
- [ ] **Step 3: Implement** `standings.py`; edit `.gitignore`, `docs/results.md`, `eval/outputs/README.md` (what is committed here and how it is regenerated).
- [ ] **Step 4: Run** → pass
- [ ] **Step 5: Checkpoint**

### Task 13: Legacy CSV adapter and the one-off correction

**Files:**

- Create: `cine_eval/cine_eval/legacy_csv.py`
- Test: `cine_eval/test/test_legacy_csv.py`

**Interfaces:**

- Produces: `record_from_legacy_csv(csv_path: Path) -> FlightRecord` for the `distance_*` files (columns `mode,time,drone_x,drone_y,drone_z,object_x,object_y,object_z,distance,target_distance,error,abs_error,within_tolerance`): `DRONE_TRUE` from drone columns, `SUBJECT_TRUE` from object columns, `FOCUS_TARGET` from `target_distance`, `FRAMING_ACTIVE` all true (the logger only wrote rows while framing), metadata `{"legacy": true, "source_csv": <name>, "logger_within_tolerance_share": <mean of that column>}`. `baseline_sim_*` files (no subject columns) raise `RecordError("no subject position")`.

- [ ] **Step 1: Failing tests:** `test_legacy_rows_become_a_record_with_target_stream` (3-row CSV in tmp), `test_legacy_logger_share_is_kept_for_the_record`, `test_file_without_subject_columns_is_rejected`.
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_legacy_csv.py -q` → FAIL; implement; run → pass.
- [ ] **Step 3: One-off re-score:** convert every `data/trajectories/2026-01/distance_*` file, score each with the `focus` channel, and add two `comparable = false` rows ("legacy kinematic sim, 2026-01, without CINE" / "with CINE") to `eval/standings.toml`. Print, per arm, the new-rule per-run mean `held_share` beside the mean `logger_within_tolerance_share`. Report both numbers to the owner; the logger means were 17.4% and 42.8% in the 2026-11-18 audit, and a mismatch is a finding, not something to tune away.
- [ ] **Step 4: Checkpoint**

### Task 14: Viewer

**Files:**

- Create: `cine_eval/cine_eval/viz/__init__.py`, `cine_eval/cine_eval/viz/series.py`, `cine_eval/cine_eval/viz/render.py`
- Test: `cine_eval/test/test_viz.py`

**Interfaces:**

- Produces:
  - `@dataclass(frozen=True) class FlightSeries` (everything drawable: paths per stream, obstacles, timelines from `metrics.npz`, filming intervals) and `flight_series(run_directory: Path) -> FlightSeries` (works without `score.json`: timelines absent, map still drawn).
  - `render_png(series: FlightSeries, out_path: Path, at_time: float | None) -> Path`
  - `render_animation(series: FlightSeries, out_path: Path, frames_per_second: int = 10) -> Path` — returns the path actually written. `.mp4` when `matplotlib.animation.writers.is_available("ffmpeg")`, else the same name with `.gif` via `PillowWriter`, and prints `"ffmpeg not available; wrote <path>"` to stderr. The image has no ffmpeg (measured 2026-11-18).
- Layout per spec "Viewer": top-down map (obstacles drawn at true size, drone path coloured by clearance with contact samples marked by a distinct marker, drone true path, flown and planned paths, subject true and estimated paths, focus circle at `at_time`) left; stacked timelines (focus error with ±tolerance band, speed with cap, clearance, data age; filming shaded) right, sharing one time axis. `matplotlib.use("Agg")`. Accessible colours: use the `dataviz` skill's palette guidance before writing the first plotting line.

- [ ] **Step 1: Failing tests:** `test_png_is_written_for_the_fixture_run` (file exists, > 10 KB), `test_animation_falls_back_to_gif_without_ffmpeg` (monkeypatch `is_available` → False; returned suffix `.gif`), `test_map_draws_without_a_score`.
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_viz.py -q` → FAIL; implement; run → pass. Also run the image test command (matplotlib 3.6.3 there).
- [ ] **Step 3: Look at the fixture PNG** and check it shows the five obstacles and the drone's path.
- [ ] **Step 4: Checkpoint**

### Task 15: Command lines (`python -m cine_eval` in the image, `cine eval` / `cine viz` on the host)

**Files:**

- Create: `cine_eval/cine_eval/__main__.py`, `cine_cli/cine_cli/evaluate.py` (module name avoids shadowing the builtin `eval`)
- Modify: `cine_cli/cine_cli/main.py` (add `eval` sub-app and `viz` command), `cine_cli/cine_cli/common.py` (move the image check out of `sim.gazebo` into `require_docker_image(environment, root, image, build_hint) -> None`, called by both), `docs/gazebo-sim.md` (a "Scoring a run" section)
- Test: `cine_eval/test/test_main.py`, `cine_cli/test/test_evaluate.py`; existing `cine_cli/test/test_sim_gazebo.py` must still pass

**Interfaces:**

- `python -m cine_eval` subcommands (argparse): `extract BAG_DIR [--out DIR]` (default `eval/runs/<bag name>`), `score RUN_DIR`, `gate RUN_DIR`, `compare --arm NAME=DIR[,DIR...] (≥2) [--repeatability DIR[,DIR...]] [--plan eval/analysis_plan.toml] --name NAME` (writes `eval/runs/compare/<NAME>.json` and `.md`), `standings [--manifest eval/standings.toml]`, `viz RUN_DIR [--out FILE] [--time T]`. Exit 4 on `BagError`/`RecordError`, 3 when `rosbag2_py` is missing for `extract`, 1 for gate REVIEW. `--json` on every subcommand prints one JSON object on stdout.
- Host: `cine eval <same subcommands and options>` and `cine viz RUN_DIR ...` build:

```
docker run --rm [--user UID:GID]
  -e CINE_EVAL_SCORER_COMMIT=<git rev-parse HEAD> -e CINE_EVAL_SCORER_DIRTY=<0|1>
  -e CINE_CONFIG_ROOT=/cine/cine_config/config -e PYTHONPATH=/cine/cine_eval:/cine/cine_config
  -e PYTHONDONTWRITEBYTECODE=1
  -v <root>:/cine:ro -v <root>/eval:/cine/eval -v <root>/docs:/cine/docs -v <bags dir>:/bags:ro
  -w /cine --entrypoint bash cine-gazebo-sim:dev
  -c 'source /opt/ros/jazzy/setup.bash && exec /opt/cine-venv/bin/python -m cine_eval "$@"' cine-eval <args>
```

Bag paths given on the host are translated to `/bags/<name>` when under the bags dir (`CINE_BAGS_DIR`, default `~/cine-bags`); others exit 2 with a message. `--user` only where `os.getuid` exists. `--dry-run` prints the command. Commit and dirty flag come from `environment.runner.capture(['git', 'rev-parse', 'HEAD'], ...)` and `['git', 'status', '--porcelain']`.

- [ ] **Step 1: Failing tests:** `test_main_gate_exits_1_on_review`, `test_main_extract_exits_4_on_unreadable_bag` (bare: monkeypatch `extract` to raise `BagError`); `test_eval_score_builds_docker_command_with_repo_read_only_and_eval_writable`, `test_eval_passes_scorer_commit_and_dirty_flag`, `test_eval_rejects_bag_outside_bags_dir`, `test_eval_without_image_exits_3` (use `RecordingRunner` from `cine_cli/test/conftest.py`).
- [ ] **Step 2: Run** `python3 -m pytest cine_eval/test/test_main.py -q` and the `cine_cli` tests → FAIL. (`cine_cli` tests need typer, which is installed nowhere on this machine today; see the execution note.)
- [ ] **Step 3: Implement**; edit `docs/gazebo-sim.md`.
- [ ] **Step 4: Run** → pass; run the full `python3 -m pytest cine_eval/test -q` and the image test command.
- [ ] **Step 5: End-to-end on a real bag:** `cine eval extract ~/cine-bags/2026-11-18_23-51-45_sim_cine_field_calm`, then `score`, `gate`, `viz --out eval/runs/2026-11-18_23-51-45_sim_cine_field_calm/flight.png`. Expected: gate `REVIEW`-free only if every rule is checked; with this bag most rules are `unchecked` and the label is `PASS` with a long unchecked list — confirm the output says so plainly.
- [ ] **Step 6: Checkpoint**

---

## Execution note

`cine_cli` tests need `typer`, which is not installed in Windows Python, WSL `python3`, or the image
(checked 2026-11-18), and `cine` itself is not installed. Running them needs a throwaway virtual
environment with `pip install -e ./cine_cli[...]`, which only installs the package's existing
declared dependency; ask the owner before creating it.
