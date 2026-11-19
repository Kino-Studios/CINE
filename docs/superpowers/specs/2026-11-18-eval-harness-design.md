# Evaluation and visualisation harness (`cine eval`, `cine viz`)

Status: design, awaiting approval. Phase 0 of the CINE roadmap.

## Why

An audit on 2026-11-18 found that the numbers in `docs/results.md` cannot be reproduced. Recomputed
from `data/trajectories/2026-01/`, the share of time within 0.1 m of the focus distance is 17.4%
without CINE and 42.8% with it (per-run mean), against the 23.6% and 76.4% the doc reports. No code
in the repository computes the table; the two arms were recorded on different days, with the subject
in different places, by different logger versions; and the CINE runs used a stand-in node that fed
the planner the subject's true position. Until results come from one reproducible scorer, no fix to
the planner, filter, costs or drift correction can be shown to help.

The owner's SAMR-squeeze-corridor repository (`BWangCN/SAMR-squeeze-corridor`, read at `3b9decc`)
already solves this problem for motion retargeting, and this design copies its structure:

- one input contract every source is converted into before scoring (`retarget/eval/contract.py`);
- a neutral measurer kept separate from the judge that holds the statistics (`harness.py` vs
  `analysis.py`, `rl/statistics.py`);
- pass/fail rules in one file, where a channel that could not run is reported as unchecked rather
  than passed (`gate.py`);
- a standings table whose every row carries the input's digest, the git commit and the command
  (`standings.py`, `eval/outputs/manifest.json`), with a `comparable` flag for runs that do not
  meet the table's conditions;
- bulky per-frame output in a gitignored `eval/runs/`, small curated output committed in
  `eval/outputs/`;
- a viewer built from source readers feeding one renderer, able to save a file headless
  (`retarget/viz/`);
- a pinned environment, so the numbers do not move when a dependency floats.

## Goals

1. Gazebo bags are the only scored input. A results row comes from a bag recorded by
   `cine sim gazebo`, nothing else.
2. Every number in `docs/results.md` is produced by `cine eval` from recorded bags, and the doc's
   table is regenerated, not hand-edited.
3. Every results row can be traced to its bag (by digest), the code that flew it (git commit and
   dirty flag) and the command that scored it.
4. A comparison between two arms reports per-run values with their spread and a paired test, never
   a single pooled figure.
5. One command renders a flight for a human to look at: a top-down map plus timelines, to PNG or
   MP4.
6. Pass/fail rules replace the smoke flight's ad-hoc success test over time, and say what they did
   not check.

## Non-goals

- Fixing the algorithm. The harness measures it; the fixes are later phases.
- Choosing how many runs each arm gets; the owner sets that later.
- Scoring the old kinematic simulator. Its CSVs are re-scored once through a small adapter so the
  doc's numbers can be corrected, and those rows are marked `comparable: false`.
- Replaying bags in rviz or Gazebo. The viewer is a plotted reconstruction.
- Scoring real-drone flights. The contract should not rule them out, but no real-flight reader is
  built now.
- Image-level scoring (shot composition from rendered frames). Camera images are not recorded
  (67 MB/s); framing is scored from detections and geometry.

## Anti-goals (fails even if it works)

- A metric that silently compares quantities in different coordinate frames or clocks.
- A skipped check that reads as a pass.
- A scoring window chosen by the algorithm under test or guessed by the scorer after the fact.
- A results row that cannot be regenerated from its recorded command and inputs.
- Pooled statistics that let one long run outweigh many short ones (the current
  `trajectory_stats.py` behaviour).
- A harness that only runs on the author's machine.

## Constraints

- No new dependencies. The simulator image `cine-gazebo-sim:dev` already provides, in
  `/opt/cine-venv/bin/python` with ROS Jazzy sourced: `rosbag2_py`, numpy, scipy, matplotlib,
  pandas, imageio and OpenCV (checked 2026-11-18).
- Constants live as JSON under `cine_config/config/` and are read through `cine_config.constants`
  (AGENTS.md).
- `cine` commands follow the existing conventions in `cine_cli/cine_cli/common.py`: `--json` prints
  one JSON object, `--dry-run` prints the command, exit code 3 means the environment is not ready.
- Bare `pytest` from the repository root runs the pure-Python tests; scoring logic must be testable
  there without ROS.
- New Python is first-party and goes through `tools/format.py`.
- One simulator at a time: earlier runs were killed by the machine's memory guard when the simulator
  and a second GPU job ran together.

## Approaches considered

**A. Everything runs inside the simulator image (chosen).** `cine eval` and `cine viz` start the
image with the repository, the bag directory and `eval/` mounted, the way `cine sim gazebo` starts
the simulator. No new dependencies; the image's versions are the pinned environment.
Costs Docker and a few seconds per command.

**B. Plain Python on the host.** Add `rosbags`, numpy, scipy and matplotlib to `cine_cli`. Runs
without Docker, but adds four dependencies whose versions would need pinning by hand.

**C. Hybrid.** Extract in the image, score and plot on the host. Two environments to keep in step,
and plotting still needs matplotlib on the host.

The owner chose A on 2026-11-18, after choosing Gazebo bags as the only scored input.

## Design

### Pipeline

```
cine sim gazebo  ──▶  bag (MCAP) + run.json         ~/cine-bags/<run>/
       │
cine eval extract ──▶ flight record                 eval/runs/<run>/record.npz + record.json
       │
cine eval score   ──▶ per-channel scores            eval/runs/<run>/score.json + metrics.npz
       │
cine eval gate    ──▶ PASS / REVIEW + unchecked     (exit code)
       │
cine eval compare ──▶ paired comparison of arms     eval/runs/compare/<name>.json + .md
       │
cine eval standings ─▶ results table + provenance   eval/outputs/results.csv, manifest.json,
                                                    docs/results.md (generated section)
cine viz <record> ──▶ map + timelines               PNG or MP4
```

Each stage reads only the previous stage's files, so any stage can be rerun alone. `score`, `gate`
and `viz` never open a bag.

### Package

A new top-level package `cine_eval/` (not a ROS package; it carries a `COLCON_IGNORE` like
`cine_cli`) with these modules:

- `record.py`: the flight-record contract (below), with `validate()`, `save()` and `load()`.
- `extract.py`: bag → flight record. The only module that imports `rosbag2_py`.
- `legacy_csv.py`: old kinematic-sim CSV → partial flight record, for the one-off correction.
- `scene.py`: obstacles and the subject's path read from `cine_gazebo/worlds/cine_field.sdf.template`
  and the subject trajectory, as shapes in the Gazebo world frame.
- `channels/`: one module per metric channel; each takes a flight record and returns a frozen
  dataclass with summary fields and per-frame arrays. Numpy only.
- `gate.py`: the pass/fail rules, thresholds read from `cine_config/config/eval/gate.json`.
- `analysis.py`: the judge. Per-run tables, spread, intervals, paired tests. Uses scipy.
- `standings.py`: builds `results.csv`, `manifest.json` and the generated section of
  `docs/results.md` from a manifest of rows.
- `viz/`: readers that turn a flight record (or a scored run) into drawable series, and one
  renderer.

`cine_cli/cine_cli/eval.py` and `viz.py` add the commands; `cine analyze` stays until `cine eval`
covers what it does, then goes in a separate change.

### Flight record (the contract)

A flight record is a set of named streams. Each stream keeps its own timestamps; nothing is
resampled at extraction, so no information is lost and no interpolation choice is baked in.

| Stream             | Source topic                                     | Fields                               |
| ------------------ | ------------------------------------------------ | ------------------------------------ |
| `drone_true`       | `/gazebo/drone_pose`                             | position (N,3), orientation (N,4)    |
| `subject_true`     | `/gazebo/subject_pose`                           | position (N,3)                       |
| `drone_estimate`   | `/mavros/local_position/odom`, `/vio/odometry`   | position, orientation, velocity      |
| `subject_estimate` | `/cinematography/target_world`                   | position                             |
| `detections`       | `/object_detection/center_point`, `bounding_box` | centre pixel (N,2), box (N,4)        |
| `setpoints`        | `/mavros/setpoint_position/local`                | position, yaw                        |
| `planned_path`     | EGO-Planner's B-spline output                    | control points and knots per message |
| `flown_path`       | the cinematic filter's B-spline output           | control points and knots per message |
| `framing_active`   | the framing flag topic                           | bool                                 |
| `costs`            | the cinematography costs topic                   | named float fields                   |
| `data_age`         | header stamp vs bag receive time, per input      | seconds                              |

Rules every stream obeys, enforced by `validate()`:

- Time is simulation time in seconds, taken from the message header stamp; the bag's receive time is
  kept alongside, because the difference between the two is the data-age metric.
- Each stream records its `frame_id`. Units are SI.
- Quaternions stay in ROS order `(x, y, z, w)` everywhere in the package, and are never converted.
- A stream that was not in the bag is absent, not empty-filled. Channels that need it report
  themselves unchecked.

`record.json` beside the record holds provenance: bag name, SHA-1 of the bag file, extractor
version, and everything from the run's `run.json` (below).

### Metric channels

Each channel resamples what it needs onto its own fixed clock (default 50 Hz, recorded in its
output) and reports SI units. Ground truth is used wherever it exists, because the harness scores
what the drone actually did, not what it believed.

| Channel     | Measures                                                                                                                                                                                                                                                              | Needs                                            |
| ----------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------ |
| `focus`     | distance drone–subject vs the configured focus distance while framing: share within tolerance, worst deviation                                                                                                                                                        | `drone_true`, `subject_true`, `framing_active`   |
| `motion`    | speed, acceleration and jerk while framing: peak and RMS                                                                                                                                                                                                              | `drone_true`, `framing_active`                   |
| `estimate`  | error of the subject estimate against truth; share of framing time with no fresh detection                                                                                                                                                                            | `subject_estimate`, `subject_true`, `detections` |
| `clearance` | minimum distance from the drone (as a sphere of the airframe's radius) to every static obstacle, to the subject (a moving upright cylinder) and to the ground while airborne, over the whole flight, not only while framing; frames in contact; collision-free yes/no | `drone_true`, `subject_true`, scene              |
| `following` | distance between the drone and its commanded setpoint; setpoint jumps at replans                                                                                                                                                                                      | `drone_true`, `setpoints`, `planned_path`        |
| `freshness` | age of depth, detections and costs when used                                                                                                                                                                                                                          | `data_age`                                       |
| `mission`   | reached the final waypoint, time taken, aborted or not                                                                                                                                                                                                                | `drone_true`, waypoint file                      |

The tolerance, focus distance, speed caps and airframe radius come from `cine_config`, read at
score time and copied into `score.json` so a later config change cannot reinterpret an old score.

**Focus target (owner's decision, 2026-11-18).** `focus` scores against the target CINE itself aims
for, as straight-line (3-D) distance from the drone to the subject, with a 0.1 m tolerance. CINE's
cost calculator resolves that target in `_resolve_desired_focus_distance`
(`cine_agent/cine_agent/cost_functions/cost_calculator.py:574`): the anchor focus distance when one
is published, otherwise `DESIRED_FOCUS_DISTANCE_M` from config. The scorer applies the same rule:
the anchor focus distance topic if it is in the bag, otherwise the config value recorded in the
run's `run.json`. Both arms are scored against the same target, so the baseline, which runs no CINE
nodes, is judged against the distance CINE would have aimed for.

### Filming window (owner's decision, 2026-11-18)

The part of a flight that is scored "while framing" is set by an explicit filming cue, not inferred.
The cue is the existing `/cinematography/framing_active` topic (`std_msgs/Bool`), published by a
new mission-cue node in `cine_gazebo` that belongs to the test setup, not to either arm.

- The cue is an input to the system under test, like a director calling "action": the trajectory
  server (`traj_server.cpp:230`), the cost node and the simulated detector already read it and
  switch to their filming behaviour. Both arms receive the identical cue, so both get the chance to
  behave as they would on a real shoot.
- When filming starts and stops is declared in the scenario, on the subject's clock (author's
  call): for example "film while the subject walks from second 10 to second 40 of its script".
  That makes the window the same in both arms whatever the drone does; a drone that arrives late
  is scored while still approaching, which is the missed shot it would be in reality.
- On a real drone the same topic would come from the operator or ground station, so the scorer
  needs no change between simulation and real flights.
- The scorer uses only the recorded cue. A bag without it has `focus` and `motion` reported as
  unchecked. That covers every bag recorded so far, since none of them ran a mission.

### Pass/fail rules

`gate.py` holds every rule. It returns a verdict with `failures` and `unchecked`; the label is
`PASS` only when there are no failures, and `unchecked` is printed beside it. First rules: no
contact with an obstacle, clearance above a minimum, mission completed, speed under the cap while
framing, focus held for at least a set share of framing time, no setpoint jump above a set size,
data age under a set bound. A run with any contact (static obstacle, subject or ground) is a
collision; the judge reports the collision-free rate beside mission success and treats a collided
run like a failed mission. Gazebo actors have no collision geometry, so contact with the subject
is only ever detected here, never by the physics engine. Thresholds live in `cine_config/config/eval/gate.json` and start as
provisional values; choosing them is the owner's call. Exit codes: 0 pass, 1 review, 3 environment
not ready, 4 unreadable input (2 is already typer's usage error in `cine_cli`).

### Judge

`cine eval compare --arm baseline=<runs> --arm cine=<runs>` pairs runs by their condition key
(world, waypoint file, wind, seed) and reports, per metric: each run's value, mean, median,
standard deviation and range per arm; Wilson 95% intervals for rates; and a paired test across
matched conditions. The default test is the exact Wilcoxon signed-rank test (no normality
assumption); the choice of test is the owner's, as in SAMR. Unpaired runs are listed, not dropped
silently. Runs below a minimum length, or that aborted, are excluded by a written rule that the
report prints with the excluded names.

### Standings and provenance

`eval/standings.toml` lists the rows of the results table: arm, condition, run directories,
whether the row is comparable, and a note. `cine eval standings` scores any row whose inputs
changed, then writes:

- `eval/outputs/results.csv`: one row per arm and condition;
- `eval/outputs/manifest.json`: for every row, the bag digests, the git commit and dirty flag the
  flight ran on, the commit the scorer ran on, and the exact `cine` command;
- the section of `docs/results.md` between generated-section markers.

`eval/runs/` is gitignored. `eval/outputs/` holds only the table, the manifest and hand-picked
figures or videos.

### Viewer

`cine viz <run> [--out file.png|file.mp4] [--time T]` draws:

- a top-down map: obstacles, the drone's true path, the flown and planned paths, the subject's true
  and estimated paths, and the focus circle around the subject at time `T`;
- stacked timelines sharing one time axis: focus-distance error with its tolerance band, speed
  against its cap, clearance, data age, with framing periods shaded.

MP4 output animates the map over time beside the timelines with a moving cursor. Rendering uses
matplotlib's non-interactive backend, so it works headless in the container.

### Changes outside `cine_eval` that the harness needs

Found while designing; they are part of Phase 0.

1. **Unclosed bags.** Every bag in `~/cine-bags` lacks `metadata.yaml`, because the runs were
   stopped without the recorder closing the file. They read correctly after `ros2 bag reindex` on a
   copy (checked on one bag: 77.8 s, 14,511 messages). The extractor reindexes a copy when
   `metadata.yaml` is missing and never writes to the original. Separately, the simulator's
   shutdown should stop the recorder cleanly.
2. **Missing topics.** No existing bag contains a mission: all were recorded with the mission off,
   so they hold only Gazebo poses, IMU and the autopilot's position. The recorded-topic list in
   `cine_gazebo/cine_gazebo/recording.py` must add the subject estimate, the costs, the framing
   flag, the planner's and the filter's B-spline outputs, and a way to keep depth-image header
   stamps without recording the images.
3. **`run.json`.** Each run writes a sidecar into its bag directory at start: git commit and dirty
   flag of every checkout in the image (CINE, CINE-Sensing, CINE-Interfaces), the launch arguments,
   the waypoint file and its digest, the seed, and a digest of `cine_config/config/`.
4. **Arms.** The baseline arm is the same launch with `use_cinematic_bspline:=false`, which already
   exists in `cine_bringup/launch/cinematic_run.launch.py:153`. `cine sim gazebo` needs to pass it
   through.
5. **Repeated runs.** A way to fly N runs per arm unattended, one simulator at a time, each with a
   recorded seed. The number of runs per arm is the owner's to set later.

### Scenario set (owner's decision, 2026-11-18)

The single existing mission passes about 5 m west of the subject's path, so plain EGO-Planner would
score near 0% focus held whatever it did, and one layout says little about the algorithm in
general. The owner wants a large, varied set instead: on the order of 30 scenarios, each a
different combination of obstacle layout, drone mission and subject path.

- A scenario is a directory under `cine_gazebo/scenarios/<name>/`: the world file (or the values
  filled into `cine_field.sdf.template`), the mission waypoint file, the subject's trajectory, and a
  `scenario.json` giving its seed and the generator version.
- Scenarios are generated by a seeded generator (author's call: no public scenario set matches
  this world and subject format, and a generator keeps every scenario reproducible from its seed).
  It must keep obstacles, waypoints and the subject inside EGO-Planner's map bounds
  (`PLANNER_MAP_SIZE_*`), keep the spawn point and waypoints clear of obstacles, and vary how close
  the plain path passes the subject, so some scenarios leave the baseline a fair chance.
- The generated scenarios are committed, so the standings rest on fixed inputs; regenerating with
  the same seed and generator version must reproduce them byte for byte.
- The scenario name joins the condition key the judge pairs on, so each scenario flown once per arm
  gives one pair.
- `cine sim gazebo --scenario <name>` selects one; the repeated-run command walks the set.

Build order (author's call): the harness first, scoring the bags that exist; then the recording
changes; then the scenario set. The harness is useful on its own, and the scenario set is about as
much work as the harness.

### Testing

- Channels, gate, judge and standings: unit tests on hand-built flight records with known answers
  (a drone circling a subject at a fixed radius scores 100% focus hold; a drone passing through a
  box scores contact). These run under bare `pytest` wherever numpy is installed.
- Extraction: a test on a small committed MCAP fixture, run inside the image; skipped with a stated
  reason where `rosbag2_py` is absent.
- Each new test is seen failing once before it passes.

## Rigour rules (added after approval, 2026-11-18)

A gap review after approval found ways the results could still mislead. Each rule below closes
one; rule 9 records the owner's decision on the speed-matched baseline.

1. **Analysis plan fixed before the scored runs.** `eval/analysis_plan.toml` names the one primary
   metric (share of filming time with focus held), the secondary metrics, the exclusion rules, the
   test and the significance level. It is committed before the runs it judges. `cine eval compare`
   records the plan's digest and labels any comparison run under a different plan "exploratory".
   Secondary metrics are corrected for multiple comparisons (Holm's method, which tightens each
   test's threshold so that looking at many metrics doesn't manufacture a significant one).
2. **Effect sizes, not only p-values.** Every comparison reports the paired median difference with
   a 95% interval (Hodges–Lehmann estimate from the Wilcoxon test, or a paired bootstrap), so a
   reader sees how big a difference is, not only whether it is unlikely to be chance.
3. **No silent exclusions.** A crashed, aborted or timed-out run is never dropped from the
   comparison. Mission success rate is reported first, per arm. For continuous metrics, a failed
   run takes the metric's worst value, and the comparison is also printed with failed pairs
   removed, so the reader sees both. Excluding failures alone would favour whichever arm fails more.
4. **Noise floor measured.** Gazebo, ArduPilot SITL (the autopilot run as software) and ROS message
   timing are not deterministic, so one scenario flown twice by one arm gives two answers. A
   repeatability block (a few scenarios, several repeats per arm) is flown once per simulator or
   image change, and its run-to-run spread is printed beside every comparison. A difference
   smaller than that spread is reported as not resolved. The number of repeats is the owner's,
   with the run counts.
5. **Held-out scenarios.** The generated scenario set is split by seed into a development part
   (used while tuning the algorithm in Phase 2) and a held-out part, scored only for rows in the
   results table. Tuning on the same scenarios that are scored would overstate the result.
6. **Real-time factor recorded and bounded.** Under load Gazebo runs slower than real time, while
   the depth model's latency is measured in wall time, so a slow simulation hides perception delay.
   `run.json` records the run's real-time factor, the ratio of simulated to wall-clock time, and
   the gate marks runs below a set floor (provisionally 0.9) as REVIEW.
7. **Environment pinned by digest.** The image tag `cine-gazebo-sim:dev` can point at different
   builds, so `run.json` records the image ID, and the ArduPilot and Gazebo versions inside it.
   Runs flown from a working tree with uncommitted changes save the diff beside the bag and its
   digest in `run.json`; standings rows built on such runs are marked `comparable: false` until
   reflown from a commit.
8. **Perception source labelled.** `run.json` records where the subject estimate came from (the
   detector on rendered images, or a node fed Gazebo's true pose). Rows fed the true pose are
   labelled "oracle perception" in the table, because they measure planning alone. Which one the
   Gazebo launch uses today is unverified: it starts with `use_simulation: 'false'`, which turns the
   simulated detector off.
9. **Smoothness reported two ways, and arms stay as they are (owner's decision, 2026-11-18).**
   CINE's slowdown is part of the method, not a fixed cap any planner could copy: the trajectory
   server drops to `focus_max_velocity_mps` (0.45 m/s, `cine_config/config/trajectory_constraints/
focus.json`) only while the filming cue is on, CINE has a subject target, and the drone is within
   the gate of the focus distance (`traj_server.cpp:769-777`, `:820`). The baseline never has a
   target, so it never slows. Capping the baseline to match would remove part of what is being
   tested, so the standings keep two arms. To still show whether CINE is smoother beyond flying
   slower, `motion` reports, while filming:
   - acceleration and jerk, peak and RMS, in m/s² and m/s³, plus peak and mean speed, so the
     slowdown is visible next to them;
   - log dimensionless jerk (squared jerk integrated over the window, scaled by duration⁵ / path
     length², then log), which does not change with speed or duration;
   - spectral arc length (SPARC: the length of the normalised speed spectrum's curve, shorter
     meaning smoother), also independent of speed and duration.
     Both duration- and speed-invariant measures are standard in motion-smoothness studies. A
     speed-matched baseline is left as a later ablation (a variant run to isolate one part of the
     method), outside the standings.
10. **Derivatives smoothed by a stated method.** Jerk computed by differencing a 50 Hz pose three
    times is mostly noise. `motion` fits a Savitzky–Golay filter (a sliding polynomial fit, in scipy)
    with a fixed window before differentiating, and records the window in `score.json`.
11. **Scorer checked end to end.** Besides unit tests on synthetic records, one committed fixture bag
    has a committed expected `score.json`. A test compares them, so any code change that moves a
    score shows up in review instead of silently changing the table.

## Calls made after the builder check

Settled by the author without changing what gets built; the owner can overturn any of them.

- **Rows no channel measures.** `docs/results.md` keeps the drift-correction error, path points
  within 10 cm, light-avoidance tilt and the hand-held bench test, but outside the generated
  section, under a heading saying they are not produced by `cine eval` and when they were last
  measured. No new channels for them in Phase 0: the drift correction is switched off in the
  Gazebo launch (`use_mlp_correction: 'false'`) because the image lacks onnxruntime, and the bench
  test has no bag.
- **Legacy CSV re-score.** The adapter recomputes from logged positions with the new `focus`
  rule, not from the logger's own `within_tolerance` column, so old and new rows use one
  definition. The logger-column figures (the 2026-11-18 audit's per-run means) are printed
  beside them once, for the record. Rows stay `comparable: false`.
- **MP4 without ffmpeg.** If the image has no ffmpeg binary for imageio, `cine viz` writes an
  animated GIF instead and says so; no dependency is added.
- **Baseline arm switches off more than the filter.** `use_cinematic_bspline:=false` also stops the
  CINE intent node (`cinematic_run.launch.py:565`, `:583`). The baseline is therefore "plain
  EGO-Planner", which is the comparison wanted; channels needing CINE-only topics report
  unchecked for that arm, never zero.

## Left to the builder

- Topic names for the costs, the anchor focus distance and the B-spline outputs, read from the
  launch files.
- How the depth header stamps reach the bag (a relay node publishing headers, or another light
  topic).
- Aligning the autopilot's local frame with the Gazebo world frame for the `estimate` channel. The
  rule is fixed: no channel compares streams in different frames unless the record carries the
  transform between them.
- Container invocation details, volume layout, and how `cine eval` finds the bag directory
  (`CINE_BAGS_DIR` already exists).
- The repeated-run command's exact shape.
- Whether to add `cine_eval` to `tools/format.py`'s first-party list (it must be formatted either
  way).
