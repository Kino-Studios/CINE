# Changelog

All notable changes to CINE are documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
Versioning follows [Semantic Versioning](https://semver.org/).

---

## [Unreleased]

Themes of ongoing work on branch `simulation` (C1–C4 phases):

- **Hygiene**: version declarations across setup.py and package.xml synced to
  root VERSION (1.0.0); stale 0.0.x strings removed.
- **Config portability**: hover_timeout and related constants migrated to
  global JSON config; hardcoded absolute paths replaced with relative/dynamic
  resolution in `find_bright_region.py` and collection scripts.
- **Error-handling hardening**: camera intrinsics source corrected; ROS node
  error paths tightened; insecure temporary file creation (CWE-377) fixed in
  collection launcher and shell scripts.
- **Deduplication**: dead debug code pruned from BsplineOptimizer; unused
  imports removed from collector scripts; redundant launch parameters
  consolidated.
- **CI tooling**: `scripts/check_versions.py` added — walks repo, asserts all
  manifests match VERSION, exits non-zero on mismatch.
- **Dependency hygiene**: requirements.txt lower bounds converted to bounded
  ranges to prevent silent major-version breaks.

---

## [1.0.0] — MLP tag (latest released)

This release represents the first full end-to-end system integration: ego
planner + MLP correction node + cinematographic adjustment pipeline running on
real hardware.

### Added

- MLP setpoint correction interceptor node (`mlp_correction`). Subscribes to
  `/drone_0_planning/cinematic_bspline`, applies learned position correction,
  publishes `/mlp/corrected_bspline`.
- Full hardware support: constants integration via JSON config files; all
  scripts wired to config rather than inline literals.
- `cinematographic_adjustment_agent` fully wired with cost-function
  calculator, waypoint sender, cinematic B-spline filter, trajectory data
  logger, noisy odometry node, and cinematic waypoint generator.
- Simulation nodes: `simulated_object_detector`, `simulated_focus_publisher`,
  `simulated_light_publisher`, `light_vector_analysis`, baseline/cinematic
  collection scripts.
- `depth_publisher` package: depth image publisher using Depth-Anything with
  CUDA acceleration.
- `cinematography_msgs`: custom ROS 2 message definitions for the system.
- MLP training completed and weights moved to external CINE-Training repo;
  only inference retained here.

### Changed

- Codebase-wide cleanup and reorganisation: consistent naming conventions,
  removed extraneous scripts and large log files, updated `.gitignore`.
- Waypoint dynamics improved: Bezier transitions, anchor point logic,
  waypoint lookahead, proper yaw handling.
- Cost functions adjusted and tuned for real flight results.
- Trajectory statistics analysis added (`trajectory_stats.py`,
  `trajectory_mean.py`).

---

## [0.3.0] — Simulation validation

Near-demo-ready trajectory generation. All core algorithmic milestones
reached in simulation before hardware integration.

### Added

- Full constants implementation with JSON config files.
- Cost-based waypoint generation with anchor point logic.
- Simulated object/light detection incorporated into planning loop.
- Waypoint lookahead implemented.
- Cinematic framing logic established.

### Changed

- Trajectory generation achieving consistent positional accuracy; Bezier
  transitions and waypoint splicing functioning.
- High-order derivative clamping via config-file constraints for smoother
  trajectories.
- Data collection infrastructure: proper ROS launch integration, data
  scaling, statistical interpolation.

### Fixed

- Intermediate velocity bug in B-spline trajectory generation.
- Anchor point logic debugged and validated in simulation.

---

## [0.2.0] — Ego planner integration

Ego planner communication, depth analysis, and first working trajectory
generation milestone.

### Added

- Ego planner map cloud integration and working trajectory generation.
- Vector light analysis in cinematographic script.
- Functioning depth analysis model (optimised).
- Waypoint sender node (renamed from room waypoint sender with legacy import
  compatibility).
- `so3_control` and `so3_quadrotor_simulator` submodules integrated.

### Changed

- Repository revamped with novel cost function node structure.
- General codebase refactoring: increased waypoint dynamics, Bezier
  incorporation, improved scalability.
- Gitignore updated to exclude large log files.

### Fixed

- Light avoidance functioning after extended debugging.
- Ego planner rebuild and stabilisation.

---

## [0.1.0] — Initial project structure

Foundation: initial commit establishing repository layout, ego planner
submodule, and early depth analysis scripts.

### Added

- Repository structure: `trajectory-optimization_ws`, `scripts`, `configs`,
  `docs` directories.
- Ego planner submodule (EGO-Planner-v2 swarm variant).
- Early depth analysis and cinematographic waypoint generator scripts.
- Initial ROS 2 package manifests for `cinematographic_adjustment_agent`,
  `cinematography_msgs`, `depth_publisher`.
- CI foundations: `.gitignore`, `requirements.txt`, `source_env.sh`.
