# CINE: Cinematic Intelligent Navigation Engine

CINE plans camera-drone flights that look like cinematography. It takes the B-spline
trajectories produced by the EGO-Planner local planner and reshapes them so the drone
keeps a subject framed on the rule-of-thirds, holds a focus distance, steers away from
bad light, and moves within velocity and acceleration limits that read as smooth on
camera. A learned correction (a small MLP run with ONNX Runtime) can further adjust the
planned trajectory before the trajectory server follows it.

CINE is the planning half of the system. Its companion repository,
[CINE-Sensing](https://github.com/Kino-Studios/CINE-Sensing), provides the sensing half
on the drone: camera, IMU, YOLO object detection and visual-inertial odometry. CINE-Sensing
publishes the object detections CINE consumes (`/object_detection/center_point`,
`/object_detection/bounding_box`, `/object_detection/dimensions`). In simulation, CINE's
own simulated detector, focus and light publishers stand in for it, so this repository
runs on its own.

## Supported platforms

| Component | Version |
| --- | --- |
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy (Python 3.12) |
| Middleware | Cyclone DDS (`rmw_cyclonedds_cpp`, set by `source_env.sh`) |
| Simulator | The quadrotor simulator bundled with EGO-Planner (`third_party/ego_planner_swarm`) |
| GPU | Optional; only the Depth-Anything depth publisher uses CUDA, and it falls back to CPU |

## Install

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp \
  python3-rosdep python3-colcon-common-extensions python3-venv
mkdir -p ~/cine_ws/src && cd ~/cine_ws/src
git clone https://github.com/Kino-Studios/CINE.git
rosdep install --from-paths . -yi --rosdistro jazzy

# pip-only packages, in a venv that can still see the ROS Python packages.
python3 -m venv --system-site-packages ~/cine_ws/venv
~/cine_ws/venv/bin/pip install -r CINE/requirements.txt

cd ~/cine_ws && colcon build --symlink-install
source src/CINE/source_env.sh
```

`source_env.sh` activates `~/cine_ws/venv` when present, sources ROS 2 Jazzy and then
the workspace's `install/setup.bash`. Source it in every new terminal.

## Quickstart

Each command starts the bundled simulator, EGO-Planner and the CINE nodes for one
pipeline.

```bash
# Cinematic pipeline: EGO-Planner trajectories reshaped by the cinematic B-spline filter.
ros2 launch cine_bringup cinematic_run.launch.py

# Baseline: plain EGO-Planner trajectories, for comparison.
ros2 launch cine_bringup cinematic_run.launch.py use_cinematic_bspline:=false

# Cinematic pipeline with the learned MLP correction (needs your own weights; see below).
ros2 launch cine_bringup cinematic_run.launch.py use_mlp_correction:=true
```

`tools/start_simulation.sh [baseline|constrained]` wraps the first two. With the
simulation running, `tools/collect_baseline_data.sh` and
`tools/collect_constrained_data.sh` record trajectory runs (see "Recorded data"). An
RViz layout for the cinematic displays is installed with `cine_bringup`:

```bash
rviz2 -d "$(ros2 pkg prefix cine_bringup)/share/cine_bringup/rviz/cinema.rviz"
```

## Packages

| Package | Path | Purpose |
| --- | --- | --- |
| `cine_msgs` | `cine_msgs/` | The `CinematographyCosts` message: per-term cinematography costs and attitude setpoints. |
| `cine_config` | `cine_config/` | All tunable constants as JSON in `cine_config/config/`, read by a Python module and a C++ library. `CINE_CONFIG_ROOT` points it at another config directory. |
| `cine_agent` | `cine_agent/` | Cost functions, the cinematic B-spline filter, waypoint generation, simulated sensors and the trajectory data logger. |
| `cine_mlp_correction` | `cine_mlp_correction/` | Applies the learned position correction to the cinematic B-spline. |
| `cine_depth` | `cine_depth/` | Publishes Depth-Anything depth images from the camera feed (optional). |
| `cine_bringup` | `cine_bringup/` | The launch file that starts a full pipeline, and the RViz layout. |
| EGO-Planner-Swarm | `third_party/ego_planner_swarm/` | Upstream local planner and quadrotor simulator, modified; changes listed in its `UPSTREAM.md`. |

## Recorded data

The trajectory data logger (`ros2 run cine_agent trajectory_data_logger`) writes one CSV
per run to its `output_dir` parameter, by default `data/trajectories` under the
directory it starts in (the collection scripts start it from the repository root). It
writes one row per sample, taken at 20 Hz from the latest odometry:

| Column | Unit | Meaning |
| --- | --- | --- |
| `time` | s | Odometry stamp minus the first stamped odometry of the run |
| `odom_x`, `odom_y`, `odom_z` | m | Drone position relative to where the run started |
| `odom_vx`, `odom_vy`, `odom_vz` | m/s | Drone velocity |
| `odom_speed` | m/s | Norm of the drone velocity |
| `planned_x`, `planned_y`, `planned_z` | m | Planned position from the current baseline B-spline at the same time, relative to the start; empty before the first plan |
| `planned_vx`, `planned_vy`, `planned_vz` | m/s | Planned velocity; empty before the first plan |
| `planned_speed` | m/s | Norm of the planned velocity; empty before the first plan |
| `cinematic_framing_active` | bool | Whether cinematic framing was active at that sample (absent from the 2025 runs in the release, which predate it) |

While framing, it also writes `distance_<mode>_<timestamp>.csv` with the drone-to-subject
distance, its target and the error against the tolerance.
`tools/analysis/trajectory_stats.py` and `tools/analysis/trajectory_mean.py` summarise
and average the runs in `data/trajectories/baseline` and `data/trajectories/cinematic`.

## MLP correction and Depth-Anything

**MLP correction.** The trained correction weights are not distributed with this
repository, and so the correction is off by default. To use your own,
`cine_mlp_correction` reads `mlp_correction.onnx`, `mlp_correction.onnx.data` and
`scaler.json` from `cine_mlp_correction/weights/` (git ignores that folder's contents; the
build installs them to the package share). Set `mlp_weights_path` in
`cine_config/config/mlp/mlp-params.json` (or the `mlp_weights_path` launch argument) to
use another directory. Without the model the node passes trajectories through unchanged.
To run it on its own: `ros2 launch cine_mlp_correction mlp_correction.launch.py`.

**Depth-Anything.** `cine_depth` imports Depth-Anything from a source checkout and loads
the `LiheYoung/depth_anything_vits14` weights from Hugging Face on first start:

```bash
git clone https://github.com/LiheYoung/Depth-Anything.git ~/cine_ws/src/Depth-Anything
touch ~/cine_ws/src/Depth-Anything/COLCON_IGNORE
~/cine_ws/venv/bin/pip install -r ~/cine_ws/src/Depth-Anything/requirements.txt
ros2 launch cine_depth depth_publisher.launch.py
```

The node finds the checkout and the venv in that layout by itself. Elsewhere, set
`depth_anything_path` and `venv_site_path` in `cine_config/config/user_params/params.json`
or as node parameters.

## Data and weights download

Large files live in the GitHub Release matching `VERSION`, not in git.
`python3 tools/download_assets.py` fetches all of them, verifies their SHA-256 and places
them where their readers look: recorded trajectories in
`data/trajectories/{baseline,cinematic}/`, their mean trajectories in `data/trajectories/`,
and the January 2026 runs in `data/trajectories/2026-01/`. The MLP weights are not among
them.

`--only <name>` fetches a single file (repeatable); `--dest <dir>` places everything under
another root. A file already present with a different checksum is refused, not
overwritten.

## Work in progress

Early work for later stages. It stays in the tree but is not started by
`cine_bringup`.

- `cine_agent/cine_agent/setpoint_adjustment/nodes/orbit_waypoint_generator_node.py`:
  generates orbit waypoints around the subject from visual-inertial pose and focus depth.
- `cine_agent/cine_agent/setpoint_sender/trapezoidal_setpoint_filter.py`: trapezoidal
  velocity profile for position setpoints, the start of a trapezoidal motion controller.
- `cine_agent/cine_agent/light_vector/vector_analysis.py`: estimates the light direction
  from the camera image.
- `cine_agent/launch/collect_baseline_data.launch.py`,
  `cine_agent/launch/collect_cinematic_data.launch.py`,
  `cine_agent/launch/collect_sim_data.launch.py`: launch-file versions of the data
  collection scripts.
- `cine_depth/launch/depth_publisher.launch.py`: starts the depth publisher, which no
  pipeline uses yet.

## Contributing

```bash
pip install pre-commit && pre-commit install
pytest                              # pure-Python tests, no ROS needed
python3 tools/format.py --check     # whitespace convention
cd ~/cine_ws && colcon test --packages-select cine_config cine_agent cine_mlp_correction
```

Python follows ruff (line length 100, single quotes, Python 3.12) and then
`python3 tools/format.py`. Use descriptive names and keep comments for the non-obvious.
Changes under `third_party/` are recorded in `third_party/ego_planner_swarm/UPSTREAM.md`.
Contributions are accepted under GPL-3.0-only.

## Citation

Cite CINE with the metadata in `CITATION.cff`. Please also cite EGO-Planner
(Zhou et al., IEEE RA-L 2021); `references.bib` has it with the other works CINE builds
on.

## Licence

GPL-3.0-only, see `LICENSE`. Bundled third-party code keeps its own licence; see
`THIRD_PARTY_NOTICES.md`.

## Acknowledgements

- [EGO-Planner-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) by the ZJU FAST
  Lab, the planner and simulator CINE builds on.
- [Depth-Anything](https://github.com/LiheYoung/Depth-Anything) by Yang et al., for
  monocular depth.
- [Boost.odeint](https://www.boost.org/doc/libs/release/libs/numeric/odeint/) by Karsten
  Ahnert and Mario Mulansky, which drives the quadrotor simulator.
