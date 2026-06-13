# CINE: Cinematic Intelligent Navigation Engine

Research system for generating and optimizing cinematographic B-spline trajectories
for autonomous quadrotor drones. CINE extends the EGO-Planner trajectory framework
with cinematographic cost functions (framing, focus distance, lighting, smoothness)
and applies an optional MLP correction layer trained in the companion
[CINE-Training](https://github.com/fishnos/CINE-Training) repository. The workspace
produces setpoint streams consumed by a PX4-compatible flight controller; the MLP
weights produced by CINE-Training are loaded at runtime via `mlp/weights/`.

## Repository layout

```
CINE/
├── trajectory-optimization_ws/          # colcon workspace
│   ├── src/
│   │   ├── ego-planner-swarm/           # EGO-Planner (B-spline planner + sim)
│   │   ├── cinematographic_adjustment_agent/  # cinematographic constraint nodes
│   │   ├── depth_publisher/             # Depth-Anything depth image node
│   │   └── cinematography_msgs/         # custom ROS 2 message definitions
│   ├── collect_baseline_data.sh         # collect EGO-Planner trajectories (no constraints)
│   ├── collect_constrained_data.sh      # collect trajectories with cinematographic constraints
│   ├── collect_constrained_data.env.example  # optional local overrides for the above
│   └── start_simulation.sh             # launch simulation (baseline or constrained)
├── mlp/
│   ├── inference/                       # mlp_correction ROS 2 package (mlp_node)
│   ├── training/                        # training code (see CINE-Training)
│   └── weights/                         # ONNX weights + scaler (downloaded separately)
├── configs/
│   ├── json_constants/                  # authoritative parameter store (JSON)
│   │   ├── camera/                      # camera intrinsics
│   │   ├── cost_functions/              # cost term weights
│   │   ├── light_avoidance/             # light orientation params
│   │   ├── mlp/                         # MLP weights path + flags
│   │   ├── sensing/                     # sensing params
│   │   ├── topics/                      # ROS topic names + QoS depth
│   │   ├── trajectory_constraints/      # velocity, acceleration, yaw limits
│   │   ├── user_params/                 # focus distance, framing anchor, drone ID
│   │   └── waypoint_generation/         # waypoint geometry params
│   ├── python-constants/                # Python dataclasses backed by JSON configs
│   └── cpp-constants/                   # C++ header generated from the same configs
├── scripts/
│   ├── download_weights.py              # fetch ONNX weights from GitHub release
│   ├── check_versions.py                # dependency version checker
│   ├── trajectory_stats.py              # compute per-dataset statistics
│   └── trajectory_mean.py              # resample + average trajectory runs
├── trajectory_data/
│   ├── baseline/                        # baseline trajectory CSVs
│   └── cinematic/                       # constrained trajectory CSVs
├── tests/                               # unit tests
├── source_env.sh                        # source ROS 2 + workspace + venv
├── cinema-config.rviz                   # RViz configuration
├── requirements.txt                     # Python dependencies (pip)
├── VERSION                              # semver string read by config system
└── reference.bib                        # bibliography
```

## Prerequisites

**System**

- Ubuntu 24.04 (ROS 2 Jazzy is the tested and targeted distro)
- ROS 2 Jazzy Desktop (`ros-jazzy-desktop`)
- CycloneDDS middleware (used by all scripts; see below)
- CUDA-capable GPU (required by `depth_publisher` for Depth-Anything inference)
- Python 3.10+

**System libraries**

```bash
sudo apt install \
  libpcl-dev \
  libvtk9-dev \
  libeigen3-dev \
  libopencv-dev \
  python3-pip \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-cv-bridge \
  ros-jazzy-pcl-conversions
```

**Python dependencies** (for `scripts/` and MLP inference outside ROS)

```bash
pip install -r requirements.txt
```

## Build

### 1. Install ROS 2 Jazzy

```bash
sudo apt install ros-jazzy-desktop
```

### 2. Set CycloneDDS as the default middleware

All provided shell scripts set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` internally,
but setting it globally avoids surprises:

```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 3. Build the colcon workspace

```bash
cd trajectory-optimization_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

The `mlp_correction` package (`mlp/inference/`) is a separate colcon package outside
the main workspace `src/`. Build it in the same workspace invocation by passing its
path explicitly, or build it in its own overlay:

```bash
# Option A — build both together from repo root
cd /path/to/CINE
colcon build --base-paths trajectory-optimization_ws/src mlp/inference
```

### 4. Source the environment

`source_env.sh` at the repo root sets `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`,
`DISPLAY`, activates the Python venv if present, and sources
`/opt/ros/jazzy/setup.bash` plus the built workspace overlay in one step:

```bash
source source_env.sh
```

The variable `_CINE_ROOT` is exported to the absolute repo root. The config
system uses `CINE_ROOT` (without leading underscore) if set; otherwise it walks
upward from `configs/python-constants/paths.py` to the directory containing
`VERSION`. No `.env` file is required for normal operation.

## Configuration

All tuneable parameters live in `configs/json_constants/`. The Python layer in
`configs/python-constants/constants.py` reads these files via `ConfigLoader`
(a process singleton) and exposes typed dataclasses (`CinematicConstraints`,
`CinematographyParams`, `UserRequestedParams`, etc.) to Python nodes and launch
files. The C++ header in `configs/cpp-constants/constants.hpp` mirrors the same
values for the EGO-Planner C++ planner.

`ConfigLoader` raises `FileNotFoundError` at import time if any expected JSON file
is absent, naming the missing path. This is intentional fail-fast behaviour: the
node will not start with a broken config.

Key files to edit:

| Parameter group                          | File                                                             |
| ---------------------------------------- | ---------------------------------------------------------------- |
| Focus distance, framing anchor, drone ID | `configs/json_constants/user_params/params.json`                 |
| Velocity, acceleration, yaw limits       | `configs/json_constants/trajectory_constraints/defaults.json`    |
| Focus-mode velocity/acceleration         | `configs/json_constants/trajectory_constraints/focus.json`       |
| Cost term weights                        | `configs/json_constants/cost_functions/cost-params.json`         |
| Light avoidance                          | `configs/json_constants/light_avoidance/orientation-params.json` |
| Camera intrinsics                        | `configs/json_constants/camera/camera-params.json`               |
| MLP weights path + enable flag           | `configs/json_constants/mlp/mlp-params.json`                     |

Paths stored in JSON configs (e.g. `mlp_weights_path`) are repo-relative strings.
`resolve_repo_path()` in `configs/python-constants/paths.py` resolves them against
the repo root at runtime. Paths escaping the repo root via `..` traversal raise
`ValueError`.

## MLP weights

The ONNX weights are not committed to this repo. Download them from the latest
GitHub release:

```bash
python scripts/download_weights.py
```

This fetches `mlp_correction.onnx`, `mlp_correction.onnx.data`, and `scaler.json`
into `mlp/weights/`, verifies SHA-256 digests, and updates
`configs/json_constants/mlp/mlp-params.json` with the repo-relative path. To pin
a specific release tag:

```bash
python scripts/download_weights.py --tag v1.2.0
```

## Running

### Simulation

`start_simulation.sh` launches `ego_planner single_run_in_sim.launch.py` with the
requested mode:

```bash
cd trajectory-optimization_ws
# Baseline: EGO-Planner without cinematographic filter
./start_simulation.sh baseline

# Constrained: EGO-Planner + cinematographic B-spline filter
./start_simulation.sh constrained
```

Both variants accept any `ros2 launch` argument appended on the command line.
`cinematic_run.launch.py` is the underlying launch file driving the full cinematic
pipeline; it is included by `single_run_in_sim.launch.py` and can be invoked
directly for advanced configuration:

```bash
ros2 launch ego_planner cinematic_run.launch.py use_mlp_correction:=true
```

Key launch arguments (defaults come from `configs/json_constants/`):

| Argument                | Default     | Effect                                    |
| ----------------------- | ----------- | ----------------------------------------- |
| `use_cinematic_bspline` | `true`      | enable cinematographic B-spline filter    |
| `use_mlp_correction`    | from config | route bspline through MLP correction node |
| `use_simulation`        | `true`      | start Gazebo/simulator nodes              |
| `drone_id`              | `0`         | drone namespace index                     |

### Data collection

**Baseline trajectories** (EGO-Planner, no cinematographic constraints):

```bash
cd trajectory-optimization_ws
./collect_baseline_data.sh
```

Data is saved to `trajectory_data/baseline/`.

**Constrained trajectories** (cinematographic filter active):

```bash
cd trajectory-optimization_ws
./collect_constrained_data.sh
```

Data is saved to `trajectory_data/cinematic/`. Primary parameters come from
`configs/json_constants/user_params/params.json`. To override individual values
for a single run without editing the JSON, copy the example file and edit it:

```bash
cp collect_constrained_data.env.example collect_constrained_data.env
# edit collect_constrained_data.env
./collect_constrained_data.sh
```

`collect_constrained_data.env` is read by the script only if it exists; values
there override the JSON config for that run only.

### Trajectory analysis

Compute per-dataset statistics:

```bash
python scripts/trajectory_stats.py \
  --trajectory-root trajectory_data \
  --datasets baseline cinematic \
  --output-dir trajectory_data
```

Resample onto a uniform time grid and compute mean trajectories:

```bash
python scripts/trajectory_mean.py \
  --trajectory-root trajectory_data \
  --datasets baseline cinematic \
  --time-step 0.005 \
  --output-dir trajectory_data
```

## Data format

Trajectory CSV columns:

| Column                    | Units                     |
| ------------------------- | ------------------------- |
| `time`                    | seconds                   |
| `pos_x`, `pos_y`, `pos_z` | meters (world frame, ENU) |
| `vel_x`, `vel_y`, `vel_z` | m/s                       |
| `acc_x`, `acc_y`, `acc_z` | m/s²                      |
| `yaw`                     | radians                   |

## Troubleshooting

**Build fails — package not found by colcon**

Ensure you ran `colcon build` from inside `trajectory-optimization_ws/` (or passed
`--base-paths` explicitly). Packages under `ego-planner-swarm/install/` are
pre-built overlays; do not re-build them unless intentionally rebuilding upstream.

**`source_env.sh` warns "venv not found"**

Create and populate the venv:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

**`FileNotFoundError` naming a config path**

`ConfigLoader` raises this when a JSON config file is absent. The error message
includes the full path. Verify `configs/json_constants/` is intact and all
subdirectory JSON files are present.

**`FileNotFoundError` naming `mlp/weights/...`**

Weights have not been downloaded. Run `python scripts/download_weights.py`.

**Nodes not communicating / topics invisible**

Check `ROS_DOMAIN_ID` matches across all terminals. `source_env.sh` sets it to
`0`. To use a different domain: `export ROS_DOMAIN_ID=<n>` before sourcing, or
edit `source_env.sh`.

**`RMW_IMPLEMENTATION` not set — DDS discovery issues**

All scripts set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` internally. If launching
nodes manually, set it in the terminal or add it to `~/.bashrc`.

**`collect_constrained_data.sh` exits with "Could not find install/setup.bash"**

The workspace must be built before running data collection. Run `colcon build`
inside `trajectory-optimization_ws/` first.

## Key technologies

- **EGO-Planner**: ESDF-free gradient-based local planner for quadrotors (B-spline trajectory representation, L-BFGS optimization)
- **ROS 2 Jazzy** with CycloneDDS middleware
- **ONNX Runtime**: MLP correction inference at runtime
- **Depth-Anything**: monocular depth estimation via `depth_publisher`
- **PCL / Eigen / OpenCV**: point cloud processing, linear algebra, computer vision

See `reference.bib` for full bibliography.

## License

See `LICENSE`.

## Citation

If you use this work, please cite:

- EGO-Planner: Zhou et al., "EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors," IEEE RA-L, 2021
- See `reference.bib` for additional citations.
