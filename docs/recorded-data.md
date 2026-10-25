# Recorded data

## Collecting runs

With the simulation running, `tools/collect_baseline_data.sh` and
`tools/collect_constrained_data.sh` record trajectory runs. The
collection scripts start the simulated focus publisher, waypoint generator and logger
themselves with their own parameters, and so they launch the simulation with
`start_cinematography_nodes:=false` to keep a second copy of those nodes from running.

## Logger output

The trajectory data logger (`ros2 run cine_agent trajectory_data_logger`) writes one CSV
per run to its `output_dir` parameter, by default `data/trajectories` under the
directory it starts in (the collection scripts start it from the repository root). It
writes one row per sample, taken at 50 Hz from the latest odometry:

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

## Downloading the data

Large files live in the GitHub Release matching `VERSION`, not in git.
`python3 tools/download_assets.py` fetches all of them, verifies their SHA-256 and places
them where their readers look: recorded trajectories in
`data/trajectories/{baseline,cinematic}/`, their mean trajectories in `data/trajectories/`,
and the January 2026 runs in `data/trajectories/2026-01/`. The MLP weights are not among
them.

`--only <name>` fetches a single file (repeatable); `--dest <dir>` places everything under
another root. A file already present with a different checksum is refused, not
overwritten.
