# Cloud simulator runs on Modal

`run_sim_modal.py` flies the CINE Gazebo simulator on rented Modal CPUs and downloads
everything the flight produced into `eval/runs/modal/<run_id>/`. It runs the same
image, entrypoint and launch file as `docker compose up` in `docker/gazebo-sim/`, so
a cloud flight is the local flight on a bigger machine.

## Setup (once)

```bash
python -m venv scripts/cloud/.venv
scripts/cloud/.venv/bin/pip install -r scripts/cloud/requirements.txt
scripts/cloud/.venv/bin/modal token new      # opens a browser to sign in
```

On Windows the venv's programs live in `scripts\cloud\.venv\Scripts\` instead of
`bin/`, and the runner is started with `scripts\cloud\.venv\Scripts\python.exe`.

The checkouts `CINE-Sensing` and `CINE-Interfaces` must sit next to `CINE`, as they do
for the compose setup. The runner refuses to start when either is missing.

## Running

Every launch command prints the plan and a cost estimate first. Nothing is launched,
and nothing is billed, unless `--confirm-spend` is given. `--fetch` prints no
estimate, since it only downloads, and neither does a run refused because a sibling
checkout is missing or the plan is invalid.

```bash
python scripts/cloud/run_sim_modal.py --mode probe                    # estimate only
python scripts/cloud/run_sim_modal.py --mode probe --confirm-spend    # ~10 min
python scripts/cloud/run_sim_modal.py --mode check --confirm-spend    # build + smoke flight
python scripts/cloud/run_sim_modal.py --confirm-spend                 # 10 min mission, scored
python scripts/cloud/run_sim_modal.py --fetch <run_id>                # download again
```

| Mode      | What it does                                                                                                                   |
| :-------- | :----------------------------------------------------------------------------------------------------------------------------- |
| `probe`   | Checks the container without building: system facts, the Xvfb display, DDS, gz transport, camera rendering, the weights fetch. |
| `check`   | Builds the workspace and runs `smoke_flight.py`, which passes or fails on its own.                                             |
| `mission` | Builds, flies for `--flight-seconds`, records a bag, and scores it with `cine_eval` (extract, score, gate).                    |

Other flags: `--cpu` (default 8 cores), `--memory-gib` (16), `--gpu` (none),
`--flight-seconds` (600), `--nav-source` (`sim` or `vins`), `--wind` (a name from
`cine_gazebo/config/wind_profiles.yaml`).

Run a probe first on a new image. The probe itself takes about 10 minutes, but the
first launch of any mode also builds the image, so the first probe costs more than
later ones. It answers the questions a full run would otherwise answer expensively:
whether DDS discovery works inside Modal's sandbox, and whether the camera renders
without a GPU.

## Cost

The estimate uses Modal's list prices read on 2026-11-20: $0.0000131 per core-second
and $0.00000222 per GiB-second, which is $0.505 an hour for 8 cores and 16 GiB. The
"typical" figure adds up assumed stage durations (build 20 min, weights 5, launch 3,
shutdown 2, scoring 5, plus the flight); none was measured on Modal yet. The
"ceiling" is the function timeout, past which Modal stops the run and billing. The
timeout counts only the function's own running time: container start-up comes on
top, and if Modal preempts the container and restarts the call, the timeout starts
again. The restarted attempt reuses the run id, so it first moves the earlier
attempt's files into `earlier_attempts/1/` (then `2/`, and so on) in the run
directory. The first run also builds the image, which takes a long time (ArduPilot and
Ceres compile from source) and is not in the estimate.

## What a run saves

The container writes straight into `/runs/<run_id>/` on the `cine-sim-runs` volume,
which Modal commits in the background while the run goes on, and commits it once more
before returning. Results survive a dropped connection, and whatever was written
before an out-of-memory kill is most likely saved too. If the terminal disconnects,
the run keeps going; `--fetch <run_id>` downloads it later. A file that cannot be
written locally is skipped with a message, and the rest still download.

| File              | Contents                                                                                                                                                                                                                                                                                                                                                    |
| :---------------- | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `result.json`     | Status, timings, the stop reason, processes that died during the flight and at shutdown, memory headroom, real-time factor, bag, gate label, cost.                                                                                                                                                                                                          |
| `launch.log`      | Everything the entrypoint printed: the colcon build, the weights fetch, every node.                                                                                                                                                                                                                                                                         |
| `plan.json`       | The plan the run was launched with.                                                                                                                                                                                                                                                                                                                         |
| `samples.jsonl`   | One line per sample, 30 s apart plus the sample's own time (up to about 30 s more): load average, Gazebo's real-time factor, and memory headroom three ways: `meminfo_available_gib` (the kernel's figure), `cgroup_headroom_gib` (the container limit minus usage, with reclaimable file cache counted as free), and `headroom_gib`, the lower of the two. |
| `deep_sample.txt` | Once, halfway through the flight: the busiest processes and the rate of each topic the flight depends on.                                                                                                                                                                                                                                                   |
| `bag/`            | The rosbag2 recording, reindexed first if shutdown left it without `metadata.yaml`.                                                                                                                                                                                                                                                                         |
| `eval/`           | `cine_eval` output for a mission.                                                                                                                                                                                                                                                                                                                           |
| `colcon_log/`     | The colcon build log, when the stack never reached launch.                                                                                                                                                                                                                                                                                                  |

The runner prints a short summary from `result.json` and exits 0 when the status is
`ok`, `check_passed` or `probe_done`, and 1 otherwise.

| Status                 | Meaning                                                                                                                                                         |
| :--------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ok`                   | The mission flew its full time, the bag closed, and no node died mid-flight.                                                                                    |
| `process_died`         | The flight finished, but at least one node died before the stop.                                                                                                |
| `no_bag`               | The flight finished, but no complete bag was found even after reindexing.                                                                                       |
| `stack_exited`         | The stack exited on its own before the flight time ran out.                                                                                                     |
| `failed_before_launch` | The build or weights fetch failed; see `colcon_log/` and `log_tail`.                                                                                            |
| `memory_stop`          | Free memory fell below 1 GiB after launch, and the stack was stopped so the bag could close. Low memory during the build is recorded but does not stop the run. |
| `deadline_stop`        | The run reached its internal deadline, set before Modal's timeout.                                                                                              |
| `check_passed`         | The smoke flight passed.                                                                                                                                        |
| `check_failed`         | The smoke flight failed.                                                                                                                                        |
| `probe_done`           | Every probe step ran; each step's exit code is in `result.json`.                                                                                                |
| `driver_error`         | The driver itself raised; the traceback is in `result.json`.                                                                                                    |

## How a run stops

The stack is started in its own process group. When the flight time is up, memory
runs low, or the deadline arrives, the driver sends SIGINT to the whole group so
rosbag2 can close its bag, waits 60 s, then sends SIGTERM, waits 30 s, then SIGKILL.
In `check` mode the first wait is 120 s, because `smoke_flight.py` runs its own
three-stage stop of up to 90 s and must finish it before the driver escalates.
The flight time in `result.json` ends when the stop is decided; the shutdown after
it is not counted. Crash lines printed after the stop are reported apart from those
before it, because nodes that exit non-zero while shutting down are not flight
failures.

The internal deadline also bounds what comes after the flight. Reindexing the bag and
each `cine_eval` step get only the time left before the deadline, and a step with
less than 60 s left is skipped: reindexing adds a warning, and scoring records
`"skipped": "out of time"` with no gate label.

## Tests

```bash
pytest scripts/cloud
```

The tests run the real driver against a fake stack, a shell script that writes a bag
when interrupted, so they need neither ROS nor Modal. The process tests are POSIX
only and skip on Windows.
