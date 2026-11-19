# Gazebo simulator

A simulator that flies the real drone software in a Gazebo Harmonic world (Gazebo is the
physics and camera simulator) without the drone. ArduPilot's own flight firmware, built to
run on a PC (called SITL, software in the loop), flies a simulated quadcopter. MAVROS (the
bridge between ROS 2 and the autopilot), the CINE-Sensing nodes that run on the Pi, and the
CINE ground nodes all run unchanged, and every perception node reads camera frames rendered
by Gazebo. It runs in one Docker container on a Linux PC. Nothing here has been run on the
author's Mac, because Docker there has no GPU passthrough; statements about what a run
does come from reading the code unless marked otherwise.

## What it does not model

- The Wi-Fi link between the Pi and the ground PC. Everything shares one container.
- Rolling shutter (the camera reading its image line by line) and motion blur.
- The Pi's limited CPU. Depth, detection and VINS run on the ground PC's hardware, so the
  simulated perception latency is optimistic.

Reports of simulated flights should say so.

## Building the image

From the CINE root:

```bash
docker compose -f docker/gazebo-sim/compose.yaml build
```

Every source in the image is pinned to an exact version, so a rebuild gives the same image:
the base image digest, the dated Ubuntu and ROS package snapshots, the ArduPilot
(`Copter-4.7.2`) and `ardupilot_gazebo` commits, and the Python packages. The pins are the
`ARG` lines and `pip install` list in `docker/gazebo-sim/Dockerfile`. The build records
every installed Ubuntu and ROS package version inside the image, at `/apt-versions.txt`;
read it with `docker run --rm --entrypoint cat cine-gazebo-sim:dev /apt-versions.txt`. The
Depth Anything 3 and YOLO weights are fetched by hash into a cache volume at container
start (`docker/gazebo-sim/fetch_weights.py`), not baked into the image.

CINE, CINE-Sensing and CINE-Interfaces are mounted from the host and built at container
start, so code edits need no image rebuild. The three repositories must sit side by side;
`CINE_SENSING_DIR` and `CINE_INTERFACES_DIR` override the paths.

## Running it: `cine sim gazebo`

```bash
cine sim gazebo                      # window, mission, calm wind, nav_source sim
cine sim gazebo --headless --wind gusty
cine sim gazebo --check              # smoke flight, exits with its result
```

The command checks that Docker and the image exist (exit code 3 if not), then runs the
container through `docker/gazebo-sim/compose.yaml`. For the Gazebo window on an X11
desktop, run `xhost +local:root` first.

| Option                   | Default | Meaning                                                                        |
| ------------------------ | ------- | ------------------------------------------------------------------------------ |
| `--nav-source sim\|vins` | `sim`   | Where the autopilot gets its position; see the next section.                   |
| `--headless`             | off     | Run Gazebo's server without a window.                                          |
| `--wind calm\|gusty`     | `calm`  | Wind profile; see "Wind".                                                      |
| `--no-mission`           | off     | Start the world and the smoke-flight starter only; no planner, no depth model. |
| `--check`                | off     | Take off, fly to one setpoint 2 m away, and exit non-zero on failure.          |
| `--json`                 | off     | Print one JSON object on stdout; the simulator's output goes to stderr.        |

The launch file `cine_gazebo/launch/gazebo_sim.launch.py` has two more arguments the
command does not pass: `world` (default `cine_field`) and `airframe` (default `iris`; see
"Which drone is simulated").

## Checking the topic exchange: `cine contract exchange`

With the simulator (or the drone) running, this compares the live ROS 2 graph with the
CINE-Interfaces topic contract and counts the messages on every contract topic:

```bash
cine contract exchange                      # simulator topics, 5 s window
cine contract exchange --mode hardware      # Pi topics, with the drone on the network
cine contract exchange --seconds 10 --json
```

It exits with code 1 when a node the contract names for that mode is missing from the
graph, or when a topic the contract gives a publisher carries no messages during the
window. A silent topic whose stream the contract marks as an accepted mismatch is reported
as info instead. Exit code 3 means ROS 2 is not sourced or `cine_interfaces` is not
installed. Rates are wall-clock messages per second, counted by
`tools/measure_topic_rates.py` in one process subscribed to every topic at once. With
`--no-mission` the planner, depth and cinematic nodes never start, so their endpoints are
reported missing.

## The two `nav_source` modes

- `sim`: ArduPilot takes its position from Gazebo's simulated GPS for the whole flight.
  VINS (the visual-inertial position estimator) still runs, and its estimate is recorded
  beside Gazebo's true pose. The planner reads the autopilot's own estimate,
  `/mavros/local_position/odom`.
- `vins`: the drone takes off on simulated GPS, flies a fixed excitation pattern, and then
  ArduPilot is switched to take its position from VINS. The planner reads `/vio/odometry`.

The excitation step exists because a single-camera VINS cannot start while hovering or
climbing straight up: it needs enough sideways motion and acceleration to fix its metric
scale. The pattern is a list of setpoints that moves the drone along all three axes and
turns it, in `cine_gazebo/config/excitation_pattern.yaml`. The flight starter
(`sitl_flight_starter`) repeats it until `/vio/odometry` arrives steadily and its velocity
agrees with the autopilot's, then switches position source, holds for a few seconds, and
starts the mission. If VINS is not ready after the allowed repeats, or the hold fails, it
lands on GPS and exits non-zero. That is a finding, not a retry.

Open design question (Task 10 finding, not yet resolved): in `vins` mode the Pi's
`vio_publisher` forwards VINS poses to `/mavros/vision_pose/pose` without rotating them
into the autopilot's heading or shifting them to the autopilot's origin, and ArduPilot's
`VISO_TYPE 1` driver does no alignment either, so the hold after the source switch will
probably fail and the flight starter lands on GPS. Whether and where to add the alignment
is the user's decision. The source reading behind this is in the next section.
This finding is from reading the code, not from a run, and is unverified.

## Handing navigation to VINS

In ArduPilot's MAVLink visual-odometry driver at the pinned commit (`fb773567`), the calls that align the external origin and yaw to the autopilot's estimate are empty, and nothing sets the flags that would align position, so VINS poses reach the EKF (the autopilot's state estimator) as MAVROS forwards them, and heading comes from the compass in both source sets. When the flight starter switches to source set 2, the EKF flags a horizontal position reset and, on the next VINS sample, snaps its position to VINS's own, so any offset between VINS's origin and the EKF origin appears as a position jump that the 0.5 m hold check reports as a failed hold. These two sentences come from reading the source, not from a run, and are unverified until `cine sim gazebo --nav-source vins` has been run.

## Wind

`--wind` picks an entry from `cine_gazebo/config/wind_profiles.yaml`. `calm` is no wind.
`gusty` is a 4 m/s mean wind blowing towards +x with random variation (standard deviations
1.5 m/s in speed, 0.3 rad in direction and 0.5 m/s vertically). The profile is written
into the world file as Gazebo's wind, so Gazebo's physics applies it to the drone.
ArduPilot's `SIM_WIND_*` parameters are not used: they act inside SITL's built-in physics model, which
is not the one flying here because Gazebo supplies the physics, so they would have no effect on
the drone. This reasoning is unverified; a gusty run settles it.

## Which drone is simulated

The default `airframe:=iris` flies the stock Iris quadcopter from `ardupilot_gazebo`, with
the real drone's camera and IMU noise. Results from it describe the Iris, not the user's
drone, and should be labelled that way.

`airframe:=source_one` (only through the launch file, not `cine sim gazebo`) flies a model
of the real 5-inch TBS Source One. Its rotor gains and its mass distribution are estimated:
the model's total mass is 0.7611 kg against the measured 0.757 kg, and the rotor control
gain was set by guess. Label its results as coming from an estimated model.

## Recorded data

Every run, smoke flight included, records a ROS 2 bag to `/bags/<date>_<nav_source>_<world>_<wind>`
inside the container, which is `~/cine-bags` on the host (`CINE_BAGS_DIR` overrides it).
The list is in `cine_gazebo/cine_gazebo/recording.py`: Gazebo's true drone and subject
poses, `/vio/odometry`, `/mavros/local_position/odom`, every setpoint stage from the
planner's `pos_cmd` to the `/mavros/setpoint_*` topics, the YOLO detections
(`/object_detection/*`), and the IMU.

It does not record `/camera/image_raw`: 1152x648 colour at 30 Hz is about 67 MB/s. To judge
whether the subject stayed in frame, run without `--headless` and watch, or use the
recorded detections. The depth image and planner camera pose are not recorded either.

## Scoring a run

Three steps turn a recorded bag into a verdict. Each runs `python -m cine_eval` inside the
`cine-gazebo-sim:dev` image; `cine eval` builds that `docker run` for you (repository
read-only, `eval/` and `docs/` writable, the bags directory read-only), and `--dry-run`
prints the command instead. A relative path is read from your current directory, and every
path must lie under the repository (the bags directory for `extract`), or the command exits 2.

```
cine eval extract ~/cine-bags/<bag>    # writes eval/runs/<bag>/record.json and record.npz
cine eval score eval/runs/<bag>        # writes score.json and metrics.npz beside the record
cine eval gate eval/runs/<bag>         # prints PASS or REVIEW; exit 1 means REVIEW
```

Every command takes `--json`: one JSON object on stdout. Exit 3 means Docker or the image is
missing, 2 means a bad argument (a bag outside the bags directory, for instance), 4 means a
bag, record or score could not be read.

Comparing two arms and rebuilding the results table:

```
cine eval compare --arm base=eval/runs/a1,eval/runs/a2 --arm cine=eval/runs/b1,eval/runs/b2 \
    --repeatability base=eval/runs/r1,eval/runs/r2 --name first
cine eval standings
```

`--repeatability NAME=DIR[,DIR...]` names repeat flights of one compared arm, and may be given
once per arm; a NAME that no `--arm` uses exits 2. The comparison is written to
`eval/runs/compare/<name>.json` and `.md`. It is labelled preregistered only when
`eval/analysis_plan.toml` is committed and unchanged: the host passes the SHA-1 of the plan at
`HEAD` into the image as `CINE_EVAL_PLAN_COMMITTED_DIGEST`, because the image has no git.
Running `python -m cine_eval compare` by hand without that variable gives "exploratory".

Current bags do not record `framing_active` or `target_world`, so the `focus`, `motion` and
`estimate` channels stay unchecked and the gate lists their rules as unchecked. A PASS on such
a bag only means that the rules that did run (collision, clearance, data age) found nothing;
read the `unchecked:` lines. The gate says REVIEW whenever the collision check did not run.

## Mission limits

- The planner mission accepts at most 5 waypoints (`MAX_WAYPOINTS` in
  `cine_bringup/cine_bringup/mission_waypoints.py`). The Gazebo list,
  `cine_gazebo/config/mission_waypoints.yaml`, has 4.
- The depth model's output is resized to 640x360 (the Gazebo camera is 1152x648) because the
  planner's depth buffer takes that size; the planner is given the intrinsics scaled to
  match. The size is the depth publisher's `output_width` and `output_height` parameters.
  Setting only one keeps the aspect ratio; both zero keeps the input size.
- The planner's map is 30 x 56 x 5 m centred on the start point, with a ceiling at 4.5 m,
  sized to hold the world's obstacles and the waypoints (`gazebo_sim.launch.py`,
  `PLANNER_MAP_SIZE_*`). A different world or waypoint list needs the map checked.

## Two-machine network check (one-off)

The simulator does not model the Wi-Fi link, so the real two-machine setup is proved once
by hand, with no results recorded:

1. On the Pi, start the CINE-Sensing nodes. On the ground PC, start the CINE ground nodes.
   Use the same `ROS_DOMAIN_ID` on both and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
2. On either machine, run `python3 -m cine_interfaces.check --live` from the
   CINE-Interfaces checkout. It compares the live topics with the shared contract and fails
   on any missing or mismatched stream.

This procedure is unverified: it has not been run.

## Unverified

- Everything in this guide about runtime behaviour: no Gazebo, Docker or ROS run has been
  seen by the author of this page. The code was read, and the unit tests that need no
  simulator pass.
- The ArduPilot behaviour in the `vins` section, as marked there.
- `--check` in `vins` mode, and whether VINS initialises on rendered images.
- The 640x360 depth size and the planner map size being enough for the full mission.
