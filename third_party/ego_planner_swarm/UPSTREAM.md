# Upstream record: ego_planner_swarm

## Origin

This directory is a modified copy of EGO-Planner-Swarm by the ZJU FAST Lab
(<https://github.com/ZJU-FAST-Lab/ego-planner-swarm>), taken from a community
ROS 2 port of that project. Which port, and which commit of it, is not
recorded: the tree was imported into this repository without history in
commit `c9525f9` (2025-11-08, "unstaged changes..").
Treat `c9525f9` as the base when comparing.

## Licence

GNU General Public License version 3, see `LICENSE` in this directory.
Third-party code bundled by upstream keeps its own licence, notably Boost.odeint
under `src/uav_simulator/so3_quadrotor_simulator/include/ode/` (Boost Software
License 1.0, `LICENSE_1_0.txt` beside it).

## Modification notice (GPLv3 section 5a)

Modified by Kino-Studios, 2025-2026. The files below differ from the imported
base. Everything else is as imported, apart from the repository housekeeping
listed under "Reorganisation".

### Behaviour changes since the base (`git diff --stat c9525f9 HEAD`, 15 files)

| File (under `src/`)                                                    | Change                                                                                                                                                                                               |
| ---------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `planner/plan_manage/src/traj_server.cpp`                              | Large extension (+1623 lines changed): cinematic B-spline input, focus target, target distance, framing and light-vector subscriptions, `CinematographyCosts` input, and visualisation publishers. |
| `planner/plan_manage/launch/cinematic_run.launch.py`                   | New. First-party CINE launch file that starts the cinematic pipeline; since moved to `cine_bringup` (see below).                                                                            |
| `planner/plan_manage/launch/single_run_in_sim.launch.py`               | Deleted.                                                                                                                                                                                             |
| `planner/plan_manage/launch/default.rviz`                              | RViz layout extended for the cinematic displays.                                                                                                                                                     |
| `planner/plan_manage/launch/advanced_param.launch.py`                  | Lower `max_vel` (1.55) and higher `max_acc` (3.7) defaults; `realworld_experiment` and `use_sim_time` launch arguments; un-namespaced cloud remap.                                                   |
| `planner/plan_manage/launch/real_hardware.launch.py`                   | Removed the depth, odometry and camera-pose topic arguments.                                                                                                                                         |
| `planner/plan_manage/launch/simulator.launch.py`                       | Passes `enable_motors: True` to the simulated controller.                                                                                                                                            |
| `planner/plan_manage/src/ego_planner_node.cpp`                         | Spins on a multi-threaded executor instead of `rclcpp::spin`.                                                                                                                                        |
| `planner/plan_manage/src/ego_replan_fsm.cpp`                           | Clamps the waypoint switch radius to 0.3 m; extra logging.                                                                                                                                           |
| `planner/plan_manage/src/planner_manager.cpp`                          | Goal-too-close threshold 0.2 m to 0.1 m, and a fallback direction instead of failing.                                                                                                               |
| `planner/plan_manage/CMakeLists.txt`, `planner/plan_manage/package.xml` | Depend on the CINE message package, `tf2`, `tf2_geometry_msgs`; compile the CINE C++ constants into `traj_server`.                                                                                  |
| `planner/bspline_opt/src/bspline_optimizer.cpp`                        | Commented-out debug code removed (no logic change).                                                                                                                                                 |
| `uav_simulator/so3_control/src/so3_control_component.cpp`              | Motors default to disabled, controlled by a new `enable_motors` parameter.                                                                                                                           |
| `uav_simulator/so3_quadrotor_simulator/launch/simulator_example.launch.py` | Passes `enable_motors: True`.                                                                                                                                                                    |

### Reorganisation for open-source release (2026-10-15)

Committed as `11a6fbc` ("Vendor ego-planner under third_party for Jazzy") and
`f4e1cc2` (package renames).

- Moved from `trajectory-optimization_ws/src/ego-planner-swarm/` to
  `third_party/ego_planner_swarm/`; `Readme.md` renamed `README.md`.
- The CINE message package was renamed `cinematography_msgs` to `cine_msgs`.
  `planner/plan_manage/{CMakeLists.txt,package.xml,src/traj_server.cpp}` were
  updated to match (package name, include path, C++ namespace). The message
  type `CinematographyCosts` and every topic name are unchanged.
- `planner/plan_manage/CMakeLists.txt`: the relative path to the CINE C++
  constants (`configs/cpp-constants`) goes up five directories instead of six,
  to match the new location.
- Removed committed colcon `install/` output, `.vscode.zip`, and a ROS 1
  `.gitignore`.
- Every `package.xml` (20 files): the placeholder maintainer, licence and description
  (`TODO`) were filled in, with `fishnos` as maintainer, ZJU FAST Lab as author and
  `GPL-3.0-only` as licence; dependency names were changed to keys rosdep resolves on
  Jazzy (`Eigen3` to `eigen`, `PCL` to `libpcl-all-dev`, `OpenCV` to `libopencv-dev`,
  `Armadillo` and `Boost` lowercased, `tf` to `tf2`), and the ROS 1 `dynamic_reconfigure`
  dependency was dropped.
- Boost.odeint: removed `libs/numeric/odeint/performance/performance.py`
  (Python 2, not parseable by Python 3), the generated HTML documentation
  (`libs/numeric/odeint/doc/html/`, `libs/numeric/odeint/index.html`); added
  `LICENSE_1_0.txt`. Nothing in the build referenced the removed files.

### Changes after the reorganisation

| File (under `src/`)                                  | Commit          | Change                                                                                                                                                                       |
| ---------------------------------------------------- | --------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `planner/plan_manage/CMakeLists.txt`                 | `a7ce900`       | `find_package(cine_config)` and link `traj_server` to `cine_config::cine_config_cpp`, replacing the relative path into the old `configs/cpp-constants` directory.         |
| `planner/plan_manage/package.xml`                    | `a7ce900`       | Depends on `cine_config`.                                                                                                                                                     |
| `planner/plan_manage/src/traj_server.cpp`            | `a7ce900`       | Includes `cine_config/constants.hpp` instead of `constants.hpp`; no logic change.                                                                                           |
| `planner/plan_manage/launch/cinematic_run.launch.py` | `a7ce900`       | Moved out of this tree to `cine_bringup/launch/cinematic_run.launch.py` (first-party). There it reads constants from `cine_config` and no longer starts the deleted `light_vector_analysis` node. |
| `planner/plan_manage/launch/advanced_param.launch.py` | 2026-11-14      | Odometry, camera-pose and depth remaps add the `drone_<id>_` prefix only to relative topic names; a name starting with `/` is used unchanged. Relative names resolve as before. |
| `planner/plan_manage/launch/advanced_param.launch.py` | 2026-11-15      | New `virtual_ceil_height` launch argument (default 2.9, the previous literal) sets `grid_map/virtual_ceil_height`; `cine_bringup/launch/cinematic_run.launch.py` passes it through so the Gazebo launch can raise the ceiling. |
| `planner/plan_manage/launch/advanced_param.launch.py` | 2026-11-24      | New `odom_depth_timeout` launch argument (default 1.0, the upstream value) sets `grid_map/odom_depth_timeout`; `cine_bringup/launch/cinematic_run.launch.py` passes it through as `planner_odom_depth_timeout` so the Gazebo launch can lengthen it to 3.0 s. |
| `planner/plan_manage/launch/advanced_param.launch.py` | 2026-11-26      | `grid_map/obstacles_inflation` raised from 0.099 to 0.25, the CINE drone's radius (`airframe_radius_m` in `cine_config/config/eval/scoring.json`). GridMap rounds it up to whole 0.1 m cells, so obstacles grow by 0.3 m instead of 0.1 m. `cine_gazebo/test/test_planner_obstacle_padding.py` keeps the two values in step. |

### Trajectory timing on the node clock (2026-11-22)

Upstream stamped trajectories with `rclcpp::Clock().now()` (system time) and `traj_server`
sampled them with a standalone `rclcpp::Clock(RCL_ROS_TIME)`, which is never attached to
`/clock` and so also reads wall time. In a simulator that runs slower than real time the
planned speed became a wall-clock speed (measured: 6.89 m/s commanded for a 0.83 m/s plan at
a real-time factor of 0.12). Every clock that stamps or samples trajectory time now comes
from the node's own clock, which follows `/clock` when `use_sim_time` is true.

| File (under `src/planner/`)                          | Change                                                                                                                                                                                                                                   |
| ---------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `plan_manage/src/traj_server.cpp`                    | A file-scope `node_clock` (the node's clock) replaces every standalone `rclcpp::Clock(RCL_ROS_TIME)`: the command callback, the B-spline handler's `time_last_`, the framing callback and the control-point marker stamp. The 10 ms command timer is `create_timer` (node clock) instead of `create_wall_timer`, so the command callback fires per simulated 10 ms and `dt` is never zero between ticks. `start_time_` and `target_stamp_` are constructed with `RCL_ROS_TIME`. |
| `plan_manage/src/planner_manager.cpp`, `plan_manage/include/ego_planner/planner_manager.h` | New member `node_clock_`, set in `initPlanModules`. Trajectory start stamps (`updateTrajInfo` in the planner, the optimizer-failure fallback and the emergency stop), the global-trajectory stamps and the reuse time `t_cur` read it. `checkCollision` (swarm only) now treats a zero stamp as "first planning not started" instead of a stamp below 1e9, which never holds on simulator time. |
| `plan_manage/src/ego_replan_fsm.cpp`                 | `node_->now()` for the execute-state and replan `t_cur`, the collision check's `t_cur` and `t_cur_global`, and the display stamp. The swarm broadcast check builds the message time with `RCL_ROS_TIME` instead of a system-time clock. |
| `bspline_opt/src/bspline_optimizer.cpp`, `bspline_opt/include/bspline_opt/bspline_optimizer.h` | New member `node_clock_`, set in `setParam`; `calcSwarmCost` and `calcMovingObjCost` read it for `t_now`. |
| `traj_utils/include/traj_utils/plan_container.hpp`   | `global_start_time_` and the two `start_time_` members are constructed with `RCL_ROS_TIME`. A default-constructed `rclcpp::Time` has the system type, and subtracting it from a node-clock reading throws. |

Clocks left on wall time because they only measure how long a computation took, with both
readings from the same clock: `planner_manager.cpp` `t_start`, `now`, `t_opt`, `t_refine`
(the optimizer and refinement timers); `bspline_optimizer.cpp` `t0`, `t1`, `t2` (the
optimizer's own iteration timing); `path_searching/src/dyn_a_star.cpp` `time_1`, `time_2`
(the A* search timeout). Left unchanged and not used by the CINE launch: the moving-obstacle
simulator and predictor (`plan_env/src/obj_generator.cpp`, `obj_predictor.cpp`), the TCP
swarm bridge (`rosmsg_tcp_bridge/src/bridge_node.cpp`), and the marker stamps in
`traj_utils/src/planning_visualization.cpp`, which only label RViz markers.
`exec_timer_` and `safety_timer_` in the replan state machine stay wall timers: they set the
loop cadence, and every time they read is on the node clock.

### Recovery from depth loss (2026-11-24)

Upstream latched the map's depth-lost flag (`flag_depth_odom_timeout_`) until restart, and the
replan state machine switched the fail-safe off on the first depth loss, so one late depth
frame ended the flight in a permanent emergency stop. Measured in the Gazebo run
`eval/runs/modal/2026-11-24_21-16-49_mission_sim_calm_4e71`: with depth estimated on CPU,
map updates arrived about 1.7 simulated seconds apart against the 1.0 s timeout. The flag now
clears on the next fresh depth and pose update, and the drone leaves emergency stop once depth
is back and it has slowed below 0.1 m/s.

| File (under `src/planner/`)                          | Change |
| ---------------------------------------------------- | ------ |
| `plan_env/src/grid_map.cpp`                          | `updateOccupancyCallback` sets `flag_depth_odom_timeout_` to false when it runs a fresh update, beside `last_occ_update_time_`. |
| `plan_manage/include/ego_planner/emergency_stop_policy.h` | New. `canLeaveEmergencyStop(fail_safe_enabled, depth_timed_out, speed)`: the upstream exit rule (fail-safe on and speed below 0.1) plus depth not timed out. |
| `plan_manage/src/ego_replan_fsm.cpp`                 | The `EMERGENCY_STOP` state leaves for `GEN_NEW_TRAJ` through `canLeaveEmergencyStop`, reading the map's depth-lost flag. `checkCollisionCallback` no longer sets `enable_fail_safe_` to false on depth loss; it still enters `EMERGENCY_STOP` on every tick while depth is lost. |
| `plan_env/test/test_depth_timeout_flag.cpp`, `plan_env/CMakeLists.txt`, `plan_env/package.xml` | New gtest: the flag rises when depth stops and clears when depth resumes. Built only with `BUILD_TESTING`; adds the `ament_cmake_gtest` test dependency. |
| `plan_manage/test/test_emergency_stop_policy.cpp`, `plan_manage/CMakeLists.txt`, `plan_manage/package.xml` | New gtest for `canLeaveEmergencyStop`. Built only with `BUILD_TESTING`; adds the `ament_cmake_gtest` test dependency. |
