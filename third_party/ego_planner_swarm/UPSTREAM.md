# Upstream record: ego_planner_swarm

## Origin

This directory is a modified copy of EGO-Planner-Swarm by the ZJU FAST Lab
(<https://github.com/ZJU-FAST-Lab/ego-planner-swarm>), taken from a community
ROS 2 port of that project. Which port, and which commit of it, is not
recorded: the tree was imported into this repository without history in
commits `03ac667` and `e3af213` (both 2025-11-08, "unstaged changes..").
Treat `03ac667` as the base when comparing.

## Licence

GNU General Public License version 3, see `LICENSE` in this directory.
Third-party code bundled by upstream keeps its own licence, notably Boost.odeint
under `src/uav_simulator/so3_quadrotor_simulator/include/ode/` (Boost Software
License 1.0, `LICENSE_1_0.txt` beside it).

## Modification notice (GPLv3 section 5a)

Modified by Kino-Studios, 2025-2026. The files below differ from the imported
base. Everything else is as imported, apart from the repository housekeeping
listed under "Reorganisation".

### Behaviour changes since the base (`git diff --stat 03ac667 HEAD`, 15 files)

| File (under `src/`)                                                    | Change                                                                                                                                                                                               |
| ---------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `planner/plan_manage/src/traj_server.cpp`                              | Large extension (+1623 lines changed): cinematic B-spline input, focus target, target distance, framing and light-vector subscriptions, `CinematographyCosts` input, and visualisation publishers. |
| `planner/plan_manage/launch/cinematic_run.launch.py`                   | New. First-party CINE launch file that starts the cinematic pipeline; it lives here until it moves to the `cine_bringup` package.                                                                   |
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
- Boost.odeint: removed `libs/numeric/odeint/performance/performance.py`
  (Python 2, not parseable by Python 3), the generated HTML documentation
  (`libs/numeric/odeint/doc/html/`, `libs/numeric/odeint/index.html`); added
  `LICENSE_1_0.txt`. Nothing in the build referenced the removed files.
