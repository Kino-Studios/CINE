# Third-party notices

CINE's own code is licensed under GPL-3.0-only (see `LICENSE`). This file lists the
code bundled in this repository that others wrote, and the components the setup
steps download, each under its own licence.

## Bundled in this repository

| Component | Where | Licence | Notes |
| --- | --- | --- | --- |
| EGO-Planner-Swarm, ZJU FAST Lab (<https://github.com/ZJU-FAST-Lab/ego-planner-swarm>), from a community ROS 2 port | `third_party/ego_planner_swarm/` | GPL-3.0 (`third_party/ego_planner_swarm/LICENSE`) | Modified; every change is recorded in `third_party/ego_planner_swarm/UPSTREAM.md`. |
| Boost.odeint (Karsten Ahnert, Mario Mulansky) | `third_party/ego_planner_swarm/src/uav_simulator/so3_quadrotor_simulator/include/ode/` | Boost Software License 1.0 (`LICENSE_1_0.txt` in that directory) | Bundled by the EGO-Planner tree for the quadrotor simulator. |
| Fast-Planner moving-object model, Boyu Zhou, HKUST Aerial Robotics Group | `third_party/ego_planner_swarm/src/planner/plan_env/` (`obj_predictor.*`, `obj_generator.cpp`, `linear_obj_model.hpp`) | LGPL-3.0-or-later (per-file headers) | Bundled by the EGO-Planner tree. |
| `FindCUDA.cmake` and its helper scripts, NVIDIA and Kitware | `third_party/ego_planner_swarm/src/uav_simulator/local_sensing/CMakeModules/` | MIT (per-file headers) | CMake modules bundled by the EGO-Planner tree. |
| `FindEigen.cmake`, Eigen authors | `third_party/ego_planner_swarm/src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake` | BSD-2-Clause (per-file header) | CMake module bundled by the EGO-Planner tree. |
| `FindEigen.cmake`, Google (Ceres Solver) | `third_party/ego_planner_swarm/src/uav_simulator/Utils/cmake_utils/cmake_modules/FindEigen.cmake` | BSD-3-Clause (per-file header) | CMake module bundled by the EGO-Planner tree. |

## Downloaded during setup, not shipped here

| Component | Fetched by | Licence |
| --- | --- | --- |
| Depth Anything 3 code (<https://github.com/ByteDance-Seed/Depth-Anything-3>) | You, with pip (see the README) | Apache-2.0 |
| Depth Anything 3 model weights `depth-anything/DA3METRIC-LARGE` (default) | `cine_depth` on first start, from Hugging Face | Apache-2.0; `DA3-SMALL` and `DA3-BASE` are also Apache-2.0, the larger `DA3-LARGE`, `DA3-GIANT` and `DA3NESTED` weights are CC BY-NC 4.0 (non-commercial) |
| ONNX Runtime, NumPy, pandas; for `cine_depth` also PyTorch, torchvision and the packages in `cine_depth/requirements.txt` | `pip install -r requirements.txt` (and `cine_depth/requirements.txt`) | Their own licences (BSD-3-Clause, MIT and others) |
| CINE trajectory datasets | `tools/download_assets.py`, from this repository's GitHub Release | Published by the CINE authors with the release; see its notes |
