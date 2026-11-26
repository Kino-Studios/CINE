#!/usr/bin/env bash
# Entry point of the gazebo-sim image.
#
# With a command (`compose run --rm gazebo-sim bash`), sets up ROS, the workspace if it has
# been built, and the venv, then runs the command without building anything.
# With no command, builds the read-only workspace under /workspace/src out of source into
# /workspace/colcon, fetches the pinned weights into /weights, and launches the simulation,
# or runs the smoke flight instead when CHECK=true.
set -euo pipefail

readonly WORKSPACE_SOURCE=/workspace/src
readonly COLCON_BASE=/workspace/colcon
readonly CINE_VENV=/opt/cine-venv

source_ros_script() {
  # ROS setup scripts read unset variables, so nounset is off while they run.
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

use_cine_venv() {
  # ROS runs Python nodes with /usr/bin/python3, ignoring PATH, so the venv joins PYTHONPATH;
  # the ArduPilot tools (sim_vehicle.py, MAVProxy) go last so their python3 never wins.
  local venv_site_packages
  venv_site_packages="$("${CINE_VENV}/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
  export PATH="${CINE_VENV}/bin:${PATH}:/opt/ardupilot/Tools/autotest:/home/ardupilot/venv-ardupilot/bin"
  export PYTHONPATH="${venv_site_packages}${PYTHONPATH:+:${PYTHONPATH}}"
}

source_ros_script /opt/ros/jazzy/setup.bash

if [ "$#" -gt 0 ]; then
  if [ -f "${COLCON_BASE}/install/setup.bash" ]; then
    source_ros_script "${COLCON_BASE}/install/setup.bash"
  fi
  use_cine_venv
  exec "$@"
fi

# The venv joins PYTHONPATH only after the build: its newer setuptools would otherwise
# replace the one colcon supports. No --symlink-install, because a symlinked Python
# package writes its egg-info into the source tree, which is mounted read-only.
cd "${COLCON_BASE}"
colcon --log-base "${COLCON_BASE}/log" build \
  --base-paths "${WORKSPACE_SOURCE}" \
  --build-base "${COLCON_BASE}/build" \
  --install-base "${COLCON_BASE}/install" \
  --cmake-args -DBUILD_TESTING=OFF
source_ros_script "${COLCON_BASE}/install/setup.bash"
use_cine_venv

python3 /opt/cine/fetch_weights.py --weights-dir /weights

if [ "${CHECK:-false}" = "true" ]; then
  exec python3 "${WORKSPACE_SOURCE}/CINE/cine_gazebo/test/smoke_flight.py"
fi

exec ros2 launch cine_gazebo gazebo_sim.launch.py \
  "nav_source:=${NAV_SOURCE:-sim}" \
  "headless:=${HEADLESS:-false}" \
  "wind:=${WIND:-calm}" \
  "mission:=${MISSION:-true}"
