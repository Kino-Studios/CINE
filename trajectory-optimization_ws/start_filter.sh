#!/usr/bin/env bash
set -euo pipefail

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/jazzy/setup.bash
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  source "${SCRIPT_DIR}/install/setup.bash"
elif [ -f "${REPO_ROOT}/install/setup.bash" ]; then
  source "${REPO_ROOT}/install/setup.bash"
else
  echo "Could not find install/setup.bash under ${SCRIPT_DIR} or ${REPO_ROOT}" >&2
  exit 1
fi

exec ros2 run cinematographic_adjustment_agent cinematic_bspline_filter "$@"
