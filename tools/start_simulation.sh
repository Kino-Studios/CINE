#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY="${DISPLAY:-:0}"

unset ROS_LOCALHOST_ONLY

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# ROS setup scripts often contain unbound variables, which fails due to 'set -u'
set +u
source /opt/ros/jazzy/setup.bash
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  source "${SCRIPT_DIR}/install/setup.bash"
elif [ -f "${REPO_ROOT}/install/setup.bash" ]; then
  source "${REPO_ROOT}/install/setup.bash"
elif [ -f "${REPO_ROOT}/../../install/setup.bash" ]; then
  # The usual layout: this repository cloned into <workspace>/src/.
  source "${REPO_ROOT}/../../install/setup.bash"
else
  echo "Could not find install/setup.bash under ${SCRIPT_DIR}, ${REPO_ROOT} or the workspace above it" >&2
  exit 1
fi
set -u

LAUNCH_ARGS=()
case "${MODE}" in
  "" ) ;;
  baseline ) LAUNCH_ARGS+=("use_cinematic_bspline:=false") ;;
  constrained|cinematic ) LAUNCH_ARGS+=("use_cinematic_bspline:=true") ;;
  * ) echo "Usage: $0 [baseline|constrained]" >&2; exit 2 ;;
esac

exec ros2 launch cine_bringup cinematic_run.launch.py "${LAUNCH_ARGS[@]}"
