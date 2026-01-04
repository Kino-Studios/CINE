#!/usr/bin/env bash
set -euo pipefail

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -z "${DISPLAY:-}" ]; then
  if [ -f /etc/resolv.conf ]; then
    export DISPLAY="$(grep -m1 nameserver /etc/resolv.conf | awk '{print $2}'):0"
  else
    export DISPLAY=":0"
  fi
fi
unset ROS_LOCALHOST_ONLY

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

if set -o | grep -q 'nounset *on'; then
  _SAVED_NOUNSET=1
  set +u
fi

AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES-}
AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE-}
COLCON_TRACE=${COLCON_TRACE-}
COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE-}
export AMENT_TRACE_SETUP_FILES AMENT_PYTHON_EXECUTABLE COLCON_TRACE COLCON_PYTHON_EXECUTABLE

source /opt/ros/jazzy/setup.bash
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  source "${SCRIPT_DIR}/install/setup.bash"
elif [ -f "${REPO_ROOT}/install/setup.bash" ]; then
  source "${REPO_ROOT}/install/setup.bash"
else
  echo "Could not find install/setup.bash under ${SCRIPT_DIR} or ${REPO_ROOT}" >&2
  exit 1
fi

if [ "${_SAVED_NOUNSET-}" = "1" ]; then
  set -u
  unset _SAVED_NOUNSET
fi

rm -rf /tmp/cyclonedds* ~/.local/share/cyclonedds* 2>/dev/null
sleep 2

GET_CONFIG="python3 ${REPO_ROOT}/configs/python-constants/get_config.py"

DRONE_ID=$($GET_CONFIG DRONE_ID)
TRAJECTORY_DATA_DIR="${REPO_ROOT}/$($GET_CONFIG TRAJECTORY_DATA_DIR)"

LOGGING_SEVERITY="${LOGGING_SEVERITY:-$($GET_CONFIG LOGGING_SEVERITY)}"
LOGGING_SEVERITY="${LOGGING_SEVERITY:-DEBUG}"
export RCUTILS_LOGGING_SEVERITY="${LOGGING_SEVERITY^^}"

ODOM_TOPIC="/drone_${DRONE_ID}_visual_slam/odom"
BASELINE_TOPIC="/drone_${DRONE_ID}_planning/bspline"
CINEMATIC_TOPIC="/drone_${DRONE_ID}_planning/cinematic_bspline"

SIM_PID=""
TAIL_PID=""
LOGGER_PID=""
WAYPOINT_PID=""

if ! ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
  "${SCRIPT_DIR}/start_simulation.sh" baseline > /tmp/ego_planner_baseline.log 2>&1 &
  SIM_PID=$!

  tail -f /tmp/ego_planner_baseline.log &
  TAIL_PID=$!

  for _ in {1..30}; do
    if ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
      echo "Simulation initialized successfully!"
      break
    fi
    echo -n "."
    sleep 1
  done

  if ! ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
    kill $SIM_PID $TAIL_PID 2>/dev/null
    exit 1
  fi
fi

ros2 run cinematographic_adjustment_agent trajectory_data_logger --mode baseline --ros-args \
  -p odom_topic:=${ODOM_TOPIC} \
  -p baseline_bspline_topic:=${BASELINE_TOPIC} \
  -p cinematic_bspline_topic:=${CINEMATIC_TOPIC} &
LOGGER_PID=$!
sleep 2

ros2 run cinematographic_adjustment_agent cinematic_waypoint_generator --ros-args -p odom_topic:=${ODOM_TOPIC} &
WAYPOINT_PID=$!

cleanup() {
  echo ""
  echo "stopping data collection."
  if [ -n "${LOGGER_PID:-}" ]; then
    kill -INT "${LOGGER_PID}" 2>/dev/null || true
    wait "${LOGGER_PID}" 2>/dev/null || true
  fi

  for pid in "${WAYPOINT_PID:-}" "${TAIL_PID:-}" "${SIM_PID:-}"; do
    if [ -n "${pid}" ]; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  exit 0
}

trap 'cleanup' INT TERM

wait
