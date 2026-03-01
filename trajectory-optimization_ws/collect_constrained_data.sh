set -euo pipefail
: "${AMENT_TRACE_SETUP_FILES=}"
: "${AMENT_PYTHON_EXECUTABLE:=python3}"
: "${LD_LIBRARY_PATH=}"
: "${COLCON_TRACE=}"
: "${COLCON_PREFIX_PATH=}"

PYTHON_BIN="$(command -v python3 || command -v python || true)"
: "${COLCON_PYTHON_EXECUTABLE:=${PYTHON_BIN:-python3}}"

export AMENT_TRACE_SETUP_FILES
export AMENT_PYTHON_EXECUTABLE
export LD_LIBRARY_PATH
export COLCON_TRACE
export COLCON_PREFIX_PATH
export COLCON_PYTHON_EXECUTABLE

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

ROS_LOG_DIR="${REPO_ROOT}/ros_logs"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

set +u
source /opt/ros/jazzy/setup.bash
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  source "${SCRIPT_DIR}/install/setup.bash"
elif [ -f "${REPO_ROOT}/install/setup.bash" ]; then
  source "${REPO_ROOT}/install/setup.bash"
else
  echo "Could not find install/setup.bash under ${SCRIPT_DIR} or ${REPO_ROOT}" >&2
  exit 1
fi
set -u

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

CONFIG_FILE="${SCRIPT_DIR}/collect_constrained_data.env"

if [ -f "${CONFIG_FILE}" ]; then
  source "${CONFIG_FILE}"
fi

USE_SIMULATED_FOCUS="${USE_SIMULATED_FOCUS:-$($GET_CONFIG USE_SIMULATED_FOCUS)}"
USE_SIMULATED_LIGHT="${USE_SIMULATED_LIGHT:-$($GET_CONFIG USE_SIMULATED_LIGHT)}"
TARGET_XYZ="${TARGET_XYZ:-$($GET_CONFIG TARGET_XYZ)}"
FOCUS_DEPTH_M="${FOCUS_DEPTH_M:-$($GET_CONFIG FOCUS_DEPTH_M)}"
FRAME_ID="${FRAME_ID:-$($GET_CONFIG FRAME_ID)}"

SIM_PID=""
TAIL_PID=""
FILTER_PID=""
LOGGER_PID=""
WAYPOINT_PID=""
DETECTOR_PID=""
COSTS_PID=""
SIMULATED_FOCUS_PID=""
SIMULATED_LIGHT_PID=""
cleanup_collection() {
  echo ""
  echo "stopping data collection."
  for pid in \
    "${LOGGER_PID:-}" \
    "${WAYPOINT_PID:-}" \
    "${FILTER_PID:-}" \
    "${SIMULATED_FOCUS_PID:-}" \
    "${SIMULATED_LIGHT_PID:-}" \
    "${COSTS_PID:-}" \
    "${DETECTOR_PID:-}" \
    "${TAIL_PID:-}" \
    "${SIM_PID:-}"; do
    if [ -n "${pid}" ]; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
stop_collection() {
  cleanup_collection
  exit 0
}
trap stop_collection INT TERM

if ! ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
  SIM_LOG=$(mktemp /tmp/ego_planner_constrained.XXXXXX)
  "${SCRIPT_DIR}/start_simulation.sh" constrained > "$SIM_LOG" 2>&1 &
  SIM_PID=$!

  tail -f "$SIM_LOG" &
  TAIL_PID=$!

  for _ in {1..30}; do
    if ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
      echo "Simulation initialized successfully"
      break
    fi
    echo -n "."
    sleep 1
  done

  if ! ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
    echo ""
    echo "[ERROR]: Simulation failed to start"
    echo "detail written to $SIM_LOG"
    kill $SIM_PID $TAIL_PID 2>/dev/null
    exit 1
  fi
fi

ros2 run cinematographic_adjustment_agent cinematic_bspline_filter &
FILTER_PID=$!
sleep 2

if [ "${USE_SIMULATED_FOCUS}" = "1" ]; then
  ros2 run cinematographic_adjustment_agent simulated_focus_publisher --ros-args \
    -p odom_topic:=${ODOM_TOPIC} \
    -p frame_id:=${FRAME_ID} \
    -p target_offset_xyz:="${TARGET_XYZ}" \
    -p target_xyz:="${TARGET_XYZ}" \
    -p focus_depth_m:=${FOCUS_DEPTH_M} &
  SIMULATED_FOCUS_PID=$!
  sleep 2
else
  ros2 run cinematographic_adjustment_agent simulated_object_detector &
  DETECTOR_PID=$!
  sleep 2

  ros2 run cinematographic_adjustment_agent cost_function_calculator --ros-args \
    -p odom_topic:=${ODOM_TOPIC} \
    -p frame_id:=${FRAME_ID} &
  COSTS_PID=$!
  sleep 2
fi

if [ "${USE_SIMULATED_LIGHT}" = "1" ]; then
  ros2 run cinematographic_adjustment_agent simulated_light_publisher --ros-args \
    -p frame_id:=map &
  SIMULATED_LIGHT_PID=$!
  sleep 1
fi

ros2 run cinematographic_adjustment_agent trajectory_data_logger --mode constrained --ros-args \
  -p odom_topic:=${ODOM_TOPIC} \
  -p baseline_bspline_topic:=${BASELINE_TOPIC} \
  -p cinematic_bspline_topic:=${CINEMATIC_TOPIC} &
LOGGER_PID=$!
sleep 2

ros2 run cinematographic_adjustment_agent cinematic_waypoint_generator --ros-args \
  -p odom_topic:=${ODOM_TOPIC} \
  -p frame_id:=${FRAME_ID} &
WAYPOINT_PID=$!

wait
cleanup_collection
