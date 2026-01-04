# easier for me to source necessary ROS2 and workspace setups

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=localhost:0

unset ROS_LOCALHOST_ONLY

_CINE_ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export _CINE_ROOT

if [ -f "$_CINE_ROOT/venv/bin/activate" ]; then
  source "$_CINE_ROOT/venv/bin/activate"
else
  echo "warning: venv not found: $_CINE_ROOT/venv/bin/activate" >&2
fi

AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES-}
export AMENT_TRACE_SETUP_FILES
AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE-}
export AMENT_PYTHON_EXECUTABLE
COLCON_TRACE=${COLCON_TRACE-}
export COLCON_TRACE
COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE-}
export COLCON_PYTHON_EXECUTABLE

if set -o | grep -q 'nounset *on'; then
  _SAVED_NOUNSET=1
  set +u
fi

source /opt/ros/jazzy/setup.bash
if [ -f "$_CINE_ROOT/trajectory-optimization_ws/install/setup.bash" ]; then
  source "$_CINE_ROOT/trajectory-optimization_ws/install/setup.bash"
else
  echo "warning: workspace setup not found: $_CINE_ROOT/trajectory-optimization_ws/install/setup.bash" >&2
fi

if [ "${_SAVED_NOUNSET-}" = "1" ]; then
  set -u
  unset _SAVED_NOUNSET
fi
