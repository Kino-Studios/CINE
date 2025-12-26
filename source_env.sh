# easier for me to source necessary ROS2 and workspace setups

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=localhost:0

unset ROS_LOCALHOST_ONLY

source ~/bspline-nvidia-isaac-tsdf/venv/bin/activate

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
source ~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/setup.bash

if [ "${_SAVED_NOUNSET-}" = "1" ]; then
  set -u
  unset _SAVED_NOUNSET
fi
