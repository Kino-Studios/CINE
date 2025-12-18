# easier for me to source necessary ROS2 and workspace setups

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=localhost:0

unset ROS_LOCALHOST_ONLY

source ~/bspline-nvidia-isaac-tsdf/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/setup.bash