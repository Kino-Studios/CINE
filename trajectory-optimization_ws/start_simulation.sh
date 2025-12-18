#!/bin/bash

# Complete startup script for ego_planner simulation and data collection

echo "Starting Ego Planner Simulation and Data Collection"
echo "===================================================="

# Source environment
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=:0
unset ROS_LOCALHOST_ONLY

source /opt/ros/jazzy/setup.bash
source ~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/setup.bash

echo "Launching ego_planner simulation..."
ros2 launch ego_planner single_run_in_sim.launch.py
