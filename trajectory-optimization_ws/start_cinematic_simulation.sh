#!/bin/bash

# Cinematographic trajectory simulation startup

echo "Starting Cinematographic Trajectory Simulation"
echo "=============================================="

# Source environment
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=:0
unset ROS_LOCALHOST_ONLY

source /opt/ros/jazzy/setup.bash
source ~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/setup.bash

echo "Launching with 16 arc waypoints (120° around object)..."
ros2 launch ego_planner cinematic_run.launch.py
