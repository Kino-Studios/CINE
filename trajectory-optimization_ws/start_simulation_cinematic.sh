#!/bin/bash
# Start simulation with cinematographic filter enabled

cd /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws

# Source environment first
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=:0
unset ROS_LOCALHOST_ONLY

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting simulation with CINEMATOGRAPHIC mode..."
echo "- Trajectory planner: CINEMATIC B-spline enabled"
echo ""

# Launch ego planner with cinematic mode enabled (only once!)
ros2 launch ego_planner single_run_in_sim.launch.py use_cinematic_bspline:=true

# Cleanup on exit
echo "Cleaning up..."
