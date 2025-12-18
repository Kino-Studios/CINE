#!/bin/bash
# Wrapper to start detector with ROS environment

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/setup.bash

# Call the executable directly (ros2 run has registration issues)
exec /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/simulated_object_detector
