#!/bin/bash

# Simple debug script to see what topics are being published

echo "Topics being published in simulation:"
echo "===================================="
echo ""

source /opt/ros/jazzy/setup.bash
source ~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/setup.bash

sleep 2

echo "B-spline related topics:"
ros2 topic list 2>/dev/null | grep -i bspline

echo ""
echo "Checking if cinematic_bspline is being published:"
ros2 topic echo /drone_0_planning/cinematic_bspline --once 2>/dev/null | head -5 || echo "No cinematic_bspline messages"

echo ""
echo "Checking if regular bspline is being published:"
ros2 topic echo /drone_0_planning/bspline --once 2>/dev/null | head -5 || echo "No bspline messages"

