#!/bin/bash
# Quick topic checker script
source install/setup.bash

echo "Starting ego_planner in background..."
ros2 launch ego_planner single_run_in_sim.launch.py > /tmp/sim.log 2>&1 &
SIM_PID=$!

echo "Waiting for system to initialize..."
sleep 15

echo ""
echo "Topics with 'bspline' or 'planning':"
ros2 topic list 2>/dev/null | grep -iE "bspline|planning" || echo "No matching topics found"

echo ""
echo "All topics:"
ros2 topic list 2>/dev/null | head -20

echo ""
echo "Killing simulation..."
kill $SIM_PID 2>/dev/null
sleep 2
pkill -f "ros2 launch" 2>/dev/null
