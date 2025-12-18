#!/bin/bash
# Simple test to check if B-spline recording works

echo "Testing B-spline recorder..."
echo ""
echo "This will:"
echo "1. List current ROS topics"
echo "2. Start the recorder"
echo "3. Show what it's subscribed to"
echo ""

cd /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws
source install/setup.bash

echo "Current topics (looking for /drone_0_planning/bspline):"
ros2 topic list | grep -E "(planning|bspline)" || echo "No bspline topics found yet"

echo ""
echo "Starting recorder (will wait 30 seconds for trajectory)..."
echo "You should see 'Waiting for first trajectory...' message"
echo ""

timeout 30 python3 bspline_recorder.py test_trajectory.json &
RECORDER_PID=$!

sleep 2

echo ""
echo "Recorder subscriptions:"
ros2 topic info /drone_0_planning/bspline 2>/dev/null || echo "/drone_0_planning/bspline topic not found"

echo ""
echo "Waiting for recording to complete (or timeout)..."
wait $RECORDER_PID

echo ""
if [ -f "test_trajectory.json" ]; then
    echo "✓ SUCCESS! test_trajectory.json created"
    cat test_trajectory.json | head -20
else
    echo "✗ FAILED - no trajectory file created"
    echo ""
    echo "Debugging steps:"
    echo "1. Make sure simulation is running: ./start_simulation.sh"
    echo "2. Send waypoints: python3 src/cinematographic_adjustment_agent/cinematographic_adjustment_agent/waypoint_automation/room_waypoint_sender.py"
    echo "3. Check topics: ros2 topic list | grep bspline"
    echo "4. Echo topic: ros2 topic echo /drone_0_planning/bspline"
fi
