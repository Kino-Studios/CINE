#!/bin/bash
# Record a reference B-spline trajectory for replay experiments

# This script:
# 1. Starts simulation and ego_planner
# 2. Waits for planner to generate first trajectory
# 3. Records the B-spline control points
# 4. Saves to reference_trajectory.json

echo "=========================================="
echo "Recording Reference Trajectory"
echo "=========================================="

# Output file
OUTPUT_FILE="${1:-reference_trajectory.json}"
FULL_PATH="$PWD/$OUTPUT_FILE"

# Clean up any existing processes
echo "Cleaning up existing processes..."
pkill -f "ros2 launch" || true
pkill -f "rviz2" || true
pkill -f "bspline_recorder" || true
sleep 2

# Start simulation and planner
echo "Starting simulation..."
cd /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws
./start_simulation.sh &
SIMULATION_PID=$!

# Wait for initialization
echo "Waiting for simulation to initialize..."
sleep 10

# Start recorder in background
echo "Starting B-spline recorder..."
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
python3 bspline_recorder.py "$FULL_PATH" &
RECORDER_PID=$!

# Send waypoints directly to trigger trajectory generation
echo "Starting waypoint sender to send 3D waypoints..."
sleep 2
ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp bash /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/start_waypoint_sender.sh &
WAYPOINT_PID=$!

# Wait for recorder to complete (it auto-shuts down after recording)
echo "Waiting for trajectory recording..."
sleep 35

# Check if file was created
if [ -f "$FULL_PATH" ]; then
    echo ""
    echo "=========================================="
    echo "✓ SUCCESS: Trajectory recorded!"
    echo "=========================================="
    echo "File: $FULL_PATH"
    echo ""
    echo "Control points saved:"
    python3 -c "import json; data=json.load(open('$FULL_PATH')); print(f\"  Order: {data['order']}\"); print(f\"  Control points: {len(data['pos_pts'])}\"); print(f\"  Knots: {len(data['knots'])}\")"
    echo ""
    echo "You can now use this trajectory for baseline and constrained runs:"
    echo "  Baseline:    ./replay_baseline_run.sh"
    echo "  Constrained: ./replay_constrained_run.sh"
else
    echo "ERROR: Failed to record trajectory!"
    echo "Check terminal output above for errors."
    echo ""
    echo "Common issues:"
    echo "  1. Check if topics are publishing: ros2 topic list | grep bspline"
    echo "  2. Check recorder output for errors"
    echo "  3. Make sure waypoint sender is running"
fi

# Clean up
echo "Cleaning up..."
kill $WAYPOINT_PID 2>/dev/null || true
kill $SIMULATION_PID 2>/dev/null || true
pkill -f "ros2 launch" || true
pkill -f "room_waypoint_sender" || true
sleep 2

echo "Done!"
