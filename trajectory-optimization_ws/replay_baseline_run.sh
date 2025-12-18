#!/bin/bash
# Replay baseline trajectory (without cinematographic filter) and collect data

# This script:
# 1. Starts simulation and traj_server (baseline mode)
# 2. Replays recorded B-spline trajectory
# 3. Logs trajectory data
# 4. Saves to baseline_replay_TIMESTAMP.csv

set -e

TRAJ_FILE="${1:-reference_trajectory.json}"
RUN_NUMBER="${2:-1}"

echo "=========================================="
echo "Replaying BASELINE Trajectory (Run $RUN_NUMBER)"
echo "=========================================="

# Check if reference trajectory exists
if [ ! -f "$TRAJ_FILE" ]; then
    echo "ERROR: Trajectory file not found: $TRAJ_FILE"
    echo "Please run: ./record_reference_trajectory.sh first"
    exit 1
fi

# Clean up any existing processes
echo "Cleaning up existing processes..."
pkill -f "ros2 launch" || true
pkill -f "rviz2" || true
pkill -f "bspline_replayer" || true
pkill -f "trajectory_data_logger" || true
sleep 2

# Start simulation
echo "Starting simulation..."
cd /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws
./start_simulation.sh &
SIMULATION_PID=$!

# Wait for initialization
echo "Waiting for simulation to initialize..."
sleep 10

# Start data logger
echo "Starting data logger (baseline mode)..."
source install/setup.bash
python3 src/cinematographic_adjustment_agent/cinematographic_adjustment_agent/data_collection/trajectory_data_logger.py --mode baseline --run-number "$RUN_NUMBER" &
LOGGER_PID=$!
sleep 2

# Start replayer (baseline = no cinematic filter)
echo "Replaying trajectory (baseline mode)..."
python3 bspline_replayer.py "$TRAJ_FILE" &
REPLAYER_PID=$!

# Wait for trajectory to complete (4 seconds should be enough)
echo "Executing trajectory..."
sleep 5

# Stop data logger
echo "Stopping data logger..."
kill $LOGGER_PID 2>/dev/null || true
sleep 1

# Find the most recent baseline CSV
LATEST_CSV=$(ls -t trajectory_data/baseline_replay_*.csv 2>/dev/null | head -1)

if [ -f "$LATEST_CSV" ]; then
    POINTS=$(wc -l < "$LATEST_CSV")
    echo ""
    echo "=========================================="
    echo "✓ SUCCESS: Baseline run $RUN_NUMBER complete"
    echo "=========================================="
    echo "Data saved: $LATEST_CSV"
    echo "Points: $POINTS"
else
    echo "WARNING: No data file found!"
fi

# Clean up
echo "Cleaning up..."
kill $REPLAYER_PID 2>/dev/null || true
kill $SIMULATION_PID 2>/dev/null || true
pkill -f "ros2 launch" || true
sleep 2

echo "Done!"
