#!/bin/bash
# Replay constrained trajectory (with cinematographic filter) and collect data

# This script:
# 1. Starts object detector
# 2. Starts cinematographic filter
# 3. Starts simulation with traj_server configured for cinematic mode
# 4. Replays recorded B-spline trajectory
# 5. Logs trajectory data
# 6. Saves to constrained_replay_TIMESTAMP.csv

set -e

TRAJ_FILE="${1:-reference_trajectory.json}"
RUN_NUMBER="${2:-1}"

echo "=========================================="
echo "Replaying CONSTRAINED Trajectory (Run $RUN_NUMBER)"
echo "=========================================="

# Check if reference trajectory exists
if [ ! -f "$TRAJ_FILE" ]; then
    echo "ERROR: Trajectory file not found: $TRAJ_FILE"
    echo "Please run: ./record_reference_trajectory.sh first"
    exit 1
fi

# Clean up any existing processes - AGGRESSIVE cleanup for DDS
echo "Cleaning up existing processes..."
pkill -9 -f "ros2 launch" 2>/dev/null || true
pkill -9 -f "ros2 run" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "ego_planner" 2>/dev/null || true
pkill -9 -f "traj_server" 2>/dev/null || true
pkill -9 -f "random_forest" 2>/dev/null || true
pkill -9 -f "bspline_replayer" 2>/dev/null || true
pkill -9 -f "trajectory_data_logger" 2>/dev/null || true
pkill -9 -f "simulated_object_detector" 2>/dev/null || true
pkill -9 -f "cinematic_bspline_filter" 2>/dev/null || true
sleep 5  # Give DDS time to release participants

cd /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws

# Source environment FIRST
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Start object detector using wrapper script (preserves ROS environment)
echo "Starting object detector..."
./start_detector.sh > /tmp/detector_$$.log 2>&1 &
DETECTOR_PID=$!
echo "  Detector PID: $DETECTOR_PID"
sleep 5
if ! ps -p $DETECTOR_PID > /dev/null; then
    echo "ERROR: Detector failed to start. Check /tmp/detector_$$.log"
    cat /tmp/detector_$$.log
    exit 1
fi

# Start cinematographic filter using wrapper script
echo "Starting cinematographic filter..."
./start_filter.sh > /tmp/filter_$$.log 2>&1 &
FILTER_PID=$!
echo "  Filter PID: $FILTER_PID"
sleep 5
if ! ps -p $FILTER_PID > /dev/null; then
    echo "ERROR: Filter failed to start. Check /tmp/filter_$$.log"
    cat /tmp/filter_$$.log
    exit 1
fi
echo "✓ Detector and filter are running"

# Start simulation with cinematic mode enabled
echo "Starting simulation (CINEMATIC MODE - traj_server will use cinematic_bspline)..."
ros2 launch ego_planner single_run_in_sim.launch.py \
    use_cinematic_bspline:=true > /tmp/sim_$$.log 2>&1 &
SIMULATION_PID=$!

# Wait for initialization - simulation needs time to create all nodes
echo "Waiting for simulation to initialize..."
sleep 15

# Check if simulation is still running
if ! ps -p $SIMULATION_PID > /dev/null; then
    echo "ERROR: Simulation failed to start!"
    cat /tmp/sim_$$.log 2>/dev/null | tail -50
    exit 1
fi
echo "✓ Simulation ready"

# Start data logger
echo "Starting data logger (constrained mode)..."
python3 src/cinematographic_adjustment_agent/cinematographic_adjustment_agent/data_collection/trajectory_data_logger.py --mode constrained --run-number "$RUN_NUMBER" > /tmp/logger_$$.log 2>&1 &
LOGGER_PID=$!
echo "  Logger PID: $LOGGER_PID"
sleep 2

# Start replayer in CINEMATIC mode
echo "Replaying trajectory (CINEMATIC mode with filter)..."
python3 bspline_replayer.py "$TRAJ_FILE" --cinematic > /tmp/replayer_$$.log 2>&1 &
REPLAYER_PID=$!
echo "  Replayer PID: $REPLAYER_PID"

# Wait for trajectory to complete
echo "Executing trajectory (waiting 8 seconds for completion)..."
sleep 8

# Check replayer status
if [ -f /tmp/replayer_$$.log ]; then
    echo "Replayer log:"
    tail -20 /tmp/replayer_$$.log
fi

# Check filter status
if [ -f /tmp/filter_$$.log ]; then
    echo "Filter log (last 10 lines):"
    tail -10 /tmp/filter_$$.log
fi

# Stop data logger
echo "Stopping data logger..."
kill $LOGGER_PID 2>/dev/null || true
sleep 2

# Find the most recent constrained CSV
LATEST_CSV=$(ls -t trajectory_data/constrained_replay_*.csv 2>/dev/null | head -1)

if [ -f "$LATEST_CSV" ]; then
    POINTS=$(wc -l < "$LATEST_CSV")
    echo ""
    echo "=========================================="
    echo "✓ SUCCESS: Constrained run $RUN_NUMBER complete"
    echo "=========================================="
    echo "Data saved: $LATEST_CSV"
    echo "Points: $POINTS"
else
    echo "WARNING: No data file found!"
fi

# Clean up - AGGRESSIVE to ensure DDS participants are freed
echo "Cleaning up..."
kill -9 $REPLAYER_PID 2>/dev/null || true
kill -9 $DETECTOR_PID 2>/dev/null || true
kill -9 $FILTER_PID 2>/dev/null || true
kill -9 $SIMULATION_PID 2>/dev/null || true
pkill -9 -f "ros2 launch" 2>/dev/null || true
pkill -9 -f "ros2 run" 2>/dev/null || true
pkill -9 -f "ego_planner" 2>/dev/null || true
sleep 3  # Give DDS time to cleanup

echo "Done!"

