#!/bin/bash

# Script to collect ONE baseline run with fresh simulation restart

set -e

cd "$(dirname "$0")"
source ../source_scripts.sh
source install/setup.bash

echo "========================================="
echo "Collecting Single Baseline Run"
echo "========================================="

# Kill any existing simulation
echo "Stopping any existing simulation..."
pkill -f "ros2 launch ego_planner" || true
pkill -f "trajectory_data_logger" || true
pkill -f "room_waypoint_sender" || true
pkill -f "simulated_object_detector" || true
sleep 2

# Start fresh simulation
echo "Starting fresh simulation..."
./start_simulation.sh &
SIM_PID=$!
echo "Simulation PID: $SIM_PID"

# Wait for simulation to initialize
echo "Waiting 10 seconds for simulation to initialize..."
sleep 10

# Run data collection
echo "Starting data collection..."
bash test_single_run.sh

# Stop simulation
echo "Stopping simulation..."
kill $SIM_PID 2>/dev/null || true
pkill -f "ros2 launch ego_planner" || true

echo "========================================="
echo "Run complete! Check trajectory_data/ for CSV"
echo "========================================="
