#!/bin/bash

# Test script to collect one run of baseline trajectory data

set -e  # Exit on error

cd "$(dirname "$0")"
source ../source_scripts.sh
source install/setup.bash

echo "==================================="
echo "Testing Single Baseline Data Run"
echo "==================================="

# Start data logger in background
echo "Starting data logger..."
trajectory_data_logger &
LOGGER_PID=$!
echo "Logger PID: $LOGGER_PID"

# Start object detector
echo "Starting object detector..."
ros2 run cinematographic_adjustment_agent simulated_object_detector \
  --ros-args -r depth/image_raw:=/drone_0_depth/image_raw &
DETECTOR_PID=$!
echo "Detector PID: $DETECTOR_PID"

# Wait for logger to initialize
sleep 3

# Send waypoints
echo "Sending waypoints..."
room_waypoint_sender &
SENDER_PID=$!
echo "Waypoint sender PID: $SENDER_PID"

# Wait for drone to complete flight
echo "Waiting for flight to complete..."
sleep 60

# Stop waypoint sender
echo "Stopping waypoint sender..."
kill $SENDER_PID 2>/dev/null || true

# Stop detector
echo "Stopping object detector..."
kill $DETECTOR_PID 2>/dev/null || true

# Stop logger (Ctrl+C triggers save)
echo "Stopping logger..."
kill -INT $LOGGER_PID
wait $LOGGER_PID 2>/dev/null || true

# Check if data was saved
echo ""
echo "==================================="
echo "Data Collection Results:"
echo "==================================="
if ls trajectory_data/*.csv 1> /dev/null 2>&1; then
    echo "✓ CSV files created:"
    ls -lh trajectory_data/*.csv | tail -5
    echo ""
    echo "Latest baseline file:"
    LATEST_BASELINE=$(ls -t trajectory_data/baseline_*.csv | head -1)
    echo "$LATEST_BASELINE"
    echo "Number of data points: $(wc -l < "$LATEST_BASELINE")"
else
    echo "✗ No CSV files found in trajectory_data/"
    exit 1
fi

echo ""
echo "Test complete!"
