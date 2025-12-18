#!/bin/bash

# Single Constrained Data Collection Run
# NOTE: Run this script 10 times, restarting simulation between each run

cd /home/vtsyp/bspline-nvidia-isaac-tsdf
source source_scripts.sh
cd trajectory-optimization_ws

echo "=========================================="
echo "  CONSTRAINED DATA COLLECTION"
echo "=========================================="
echo ""

# Check simulation is running
if ! ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
    echo "ERROR: Simulation not running!"
    echo "Start simulation first: ./start_simulation.sh"
    exit 1
fi

# Start cinematic filter
echo "Starting cinematographic B-spline filter..."
~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/cinematic_bspline_filter &
FILTER_PID=$!
sleep 2

echo "Starting data logger..."
~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/trajectory_data_logger &
LOGGER_PID=$!
sleep 2

echo "Sending waypoints..."
~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/room_waypoint_sender

echo "Collecting data for 30 seconds..."
sleep 30

echo "Stopping data logger and filter..."
kill -INT $LOGGER_PID 2>/dev/null
kill -INT $FILTER_PID 2>/dev/null
wait $LOGGER_PID 2>/dev/null
wait $FILTER_PID 2>/dev/null

echo ""
echo "=========================================="
echo "  RUN COMPLETE!"
echo "=========================================="
echo ""
echo "To do another run:"
echo "  1. Stop simulation (Ctrl+C in sim terminal)"
echo "  2. Restart simulation: ./start_simulation.sh"
echo "  3. Run this script again"
echo ""
