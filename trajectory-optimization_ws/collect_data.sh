#!/bin/bash

# Complete Data Collection Script for Ego Planner with Cinematographic Constraints
# This script runs the full simulation and collects trajectory comparison data

echo "=================================================="
echo "  Ego Planner Data Collection System"
echo "=================================================="
echo ""

# Source ROS environment
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=:0
unset ROS_LOCALHOST_ONLY

source /opt/ros/jazzy/setup.bash
source ~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/setup.bash

# Check if simulation is already running
if ! ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
    echo "✗ ERROR: Ego planner simulation is not running!"
    echo ""
    echo "Please start the simulation first in a separate terminal:"
    echo "  cd ~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws"
    echo "  ./start_simulation.sh"
    echo ""
    echo "Or manually:"
    echo "  source source_scripts.sh  (from repo root)"
    echo "  ros2 launch ego_planner single_run_in_sim.launch.py"
    echo ""
    exit 1
fi

echo "✓ Ego planner simulation detected"
echo ""
echo "Starting data collection components..."
echo ""

# Start cinematic bspline filter to apply velocity constraints
echo "Starting cinematographic B-spline filter (applies velocity constraints)..."
~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/cinematic_bspline_filter &
FILTER_PID=$!
sleep 2

# Start data logger in background
echo "Starting trajectory data logger..."
~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/trajectory_data_logger &
LOGGER_PID=$!
sleep 2

# Start waypoint sender in background
echo "Starting room waypoint sender..."
~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/room_waypoint_sender &
WAYPOINT_PID=$!

echo ""
echo "=================================================="
echo "  Data Collection Active"
echo "=================================================="
echo ""
echo "System Status:"
echo "  • Cinematic B-spline filter running (PID: $FILTER_PID)"
echo "  • Trajectory logger running (PID: $LOGGER_PID)"
echo "  • Waypoint sender running (PID: $WAYPOINT_PID)"
echo "  • Sending 5 waypoints around the room"
echo "  • Applying cinematographic velocity constraints in real-time"
echo "  • Logging trajectory data to CSV files"
echo ""
echo "Trajectory flow:"
echo "  ego_planner → planning/bspline → cinematic_filter → planning/cinematic_bspline → traj_server"
echo ""
echo "The drone will fly with your algorithm's velocity constraints applied!"
echo "Data will be saved to: ./trajectory_data/"
echo ""
echo "Press Ctrl+C when finished to save data and exit..."
echo "=================================================="
echo ""

# Wait for user interrupt
trap "echo ''; echo 'Stopping data collection...'; kill $FILTER_PID $LOGGER_PID $WAYPOINT_PID 2>/dev/null; exit 0" INT TERM

# Keep script running
wait
