#!/bin/bash

# Constrained Trajectory Data Collection
# Collects data from cinematographic velocity-constrained trajectories

echo "=================================================="
echo "  CINEMATOGRAPHIC CONSTRAINED Data Collection"
echo "=================================================="
echo ""

# Source ROS environment
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
unset ROS_LOCALHOST_ONLY

source /opt/ros/jazzy/setup.bash
source ~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/setup.bash

# Check if simulation is running
if ! ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
    echo "✗ ERROR: Ego planner simulation is not running!"
    echo ""
    echo "For CONSTRAINED data collection, you need to:"
    echo "  1. Stop any running simulation (Ctrl+C in simulation terminal)"
    echo "  2. Edit launch file to set use_cinematic_bspline=True"
    echo "  3. Restart simulation: ./start_simulation.sh"
    echo ""
    echo "Or use ros2 param to change it dynamically (if supported)"
    exit 1
fi

echo "✓ Ego planner simulation detected"
echo "⚠ This script collects CONSTRAINED (cinematographic) trajectory data"
echo "⚠ Make sure use_cinematic_bspline parameter is TRUE in the simulation"
echo ""

# Start cinematic B-spline filter
echo "Starting cinematographic B-spline filter..."
~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/cinematic_bspline_filter &
FILTER_PID=$!
sleep 2

# Start data logger
echo "Starting trajectory data logger..."
~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/trajectory_data_logger &
LOGGER_PID=$!
sleep 2

# Start waypoint sender
echo "Starting room waypoint sender..."
~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/room_waypoint_sender &
WAYPOINT_PID=$!

echo ""
echo "=================================================="
echo "  CONSTRAINED Data Collection Active"
echo "=================================================="
echo ""
echo "System Status:"
echo "  • Mode: CINEMATOGRAPHIC (velocity-constrained)"
echo "  • Cinematic filter running (PID: $FILTER_PID)"
echo "  • Trajectory logger running (PID: $LOGGER_PID)"
echo "  • Waypoint sender running (PID: $WAYPOINT_PID)"
echo ""
echo "Trajectory flow:"
echo "  ego_planner → planning/bspline → cinematic_filter"
echo "  → planning/cinematic_bspline → traj_server → drone"
echo ""
echo "Data will be saved to: ./trajectory_data/constrained_sim_*.csv"
echo ""
echo "Press Ctrl+C when finished..."
echo "=================================================="
echo ""

# Wait for user interrupt
trap "echo ''; echo 'Stopping data collection...'; kill $FILTER_PID $LOGGER_PID $WAYPOINT_PID 2>/dev/null; exit 0" INT TERM

# Keep script running
wait
