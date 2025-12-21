#!/bin/bash

# Baseline Trajectory Data Collection
# Collects data from unconstrained ego_planner trajectories

echo "=================================================="
echo "  BASELINE Trajectory Data Collection"
echo "=================================================="
echo ""

# Source ROS environment
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
unset ROS_LOCALHOST_ONLY

source /opt/ros/jazzy/setup.bash
source ~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/setup.bash

# Clean up any stale middleware
rm -rf /tmp/cyclonedds* ~/.local/share/cyclonedds* 2>/dev/null
sleep 2

# Check if simulation is running with BASELINE mode
if ! ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
    echo "⏳ Ego planner simulation not detected. Starting it now..."
    echo ""
    
    # Start simulation with visible output
    cd ~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws
    echo "Starting simulation (output will be shown below)..."
    echo "================================================================"
    ./start_simulation.sh > /tmp/ego_planner_baseline.log 2>&1 &
    SIM_PID=$!
    
    # Show output in real-time
    tail -f /tmp/ego_planner_baseline.log &
    TAIL_PID=$!
    
    echo "⏳ Waiting for simulation to initialize (30 seconds)..."
    for i in {1..30}; do
        if ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
            echo "✓ Simulation initialized successfully!"
            break
        fi
        echo -n "."
        sleep 1
    done
    
    if ! ros2 node list 2>/dev/null | grep -q "drone_0_ego_planner_node"; then
        echo ""
        echo "✗ ERROR: Simulation failed to start!"
        echo "Check /tmp/ego_planner_baseline.log for details"
        kill $SIM_PID $TAIL_PID 2>/dev/null
        exit 1
    fi
else
    echo "✓ Ego planner simulation already running"
fi

echo ""
echo "⚠ This script collects BASELINE (unconstrained) trajectory data"
echo "⚠ Using waypoint-based trajectory generation (ego_planner)"
echo ""

# Start data logger
echo "Starting trajectory data logger..."
~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/trajectory_data_logger &
LOGGER_PID=$!
sleep 2

~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/cinematic_waypoint_generator &
WAYPOINT_PID=$!

echo ""
echo "=================================================="
echo "  BASELINE Data Collection Active"
echo "=================================================="
echo ""
echo "System Status:"
echo "  • Mode: BASELINE (unconstrained)"
echo "  • Trajectory logger running (PID: $LOGGER_PID)"
echo "  • Waypoint sender running (PID: $WAYPOINT_PID)"
echo "  • Logging data from planning/bspline"
echo ""
echo "Data will be saved to: ./trajectory_data/baseline_sim_*.csv"
echo ""
echo "Press Ctrl+C when finished..."
echo "=================================================="
echo ""

# Wait for user interrupt
trap "echo ''; echo 'Stopping data collection...'; kill $LOGGER_PID $WAYPOINT_PID $TAIL_PID 2>/dev/null; exit 0" INT TERM

# Keep script running
wait
