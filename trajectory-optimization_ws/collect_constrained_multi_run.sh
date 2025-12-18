#!/bin/bash

# Collect multiple short constrained trajectory runs (workaround for ego_planner crash)
# This script runs multiple sequential shorter collections to work around the executor issue

TOTAL_RUNS=5
RUN_TIMEOUT=25  # 25 second timeout per run (ego_planner crashes around 1-2s, give it time to collect)

echo "=================================================="
echo "  CONSTRAINED Multi-Run Trajectory Collection"
echo "  (Workaround for early ego_planner crashes)"
echo "=================================================="
echo ""
echo "Configuration:"
echo "  • Total runs: ${TOTAL_RUNS}"
echo "  • Timeout per run: ${RUN_TIMEOUT}s"
echo "  • Mode: CONSTRAINED with cinematographic filter"
echo ""

for i in $(seq 1 $TOTAL_RUNS); do
    echo "=================================================="
    echo "  Run ${i} of ${TOTAL_RUNS}"
    echo "=================================================="
    
    # Kill any remaining processes
    pkill -9 -f "ros2\|launch\|waypoint\|filter\|logger" 2>/dev/null || true
    sleep 2
    
    # Run collection
    cd /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws
    echo "Starting collection run ${i}..."
    timeout ${RUN_TIMEOUT} ./collect_constrained_data_full.sh 2>&1 | tail -50
    
    echo ""
    echo "Run ${i} completed (or timed out). Waiting before next run..."
    sleep 3
done

# Final cleanup
echo ""
echo "=================================================="
echo "  Collection Complete"
echo "=================================================="
pkill -9 -f "ros2\|launch\|waypoint\|filter\|logger" 2>/dev/null || true

# Show what was collected
echo ""
echo "Data files collected:"
ls -lh /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/trajectory_data/cinematic_sim_*.csv 2>/dev/null | wc -l
echo "files"

echo ""
echo "Sample of collected files:"
ls -lt /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/trajectory_data/cinematic_sim_*.csv 2>/dev/null | head -3
