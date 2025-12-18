#!/bin/bash

# Helper script to toggle cinematographic constraint mode
# Usage: ./set_cinematic_mode.sh [baseline|constrained]

MODE=$1
LAUNCH_FILE=~/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/src/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py

if [ -z "$MODE" ]; then
    echo "Usage: $0 [baseline|constrained]"
    echo ""
    echo "  baseline    - Use unconstrained ego_planner trajectories"
    echo "  constrained - Use cinematographic velocity-constrained trajectories"
    exit 1
fi

if [ "$MODE" == "baseline" ]; then
    echo "Setting mode to BASELINE (unconstrained)..."
    sed -i "s/'traj_server\/use_cinematic_bspline': True/'traj_server\/use_cinematic_bspline': False/" "$LAUNCH_FILE"
    echo "✓ Launch file updated"
    echo ""
    echo "traj_server will now use: planning/bspline"
    echo ""
    echo "To apply changes:"
    echo "  1. Stop simulation (Ctrl+C)"
    echo "  2. Rebuild: colcon build --packages-select ego_planner"
    echo "  3. Restart: ./start_simulation.sh"
    
elif [ "$MODE" == "constrained" ]; then
    echo "Setting mode to CONSTRAINED (cinematographic)..."
    sed -i "s/'traj_server\/use_cinematic_bspline': False/'traj_server\/use_cinematic_bspline': True/" "$LAUNCH_FILE"
    echo "✓ Launch file updated"
    echo ""
    echo "traj_server will now use: planning/cinematic_bspline"
    echo ""
    echo "To apply changes:"
    echo "  1. Stop simulation (Ctrl+C)"
    echo "  2. Rebuild: colcon build --packages-select ego_planner"
    echo "  3. Restart: ./start_simulation.sh"
    echo "  4. Start cinematic filter: (automatic in collect_constrained_data.sh)"
    
else
    echo "Invalid mode: $MODE"
    echo "Use 'baseline' or 'constrained'"
    exit 1
fi
