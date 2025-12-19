#!/usr/bin/env bash
# Toggles ego planner and cinematographic adjustment algorithm.

set -euo pipefail

MODE="${1:-}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_FILE="${SCRIPT_DIR}/src/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py"

inplace_sed() {
    local expr="$1"
    local file="$2"

    if command -v gsed >/dev/null 2>&1; then
        gsed -i -e "$expr" "$file"
    else
        # BSD sed (macOS) requires a backup suffix; '' means no backup file.
        sed -i '' -e "$expr" "$file"
    fi
}

if [ -z "$MODE" ]; then
    echo "Usage: $0 [baseline|constrained]"
    echo ""
    echo "  baseline    - Use unconstrained ego_planner trajectories"
    echo "  constrained - Use cinematographic velocity-constrained trajectories"
    exit 1
fi

if [ ! -f "$LAUNCH_FILE" ]; then
    echo "Launch file not found: $LAUNCH_FILE" >&2
    exit 1
fi

if [ "$MODE" = "baseline" ]; then
    echo "Setting mode to BASELINE (unconstrained)..."
    inplace_sed "s/'traj_server\\/use_cinematic_bspline': True/'traj_server\\/use_cinematic_bspline': False/" "$LAUNCH_FILE"
    echo "✓ Launch file updated"
    echo ""
    echo "traj_server will now use: planning/bspline"
    echo ""
    echo "To apply changes:"
    echo "  1. Stop simulation (Ctrl+C)"
    echo "  2. Rebuild: colcon build --packages-select ego_planner"
    echo "  3. Restart: ./start_simulation.sh"
    
elif [ "$MODE" = "constrained" ]; then
    echo "Setting mode to CONSTRAINED (cinematographic)..."
    inplace_sed "s/'traj_server\\/use_cinematic_bspline': False/'traj_server\\/use_cinematic_bspline': True/" "$LAUNCH_FILE"
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
    exit 1
fi
