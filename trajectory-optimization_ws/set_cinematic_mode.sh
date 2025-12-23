#!/usr/bin/env bash
# Toggles ego planner cinematic mode by updating the default value for the
# `use_cinematic_bspline` launch argument.

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

toggle_default() {
    local value="$1" # true|false
    inplace_sed "/DeclareLaunchArgument('use_cinematic_bspline'/{s/default_value='[^']*'/default_value='${value}'/;}" "$LAUNCH_FILE"
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
    toggle_default "false"
    echo "Set to baseline."
    
elif [ "$MODE" = "constrained" ]; then
    toggle_default "true"
    echo "Set to constrained."
    
else
    echo "Invalid mode: $MODE"
    exit 1
fi
