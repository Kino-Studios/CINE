#!/bin/bash
# Collect 10 baseline runs using replay mode

set -e

TRAJ_FILE="${1:-reference_trajectory.json}"
NUM_RUNS=10

echo "=========================================="
echo "Collecting $NUM_RUNS Baseline Replay Runs"
echo "=========================================="

# Check if reference trajectory exists
if [ ! -f "$TRAJ_FILE" ]; then
    echo "ERROR: Trajectory file not found: $TRAJ_FILE"
    echo "Please run: ./record_reference_trajectory.sh first"
    exit 1
fi

for i in $(seq 1 $NUM_RUNS); do
    echo ""
    echo "=========================================="
    echo "Baseline Run $i/$NUM_RUNS"
    echo "=========================================="
    
    ./replay_baseline_run.sh "$TRAJ_FILE" "$i"
    
    # Small delay between runs
    if [ $i -lt $NUM_RUNS ]; then
        echo "Waiting before next run..."
        sleep 3
    fi
done

echo ""
echo "=========================================="
echo "✓ All $NUM_RUNS baseline runs complete!"
echo "=========================================="
echo ""
echo "Data files:"
ls -1t trajectory_data/baseline_replay_*.csv | head -$NUM_RUNS

echo ""
echo "Next steps:"
echo "  1. Run: ./collect_10_constrained_replay_runs.sh"
echo "  2. Analyze: python3 analyze_replay_statistics.py"
