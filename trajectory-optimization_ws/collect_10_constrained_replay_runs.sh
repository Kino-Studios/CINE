#!/bin/bash
# Collect 10 constrained runs using replay mode

set -e

TRAJ_FILE="${1:-reference_trajectory.json}"
NUM_RUNS=10

echo "=========================================="
echo "Collecting $NUM_RUNS Constrained Replay Runs"
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
    echo "Constrained Run $i/$NUM_RUNS"
    echo "=========================================="
    
    ./replay_constrained_run.sh "$TRAJ_FILE" "$i"
    
    # Small delay between runs
    if [ $i -lt $NUM_RUNS ]; then
        echo "Waiting before next run..."
        sleep 3
    fi
done

echo ""
echo "=========================================="
echo "✓ All $NUM_RUNS constrained runs complete!"
echo "=========================================="
echo ""
echo "Data files:"
ls -1t trajectory_data/constrained_replay_*.csv | head -$NUM_RUNS

echo ""
echo "Next step:"
echo "  Analyze: python3 analyze_replay_statistics.py"
