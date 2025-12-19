cd "$(dirname "$0")"
source ../source_scripts.sh
source install/setup.bash

trajectory_data_logger &
LOGGER_PID=$!

ros2 run cinematographic_adjustment_agent simulated_object_detector \
  --ros-args -r depth/image_raw:=/drone_0_depth/image_raw &
DETECTOR_PID=$!

sleep 3

room_waypoint_sender &
SENDER_PID=$!

sleep 30

kill $SENDER_PID 2>/dev/null || true
kill $DETECTOR_PID 2>/dev/null || true
kill -INT $LOGGER_PID
wait $LOGGER_PID 2>/dev/null || true

if ls trajectory_data/*.csv 1> /dev/null 2>&1; then
    ls -lh trajectory_data/*.csv | tail -5

    echo "latest file saved:"
    LATEST_BASELINE=$(ls -t trajectory_data/baseline_*.csv | head -1)
    echo "$LATEST_BASELINE"
    echo "# data points: $(wc -l < "$LATEST_BASELINE")"
else
    exit 1
fi
