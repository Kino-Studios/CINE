#!/bin/bash

cd /home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws
source install/setup.bash

echo "Starting trajectory data logger..."
echo "This will log ego_planner trajectory data to CSV files"
echo "Press Ctrl+C to stop and save data"
echo ""

/home/vtsyp/bspline-nvidia-isaac-tsdf/trajectory-optimization_ws/install/cinematographic_adjustment_agent/bin/trajectory_data_logger
