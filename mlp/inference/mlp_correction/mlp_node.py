# TODO: Implement ROS2 interceptor node for MLP setpoint correction.
#
# Node name: mlp_correction_node
#
# Subscriptions:
#   /drone_0_planning/cinematic_bspline  (traj_utils/Bspline)
#       — output of CinematicBsplineFilter
#
# Publications:
#   /mlp/corrected_bspline               (traj_utils/Bspline)
#       — Bspline with corrected control points (pos_pts + Δpos)
#   /mlp/correction_delta                (geometry_msgs/Vector3Stamped)
#       — mean(Δpos) across all control points; frame_id = "odom_world"
#       — publishes zero vector in passthrough mode (ONNX not loaded)
#
# Parameters:
#   mlp_weights_path (string): path to mlp/weights/ directory
#
# Startup:
#   1. Read mlp_weights_path parameter
#   2. Instantiate MLPLoader(mlp_weights_path) — loads ONNX + scaler
#   3. If ONNX missing: log warning, run in passthrough mode
#
# Callback (on each Bspline message):
#   1. Extract control points pos_pts and knots from msg
#   2. Compute dt via median of positive knot diffs (same as _estimate_dt in bspline filter)
#   3. corrected_pts = loader.correct(pos_pts, dt)
#   4. Rebuild Bspline msg with corrected_pts, publish on /mlp/corrected_bspline
#   5. Compute mean_delta = mean(corrected_pts - pos_pts, axis=0)
#      Publish as Vector3Stamped on /mlp/correction_delta
#
# Phase 1 (dark launch): no downstream node subscribes to /mlp/corrected_bspline.
# Phase 2 (live):        reconfigure downstream Bspline-to-PoseStamped converter
#                        to subscribe to /mlp/corrected_bspline instead of
#                        /drone_0_planning/cinematic_bspline (one launch param change).


def main(args=None):
    # TODO: implement
    pass
