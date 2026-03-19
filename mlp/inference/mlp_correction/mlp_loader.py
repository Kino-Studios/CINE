# TODO: Implement ONNX model loader for MLP setpoint correction.
#
# This module loads mlp_correction.onnx and scaler.json from a weights directory
# and exposes a single inference function.
#
# Public interface:
#
#   class MLPLoader:
#       def __init__(self, weights_dir: str) -> None:
#           """Load ONNX model and scaler from weights_dir.
#           weights_dir must contain:
#             - mlp_correction.onnx
#             - scaler.json  (keys: mean[6], std[6], feature_names[6])
#           """
#
#       def correct(self, pos_pts: np.ndarray, dt: float) -> np.ndarray:
#           """Apply MLP correction to B-spline control points.
#
#           Args:
#               pos_pts: (N, 3) control points in odom_world frame.
#               dt:      estimated knot interval in seconds (from Bspline message).
#
#           Returns:
#               (N, 3) corrected control points (pos_pts + Δpos).
#
#           Input construction (per control point i):
#               pos = pos_pts[i]                                   # [x, y, z]
#               vel = (pos_pts[i+1] - pos_pts[i-1]) / (2 * dt)   # central diff
#               vel at i=0:   forward diff  (pos_pts[1]   - pos_pts[0])   / dt
#               vel at i=N-1: backward diff (pos_pts[N-1] - pos_pts[N-2]) / dt
#               input_6d = normalize([pos, vel], scaler)
#
#           Batch call: all N rows fed as (N, 6) to ONNX in one session.run().
#           Passthrough: if model not loaded, return pos_pts unchanged.
#           """
