import numpy as np
from geometry_msgs.msg import Point
from traj_utils.msg import Bspline


def msg_to_arrays(msg: Bspline):
    pos_pts = np.array([[p.x, p.y, p.z] for p in msg.pos_pts], dtype=float)
    knots = np.array(msg.knots, dtype=float)
    return pos_pts, knots


def arrays_to_msg(original: Bspline, pos_pts: np.ndarray) -> Bspline:
    out = Bspline()
    out.drone_id = original.drone_id
    out.order = original.order
    out.traj_id = original.traj_id
    out.start_time = original.start_time
    out.knots = list(original.knots)
    out.yaw_pts = list(original.yaw_pts)
    out.yaw_dt = original.yaw_dt
    out.pos_pts = []

    for row in pos_pts:
        pt = Point()
        pt.x, pt.y, pt.z = float(row[0]), float(row[1]), float(row[2])
        out.pos_pts.append(pt)
        
    return out
