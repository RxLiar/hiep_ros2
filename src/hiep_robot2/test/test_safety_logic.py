import sys, math
import os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
from hiep_robot2.safety_logic import *

p = Profile.from_dict({"length_m": 1.8, "width_m": 0.7, "lidar_x_m": 0.9, "lidar_y_m": 0.0,
                       "stop": {"front": 0.25, "rear": 0.25, "left": 0.1, "right": 0.1},
                       "slow": {"front": 1.0, "rear": 0.6, "left": 0.4, "right": 0.4}})
print("footprint:", p.nav2_footprint())
assert p.footprint_rect() == (-0.9, 0.9, -0.35, 0.35)
print("stop rect:", p.zone_rect("stop"), "| slow rect:", p.zone_rect("slow"))

def flags_for(pts_xy, mp=1):
    c = classify(np.array(pts_xy, float), p)
    d = Debouncer(); counts = dict(stop_front=c.stop_front, stop_rear=c.stop_rear, slow_front=c.slow_front, slow_rear=c.slow_rear)
    return d.update(0.0, counts, mp), c

# --- nothing around
f, c = flags_for([[5.0, 0.0]]); assert state_name(f) == "ok" and gate(0.5, 0.2, f, p) == (0.5, 0.2)
# --- obstacle 0.6 m in front of the bumper (x = 0.9 + 0.6): slow zone only
f, c = flags_for([[1.5, 0.0]]); assert state_name(f) == "slow", f
print("slow zone fwd cmd 0.5 ->", gate(0.5, 0.4, f, p)); assert gate(0.5, 0.4, f, p) == (0.15, 0.30)
assert gate(-0.4, 0.0, f, p) == (-0.4, 0.0), "reversing away must not be limited"
# --- obstacle 0.1 m in front: stop zone
f, c = flags_for([[1.0, 0.0]]); assert state_name(f) == "stop"
print("stop zone fwd ->", gate(0.5, 0.3, f, p), "| reverse ->", gate(-0.3, 0.2, f, p), "| nearest %.2f m" % c.nearest_m)
assert gate(0.5, 0.3, f, p) == (0.0, 0.0)
assert gate(-0.3, 0.2, f, p) == (-0.3, 0.0), "reverse away allowed, rotation blocked"
# --- person behind
f, c = flags_for([[-1.0, 0.0]]); assert state_name(f) == "stop" and gate(0.5, 0, f, p) == (0.5, 0.0), "rear stop must not block forward"
assert gate(-0.2, 0, f, p) == (0.0, 0.0)
# --- side obstacle in the slow band while driving forward (x beyond centre => front)
f, c = flags_for([[0.5, 0.6]]); print("side obstacle:", state_name(f)); assert state_name(f) == "slow" and gate(0.5, 0, f, p)[0] == 0.15
# --- own body is ignored
f, c = flags_for([[0.5, 0.2], [-0.5, -0.3]]); assert state_name(f) == "ok" and c.nearest_m is None
# --- debounce: needs min_points, then holds
d = Debouncer(hold_s=0.3)
assert not d.update(0.0, {"stop_front": 2}, 3)["stop_front"]
assert d.update(0.1, {"stop_front": 3}, 3)["stop_front"]
assert d.update(0.3, {"stop_front": 0}, 3)["stop_front"] and not d.update(0.5, {"stop_front": 0}, 3)["stop_front"]
# --- scan to base points with a lidar yaw/offset: obstacle straight ahead of a lidar rotated 90 deg
q = Profile.from_dict({"lidar_x_m": 0.0, "lidar_y_m": 0.5, "lidar_yaw_deg": 90})
ranges = [3.0, 2.0, 4.0]; pts = scan_to_base_points(ranges, -0.1, 0.1, 0.05, 20, q)
print("rotated lidar points:", np.round(pts, 2).tolist()); assert abs(pts[1][0]) < 1e-6 and abs(pts[1][1] - 2.5) < 1e-6
# --- invalid values are clamped, slow >= stop
bad = Profile.from_dict({"length_m": -3, "stop": {"front": 2.0}, "slow": {"front": 0.5}})
assert bad.length_m == 0.1 and bad.slow["front"] >= bad.stop["front"]
inf = scan_to_base_points([float("inf"), float("nan"), 1.0, 0.0], 0, 0.1, 0.05, 20, Profile()); assert len(inf) == 1
print("SAFETY LOGIC OK")
