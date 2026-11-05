import sys, math, os
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import numpy as np
from hiep_robot2.safety_logic import *

p = Profile.from_dict({"length_m": 1.8, "width_m": 0.7, "lidar_x_m": 0.9, "lidar_y_m": 0.0,
                       "stop": {"front": 0.25, "rear": 0.25, "left": 0.1, "right": 0.1},
                       "slow": {"front": 1.0, "rear": 0.6, "left": 0.4, "right": 0.4}})
print("footprint:", p.nav2_footprint())
assert p.footprint_rect() == (-0.9, 0.9, -0.35, 0.35)
print("stop rect:", p.zone_rect("stop"), "| slow rect:", p.zone_rect("slow"))

def counts_of(c):
    return dict(stop_front=c.stop_front, stop_rear=c.stop_rear, stop_side=c.stop_side,
                slow_front=c.slow_front, slow_rear=c.slow_rear, slow_side=c.slow_side)

def flags_for(pts_xy, mp=1):
    c = classify(np.array(pts_xy, float), p)
    d = Debouncer(); counts = counts_of(c)
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

# ---------------- bypass (fork tip) ----------------
import time as _t
fp = Profile.from_dict({"length_m": 2.4, "width_m": 1.0, "lidar_x_m": -0.6,
                        "stop": {"front": 0.3, "rear": 0.4, "left": 0.1, "right": 0.1},
                        "slow": {"front": 1.0, "rear": 0.8, "left": 0.4, "right": 0.4}})
tip = fp.footprint_rect()[1]                                   # fork tip x
pallet = np.array([[tip + 0.15, -0.3], [tip + 0.15, 0.0], [tip + 0.15, 0.3]])     # pallet 0.15 m in front of the tips
c0 = classify(pallet, fp); d0 = Debouncer(); f0 = d0.update(0.0, counts_of(c0), 3)
assert state_name(f0) == "stop" and gate(0.1, 0, f0, fp) == (0.0, 0.0), "pallet blocks without bypass"
bm = BypassManager(); now = 100.0
bm.update(now, {"id": "fork_insert", "ttl": 1.0, "rects": [[tip - 1.0, tip + 0.6, -0.7, 0.7]], "max_linear": 0.10})
c1 = classify(pallet, fp, ignore_rects=bm.rects(now)); assert c1.stop_any == 0 and c1.slow_any == 0
caps = bm.speed_caps(now); print("bypass caps:", caps); assert caps == (0.10, 0.20)
assert cap_speed(0.5, 0.5, caps) == (0.10, 0.20) and cap_speed(-0.5, 0, caps) == (-0.10, 0)
# a person to the SIDE of the pallet area (outside the bypass rectangle) still stops the robot
person = np.array([[tip + 0.15, 1.0 - 0.0 + 0.0]]); person = np.array([[tip - 0.1, 0.85], [tip - 0.1, 0.84], [tip - 0.05, 0.85]])
c2 = classify(person, fp, ignore_rects=bm.rects(now)); print("person at side with bypass -> stop_any", c2.stop_any, "slow_any", c2.slow_any); assert c2.stop_any + c2.slow_any > 0
# REAR zone is never bypassed: someone behind while reversing out
rear = np.array([[fp.footprint_rect()[0] - 0.2, 0.0]] * 3)
c3 = classify(rear, fp, ignore_rects=bm.rects(now)); f3 = Debouncer().update(0.0, counts_of(c3), 3)
assert state_name(f3) == "stop" and gate(-0.1, 0, f3, fp) == (0.0, 0.0), "rear safety must stay active while a front bypass is on"
# expiry, cap on ttl, withdraw, bad input
assert bm.active(now + 1.5) == {}, "bypass must expire"
bm.update(now, {"id": "x", "ttl": 99, "rects": [[0, 1, 0, 1]]}); assert bm.active(now + 2.9) and not bm.active(now + 3.1), "ttl capped at 3 s"
bm.update(now, {"id": "x", "rects": [[0, 1, 0, 1]]}); bm.update(now, {"id": "x", "active": False}); assert not bm.active(now)
bm.update(now, {"nope": 1}); bm.update(now, {"id": "y", "rects": [["a", 1, 2, 3]]}); assert not bm.active(now)
bm.update(now, {"id": "z", "rects": [[0, 1, 0, 1]], "max_linear": 5.0}); assert bm.speed_caps(now)[0] == 0.30, "speed cap itself is capped"
print("BYPASS OK")

# ---------------- sectors: BESIDE the body counts for both directions ----------------
sp_ = Profile.from_dict({"length_m": 1.8, "width_m": 0.7, "lidar_x_m": 0.9,
                         "stop": {"front": 0.25, "rear": 0.25, "left": 0.10, "right": 0.10},
                         "slow": {"front": 1.0, "rear": 0.6, "left": 0.4, "right": 0.4}})
beside_rear = np.array([[-0.5, 0.6], [-0.5, 0.62], [-0.45, 0.6]])         # person walking beside the REAR half, 0.25 m from the flank
c = classify(beside_rear, sp_); f = Debouncer().update(0.0, counts_of(c), 1)
print("beside rear half: slow_side", c.slow_side, "| fwd cmd ->", gate(0.5, 0, f, sp_), "| rev cmd ->", gate(-0.5, 0, f, sp_))
assert c.slow_side == 3 and gate(0.5, 0, f, sp_)[0] == 0.15 and gate(-0.5, 0, f, sp_)[0] == -0.15
touching = np.array([[0.0, 0.40], [0.0, 0.41], [0.1, 0.40]])               # 5 cm from the flank: stop band
c = classify(touching, sp_); f = Debouncer().update(0.0, counts_of(c), 1)
assert c.stop_side == 3 and gate(0.5, 0.1, f, sp_) == (0.0, 0.0) and gate(-0.5, 0.1, f, sp_) == (0.0, 0.0), "side stop blocks both directions"
# beyond the rear edge still does not block driving forward
c = classify(np.array([[-1.1, 0.0]] * 3), sp_); f = Debouncer().update(0.0, counts_of(c), 1)
assert c.stop_rear == 3 and gate(0.5, 0, f, sp_) == (0.5, 0.0)
# explicit sensor pose (TF) replaces the manual numbers: a scan taken in base_link must not be shifted by lidar_x
ranges = [2.0]
manual = scan_to_base_points(ranges, 0.0, 0.1, 0.05, 20, sp_)               # profile says lidar is 0.9 m forward
via_tf = scan_to_base_points(ranges, 0.0, 0.1, 0.05, 20, sp_, pose=(0.0, 0.0, 0.0))
print("same range, manual pose x=%.1f vs TF pose x=%.1f" % (manual[0][0], via_tf[0][0])); assert abs(manual[0][0] - 2.9) < 1e-9 and abs(via_tf[0][0] - 2.0) < 1e-9
# signature: stable and sensitive
assert sp_.signature() == Profile.from_dict(sp_.to_dict()).signature() and len(sp_.signature()) == 8
other = Profile.from_dict(dict(sp_.to_dict(), stop={"front": 0.30, "rear": 0.25, "left": 0.10, "right": 0.10}))
assert other.signature() != sp_.signature()
print("SIGNATURE", sp_.signature())
print("SECTORS OK")
