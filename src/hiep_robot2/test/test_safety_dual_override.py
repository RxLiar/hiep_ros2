"""Two lidars + engineer override of the safety node (no ROS needed)."""
import json, os, sys, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, HERE)
import mock_ros as M
M.install()
import numpy as np
from hiep_robot2.safety_monitor_node import SafetyMonitorNode
from hiep_robot2.safety_logic import Profile, OverrideManager

d = tempfile.mkdtemp(); prof = d + "/robot_profile.json"
P = {"length_m": 1.8, "width_m": 0.7,
     "stop": {"front": 0.25, "rear": 0.25, "left": 0.1, "right": 0.1}, "slow": {"front": 1.0, "rear": 0.6, "left": 0.4, "right": 0.4}}
json.dump({"active": "A", "profiles": {"A": P}}, open(prof, "w"))
M.Node.declare_parameter = lambda s, n, v: s._p.__setitem__(n, prof if n == "profile_file" else (False if n == "use_tf" else v))
n = SafetyMonitorNode()
assert n._active_idx() == [0] and "/scan" in n.subscriptions

def scan(front_m=None, frame="laser"):
    s = M.Scan(); s.header.frame_id = frame
    s.angle_min = -np.pi; s.angle_increment = np.pi / 180; s.range_min = 0.05; s.range_max = 20.0
    r = np.full(360, 15.0)
    if front_m is not None: r[178:183] = front_m
    s.ranges = list(r); return s
def command(lin, ang):
    m = M.Twist(); m.linear.x, m.angular.z = lin, ang; n._cmd_cb(m)
def out():
    n._tick(); o = n.out_pub.sent[-1]; return round(o.linear.x, 3), round(o.angular.z, 3)

# ---- 1) switch to dual mode live: the node re-subscribes to the two topics
cfg = dict(P, name="A", lidar_mode="dual", lidar1_topic="/front_lidar/scan", lidar2_topic="/rear_lidar/scan",
           lidar_x_m=0.9, lidar_y_m=0.0, lidar_yaw_deg=0.0, lidar2_x_m=-0.9, lidar2_y_m=0.0, lidar2_yaw_deg=180.0)
n._config_cb(M.String(json.dumps(cfg)))
assert n._active_idx() == [0, 1] and "/front_lidar/scan" in n.subscriptions and "/rear_lidar/scan" in n.subscriptions
assert "/scan" not in n.subscriptions, "the merged topic must be dropped in dual mode"
n.debounce.__init__(0.0)

# ---- 2) one lidar only => no_scan (a lost lidar is a lost safety function)
command(0.5, 0.0)
n.subscriptions["/front_lidar/scan"](scan())
assert n._mode() == "no_scan" and out() == (0.0, 0.0), n._mode()
n.subscriptions["/rear_lidar/scan"](scan())
assert n._mode() == "ok" and out() == (0.5, 0.0)

# ---- 3) the REAR lidar uses its own pose (yaw 180): an obstacle behind the robot is seen as behind it
# rear lidar at x=-0.9 looking backwards (yaw 180): a hit straight ahead of the sensor (angle 0) is behind the robot
n.subscriptions["/front_lidar/scan"](scan())
rear = scan(front_m=0.3)                        # 0.3 m in front of the rear lidar = x = -1.2 (0.3 m behind the rear edge)
n.subscriptions["/rear_lidar/scan"](rear)
res = n._nearest
print("rear obstacle:", res.stop_rear, res.slow_rear, round(res.nearest_m, 2))
assert res.stop_rear == 0 and res.slow_rear >= 3 and res.slow_front == 0
command(-0.5, 0.0); n.subscriptions["/rear_lidar/scan"](rear); n.subscriptions["/front_lidar/scan"](scan())
assert out() == (-0.15, 0.0), out()             # reversing towards it => slow
command(0.5, 0.0); assert out() == (0.5, 0.0)   # driving away is free

# ---- 4) state + points for the HMI
n._publish_state(); st = json.loads(n.state_pub.sent[-1].data)
assert st["lidar_mode"] == "dual" and [l["idx"] for l in st["lidars"]] == [0, 1] and st["scan_age_s"] < 0.5
assert st["lidars"][1]["pose"][2] == 180.0 or st["lidars"][1]["pose_source"] == "profile"
n._publish_points(); pts = json.loads(n.points_pub.sent[-1].data)
assert len(pts["n"]) == 2 and pts["n"][0] > 0 and pts["n"][1] > 0 and len(pts["pts"]) == 2 * sum(pts["n"])
assert st["profile_sig"] == Profile.from_dict(cfg).signature()

# ---- 5) engineer override
n.debounce.__init__(0.0)
n.subscriptions["/front_lidar/scan"](scan(front_m=0.15)); n.subscriptions["/rear_lidar/scan"](scan())
command(0.5, 0.2); assert n._mode() == "stop" and out() == (0.0, 0.0)
n._override_cb(M.String(json.dumps({"id": "hmi_teleop", "active": True, "ttl": 1.0})))
command(0.5, 0.2); assert n._mode() == "override" and out() == (0.5, 0.2), "zones off while overridden"
n._publish_state(); assert json.loads(n.state_pub.sent[-1].data)["override"] == ["hmi_teleop"]
# override also lets the robot move with NO scan (engineer teleop without lidar)
n._scans.clear(); command(0.4, 0.0); assert n._mode() == "override" and out() == (0.4, 0.0)
# turning it off puts the zones back
n._override_cb(M.String(json.dumps({"id": "hmi_teleop", "active": False})))
assert n._mode() == "no_scan" and out() == (0.0, 0.0)
# it expires by itself when the HMI stops refreshing it
n._override_cb(M.String(json.dumps({"id": "hmi_teleop", "active": True, "ttl": 0.1})))
assert n._mode() == "override"; time.sleep(0.15); assert n._mode() == "no_scan", "must expire without refresh"
# the ttl is capped, garbage is ignored
om = OverrideManager(); om.update(100.0, {"id": "x", "ttl": 999}); assert om.is_active(101.9) and not om.is_active(102.1)
om.update(100.0, "garbage"); om.update(100.0, {"ttl": "abc"})
# the override never overrides a profile that was switched off (state stays "disabled")
n._config_cb(M.String(json.dumps(dict(cfg, safety_enabled=False))))
n._override_cb(M.String(json.dumps({"id": "hmi_teleop", "active": True}))); assert n._mode() == "disabled"

# ---- 6) back to a single merged scan
n._config_cb(M.String(json.dumps(dict(cfg, lidar_mode="merged", safety_enabled=True))))
assert n._active_idx() == [0] and "/scan" in n.subscriptions and "/rear_lidar/scan" not in n.subscriptions
print("DUAL + OVERRIDE OK")
