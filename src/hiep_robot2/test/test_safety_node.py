import json, os, sys, tempfile, time, types
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, HERE)
import mock_ros as M
M.install()
import numpy as np
from hiep_robot2.safety_monitor_node import SafetyMonitorNode
from hiep_robot2.safety_logic import Profile

d = tempfile.mkdtemp(); prof = d + "/robot_profile.json"
P = {"length_m": 1.8, "width_m": 0.7, "lidar_x_m": 0.9, "lidar_y_m": 0.0,
     "stop": {"front": 0.25, "rear": 0.25, "left": 0.1, "right": 0.1}, "slow": {"front": 1.0, "rear": 0.6, "left": 0.4, "right": 0.4}}
json.dump({"active": "AGV-1800", "profiles": {"AGV-1800": P}}, open(prof, "w"))
M.Node.declare_parameter = lambda s, n, v: s._p.__setitem__(n, prof if n == "profile_file" else (False if n == "use_tf" else v))
n = SafetyMonitorNode()
print("profile from file:", n.profile.name, "| sig", n.profile.signature())

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

# 1) the gate works with the manual lidar pose (use_tf false)
command(0.5, 0.2); assert out() == (0.0, 0.0) and n._mode() == "no_scan"
n._scan_cb(scan()); assert out() == (0.5, 0.2)
n._scan_cb(scan(front_m=0.6)); command(0.5, 0.2); print("person 0.6 m ahead ->", out(), n._mode()); assert n._mode() == "slow" and out() == (0.15, 0.2)
n._scan_cb(scan(front_m=0.15)); command(0.5, 0.2); assert n._mode() == "stop" and out() == (0.0, 0.0)

# 2) TF pose: the scan is expressed in base_link (merged scan), so the lidar offset must NOT be added
class Buf:
    def __init__(s, x=0.0, y=0.0, fail=False): s.x, s.y, s.fail = x, y, fail
    def lookup_transform(s, base, frame, t):
        if s.fail: raise RuntimeError("frame does not exist")
        q = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        return types.SimpleNamespace(transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=s.x, y=s.y), rotation=q))
n.debounce.__init__(0.0)                                     # no hold, so the test is not time dependent
n._tf_buffer = Buf(0.0, 0.0)
n._scan_cb(scan(front_m=1.3, frame="base_link")); command(0.5, 0.0)
print("TF pose:", n._pose_info, "| person at 1.3 m from the centre ->", out(), n._mode())
assert n._pose_info[3] == "tf" and n._pose_info[:2] == (0.0, 0.0)
# same obstacle with the WRONG manual pose (0.9 m forward) would be 2.2 m from the centre: not even in the slow zone
n._tf_buffer = None; n._scan_cb(scan(front_m=1.3, frame="base_link")); command(0.5, 0.0)
print("manual pose (0.9 m) ->", n._mode(), "(the obstacle was shifted out of the zone)"); assert n._mode() == "ok"
n._tf_buffer = Buf(fail=True); n._scan_cb(scan(front_m=0.6)); assert n._pose_info[3] == "profile" and n._tf_error
n._tf_buffer = Buf(0.9, 0.0); n.base_frame = "laser"; n._scan_cb(scan(front_m=0.6, frame="laser")); assert n._pose_info[3] == "same_frame"
n.base_frame = "base_footprint"; n._tf_buffer = None; n._tf_error = None

# 3) diagnostics in /safety_state
n.out_pub.subs = 1; n.get_subscriptions_info_by_topic = lambda t: [types.SimpleNamespace(node_name="safety_monitor"), types.SimpleNamespace(node_name="keya_driver_bridge")]
n._refresh_graph(); n._scan_cb(scan(front_m=0.6)); command(0.5, 0.2); n._tick(); n._publish_state()
st = json.loads(n.state_pub.sent[-1].data); print("state keys:", sorted(st))
assert st["profile_sig"] == n.profile.signature() and st["out_subscribers"] == 1
assert st["direct_consumers"] == ["keya_driver_bridge"], "a bridge still listening to /cmd_vel bypasses the gate"
assert st["cmd_in"] == [0.5, 0.2] and st["cmd_out"][0] == 0.15, st
assert st["pose_source"] == "profile" and st["lidar_pose"][0] == 0.9

# 4) live config changes the signature and the zones
cfg = dict(P, name="AGV-1800", stop={"front": 0.5, "rear": 0.25, "left": 0.1, "right": 0.1})
n._config_cb(M.String(json.dumps(cfg))); n._publish_state(); st2 = json.loads(n.state_pub.sent[-1].data)
assert st2["profile_sig"] != st["profile_sig"] and st2["profile_sig"] == Profile.from_dict(cfg).signature()
# 5) side hazard beside the rear half slows forward travel
n.debounce.__init__(0.0); s = M.Scan(); s.angle_min = np.pi / 2 - 0.05; s.angle_increment = 0.05; s.range_min = 0.05; s.range_max = 20
s.ranges = [0.55, 0.55, 0.55]; n._tf_buffer = Buf(-0.5, 0.0); n._scan_cb(s); command(0.5, 0.0)    # lidar at x=-0.5, person 0.55 m to its left = beside the rear half
print("beside the body ->", n._mode(), out()); assert n._mode() in ("slow", "stop")
print("SAFETY NODE OK")
