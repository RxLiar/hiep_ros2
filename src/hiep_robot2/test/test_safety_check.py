import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from hiep_robot2.safety_check import evaluate, OK, FAIL, WARN
good = {"state": {"state": "ok", "enabled": True, "profile": "FL", "scan_age_s": 0.05, "scan_frame": "base_link", "pose_source": "tf",
                  "lidar_pose": [0.0, 0.0, 0.0], "base_frame": "base_footprint", "profile_sig": "abc", "input_topic": "/cmd_vel", "output_topic": "/cmd_vel_safe",
                  "cmd_in": [0.3, 0], "cmd_out": [0.3, 0], "nearest_m": None},
        "out_subscriber_names": ["keya_driver_bridge"], "in_subscriber_names": ["safety_monitor"], "out_publisher_names": ["safety_monitor"], "hmi_signature": "abc"}
res = evaluate(good); print("\n".join(f"{l} {t}" for l, t, _ in res)); assert not [r for r in res if r[0] == FAIL]
import copy
def fails(info): return [t for l, t, _ in evaluate(info) if l == FAIL]
bad = copy.deepcopy(good); bad["state"] = None; assert "not running" in fails(bad)[0]
bad = copy.deepcopy(good); bad["out_subscriber_names"] = []; f = fails(bad); print("no consumer ->", f[0]); assert "NOBODY listens" in f[0]
bad = copy.deepcopy(good); bad["in_subscriber_names"] = ["safety_monitor", "keya_driver_bridge", "ros_gz_bridge"]; f = fails(bad); print("direct ->", f[0]); assert "keya_driver_bridge" in f[0] and "ros_gz_bridge" in f[0]
bad = copy.deepcopy(good); bad["hmi_signature"] = "zzz"; assert "another configuration" in fails(bad)[0]
bad = copy.deepcopy(good); bad["state"]["scan_age_s"] = 3.0; assert "no fresh /scan" in fails(bad)[0]
bad = copy.deepcopy(good); bad["state"]["enabled"] = False; assert "DISABLED" in fails(bad)[0]
warn = copy.deepcopy(good); warn["state"]["pose_source"] = "profile"; warn["state"]["lidar_pose"] = [0.9, 0, 0]; warn["state"]["tf_error"] = "no frame"
assert any(l == WARN and "MANUAL" in t for l, t, _ in evaluate(warn))
print("SAFETY CHECK OK")
