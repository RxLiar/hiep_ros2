"""Two-lidar profile fields, signature parity with the ROS node, and the safety_diag output (no Qt / ROS needed)."""
import os, random, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from agv_hmi.core import robot_profile as RP, safety_diag as SD


def test_defaults_and_clamping():
    d = RP.normalize(None)
    assert d["lidar_mode"] == "merged" and d["lidar2_yaw_deg"] == 180.0
    d = RP.normalize({"lidar_mode": "bogus", "lidar1_topic": "no slash", "lidar2_topic": "/rear/scan",
                      "lidar2_x_m": 99, "lidar2_yaw_deg": -999})
    assert d["lidar_mode"] == "merged" and d["lidar1_topic"] == "/front_lidar/scan"
    assert d["lidar2_topic"] == "/rear/scan" and d["lidar2_x_m"] == 5.0 and d["lidar2_yaw_deg"] == -180.0


def test_signature_changes_with_lidar2():
    a = RP.normalize({"lidar_mode": "dual"}); b = dict(a, lidar2_x_m=-1.2)
    assert RP.signature(a) != RP.signature(b)
    assert RP.signature(a) != RP.signature(dict(a, lidar_mode="merged"))


def test_signature_parity_with_ros_node():
    try:
        sys.path.insert(0, os.path.join(HERE, "..", "..", "hiep_robot2"))
        from hiep_robot2.safety_logic import Profile
    except ImportError:
        return                                   # hiep_robot2 not next to this package: nothing to compare
    random.seed(3)
    for _ in range(100):
        d = RP.normalize({"length_m": random.uniform(.5, 3), "lidar_mode": random.choice(["merged", "dual"]),
                          "lidar2_x_m": random.uniform(-3, 3), "lidar2_topic": random.choice(["/a", "bad", "/r/s"])})
        assert RP.signature(d) == Profile.from_dict(d).signature()


def test_diag_two_lidars_and_override():
    st = {"state": "ok", "enabled": True, "out_subscribers": 1, "direct_consumers": [], "lidar_mode": "dual",
          "override": ["hmi_teleop"], "profile_sig": "x",
          "lidars": [{"idx": 0, "topic": "/f", "age_s": 0.1, "pose": [0.9, 0, 0], "pose_source": "tf"},
                     {"idx": 1, "topic": "/r", "age_s": None}]}
    keys = [(lvl, k) for lvl, k, _ in SD.diagnose(st, "x", True)]
    assert (SD.WARN, "sd_override") in keys and (SD.OK, "sd_lidar_ok") in keys and (SD.FAIL, "sd_lidar_lost") in keys
    assert not any(k == "sd_no_scan" for _, k in keys)
    st["lidar_mode"] = "merged"; st["scan_age_s"] = 0.1
    assert any(k == "sd_scan_ok" for _, k, _a in SD.diagnose(st, "x", True))


if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f(); print("ok", n)
