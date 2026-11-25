import os, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from agv_fleet_domains import bridge_config as B

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config")


def test_generated_yaml_equals_the_shipped_files():
    for rid, dom, fn in (("AGV001", 21, "agv001_fleet_bridge.yaml"), ("AGV002", 22, "agv002_fleet_bridge.yaml")):
        assert B.bridge_yaml_text(rid, dom, 20) == open(os.path.join(CFG, fn)).read()


def test_write_bridge_config_for_a_new_robot():
    d = tempfile.mkdtemp()
    p = B.write_bridge_config("agv007", 27, 20, d)
    text = open(p).read()
    assert p.endswith("agv007_fleet_bridge.yaml") and "from_domain: 27" in text and "to_domain: 20" in text
    assert "/cmd_vel" not in text.replace("/cmd_vel,", "").split("topics:")[1], "navigation topics must never be bridged"


def test_validation():
    assert B.validate("agv003", "23", "20") == ("AGV003", 23, 20)
    for bad in (("A1", 21, 20), ("1AGV", 21, 20), ("AGV001", 20, 20), ("AGV001", 300, 20), ("AGV 01", 21, 20)):
        try:
            B.validate(*bad)
        except ValueError:
            continue
        raise AssertionError(bad)


if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f(); print("ok", n)
