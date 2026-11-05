#!/usr/bin/env python3
"""ros2 run hiep_robot2 safety_check  -  why do the safety zones have no effect?

Looks at the running system for ~3 s and prints a PASS / FAIL list:
  * is safety_monitor running and does it see /scan?
  * does a motor bridge listen to /cmd_vel_safe (otherwise the gate is not in the path)?
  * does anything still listen to /cmd_vel directly (it would bypass the gate)?
  * does the node run the configuration that the HMI applied?
  * where does it think the lidar is (TF or the manual profile pose)?
"""
import json
import sys
import time

OK, FAIL, WARN = "PASS", "FAIL", "WARN"


def evaluate(info: dict) -> list:
    """info -> list of (level, text, hint). Pure function (unit tested)."""
    out = []
    st = info.get("state")
    if st is None:
        out.append((FAIL, "No /safety_state received: safety_monitor is not running.",
                    "ros2 launch hiep_robot2 safety_monitor.launch.py   (or agv_hardware.launch.py profile:=real)"))
        return out
    out.append((OK, f"safety_monitor running, state={st.get('state')}, profile='{st.get('profile')}'", ""))
    if not st.get("enabled", True):
        out.append((FAIL, "The safety layer is DISABLED in the profile (safety_enabled=false).", "Robot & Safety page -> enable the safety layer -> Apply & save"))
    in_t, out_t = st.get("input_topic", "/cmd_vel"), st.get("output_topic", "/cmd_vel_safe")
    subs = info.get("out_subscribers")
    names = info.get("out_subscriber_names") or []
    if not subs and not names:
        out.append((FAIL, f"NOBODY listens to {out_t}: the gate is not in the path, the zones have NO effect.",
                    f"Start the motor bridge with cmd_vel_topic:={out_t} (agv_hardware.launch.py safety:=true) or remap your motor node to {out_t}"))
    else:
        out.append((OK, f"{out_t} is consumed by: {', '.join(names) if names else str(subs) + ' subscriber(s)'}", ""))
    direct = [n for n in (info.get("in_subscriber_names") or []) if n != "safety_monitor"]
    if direct:
        out.append((FAIL, f"These nodes still listen to {in_t} directly and ignore the safety zones: {', '.join(direct)}",
                    f"Make them listen to {out_t} instead (bridge parameter cmd_vel_topic, or a remap in your launch file / Gazebo bridge)"))
    else:
        out.append((OK, f"No other node listens to {in_t} directly.", ""))
    pubs = [n for n in (info.get("out_publisher_names") or []) if n != "safety_monitor"]
    if pubs:
        out.append((WARN, f"Other nodes also PUBLISH {out_t}: {', '.join(pubs)}", "Only safety_monitor should publish it"))
    age = st.get("scan_age_s")
    if age is None or age > 0.5:
        out.append((FAIL, "safety_monitor receives no fresh /scan.", "Check the lidar driver / merged scan topic name (scan_topic parameter)"))
    else:
        out.append((OK, f"/scan is received (frame '{st.get('scan_frame')}', {age:.2f} s old)", ""))
    src = st.get("pose_source")
    pose = st.get("lidar_pose")
    if src in ("tf", "same_frame"):
        out.append((OK, f"Lidar pose from TF ({src}): x={pose[0]} y={pose[1]} yaw={pose[2]} deg in '{st.get('base_frame')}'", ""))
    else:
        out.append((WARN, f"Lidar pose comes from the MANUAL profile numbers x={pose[0]} y={pose[1]} yaw={pose[2]} ({st.get('tf_error') or 'tf disabled'}).",
                    "If this is not the real lidar position (e.g. a merged scan expressed in base_link) every zone is shifted. Fix TF or the lidar pose in the profile"))
    want = info.get("hmi_signature")
    if want and want != st.get("profile_sig"):
        out.append((FAIL, f"The node runs another configuration (sig {st.get('profile_sig')}) than the HMI profile ({want}).",
                    "Robot & Safety page -> Apply & save (the node reads ~/.agv_hmi/robot_profile.json and /safety_config)"))
    elif want:
        out.append((OK, "The node runs the same configuration as the HMI profile.", ""))
    cin, cout = st.get("cmd_in") or [0, 0], st.get("cmd_out") or [0, 0]
    out.append((OK if st.get("state") == "ok" else WARN,
                f"Live: command in {cin} -> out {cout}, nearest object {st.get('nearest_m')} m, state {st.get('state')}", ""))
    return out


def main(args=None) -> None:
    import os
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String
    rclpy.init(args=args)
    node = rclpy.create_node("safety_check")
    box = {}
    node.create_subscription(String, "/safety_state", lambda m: box.__setitem__("state", json.loads(m.data)), 10)
    t_end = time.monotonic() + 3.0
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    st = box.get("state") or {}
    in_t, out_t = st.get("input_topic", "/cmd_vel"), st.get("output_topic", "/cmd_vel_safe")
    names = lambda infos: sorted({i.node_name for i in infos})
    info = {"state": box.get("state"),
            "out_subscriber_names": names(node.get_subscriptions_info_by_topic(out_t)),
            "in_subscriber_names": names(node.get_subscriptions_info_by_topic(in_t)),
            "out_publisher_names": names(node.get_publishers_info_by_topic(out_t))}
    try:
        with open(os.path.expanduser("~/.agv_hmi/robot_profile.json"), encoding="utf-8") as f:
            from hiep_robot2.safety_logic import load_active_profile
            info["hmi_signature"] = load_active_profile("~/.agv_hmi/robot_profile.json").signature()
    except Exception:
        pass
    failed = 0
    for level, text, hint in evaluate(info):
        print(f"[{level}] {text}")
        if hint and level != OK:
            print(f"       -> {hint}")
        failed += level == FAIL
    print("\nRESULT:", "the safety chain looks fine" if not failed else f"{failed} problem(s) found - fix the FAIL lines first")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
