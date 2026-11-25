"""safety_diag.py - turn a /safety_state message into a PASS / WARN / FAIL list for the HMI."""
from __future__ import annotations

OK, WARN, FAIL = "ok", "warn", "fail"


def diagnose(state: dict | None, local_sig: str | None, state_fresh: bool = True) -> list[tuple[str, str, tuple]]:
    """-> [(level, i18n key, args)]. The HMI shows tr(key, *args) and tr(key + '_hint') for non-OK items."""
    if not state or not state_fresh:
        return [(FAIL, "sd_not_running", ())]
    out = [(OK, "sd_running", (state.get("state", "?"), state.get("profile", "?")))]
    if state.get("enabled") is False:
        out.append((FAIL, "sd_disabled", ()))
    subs = state.get("out_subscribers")
    topic = state.get("output_topic", "/cmd_vel_safe")
    if subs == 0:
        out.append((FAIL, "sd_no_consumer", (topic,)))
    elif subs is None:
        out.append((WARN, "sd_consumer_unknown", (topic,)))
    else:
        out.append((OK, "sd_consumer_ok", (topic, subs)))
    direct = state.get("direct_consumers")
    if direct:
        out.append((FAIL, "sd_direct", (state.get("input_topic", "/cmd_vel"), ", ".join(direct))))
    elif direct is not None:
        out.append((OK, "sd_direct_ok", (state.get("input_topic", "/cmd_vel"),)))
    if state.get("override"):
        out.append((WARN, "sd_override", (", ".join(state["override"]),)))
    if state.get("lidar_mode") == "dual" and state.get("lidars"):
        # two lidars: report each one, so it is obvious which of them is missing
        for lid in state["lidars"]:
            n, topic, age = int(lid.get("idx", 0)) + 1, lid.get("topic", "?"), lid.get("age_s")
            if age is None or age > 0.5:
                out.append((FAIL, "sd_lidar_lost", (n, topic)))
            else:
                pose = lid.get("pose") or [0, 0, 0]
                from_tf = lid.get("pose_source") in ("tf", "same_frame")
                out.append((OK if from_tf else WARN, "sd_lidar_ok" if from_tf else "sd_lidar_profile",
                            (n, topic, pose[0], pose[1], pose[2])))
    else:
        age = state.get("scan_age_s")
        if age is None or age > 0.5:
            out.append((FAIL, "sd_no_scan", ()))
        else:
            out.append((OK, "sd_scan_ok", (state.get("scan_frame", "?"),)))
        pose = state.get("lidar_pose") or [0, 0, 0]
        if state.get("pose_source") in ("tf", "same_frame"):
            out.append((OK, "sd_pose_tf", (pose[0], pose[1], pose[2], state.get("base_frame", ""))))
        else:
            out.append((WARN, "sd_pose_profile", (pose[0], pose[1], pose[2])))
    if local_sig and state.get("profile_sig") and local_sig != state.get("profile_sig"):
        out.append((FAIL, "sd_sig_bad", ()))
    elif local_sig:
        out.append((OK, "sd_sig_ok", ()))
    cin, cout = state.get("cmd_in") or [0, 0], state.get("cmd_out") or [0, 0]
    out.append((OK if state.get("state") == "ok" else WARN, "sd_live",
                (cin[0], cin[1], cout[0], cout[1], state.get("state", "?"))))
    return out
