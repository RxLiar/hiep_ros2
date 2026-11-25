"""Pure helpers (no ROS imports) to build the domain_bridge YAML of ANY robot.

Only the /fleet/* topics cross the domain boundary; navigation / TF / velocity topics stay in the robot domain.
The text produced for robot domain 21 -> fleet domain 20 is identical to config/agv001_fleet_bridge.yaml.
"""
from __future__ import annotations

import os
import re
import tempfile

ROBOT_ID_RE = re.compile(r"^[A-Z][A-Z0-9_-]{2,31}$")
MAX_DOMAIN = 232

_HEADER = """\
# Only Fleet topics cross the domain boundary.
# Navigation topics such as /cmd_vel, /amcl_pose, /odom, /scan,
# /map, /tf and /tf_static are intentionally NOT bridged.
"""

_TOPICS = """\

topics:
  /fleet/robot_state:
    type: std_msgs/msg/String
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 50

  /fleet/events:
    type: std_msgs/msg/String
    qos:
      reliability: reliable
      durability: volatile
      history: keep_last
      depth: 20

  # Fleet Server -> local AGV HMI Fleet Monitor
  /fleet/snapshot:
    type: std_msgs/msg/String
    reversed: true
    qos:
      reliability: reliable
      durability: transient_local
      history: keep_last
      depth: 1

  # Optional central Fleet command -> local Fleet Agent
  /fleet/map_context:
    type: std_msgs/msg/String
    reversed: true
    qos:
      reliability: reliable
      durability: transient_local
      history: keep_last
      depth: 20

  # Reserved for future pause/resume/cancel commands.
  /fleet/command:
    type: std_msgs/msg/String
    reversed: true
    qos:
      reliability: reliable
      durability: volatile
      history: keep_last
      depth: 10
"""


def validate(robot_id: str, robot_domain: int, fleet_domain: int) -> tuple[str, int, int]:
    rid = str(robot_id or "").strip().upper()
    if not ROBOT_ID_RE.fullmatch(rid):
        raise ValueError("robot_id must be 3-32 uppercase letters, digits, '_' or '-'")
    rd, fd = int(robot_domain), int(fleet_domain)
    for d in (rd, fd):
        if not 0 <= d <= MAX_DOMAIN:
            raise ValueError(f"ROS domain id must be 0..{MAX_DOMAIN}")
    if rd == fd:
        raise ValueError("robot_domain and fleet_domain must differ (a bridge needs two domains)")
    return rid, rd, fd


def bridge_yaml_text(robot_id: str, robot_domain: int, fleet_domain: int) -> str:
    rid, rd, fd = validate(robot_id, robot_domain, fleet_domain)
    return (_HEADER + f"name: {rid.lower()}_fleet_bridge\nfrom_domain: {rd}\nto_domain: {fd}\n" + _TOPICS)


def write_bridge_config(robot_id: str, robot_domain: int, fleet_domain: int, directory: str | None = None) -> str:
    """Write the YAML to a per-robot file and return its path (re-written every launch)."""
    rid, _, _ = validate(robot_id, robot_domain, fleet_domain)
    directory = directory or os.path.join(os.path.expanduser("~"), ".agv_hmi", "fleet")
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, f"{rid.lower()}_fleet_bridge.yaml")
    fd, tmp = tempfile.mkstemp(prefix=".bridge.", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(bridge_yaml_text(robot_id, robot_domain, fleet_domain))
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    return path
