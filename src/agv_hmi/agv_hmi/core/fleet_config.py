"""fleet_config.py - settings and logic behind the Fleet "control" panel (no Qt, unit-testable).

The HMI starts the two fleet services itself, instead of typing ros2 commands:

* robot side  : ``ros2 launch agv_fleet_domains robot_fleet_side.launch.py``  (Fleet Agent + domain_bridge)
* fleet server: ``ros2 launch agv_fleet_domains fleet_server_domain20.launch.py`` (Fleet Server in the Fleet domain)

Settings are stored in ``~/.agv_hmi/fleet_config.json``.
"""
from __future__ import annotations

import json
import os
import re
import tempfile
from dataclasses import asdict, dataclass, fields

CONFIG_PATH = os.path.join(os.path.expanduser("~"), ".agv_hmi", "fleet_config.json")
LOG_DIR = os.path.join(os.path.expanduser("~"), ".agv_hmi", "logs")
ROBOT_ID_RE = re.compile(r"^[A-Z][A-Z0-9_-]{2,31}$")
MAX_DOMAIN = 232
DEFAULT_FLEET_DOMAIN = 20
DEFAULT_ROBOT_DOMAIN = 21
REQUIRED_PACKAGES = ("agv_fleet", "agv_fleet_domains", "domain_bridge")
GENERIC_LAUNCH = "robot_fleet_side.launch.py"
SERVER_LAUNCH = "fleet_server_domain20.launch.py"


@dataclass
class FleetConfig:
    robot_id: str = "AGV001"
    robot_name: str = ""
    robot_domain: int = DEFAULT_ROBOT_DOMAIN
    fleet_domain: int = DEFAULT_FLEET_DOMAIN
    source_namespace: str = ""
    publish_hz: float = 5.0
    snapshot_hz: float = 5.0
    warning_timeout_sec: float = 1.5
    offline_timeout_sec: float = 3.0
    auto_connect: bool = False          # connect this robot when the HMI starts
    auto_server: bool = False           # start Fleet Server when the HMI starts

    @classmethod
    def from_dict(cls, data: dict | None) -> "FleetConfig":
        base = cls()
        data = data if isinstance(data, dict) else {}
        out = {}
        for f in fields(cls):
            if f.name not in data:
                continue
            try:
                v = data[f.name]
                cur = getattr(base, f.name)
                out[f.name] = (bool(v) if isinstance(cur, bool) else int(v) if isinstance(cur, int)
                               else float(v) if isinstance(cur, float) else str(v))
            except (TypeError, ValueError):
                pass
        cfg = cls(**out)
        cfg.robot_id = cfg.robot_id.strip().upper()
        cfg.source_namespace = cfg.source_namespace.strip().strip("/")
        return cfg

    def display_name(self) -> str:
        return self.robot_name.strip() or self.robot_id


def validate(cfg: FleetConfig) -> list[tuple[str, tuple]]:
    """[(i18n key, args)] of everything wrong with the settings; empty list = OK."""
    errs: list[tuple[str, tuple]] = []
    if not ROBOT_ID_RE.fullmatch(cfg.robot_id or ""):
        errs.append(("fc_err_robot_id", ()))
    for label, d in (("robot", cfg.robot_domain), ("fleet", cfg.fleet_domain)):
        if not 0 <= d <= MAX_DOMAIN:
            errs.append(("fc_err_domain_range", (label, MAX_DOMAIN)))
    if cfg.robot_domain == cfg.fleet_domain:
        errs.append(("fc_err_same_domain", ()))
    if not 0.5 <= cfg.publish_hz <= 20 or not 0.5 <= cfg.snapshot_hz <= 20:
        errs.append(("fc_err_rate", ()))
    if cfg.offline_timeout_sec <= cfg.warning_timeout_sec:
        errs.append(("fc_err_timeouts", ()))
    return errs


def _num(x: float) -> str:
    return f"{x:g}"


def agent_cmd(cfg: FleetConfig) -> list[str]:
    """Fleet Agent + domain_bridge of THIS robot. robot_id is part of the command so the process can be
    recognised (and cleaned up) per robot even when two robots are simulated on one computer."""
    cmd = ["ros2", "launch", "agv_fleet_domains", GENERIC_LAUNCH,
           f"robot_id:={cfg.robot_id}", f"robot_domain:={cfg.robot_domain}", f"fleet_domain:={cfg.fleet_domain}",
           f"publish_hz:={_num(cfg.publish_hz)}"]
    if cfg.robot_name.strip():
        cmd.append(f"robot_name:={cfg.robot_name.strip()}")
    if cfg.source_namespace:
        cmd.append(f"source_namespace:={cfg.source_namespace}")
    return cmd


def server_cmd(cfg: FleetConfig) -> list[str]:
    return ["ros2", "launch", "agv_fleet_domains", SERVER_LAUNCH,
            f"fleet_domain:={cfg.fleet_domain}", f"snapshot_hz:={_num(cfg.snapshot_hz)}",
            f"warning_timeout_sec:={_num(cfg.warning_timeout_sec)}", f"offline_timeout_sec:={_num(cfg.offline_timeout_sec)}"]


# -- storage ---------------------------------------------------------------------------
def default_config(env: dict | None = None) -> FleetConfig:
    """First start: take the robot id and domain from the environment the HMI was started in."""
    env = os.environ if env is None else env
    cfg = FleetConfig()
    rid = str(env.get("AGV_ROBOT_ID", "")).strip().upper()
    if ROBOT_ID_RE.fullmatch(rid):
        cfg.robot_id = rid
    dom = hmi_domain(env)
    if dom is not None and dom != cfg.fleet_domain:
        cfg.robot_domain = dom
    return cfg


def load(path: str = CONFIG_PATH, env: dict | None = None) -> FleetConfig:
    try:
        with open(path, "r", encoding="utf-8") as fh:
            return FleetConfig.from_dict(json.load(fh))
    except (OSError, ValueError):
        return default_config(env)


def save(cfg: FleetConfig, path: str = CONFIG_PATH) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".fleet.", suffix=".tmp", dir=os.path.dirname(path))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(asdict(cfg), fh, ensure_ascii=False, indent=2)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def remember_robot_id(robot_id: str, path: str = os.path.join(os.path.expanduser("~"), ".agv_hmi", "config.json")) -> None:
    """The HMI ROS node reads robot_id from ~/.agv_hmi/config.json (to address /fleet/map_context): keep it in step."""
    try:
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            data = data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            data = {}
        if data.get("robot_id") == robot_id:
            return
        data["robot_id"] = robot_id
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(data, fh, ensure_ascii=False, indent=2)
    except OSError:
        pass


# -- checks and explanations ---------------------------------------------------------------
def hmi_domain(env: dict | None = None) -> int | None:
    """ROS_DOMAIN_ID of the process the HMI runs in (unset = ROS default 0 -> None)."""
    env = os.environ if env is None else env
    try:
        return int(str(env.get("ROS_DOMAIN_ID", "")).strip())
    except ValueError:
        return None


def domain_hints(cfg: FleetConfig, hmi_dom: int | None) -> list[tuple[str, str, tuple]]:
    """[(level, i18n key, args)] - will THIS HMI see the fleet data with these settings?"""
    d = 0 if hmi_dom is None else hmi_dom
    if d == cfg.robot_domain:
        return [("ok", "fc_dom_ok", (d,))]
    if d == cfg.fleet_domain:
        return [("info", "fc_dom_fleet", (d,))]
    return [("warn", "fc_dom_mismatch", (d, cfg.robot_domain, cfg.fleet_domain))]


def check_packages(runner, exists=os.path.exists) -> dict[str, bool]:
    """runner(["ros2","pkg","prefix",name]) -> (returncode, stdout). True = installed (and, for
    agv_fleet_domains, new enough to contain the generic robot launch file)."""
    out: dict[str, bool] = {}
    for name in REQUIRED_PACKAGES:
        try:
            rc, text = runner(["ros2", "pkg", "prefix", name])
        except Exception:
            rc, text = 1, ""
        ok = rc == 0 and bool(text.strip())
        if ok and name == "agv_fleet_domains":
            prefix = text.strip().splitlines()[-1]
            ok = exists(os.path.join(prefix, "share", name, "launch", GENERIC_LAUNCH))
        out[name] = ok
    return out


def explain_state(agent_running: bool, server_running: bool, snapshot_age: float | None,
                  robots: list[dict], my_id: str, stale_after: float = 3.0) -> tuple[str, str, tuple]:
    """One sentence for the operator: (level ok/warn/bad/info, i18n key, args)."""
    fresh = snapshot_age is not None and snapshot_age <= stale_after
    ids = {str(r.get("robot_id", "")).upper(): r for r in robots if isinstance(r, dict)}
    if fresh:
        mine = ids.get(my_id.upper())
        if mine is not None and mine.get("fleet_health", "online") == "online":
            return "ok", "fc_st_connected", (my_id, len(ids))
        if mine is not None:
            return "warn", "fc_st_my_stale", (my_id, mine.get("fleet_health", "?"))
        if agent_running:
            return "warn", "fc_st_not_listed", (my_id,)
        return "ok", "fc_st_snapshot", (len(ids),)
    if not agent_running and not server_running:
        return "info", "fc_st_nothing", ()
    if agent_running and not server_running:
        return "warn", "fc_st_agent_waiting", ()
    if server_running and not agent_running:
        return "warn", "fc_st_server_no_robot", ()
    return "warn", "fc_st_both_waiting", ()
