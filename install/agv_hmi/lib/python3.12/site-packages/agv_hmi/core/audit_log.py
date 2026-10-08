"""audit_log.py - who did what, when (append-only JSON lines)."""
from __future__ import annotations

import json
import os
from datetime import datetime

AUDIT_FILE = os.path.expanduser("~/.agv_hmi/audit.jsonl")
_user = {"name": "-", "role": "-"}


def set_user(name: str, role: str) -> None:
    _user["name"], _user["role"] = name or "-", role or "-"


def audit(action: str, detail: str = "") -> None:
    rec = {"at": datetime.now().isoformat(timespec="seconds"),
           "user": _user["name"], "role": _user["role"],
           "action": action, "detail": detail}
    try:
        os.makedirs(os.path.dirname(AUDIT_FILE), exist_ok=True)
        with open(AUDIT_FILE, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    except OSError:
        pass


def read_recent(limit: int = 500) -> list[dict]:
    try:
        with open(AUDIT_FILE, encoding="utf-8") as f:
            lines = f.readlines()[-limit:]
    except OSError:
        return []
    out = []
    for ln in lines:
        try:
            out.append(json.loads(ln))
        except ValueError:
            continue
    return list(reversed(out))
