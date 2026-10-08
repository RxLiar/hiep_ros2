"""backup.py - zip / restore maps, routes and HMI settings."""
from __future__ import annotations

import os
import zipfile
from datetime import datetime

HOME = os.path.expanduser("~")
# (path relative to $HOME). Thumbnails/log files are rebuilt, so they are skipped.
_INCLUDE = ["maps", "agv_routes", ".agv_hmi/config.json", ".agv_hmi/hmi_prefs.json",
            ".agv_hmi/stations.json", ".agv_hmi/schedule.json", ".agv_hmi/mission_logs"]
_ALLOWED = tuple(p if p.endswith(".json") else p + "/" for p in _INCLUDE) + tuple(
    p for p in _INCLUDE if p.endswith(".json"))


def default_name() -> str:
    return f"agv_hmi_backup_{datetime.now():%Y%m%d_%H%M%S}.zip"


def make_backup(zip_path: str, home: str = HOME) -> int:
    """Returns the number of files written."""
    n = 0
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
        for rel in _INCLUDE:
            full = os.path.join(home, rel)
            if os.path.isfile(full):
                z.write(full, rel); n += 1
            elif os.path.isdir(full):
                for root, _, files in os.walk(full):
                    for f in files:
                        fp = os.path.join(root, f)
                        z.write(fp, os.path.relpath(fp, home)); n += 1
    return n


def restore_backup(zip_path: str, home: str = HOME) -> int:
    """Extract a backup made by make_backup(). Unsafe/unknown paths are skipped."""
    n = 0
    with zipfile.ZipFile(zip_path) as z:
        for info in z.infolist():
            name = info.filename.replace("\\", "/")
            if info.is_dir() or name.startswith("/") or ".." in name.split("/"):
                continue
            if not (name in _INCLUDE or name.startswith(tuple(p + "/" for p in _INCLUDE))):
                continue
            dest = os.path.join(home, name)
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            with z.open(info) as src, open(dest, "wb") as out:
                out.write(src.read())
            n += 1
    return n
