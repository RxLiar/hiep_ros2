"""Route storage with Fleet map context.

Backward compatible with legacy routes that only contain ``map_path``.
New/updated routes also store ``map_id``, ``map_version`` and
``map_checksum`` so Fleet Agent can change maps automatically when a route is
started.
"""
from __future__ import annotations

import json
import os
import tempfile
import uuid
from datetime import datetime
from typing import Optional

ROUTES_DIR = os.path.expanduser("~/agv_routes")


def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _routes_file() -> str:
    os.makedirs(ROUTES_DIR, exist_ok=True)
    return os.path.join(ROUTES_DIR, "routes.json")


def load_all() -> list[dict]:
    path = _routes_file()
    if not os.path.exists(path):
        return []
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, list) else []
    except Exception as exc:
        print(f"[RouteManager] load failed: {exc}")
        return []


def save_all(routes: list[dict]):
    path = _routes_file()
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix="routes_", suffix=".json", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(routes, f, ensure_ascii=False, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, path)
    finally:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)


def _apply_map_context(
    route: dict,
    *,
    map_path: str,
    map_id: str = "",
    map_version: int = 1,
    map_checksum: str = "",
) -> None:
    route["map_path"] = str(map_path or "")
    route["map_id"] = str(map_id or "").strip().upper()
    route["map_version"] = max(1, int(map_version or 1)) if route["map_id"] else 0
    route["map_checksum"] = str(map_checksum or "").strip().lower()


def create_route(
    name: str,
    map_path: str = "",
    waypoints: Optional[list] = None,
    map_id: str = "",
    map_version: int = 1,
    map_checksum: str = "",
) -> dict:
    route = {
        "id": str(uuid.uuid4()),
        "name": name,
        "waypoints": waypoints or [],
        "created": _now(),
        "modified": _now(),
    }
    _apply_map_context(
        route,
        map_path=map_path,
        map_id=map_id,
        map_version=map_version,
        map_checksum=map_checksum,
    )
    routes = load_all()
    routes.append(route)
    save_all(routes)
    return route


def update_route(
    route_id: str,
    name: str,
    map_path: str,
    waypoints: list,
    map_id: str = "",
    map_version: int = 1,
    map_checksum: str = "",
) -> bool:
    routes = load_all()
    for route in routes:
        if route.get("id") == route_id:
            route["name"] = name
            route["waypoints"] = waypoints
            _apply_map_context(
                route,
                map_path=map_path,
                map_id=map_id,
                map_version=map_version,
                map_checksum=map_checksum,
            )
            route["modified"] = _now()
            save_all(routes)
            return True
    return False


def attach_map_context(
    route_id: str,
    *,
    map_id: str,
    map_version: int,
    map_path: str,
    map_checksum: str = "",
) -> bool:
    """Migrate a legacy route or refresh its canonical Fleet map fields."""
    routes = load_all()
    for route in routes:
        if route.get("id") == route_id:
            _apply_map_context(
                route,
                map_path=map_path,
                map_id=map_id,
                map_version=map_version,
                map_checksum=map_checksum,
            )
            route["modified"] = _now()
            save_all(routes)
            return True
    return False


def delete_route(route_id: str) -> bool:
    routes = load_all()
    new = [route for route in routes if route.get("id") != route_id]
    if len(new) == len(routes):
        return False
    save_all(new)
    return True


def get_route(route_id: str) -> Optional[dict]:
    for route in load_all():
        if route.get("id") == route_id:
            return route
    return None


def make_waypoint(
    label: str,
    x: float,
    y: float,
    action: str = "Không có",
    delay: int = 3,
    conveyor_id: Optional[int] = None,
    conveyor_mode: Optional[str] = None,
) -> dict:
    return {
        "label": label,
        "x": x,
        "y": y,
        "action": action,
        "delay": delay,
        "conveyor_id": conveyor_id,
        "conveyor_mode": conveyor_mode,
    }
