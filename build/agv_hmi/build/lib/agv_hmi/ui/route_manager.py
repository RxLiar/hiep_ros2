"""
route_manager.py — Quản lý lộ trình lưu trữ JSON

Mỗi route:
{
  "id": "uuid4",
  "name": "Khu A → Kho B",
  "map_path": "/home/.../maps/floor1.yaml",
  "waypoints": [
    {"label": "1", "x": 1.2, "y": 3.4, "action": "Không có", "delay": 3,
     "conveyor_id": null, "conveyor_mode": null}
  ],
  "created": "2025-01-01T00:00:00",
  "modified": "2025-01-01T00:00:00"
}
"""
import json
import os
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
    except Exception:
        return []


def save_all(routes: list[dict]):
    with open(_routes_file(), "w", encoding="utf-8") as f:
        json.dump(routes, f, ensure_ascii=False, indent=2)


def create_route(name: str, map_path: str = "", waypoints: Optional[list] = None) -> dict:
    route = {
        "id": str(uuid.uuid4()),
        "name": name,
        "map_path": map_path,
        "waypoints": waypoints or [],
        "created": _now(),
        "modified": _now(),
    }
    routes = load_all()
    routes.append(route)
    save_all(routes)
    return route


def update_route(route_id: str, name: str, map_path: str, waypoints: list) -> bool:
    routes = load_all()
    for r in routes:
        if r["id"] == route_id:
            r["name"] = name
            r["map_path"] = map_path
            r["waypoints"] = waypoints
            r["modified"] = _now()
            save_all(routes)
            return True
    return False


def delete_route(route_id: str) -> bool:
    routes = load_all()
    new = [r for r in routes if r["id"] != route_id]
    if len(new) == len(routes):
        return False
    save_all(new)
    return True


def get_route(route_id: str) -> Optional[dict]:
    for r in load_all():
        if r["id"] == route_id:
            return r
    return None


def make_waypoint(label: str, x: float, y: float,
                  action: str = "Không có", delay: int = 3,
                  conveyor_id: Optional[int] = None,
                  conveyor_mode: Optional[str] = None) -> dict:
    return {
        "label": label,
        "x": x,
        "y": y,
        "action": action,
        "delay": delay,
        "conveyor_id": conveyor_id,
        "conveyor_mode": conveyor_mode,
    }