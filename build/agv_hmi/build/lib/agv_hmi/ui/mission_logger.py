"""
mission_logger.py — Ghi lại lịch sử thực hiện lộ trình

Mỗi record:
{
  "id": "uuid4",
  "route_id": "...",
  "route_name": "Khu A → Kho B",
  "started_at": "2025-01-01T10:00:00",
  "finished_at": "2025-01-01T10:05:00",
  "duration_sec": 300,
  "status": "success" | "failed" | "cancelled",
  "waypoints_done": 3,
  "waypoints_total": 4,
  "cargo_count": 1,        # số lần băng tải hoàn thành task
  "notes": ""
}
"""
import json
import os
import uuid
from datetime import datetime
from typing import Optional

LOG_DIR  = os.path.expanduser("~/.agv_hmi/mission_logs")
LOG_FILE = os.path.join(LOG_DIR, "history.json")


def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _ensure_dir():
    os.makedirs(LOG_DIR, exist_ok=True)


def load_all() -> list[dict]:
    _ensure_dir()
    if not os.path.exists(LOG_FILE):
        return []
    try:
        with open(LOG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, list) else []
    except Exception:
        return []


def save_all(records: list[dict]):
    _ensure_dir()
    with open(LOG_FILE, "w", encoding="utf-8") as f:
        json.dump(records, f, ensure_ascii=False, indent=2)


def start_record(route_id: str, route_name: str, waypoints_total: int) -> dict:
    """Tạo record mới khi bắt đầu mission. Trả về record dict (chưa lưu)."""
    return {
        "id": str(uuid.uuid4()),
        "route_id": route_id,
        "route_name": route_name,
        "started_at": _now(),
        "finished_at": "",
        "duration_sec": 0,
        "status": "running",
        "waypoints_done": 0,
        "waypoints_total": waypoints_total,
        "cargo_count": 0,
        "notes": "",
    }


def finish_record(record: dict, status: str,
                  waypoints_done: int, cargo_count: int = 0,
                  notes: str = "") -> dict:
    """Hoàn chỉnh record và lưu vào file."""
    now = _now()
    record["finished_at"] = now
    record["status"] = status
    record["waypoints_done"] = waypoints_done
    record["cargo_count"] = cargo_count
    record["notes"] = notes

    try:
        started = datetime.fromisoformat(record["started_at"])
        finished = datetime.fromisoformat(now)
        record["duration_sec"] = int((finished - started).total_seconds())
    except Exception:
        record["duration_sec"] = 0

    records = load_all()
    records.insert(0, record)          # mới nhất lên đầu
    records = records[:500]            # giới hạn 500 bản ghi
    save_all(records)
    return record


def delete_record(record_id: str) -> bool:
    records = load_all()
    new = [r for r in records if r.get("id") != record_id]
    if len(new) == len(records):
        return False
    save_all(new)
    return True


def clear_all():
    save_all([])


def get_stats(route_id: Optional[str] = None) -> dict:
    """Thống kê nhanh: tổng lần chạy, thành công, hàng vận chuyển."""
    records = load_all()
    if route_id:
        records = [r for r in records if r.get("route_id") == route_id]

    total   = len(records)
    success = sum(1 for r in records if r.get("status") == "success")
    failed  = sum(1 for r in records if r.get("status") == "failed")
    cancelled = sum(1 for r in records if r.get("status") == "cancelled")
    cargo   = sum(r.get("cargo_count", 0) for r in records)

    return {
        "total": total,
        "success": success,
        "failed": failed,
        "cancelled": cancelled,
        "cargo_total": cargo,
    }