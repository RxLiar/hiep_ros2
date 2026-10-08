"""scheduler.py - weekly schedule jobs (persisted) for routes."""
from __future__ import annotations

import json
import os
import uuid
from datetime import datetime

SCHEDULE_FILE = os.path.expanduser("~/.agv_hmi/schedule.json")


class Scheduler:
    def __init__(self, path: str = SCHEDULE_FILE):
        self._path = path
        self.jobs: list[dict] = self._load()

    def _load(self) -> list[dict]:
        try:
            with open(self._path, encoding="utf-8") as f:
                d = json.load(f)
            return d if isinstance(d, list) else []
        except (OSError, ValueError):
            return []

    def save(self) -> None:
        os.makedirs(os.path.dirname(self._path), exist_ok=True)
        with open(self._path, "w", encoding="utf-8") as f:
            json.dump(self.jobs, f, ensure_ascii=False, indent=2)

    def add(self, route_id: str, route_name: str, hhmm: str, days: list[int]) -> dict:
        job = {"id": uuid.uuid4().hex[:8], "route_id": route_id, "route_name": route_name,
               "time": hhmm, "days": sorted(set(days)) or list(range(7)), "enabled": True, "last_date": ""}
        self.jobs.append(job)
        self.save()
        return job

    def remove(self, job_id: str) -> None:
        self.jobs = [j for j in self.jobs if j["id"] != job_id]
        self.save()

    def set_enabled(self, job_id: str, on: bool) -> None:
        for j in self.jobs:
            if j["id"] == job_id:
                j["enabled"] = bool(on)
        self.save()

    def due(self, now: datetime | None = None) -> list[dict]:
        """Jobs whose time has come today (each fires once per day)."""
        now = now or datetime.now()
        today, hhmm, out = now.date().isoformat(), now.strftime("%H:%M"), []
        for j in self.jobs:
            if (j.get("enabled") and now.weekday() in j.get("days", []) and j.get("last_date") != today
                    and j.get("time", "99:99") <= hhmm):
                # only fire within 5 minutes of the planned time, so reopening the
                # HMI hours later does not start an old job
                h, m = map(int, j["time"].split(":"))
                if (now.hour * 60 + now.minute) - (h * 60 + m) <= 5:
                    j["last_date"] = today
                    out.append(j)
        if out:
            self.save()
        return out
