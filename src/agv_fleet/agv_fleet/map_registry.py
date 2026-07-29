"""Generate and store stable map IDs such as M02101."""
from __future__ import annotations

import json
import os
import secrets
import tempfile
from pathlib import Path
from typing import Any

from .protocol import validate_map_id

DEFAULT_REGISTRY = Path.home() / ".agv_hmi" / "fleet_maps" / "registry.json"


class MapRegistry:
    def __init__(self, path: str | os.PathLike[str] = DEFAULT_REGISTRY):
        self.path = Path(path).expanduser()

    def load(self) -> dict[str, Any]:
        if not self.path.exists():
            return {"maps": {}}
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if isinstance(data, dict) and isinstance(data.get("maps"), dict):
                return data
        except Exception:
            pass
        return {"maps": {}}

    def save(self, data: dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp_path = tempfile.mkstemp(prefix="registry_", suffix=".json", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(data, handle, ensure_ascii=False, indent=2)
            os.replace(tmp_path, self.path)
        finally:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)

    def generate_id(self) -> str:
        data = self.load()
        existing = set(data["maps"].keys())
        for _ in range(2000):
            map_id = f"M{secrets.randbelow(100000):05d}"
            if map_id not in existing:
                return map_id
        raise RuntimeError("unable to allocate a unique map ID")

    def register(self, name: str, yaml_path: str, version: int = 1, map_id: str = "") -> dict[str, Any]:
        data = self.load()
        selected_id = validate_map_id(map_id) if map_id else self.generate_id()
        record = {
            "map_id": selected_id,
            "name": str(name or selected_id),
            "version": max(1, int(version)),
            "yaml_path": str(Path(yaml_path).expanduser()),
        }
        data["maps"][selected_id] = record
        self.save(data)
        return record


def main() -> None:
    registry = MapRegistry()
    print(registry.generate_id())
