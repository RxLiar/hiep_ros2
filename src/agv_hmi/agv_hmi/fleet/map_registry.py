"""Register HMI maps in the local Fleet map repository.

Each new map receives a stable identifier in the form ``M`` + five digits.
The canonical Fleet copy is stored under::

    ~/.agv_hmi/fleet_maps/<MAP_ID>/active.yaml
    ~/.agv_hmi/fleet_maps/<MAP_ID>/map.<ext>
    ~/.agv_hmi/fleet_maps/<MAP_ID>/manifest.json

Routes use the canonical ``active.yaml`` so Nav2 and Fleet Monitor always read
exactly the same YAML/image pair.
"""
from __future__ import annotations

import hashlib
import json
import os
import secrets
import shutil
import tempfile
import time
from pathlib import Path
from typing import Any

import yaml

MAP_ID_PREFIX = "M"
MAP_ID_DIGITS = 5
FLEET_MAP_ROOT = Path.home() / ".agv_hmi" / "fleet_maps"
REGISTRY_PATH = FLEET_MAP_ROOT / "registry.json"


def _atomic_write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(data, handle, ensure_ascii=False, indent=2)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_name, path)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


def _normalize_map_id(value: str) -> str:
    map_id = str(value or "").strip().upper()
    if len(map_id) != 6 or not map_id.startswith("M") or not map_id[1:].isdigit():
        raise ValueError("Map ID phải có dạng M + 5 chữ số, ví dụ M02101")
    return map_id


def _resolve_image(yaml_path: Path, metadata: dict[str, Any]) -> Path:
    image_value = str(metadata.get("image", "")).strip()
    if not image_value:
        raise ValueError(f"Map YAML thiếu trường image: {yaml_path}")
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    if not image_path.is_file():
        raise FileNotFoundError(f"Không tìm thấy map image: {image_path}")
    return image_path


def _map_fingerprint(yaml_path: Path) -> tuple[str, dict[str, Any], Path]:
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    if not isinstance(metadata, dict):
        raise ValueError(f"Map YAML không hợp lệ: {yaml_path}")
    image_path = _resolve_image(yaml_path, metadata)

    normalized = {
        "resolution": metadata.get("resolution"),
        "origin": metadata.get("origin"),
        "negate": metadata.get("negate", 0),
        "occupied_thresh": metadata.get("occupied_thresh"),
        "free_thresh": metadata.get("free_thresh"),
        "mode": metadata.get("mode", "trinary"),
    }
    digest = hashlib.sha256()
    digest.update(
        json.dumps(normalized, sort_keys=True, separators=(",", ":")).encode("utf-8")
    )
    digest.update(b"\0")
    with image_path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest(), metadata, image_path


class HmiFleetMapRegistry:
    """Local registry used by Mapping, Routes and Fleet Monitor."""

    def __init__(
        self,
        root: str | os.PathLike[str] = FLEET_MAP_ROOT,
        registry_path: str | os.PathLike[str] | None = None,
    ) -> None:
        self.root = Path(root).expanduser()
        self.registry_path = (
            Path(registry_path).expanduser()
            if registry_path is not None
            else self.root / "registry.json"
        )
        self.root.mkdir(parents=True, exist_ok=True)

    def load(self) -> dict[str, Any]:
        if not self.registry_path.exists():
            return {"schema": "agv_fleet_map_registry_v1", "maps": {}}
        try:
            data = json.loads(self.registry_path.read_text(encoding="utf-8"))
            if isinstance(data, dict) and isinstance(data.get("maps"), dict):
                return data
        except Exception as exc:
            print(f"[FleetMapRegistry] Không đọc được registry: {exc}")
        return {"schema": "agv_fleet_map_registry_v1", "maps": {}}

    def save(self, data: dict[str, Any]) -> None:
        data["schema"] = "agv_fleet_map_registry_v1"
        data.setdefault("maps", {})
        _atomic_write_json(self.registry_path, data)

    def generate_id(self, data: dict[str, Any] | None = None) -> str:
        registry = data if data is not None else self.load()
        existing = set(registry.get("maps", {}).keys())
        existing.update(
            path.name for path in self.root.glob("M[0-9][0-9][0-9][0-9][0-9]")
            if path.is_dir()
        )
        for _ in range(5000):
            map_id = f"M{secrets.randbelow(100000):05d}"
            if map_id not in existing:
                return map_id
        raise RuntimeError("Không thể tạo Map ID không trùng sau 5000 lần thử")

    def _find_by_checksum(
        self, data: dict[str, Any], checksum: str
    ) -> dict[str, Any] | None:
        candidates = [
            record
            for record in data.get("maps", {}).values()
            if isinstance(record, dict) and record.get("checksum") == checksum
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda item: float(item.get("updated_at", 0.0)))

    def _find_latest_by_source(
        self, data: dict[str, Any], source_yaml: Path
    ) -> dict[str, Any] | None:
        source = str(source_yaml.resolve())
        candidates = [
            record
            for record in data.get("maps", {}).values()
            if isinstance(record, dict) and record.get("source_yaml") == source
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda item: float(item.get("updated_at", 0.0)))

    def _adopt_existing_directory(
        self, data: dict[str, Any], checksum: str
    ) -> dict[str, Any] | None:
        """Import a manually created Mxxxxx/active.yaml into registry.json."""
        for map_dir in sorted(self.root.glob("M[0-9][0-9][0-9][0-9][0-9]")):
            if not map_dir.is_dir():
                continue
            active_yaml = map_dir / "active.yaml"
            if not active_yaml.is_file():
                continue
            try:
                existing_checksum, metadata, image_path = _map_fingerprint(active_yaml)
            except Exception:
                continue
            if existing_checksum != checksum:
                continue

            map_id = _normalize_map_id(map_dir.name)
            manifest_path = map_dir / "manifest.json"
            manifest: dict[str, Any] = {}
            if manifest_path.is_file():
                try:
                    loaded = json.loads(manifest_path.read_text(encoding="utf-8"))
                    if isinstance(loaded, dict):
                        manifest = loaded
                except Exception:
                    manifest = {}
            now = time.time()
            record = {
                "schema": "agv_fleet_map_manifest_v1",
                "map_id": map_id,
                "map_version": max(1, int(manifest.get("map_version", 1))),
                "name": str(manifest.get("name") or map_id),
                "checksum": checksum,
                "source_yaml": str(manifest.get("source_yaml") or active_yaml.resolve()),
                "source_image": str(manifest.get("source_image") or image_path.resolve()),
                "active_yaml": str(active_yaml.resolve()),
                "active_image": str(image_path.resolve()),
                "created_at": float(manifest.get("created_at", now)),
                "updated_at": float(manifest.get("updated_at", now)),
            }
            _atomic_write_json(manifest_path, record)
            data.setdefault("maps", {})[map_id] = record
            self.save(data)
            print(f"[FleetMapRegistry] Đã nhận map thủ công hiện có: {map_id}")
            return record
        return None

    def _write_canonical_copy(
        self,
        map_id: str,
        version: int,
        source_yaml: Path,
        metadata: dict[str, Any],
        source_image: Path,
        checksum: str,
        name: str,
    ) -> dict[str, Any]:
        map_dir = self.root / map_id
        map_dir.mkdir(parents=True, exist_ok=True)

        image_suffix = source_image.suffix.lower() or ".pgm"
        canonical_image = map_dir / f"map{image_suffix}"
        canonical_yaml = map_dir / "active.yaml"
        manifest_path = map_dir / "manifest.json"

        shutil.copy2(source_image, canonical_image)
        active_metadata = dict(metadata)
        active_metadata["image"] = canonical_image.name
        canonical_yaml.write_text(
            yaml.safe_dump(active_metadata, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )

        now = time.time()
        manifest = {
            "schema": "agv_fleet_map_manifest_v1",
            "map_id": map_id,
            "map_version": int(version),
            "name": str(name or map_id),
            "checksum": checksum,
            "source_yaml": str(source_yaml.resolve()),
            "source_image": str(source_image.resolve()),
            "active_yaml": str(canonical_yaml.resolve()),
            "active_image": str(canonical_image.resolve()),
            "updated_at": now,
        }
        previous = {}
        if manifest_path.exists():
            try:
                previous = json.loads(manifest_path.read_text(encoding="utf-8"))
            except Exception:
                previous = {}
        manifest["created_at"] = float(previous.get("created_at", now))
        _atomic_write_json(manifest_path, manifest)
        return manifest

    def register_new_map(self, yaml_path: str, name: str = "") -> dict[str, Any]:
        """Allocate a new Map ID for a newly saved Mapping result."""
        source_yaml = Path(yaml_path).expanduser().resolve()
        if not source_yaml.is_file():
            raise FileNotFoundError(f"Không tìm thấy map YAML: {source_yaml}")
        checksum, metadata, source_image = _map_fingerprint(source_yaml)
        data = self.load()
        map_id = self.generate_id(data)
        version = 1
        record = self._write_canonical_copy(
            map_id=map_id,
            version=version,
            source_yaml=source_yaml,
            metadata=metadata,
            source_image=source_image,
            checksum=checksum,
            name=name or source_yaml.stem,
        )
        data.setdefault("maps", {})[map_id] = record
        self.save(data)
        print(
            f"[FleetMapRegistry] Đã tạo {map_id} v{version}: "
            f"{record['active_yaml']}"
        )
        return dict(record)

    def ensure_registered(
        self,
        yaml_path: str,
        name: str = "",
        preferred_map_id: str = "",
    ) -> dict[str, Any]:
        """Return an existing registration for identical map data or create one.

        Legacy routes containing only ``map_path`` are migrated automatically.
        If the YAML/image content is already registered, its existing Map ID is
        reused. Changed map content receives a new Map ID rather than silently
        reusing coordinates from an older map.
        """
        source_yaml = Path(yaml_path).expanduser().resolve()
        if not source_yaml.is_file():
            raise FileNotFoundError(f"Không tìm thấy map YAML: {source_yaml}")
        checksum, metadata, source_image = _map_fingerprint(source_yaml)
        data = self.load()

        record: dict[str, Any] | None = None
        if preferred_map_id:
            selected_id = _normalize_map_id(preferred_map_id)
            candidate = data.get("maps", {}).get(selected_id)
            if isinstance(candidate, dict) and candidate.get("checksum") == checksum:
                record = candidate

        if record is None:
            record = self._find_by_checksum(data, checksum)
        if record is None:
            # Preserve manually registered maps such as M02101 even when an
            # older version of the project did not create registry.json.
            record = self._adopt_existing_directory(data, checksum)

        if record is not None:
            # Repair missing canonical files if needed.
            active_yaml = Path(str(record.get("active_yaml", ""))).expanduser()
            active_image = Path(str(record.get("active_image", ""))).expanduser()
            if not active_yaml.is_file() or not active_image.is_file():
                record = self._write_canonical_copy(
                    map_id=str(record["map_id"]),
                    version=max(1, int(record.get("map_version", 1))),
                    source_yaml=source_yaml,
                    metadata=metadata,
                    source_image=source_image,
                    checksum=checksum,
                    name=name or str(record.get("name") or source_yaml.stem),
                )
                data.setdefault("maps", {})[record["map_id"]] = record
                self.save(data)
            return dict(record)

        # The source path may have been overwritten with a genuinely different
        # map. Allocate a new ID so old routes remain bound to their old map.
        return self.register_new_map(str(source_yaml), name=name or source_yaml.stem)

    def get(self, map_id: str) -> dict[str, Any] | None:
        selected_id = _normalize_map_id(map_id)
        record = self.load().get("maps", {}).get(selected_id)
        return dict(record) if isinstance(record, dict) else None
