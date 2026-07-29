"""Map widget for drawing multiple AGVs on one shared map."""
from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np
import yaml
from PyQt6.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PyQt6.QtWidgets import QWidget


class FleetMapWidget(QWidget):
    robot_selected = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 320)
        self.setMouseTracking(True)
        self._map_id = ""
        self._robots: list[dict] = []
        self._selected_robot_id = ""
        self._pixmap: QPixmap | None = None
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._resolution = 0.05
        self._map_w = 0
        self._map_h = 0
        self._robot_screen_positions: dict[str, QPointF] = {}

    def set_map_id(self, map_id: str) -> None:
        map_id = str(map_id or "")
        if map_id == self._map_id:
            return
        self._map_id = map_id
        self._load_registered_map(map_id)
        self.update()

    def set_robots(self, robots: list[dict]) -> None:
        self._robots = list(robots or [])
        self.update()

    def set_selected_robot(self, robot_id: str) -> None:
        self._selected_robot_id = str(robot_id or "")
        self.update()

    def load_map_file(self, yaml_path: str) -> bool:
        try:
            path = Path(yaml_path).expanduser()
            meta = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            image_path = Path(str(meta.get("image", "")))
            if not image_path.is_absolute():
                image_path = path.parent / image_path
            raw = QImage(str(image_path)).convertToFormat(QImage.Format.Format_Grayscale8)
            if raw.isNull():
                return False

            self._resolution = float(meta.get("resolution", 0.05))
            origin = meta.get("origin", [0.0, 0.0, 0.0])
            self._origin_x = float(origin[0])
            self._origin_y = float(origin[1])
            self._map_w = raw.width()
            self._map_h = raw.height()

            ptr = raw.constBits()
            ptr.setsize(raw.sizeInBytes())
            arr = np.frombuffer(ptr, dtype=np.uint8).reshape(
                (self._map_h, raw.bytesPerLine())
            )[:, : self._map_w].copy()
            if int(meta.get("negate", 0)):
                arr = 255 - arr
            rgb = np.stack([arr, arr, arr], axis=2)
            rgb = np.ascontiguousarray(rgb)
            qimg = QImage(
                rgb.data,
                self._map_w,
                self._map_h,
                self._map_w * 3,
                QImage.Format.Format_RGB888,
            ).copy()
            self._pixmap = QPixmap.fromImage(qimg)
            self.update()
            return True
        except Exception:
            self._pixmap = None
            return False

    def _load_registered_map(self, map_id: str) -> None:
        self._pixmap = None
        if not map_id:
            return
        base = Path.home() / ".agv_hmi" / "fleet_maps" / map_id
        candidates = [
            base / "active.yaml",
            base / f"{map_id}.yaml",
            Path.home() / "maps" / f"{map_id}.yaml",
        ]
        for candidate in candidates:
            if candidate.exists() and self.load_map_file(str(candidate)):
                return

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#0D1117"))
        self._robot_screen_positions.clear()

        if self._pixmap is not None and not self._pixmap.isNull():
            rect = self._map_target_rect()
            painter.drawPixmap(rect, self._pixmap, self._pixmap.rect())
            self._draw_robots_on_map(painter, rect)
        else:
            self._draw_grid(painter)
            self._draw_robots_auto_fit(painter)

        painter.setPen(QColor("#8B949E"))
        painter.setFont(QFont("Sans", 10))
        caption = self._map_id or "No map selected"
        painter.drawText(12, 22, caption)

    def _map_target_rect(self) -> QRectF:
        if self._map_w <= 0 or self._map_h <= 0:
            return QRectF(self.rect())
        margin = 16.0
        available_w = max(1.0, self.width() - margin * 2)
        available_h = max(1.0, self.height() - margin * 2)
        scale = min(available_w / self._map_w, available_h / self._map_h)
        width = self._map_w * scale
        height = self._map_h * scale
        return QRectF(
            (self.width() - width) / 2.0,
            (self.height() - height) / 2.0,
            width,
            height,
        )

    def _world_to_map_screen(self, x: float, y: float, rect: QRectF) -> QPointF:
        mx = (x - self._origin_x) / self._resolution
        my = self._map_h - (y - self._origin_y) / self._resolution
        return QPointF(
            rect.left() + mx / max(1, self._map_w) * rect.width(),
            rect.top() + my / max(1, self._map_h) * rect.height(),
        )

    def _draw_robots_on_map(self, painter: QPainter, rect: QRectF) -> None:
        for robot in self._robots:
            pos = self._world_to_map_screen(
                float(robot.get("x", 0.0)),
                float(robot.get("y", 0.0)),
                rect,
            )
            self._draw_robot(painter, robot, pos)

    def _draw_grid(self, painter: QPainter) -> None:
        painter.save()
        painter.setPen(QPen(QColor("#21262D"), 1))
        step = 40
        for x in range(0, self.width(), step):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), step):
            painter.drawLine(0, y, self.width(), y)
        painter.restore()

    def _draw_robots_auto_fit(self, painter: QPainter) -> None:
        if not self._robots:
            painter.setPen(QColor("#8B949E"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Waiting for fleet data...")
            return

        xs = [float(robot.get("x", 0.0)) for robot in self._robots]
        ys = [float(robot.get("y", 0.0)) for robot in self._robots]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        if math.isclose(min_x, max_x):
            min_x -= 1.0
            max_x += 1.0
        if math.isclose(min_y, max_y):
            min_y -= 1.0
            max_y += 1.0
        pad_x = max(1.0, (max_x - min_x) * 0.25)
        pad_y = max(1.0, (max_y - min_y) * 0.25)
        min_x -= pad_x
        max_x += pad_x
        min_y -= pad_y
        max_y += pad_y

        margin = 36.0
        for robot in self._robots:
            x = float(robot.get("x", 0.0))
            y = float(robot.get("y", 0.0))
            sx = margin + (x - min_x) / (max_x - min_x) * max(1.0, self.width() - 2 * margin)
            sy = self.height() - margin - (y - min_y) / (max_y - min_y) * max(1.0, self.height() - 2 * margin)
            self._draw_robot(painter, robot, QPointF(sx, sy))

    def _robot_color(self, robot: dict) -> QColor:
        health = str(robot.get("fleet_health", "online"))
        error = str(robot.get("error_level", "ok"))
        if health == "offline" or str(robot.get("connection")) == "offline":
            return QColor("#6E7681")
        if error == "error":
            return QColor("#F85149")
        if error == "warn" or health == "stale":
            return QColor("#E3B341")
        if bool(robot.get("moving")):
            return QColor("#58A6FF")
        return QColor("#3FB950")

    def _draw_robot(self, painter: QPainter, robot: dict, pos: QPointF) -> None:
        robot_id = str(robot.get("robot_id", "?"))
        self._robot_screen_positions[robot_id] = pos
        selected = robot_id == self._selected_robot_id
        radius = 13.0 if selected else 10.0
        color = self._robot_color(robot)

        painter.save()
        if selected:
            painter.setPen(QPen(QColor("#FFFFFF"), 2))
            painter.setBrush(QColor(255, 255, 255, 35))
            painter.drawEllipse(pos, radius + 7, radius + 7)

        painter.setBrush(color)
        painter.setPen(QPen(QColor("#FFFFFF"), 1.5))
        painter.drawEllipse(pos, radius, radius)

        yaw = float(robot.get("yaw", 0.0))
        tip = QPointF(pos.x() + math.cos(yaw) * 20, pos.y() - math.sin(yaw) * 20)
        painter.setPen(QPen(QColor("#FFFFFF"), 2))
        painter.drawLine(pos, tip)

        painter.setPen(QColor("#E6EDF3"))
        painter.setFont(QFont("Sans", 9, QFont.Weight.Bold))
        painter.drawText(QPointF(pos.x() + 14, pos.y() - 12), robot_id)
        painter.restore()

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return
        point = event.position()
        nearest_id = ""
        nearest_dist = 24.0
        for robot_id, robot_point in self._robot_screen_positions.items():
            distance = math.hypot(point.x() - robot_point.x(), point.y() - robot_point.y())
            if distance < nearest_dist:
                nearest_id = robot_id
                nearest_dist = distance
        if nearest_id:
            self.robot_selected.emit(nearest_id)
