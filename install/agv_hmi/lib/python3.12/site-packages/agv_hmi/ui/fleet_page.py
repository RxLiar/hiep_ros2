"""Fleet Monitor screen for viewing multiple AGVs from /fleet/snapshot."""
from __future__ import annotations

import json

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from agv_hmi.ui.fleet_map_widget import FleetMapWidget
from agv_hmi.ui.i18n import tr


class _MetricCard(QFrame):
    def __init__(self, title: str, value: str = "0"):
        super().__init__()
        self.setObjectName("Card")
        self.setStyleSheet(
            "QFrame#Card{background:#161B22;border:1px solid #30363D;border-radius:10px;}"
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(2)
        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("font-size:10px;color:#8B949E;background:transparent;")
        self.value_label = QLabel(value)
        self.value_label.setStyleSheet("font-size:20px;font-weight:700;color:#E6EDF3;background:transparent;")
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)

    def set_title(self, text: str) -> None:
        self.title_label.setText(text)

    def set_value(self, value) -> None:
        self.value_label.setText(str(value))


class FleetPage(QWidget):
    def __init__(self):
        super().__init__()
        self._robots: list[dict] = []
        self._selected_robot_id = ""
        self._current_map_filter = ""
        self._build()

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(18, 16, 18, 18)
        root.setSpacing(12)

        header = QHBoxLayout()
        self._title = QLabel(tr("fleet_title"))
        self._title.setStyleSheet("font-size:18px;font-weight:700;color:#E6EDF3;")
        header.addWidget(self._title)
        header.addStretch()
        self._map_label = QLabel(tr("fleet_map_filter"))
        self._map_combo = QComboBox()
        self._map_combo.setMinimumWidth(150)
        self._map_combo.currentIndexChanged.connect(self._on_map_changed)
        header.addWidget(self._map_label)
        header.addWidget(self._map_combo)
        root.addLayout(header)

        self._transport_status = QLabel(
            "⏳ Waiting for /fleet/snapshot — start fleet_server and fleet_agent"
        )
        self._transport_status.setWordWrap(True)
        self._transport_status.setStyleSheet(
            "padding:7px 10px;border:1px solid #9E6A03;border-radius:6px;"
            "background:#2E2000;color:#E3B341;font-size:11px;"
        )
        root.addWidget(self._transport_status)

        metrics = QGridLayout()
        metrics.setSpacing(10)
        self._metric_total = _MetricCard(tr("fleet_total"))
        self._metric_online = _MetricCard(tr("fleet_online"))
        self._metric_moving = _MetricCard(tr("fleet_moving"))
        self._metric_errors = _MetricCard(tr("fleet_errors"))
        for column, card in enumerate([
            self._metric_total,
            self._metric_online,
            self._metric_moving,
            self._metric_errors,
        ]):
            metrics.addWidget(card, 0, column)
        root.addLayout(metrics)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._build_table_panel())
        splitter.addWidget(self._build_map_panel())
        splitter.addWidget(self._build_detail_panel())
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 4)
        splitter.setStretchFactor(2, 2)
        splitter.setSizes([360, 660, 300])
        root.addWidget(splitter)

    def _build_table_panel(self) -> QWidget:
        box = QGroupBox(tr("fleet_robot_list"))
        layout = QVBoxLayout(box)
        self._table = QTableWidget(0, 7)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.verticalHeader().setVisible(False)
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self._table.horizontalHeader().setStretchLastSection(True)
        self._table.itemSelectionChanged.connect(self._on_table_selection)
        layout.addWidget(self._table)
        return box

    def _build_map_panel(self) -> QWidget:
        box = QGroupBox(tr("fleet_shared_map"))
        layout = QVBoxLayout(box)
        self._map_widget = FleetMapWidget()
        self._map_widget.robot_selected.connect(self._select_robot)
        layout.addWidget(self._map_widget)
        self._map_box = box
        return box

    def _build_detail_panel(self) -> QWidget:
        box = QGroupBox(tr("fleet_robot_detail"))
        layout = QVBoxLayout(box)
        self._detail_labels: dict[str, QLabel] = {}
        for key in (
            "robot_id",
            "robot_name",
            "map_id",
            "position",
            "battery",
            "connection",
            "nav_state",
            "mission",
            "waypoint",
            "error",
            "last_seen",
        ):
            label = QLabel("—")
            label.setWordWrap(True)
            label.setStyleSheet("font-size:12px;color:#E6EDF3;")
            self._detail_labels[key] = label
            layout.addWidget(label)
        layout.addStretch()
        self._detail_box = box
        self._render_detail(None)
        return box

    
    def update_snapshot_json(self, payload: str) -> None:
        try:
            data = json.loads(payload or "{}")
            robots = data.get("robots", [])
            if not isinstance(robots, list):
                raise ValueError("snapshot field 'robots' is not a list")
        except Exception as exc:
            self._transport_status.setText(
                f"❌ Invalid /fleet/snapshot payload: {exc}"
            )
            self._transport_status.setStyleSheet(
                "padding:7px 10px;border:1px solid #8B0000;border-radius:6px;"
                "background:#3D1515;color:#F85149;font-size:11px;"
            )
            return

        self._robots = [robot for robot in robots if isinstance(robot, dict)]
        if self._robots:
            self._transport_status.setText(
                f"✅ /fleet/snapshot connected — {len(self._robots)} AGV received"
            )
            self._transport_status.setStyleSheet(
                "padding:7px 10px;border:1px solid #2EA043;border-radius:6px;"
                "background:#1B3629;color:#3FB950;font-size:11px;"
            )
        else:
            self._transport_status.setText(
                "⚠ /fleet/snapshot is connected, but Fleet Server has not discovered any AGV. "
                "Check /fleet/robot_state."
            )
            self._transport_status.setStyleSheet(
                "padding:7px 10px;border:1px solid #9E6A03;border-radius:6px;"
                "background:#2E2000;color:#E3B341;font-size:11px;"
            )
        summary = data.get("summary", {}) if isinstance(data.get("summary"), dict) else {}
        self._metric_total.set_value(summary.get("total", len(self._robots)))
        self._metric_online.set_value(summary.get("online", 0))
        self._metric_moving.set_value(summary.get("moving", 0))
        self._metric_errors.set_value(summary.get("errors", 0))
        self._refresh_map_combo()
        self._refresh_view()

    def _refresh_map_combo(self) -> None:
        maps = sorted({str(robot.get("map_id", "")) for robot in self._robots if robot.get("map_id")})
        current = self._current_map_filter
        self._map_combo.blockSignals(True)
        self._map_combo.clear()
        self._map_combo.addItem(tr("fleet_all_maps"), "")
        for map_id in maps:
            self._map_combo.addItem(map_id, map_id)
        target = self._map_combo.findData(current)
        self._map_combo.setCurrentIndex(target if target >= 0 else 0)
        self._map_combo.blockSignals(False)
        self._current_map_filter = str(self._map_combo.currentData() or "")

    def _filtered_robots(self) -> list[dict]:
        if not self._current_map_filter:
            return list(self._robots)
        return [robot for robot in self._robots if robot.get("map_id") == self._current_map_filter]

    def _refresh_view(self) -> None:
        robots = self._filtered_robots()
        self._refresh_table(robots)
        self._map_widget.set_map_id(self._current_map_filter)
        self._map_widget.set_robots(robots)
        selected = next((r for r in robots if r.get("robot_id") == self._selected_robot_id), None)
        if selected is None and robots:
            selected = robots[0]
            self._selected_robot_id = str(selected.get("robot_id", ""))
        self._map_widget.set_selected_robot(self._selected_robot_id)
        self._render_detail(selected)

    def _refresh_table(self, robots: list[dict]) -> None:
        headers = [
            tr("fleet_col_robot"),
            tr("fleet_col_status"),
            tr("fleet_col_map"),
            tr("fleet_col_battery"),
            tr("fleet_col_mission"),
            tr("fleet_col_waypoint"),
            tr("fleet_col_error"),
        ]
        self._table.setHorizontalHeaderLabels(headers)
        self._table.setRowCount(len(robots))
        selected_row = -1
        for row, robot in enumerate(robots):
            health = str(robot.get("fleet_health", "online"))
            connection = str(robot.get("connection", "online"))
            status = health if health != "online" else connection
            battery = robot.get("battery_percent", -1)
            battery_text = "—" if int(battery) < 0 else f"{battery}%"
            wp_current = int(robot.get("waypoint_current", 0) or 0)
            wp_total = int(robot.get("waypoint_total", 0) or 0)
            waypoint = f"{wp_current}/{wp_total}" if wp_total else "—"
            values = [
                str(robot.get("robot_id", "—")),
                status,
                f"{robot.get('map_id', '—')} v{robot.get('map_version', 1)}",
                battery_text,
                str(robot.get("route_name") or robot.get("mission_id") or "—"),
                waypoint,
                str(robot.get("error_message") or robot.get("error_level", "ok")),
            ]
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if column in {2, 3, 5}:
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self._table.setItem(row, column, item)
            if values[0] == self._selected_robot_id:
                selected_row = row
        if selected_row >= 0:
            self._table.selectRow(selected_row)

    def _on_map_changed(self) -> None:
        self._current_map_filter = str(self._map_combo.currentData() or "")
        self._refresh_view()

    def _on_table_selection(self) -> None:
        row = self._table.currentRow()
        if row < 0:
            return
        item = self._table.item(row, 0)
        if item:
            self._select_robot(item.text())

    def _select_robot(self, robot_id: str) -> None:
        self._selected_robot_id = str(robot_id or "")
        self._map_widget.set_selected_robot(self._selected_robot_id)
        robot = next((r for r in self._filtered_robots() if r.get("robot_id") == self._selected_robot_id), None)
        self._render_detail(robot)

    def _render_detail(self, robot: dict | None) -> None:
        if not robot:
            for label in self._detail_labels.values():
                label.setText("—")
            return
        battery = int(robot.get("battery_percent", -1) or -1)
        wp_current = int(robot.get("waypoint_current", 0) or 0)
        wp_total = int(robot.get("waypoint_total", 0) or 0)
        values = {
            "robot_id": f"{tr('fleet_detail_id')}: {robot.get('robot_id', '—')}",
            "robot_name": f"{tr('fleet_detail_name')}: {robot.get('robot_name', '—')}",
            "map_id": f"{tr('fleet_detail_map')}: {robot.get('map_id', '—')} v{robot.get('map_version', 1)}",
            "position": f"{tr('fleet_detail_position')}: X {float(robot.get('x', 0.0)):.2f}, Y {float(robot.get('y', 0.0)):.2f}",
            "battery": f"{tr('fleet_detail_battery')}: {'—' if battery < 0 else str(battery) + '%'}",
            "connection": f"{tr('fleet_detail_connection')}: {robot.get('fleet_health', robot.get('connection', '—'))}",
            "nav_state": f"{tr('fleet_detail_nav')}: {robot.get('nav_state', '—')}",
            "mission": f"{tr('fleet_detail_mission')}: {robot.get('route_name') or robot.get('mission_id') or '—'}",
            "waypoint": f"{tr('fleet_detail_waypoint')}: {wp_current}/{wp_total}" if wp_total else f"{tr('fleet_detail_waypoint')}: —",
            "error": f"{tr('fleet_detail_error')}: {robot.get('error_message') or robot.get('error_level', 'ok')}",
            "last_seen": f"{tr('fleet_detail_last_seen')}: {float(robot.get('last_seen_sec', 0.0)):.1f}s",
        }
        for key, text in values.items():
            self._detail_labels[key].setText(text)

    def retranslate(self) -> None:
        self._title.setText(tr("fleet_title"))
        self._map_label.setText(tr("fleet_map_filter"))
        self._metric_total.set_title(tr("fleet_total"))
        self._metric_online.set_title(tr("fleet_online"))
        self._metric_moving.set_title(tr("fleet_moving"))
        self._metric_errors.set_title(tr("fleet_errors"))
        self._map_box.setTitle(tr("fleet_shared_map"))
        self._detail_box.setTitle(tr("fleet_robot_detail"))
        self._refresh_map_combo()
        self._refresh_view()
