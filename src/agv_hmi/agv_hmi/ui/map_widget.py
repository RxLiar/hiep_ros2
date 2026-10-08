import math
import os
import yaml
import numpy as np

from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import (
    QPainter,
    QColor,
    QImage,
    QPen,
    QBrush,
    QPolygon,
    QPixmap,
    QFont,
    QPainterPath,   # FIX 1: thêm QPainterPath cho _draw_nav_path()
)
from PyQt6.QtCore import Qt, QPoint, QPointF, QRectF, QLineF, pyqtSignal
#                                               ^^^^^ FIX 2: thêm QRectF, bỏ inline import trong _draw_robot()


ROBOT_L = 1.800
ROBOT_W = 0.700

C_BG         = QColor(13,  17,  23)
C_ROBOT_FILL = QColor(55,  138, 221, 200)
C_ROBOT_EDGE = QColor(255, 255, 255, 220)
C_ARROW      = QColor(248, 81,  73)
C_SCAN       = QColor(50,  220, 120, 150)
C_GOAL_DONE  = QColor(63,  185, 80)
C_GOAL_CURR  = QColor(88,  166, 255)
C_GOAL_END   = QColor(248, 81,  73)
C_POSE_EST   = QColor(230, 179, 65,  200)
C_NAV_PATH   = QColor(63,  185, 80,  180)

# Map palette (soft, high-contrast walls)
C_MAP_UNKNOWN = (30, 35, 44)
C_MAP_FREE    = (232, 236, 242)
C_MAP_OCC     = (17, 21, 28)
C_WALL        = QColor(17, 21, 28)
C_GRID_MINOR  = QColor(0, 0, 0, 22)
C_GRID_MAJOR  = QColor(0, 0, 0, 55)
C_MEASURE     = QColor(255, 196, 61)

MODE_GOAL = "goal"
MODE_POSE = "pose"
MODE_MEASURE = "measure"
MODE_ZONE = "zone"


class MapWidget(QWidget):
    goal_selected    = pyqtSignal(float, float)
    pose_estimate_set = pyqtSignal(float, float, float)
    zones_changed = pyqtSignal()

    def __init__(self):
        super().__init__()

        self._origin_x   = 0.0
        self._origin_y   = 0.0
        self._resolution = 0.05
        self._map_w      = 0
        self._map_h      = 0
        self._pixmap: QPixmap | None = None

        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0

        self._scan_pts:  list[tuple[float, float]]        = []
        self._waypoints: list[tuple[float, float, str]]   = []
        self._pose_est:  tuple[float, float, float] | None = None
        self._nav_path:  list[tuple[float, float]]        = []

        self._mode = MODE_GOAL

        # Clean-map data (cell coordinates, row 0 = lowest y)
        self._grid: np.ndarray | None = None
        self._segments: list[tuple[float, float, float, float]] = []
        self._wall_thickness_m = 0.10
        self._follow = False
        self._show_grid = True
        self._show_minimap = True
        self._measure_pts: list[tuple[float, float]] = []
        self._zones: list[dict] = []            # {'kind': 'keepout'|'slow', 'pts': [[x, y], ...]}
        self._zone_draft: list[tuple[float, float]] = []
        self._zone_kind = "keepout"
        self._robot_len = ROBOT_L
        self._robot_wid = ROBOT_W
        self._robot_cx = 0.0
        self._safety: dict | None = None

        self._zoom      = 1.0
        self._pan_x     = 0.0
        self._pan_y     = 0.0
        self._drag_start = None
        self._pan_start  = (0.0, 0.0)

        self._pose_dragging    = False
        self._pose_drag_origin: tuple[float, float] | None = None

        self.setMinimumSize(400, 300)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.WheelFocus)

    # ── Public API ──────────────────────────────────────────────────

    def set_mode(self, mode: str):
        self._mode = mode
        self.setCursor(
            Qt.CursorShape.CrossCursor if mode in (MODE_POSE, MODE_MEASURE, MODE_ZONE)
            else Qt.CursorShape.ArrowCursor
        )
        if mode != MODE_MEASURE:
            self._measure_pts = []
        if mode != MODE_ZONE:
            self._zone_draft = []
        self.update()

    # ── Robot size / safety overlay ─────────────────────────────────

    def set_robot_size(self, length_m: float, width_m: float, center_x_m: float = 0.0):
        """Footprint used when drawing the robot (centre offset from base_link, + forward)."""
        self._robot_len, self._robot_wid, self._robot_cx = float(length_m), float(width_m), float(center_x_m)
        self.update()

    def set_safety_overlay(self, profile: dict | None):
        """Draw the STOP / SAFETY rectangles around the robot (None hides them)."""
        self._safety = profile
        self.update()

    # ── Zones (keep-out / slow) ─────────────────────────────────────

    def begin_zone(self, kind: str):
        self._zone_kind = kind
        self._zone_draft = []
        self.set_mode(MODE_ZONE)
        self.setFocus()

    def undo_zone(self):
        if self._zone_draft:
            self._zone_draft = []
        elif self._zones:
            self._zones.pop()
        self.zones_changed.emit()
        self.update()

    def get_zones(self) -> list[dict]:
        return [dict(z, pts=[list(p) for p in z["pts"]]) for z in self._zones]

    def set_zones(self, zones: list[dict]):
        self._zones = [dict(z) for z in zones]
        self.update()

    def keyPressEvent(self, ev):
        if ev.key() == Qt.Key.Key_Escape and self._mode == MODE_ZONE:
            self._zone_draft = []
            self.set_mode(MODE_GOAL)
            return
        super().keyPressEvent(ev)

    def mouseDoubleClickEvent(self, ev):
        if self._mode == MODE_ZONE and ev.button() == Qt.MouseButton.LeftButton:
            if len(self._zone_draft) >= 3:
                self._zones.append({"kind": self._zone_kind, "pts": [list(p) for p in self._zone_draft]})
                self.zones_changed.emit()
            self._zone_draft = []
            self.set_mode(MODE_GOAL)
            return
        super().mouseDoubleClickEvent(ev)

    def _draw_zones(self, painter: QPainter):
        painter.save()
        for z in self._zones:
            keep = z["kind"] == "keepout"
            col = QColor(248, 81, 73) if keep else QColor(240, 136, 62)
            poly = QPolygon([QPoint(int(x), int(y)) for x, y in (self._w2px(px, py) for px, py in z["pts"])])
            painter.setPen(QPen(col, max(1.5, 2 / self._zoom)))
            fill = QColor(col); fill.setAlpha(70)
            painter.setBrush(QBrush(fill))
            painter.drawPolygon(poly)
        if self._zone_draft:
            col = QColor(248, 81, 73) if self._zone_kind == "keepout" else QColor(240, 136, 62)
            pts = [QPointF(*self._w2px(x, y)) for x, y in self._zone_draft]
            painter.setPen(QPen(col, max(1.5, 2 / self._zoom), Qt.PenStyle.DashLine))
            for a, b in zip(pts, pts[1:]):
                painter.drawLine(a, b)
            painter.setBrush(QBrush(col)); painter.setPen(Qt.PenStyle.NoPen)
            for pt in pts:
                painter.drawEllipse(pt, max(2.0, 4 / self._zoom), max(2.0, 4 / self._zoom))
        painter.restore()

    # ── View options ────────────────────────────────────────────────

    def set_follow(self, on: bool):
        self._follow = bool(on)
        if on:
            self._center_on_robot()
        self.update()

    def set_show_grid(self, on: bool):
        self._show_grid = bool(on)
        self.update()

    def set_show_minimap(self, on: bool):
        self._show_minimap = bool(on)
        self.update()

    def get_grid(self):
        """(grid int16 [h,w] row0=lowest y, resolution, origin_x, origin_y) or None."""
        if self._grid is None:
            return None
        return self._grid.copy(), self._resolution, self._origin_x, self._origin_y

    def set_grid(self, grid: np.ndarray, resolution: float, origin_x: float,
                 origin_y: float, segments=None, wall_thickness_m: float = 0.10,
                 reset_view: bool = False):
        """Show an occupancy grid (optionally with vector wall segments on top)."""
        h, w = grid.shape
        first = self._pixmap is None
        self._grid = grid.astype(np.int16)
        self._segments = list(segments or [])
        self._wall_thickness_m = wall_thickness_m
        self._resolution, self._origin_x, self._origin_y = float(resolution), float(origin_x), float(origin_y)
        self._map_w, self._map_h = w, h
        self._render_grid()
        if reset_view or first:
            self._reset_view()
        self.update()

    def _render_grid(self):
        data = self._grid
        h, w = data.shape
        img = np.empty((h, w, 3), dtype=np.uint8)
        img[:] = C_MAP_UNKNOWN
        img[data == 0] = C_MAP_FREE
        occ = data > 50
        if self._segments:
            from agv_hmi.core.map_cleaner import rasterize_segments
            covered = rasterize_segments(
                data.shape, self._segments,
                self._wall_thickness_m / self._resolution + 2.0)
            img[occ & covered] = C_MAP_FREE      # vector walls are drawn on top
            img[occ & ~covered] = C_MAP_OCC
        else:
            img[occ] = C_MAP_OCC
        img = np.ascontiguousarray(np.flipud(img))
        qimg = QImage(img.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()
        self._pixmap = QPixmap.fromImage(qimg)

    def update_nav_path(self, pts: list[tuple[float, float]]):
        """Cập nhật đường path Nav2 sẽ đi (world coordinates)."""
        self._nav_path = pts
        self.update()

    def clear_nav_path(self):
        """Xoá đường path Nav2."""
        self._nav_path = []
        self.update()

    def update_map(self, msg, reset_view: bool = False):
        first_map = self._pixmap is None

        w = int(msg.info.width)
        h = int(msg.info.height)
        if w <= 0 or h <= 0:
            return

        self._origin_x   = float(msg.info.origin.position.x)
        self._origin_y   = float(msg.info.origin.position.y)
        self._resolution = float(msg.info.resolution)
        self._map_w      = w
        self._map_h      = h

        data = np.array(msg.data, dtype=np.int16).reshape((h, w))
        self._segments = []
        self.set_grid(data, self._resolution, self._origin_x, self._origin_y,
                      reset_view=(reset_view or first_map))

    def load_map_file(self, yaml_path: str):
        with open(yaml_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        pgm = meta.get("image", "")
        if not os.path.isabs(pgm):
            pgm = os.path.join(os.path.dirname(yaml_path), pgm)

        raw = QImage(pgm).convertToFormat(QImage.Format.Format_Grayscale8)
        if raw.isNull():
            print(f"[MapWidget] Không đọc được: {pgm}")
            return

        self._resolution = float(meta.get("resolution", 0.05))
        origin           = meta.get("origin", [0.0, 0.0, 0.0])
        self._origin_x   = float(origin[0])
        self._origin_y   = float(origin[1])

        negate    = int(meta.get("negate", 0))
        w, h      = raw.width(), raw.height()

        ptr = raw.constBits()
        ptr.setsize(raw.sizeInBytes())
        arr = np.frombuffer(ptr, dtype=np.uint8).reshape(
            (h, raw.bytesPerLine()))[:, :w].copy()
        if negate:
            arr = 255 - arr

        # PGM -> occupancy (-1 / 0 / 100). The file is stored top-down, the grid
        # has row 0 at the lowest y, so flip once here.
        occ_t = float(meta.get("occupied_thresh", 0.65))
        free_t = float(meta.get("free_thresh", 0.196))
        p_occ = (255 - arr.astype(np.float32)) / 255.0
        grid = np.full((h, w), -1, dtype=np.int16)
        grid[p_occ < free_t] = 0
        grid[p_occ > occ_t] = 100
        grid = np.ascontiguousarray(np.flipud(grid))

        from agv_hmi.core.map_io import load_walls
        from agv_hmi.core.zones import load_zones
        segs, thick = load_walls(yaml_path)
        self._zones = load_zones(os.path.splitext(yaml_path)[0])
        self._measure_pts = []
        self.set_grid(grid, self._resolution, self._origin_x, self._origin_y,
                      segments=segs, wall_thickness_m=thick, reset_view=True)

    def clear_map(self):
        self._pixmap     = None
        self._grid       = None
        self._segments   = []
        self._zones      = []
        self._zone_draft = []
        self._measure_pts = []
        self._map_w      = 0
        self._map_h      = 0
        self._scan_pts   = []
        self._waypoints  = []
        self._pose_est   = None
        self._nav_path   = []    # FIX 4: xoá path khi load map mới
        self._zoom       = 1.0
        self._pan_x      = 0.0
        self._pan_y      = 0.0
        self.update()

    def update_pose(self, x: float, y: float, yaw: float):
        self.robot_x   = float(x)
        self.robot_y   = float(y)
        self.robot_yaw = float(yaw)
        if self._follow and self._drag_start is None:
            self._center_on_robot()
        self.update()

    def _center_on_robot(self):
        if self._map_w <= 0:
            return
        px, py = self._w2px(self.robot_x, self.robot_y)
        s = self._zoom
        self._pan_x = self.width() / 2 - px * s
        self._pan_y = self.height() / 2 - py * s

    def update_scan(self, scan_pts: list[tuple[float, float]]):
        self._scan_pts = scan_pts
        self.update()

    def set_waypoints(self, wps: list[tuple[float, float, str]]):
        self._waypoints = wps
        self.update()

    def clear_waypoints(self):
        self._waypoints = []
        self.update()

    def clear_pose_estimate(self):
        self._pose_est = None
        self.update()

    # ── Mouse events ────────────────────────────────────────────────

    def mousePressEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton:
            wx, wy = self._widget_to_world(ev.pos().x(), ev.pos().y())
            if self._mode == MODE_ZONE:
                self._zone_draft.append((wx, wy))
                self.update()
            elif self._mode == MODE_MEASURE:
                if len(self._measure_pts) >= 2:
                    self._measure_pts = []
                self._measure_pts.append((wx, wy))
                self.update()
            elif self._mode == MODE_POSE:
                self._pose_drag_origin = (wx, wy)
                self._pose_dragging    = True
                self._pose_est         = (wx, wy, 0.0)
            else:
                self.goal_selected.emit(wx, wy)

        elif ev.button() == Qt.MouseButton.RightButton:
            self._drag_start = ev.pos()
            self._pan_start  = (self._pan_x, self._pan_y)
            self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, ev):
        if self._drag_start is not None:
            dx = ev.pos().x() - self._drag_start.x()
            dy = ev.pos().y() - self._drag_start.y()
            self._pan_x = self._pan_start[0] + dx
            self._pan_y = self._pan_start[1] + dy
            self.update()

        elif self._pose_dragging and self._pose_drag_origin:
            ox, oy = self._pose_drag_origin
            cx, cy = self._widget_to_world(ev.pos().x(), ev.pos().y())
            yaw    = math.atan2(cy - oy, cx - ox)
            self._pose_est = (ox, oy, yaw)
            self.update()

    def mouseReleaseEvent(self, ev):
        if ev.button() == Qt.MouseButton.RightButton:
            self._drag_start = None
            self.setCursor(
                Qt.CursorShape.CrossCursor if self._mode == MODE_POSE
                else Qt.CursorShape.ArrowCursor
            )
        elif ev.button() == Qt.MouseButton.LeftButton and self._pose_dragging:
            self._pose_dragging = False
            if self._pose_est:
                x, y, yaw = self._pose_est
                self.pose_estimate_set.emit(x, y, yaw)

    def wheelEvent(self, ev):
        factor = 1.15 if ev.angleDelta().y() > 0 else 1 / 1.15
        mx, my = ev.position().x(), ev.position().y()
        self._pan_x = mx + (self._pan_x - mx) * factor
        self._pan_y = my + (self._pan_y - my) * factor
        self._zoom  = max(0.1, min(self._zoom * factor, 30.0))
        self.update()

    def resizeEvent(self, ev):
        if self._pixmap is not None:
            self._reset_view()

    # ── Paint ───────────────────────────────────────────────────────

    def paintEvent(self, ev):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)
        painter.fillRect(self.rect(), C_BG)

        if self._pixmap is None or self._pixmap.isNull():
            painter.setPen(QPen(QColor(139, 148, 158)))
            fnt = QFont()
            fnt.setPixelSize(14)
            painter.setFont(fnt)
            painter.drawText(
                self.rect(),
                Qt.AlignmentFlag.AlignCenter,
                "Chờ map...\n(SLAM đang chạy hoặc load file)",
            )
            return

        bs = self._base_scale()
        pw = int(self._map_w * bs)
        ph = int(self._map_h * bs)

        painter.save()
        painter.translate(self._pan_x, self._pan_y)
        painter.scale(self._zoom, self._zoom)

        painter.drawPixmap(0, 0, pw, ph, self._pixmap)

        self._draw_grid_lines(painter, bs)
        self._draw_wall_vectors(painter, bs)
        self._draw_zones(painter)
        self._draw_scan(painter)
        self._draw_nav_path(painter)      # vẽ path trước waypoints để WP nằm trên
        self._draw_waypoints(painter)
        self._draw_pose_estimate(painter)
        self._draw_robot(painter, bs)

        self._draw_measure(painter)
        painter.restore()

        self._draw_scale_bar(painter, bs)
        if self._show_minimap:
            self._draw_minimap(painter)
        if self._mode == MODE_POSE:
            self._draw_pose_mode_banner(painter)

    # ── Draw helpers ────────────────────────────────────────────────

    def _draw_wall_vectors(self, painter: QPainter, bs: float):
        if not self._segments:
            return
        pen = QPen(C_WALL, max(1.0, self._wall_thickness_m / self._resolution * bs))
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        painter.save()
        painter.setPen(pen)
        h = self._map_h
        for x0, y0, x1, y1 in self._segments:
            painter.drawLine(QLineF((x0 + 0.5) * bs, (h - (y0 + 0.5)) * bs,
                                    (x1 + 0.5) * bs, (h - (y1 + 0.5)) * bs))
        painter.restore()

    def _draw_grid_lines(self, painter: QPainter, bs: float):
        if not self._show_grid or self._map_w <= 0:
            return
        step_px = bs / self._resolution            # 1 m in scene px
        if step_px * self._zoom < 14:              # too dense to be useful
            return
        painter.save()
        ox, oy = self._origin_x, self._origin_y
        wx0 = math.ceil(ox)
        wx1 = math.floor(ox + self._map_w * self._resolution)
        wy0 = math.ceil(oy)
        wy1 = math.floor(oy + self._map_h * self._resolution)
        top, bottom = 0.0, self._map_h * bs
        left, right = 0.0, self._map_w * bs
        for wx in range(wx0, wx1 + 1):
            painter.setPen(QPen(C_GRID_MAJOR if wx % 5 == 0 else C_GRID_MINOR, max(0.5, 1 / self._zoom)))
            px, _ = self._w2px(wx, oy)
            painter.drawLine(QLineF(px, top, px, bottom))
        for wy in range(wy0, wy1 + 1):
            painter.setPen(QPen(C_GRID_MAJOR if wy % 5 == 0 else C_GRID_MINOR, max(0.5, 1 / self._zoom)))
            _, py = self._w2px(ox, wy)
            painter.drawLine(QLineF(left, py, right, py))
        painter.restore()

    def _draw_measure(self, painter: QPainter):
        pts = self._measure_pts
        if not pts:
            return
        painter.save()
        px = [self._w2px(x, y) for x, y in pts]
        pen = QPen(C_MEASURE, max(1.5, 2.5 / self._zoom))
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(pen)
        painter.setBrush(QBrush(C_MEASURE))
        r = max(2.0, 4.0 / self._zoom)
        for x, y in px:
            painter.drawEllipse(QPointF(x, y), r, r)
        if len(px) == 2:
            painter.drawLine(QPointF(*px[0]), QPointF(*px[1]))
            d = math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
            f = QFont()
            f.setPixelSize(max(8, int(14 / self._zoom)))
            f.setBold(True)
            painter.setFont(f)
            mx, my = (px[0][0] + px[1][0]) / 2, (px[0][1] + px[1][1]) / 2
            painter.drawText(QPointF(mx + r * 2, my - r * 2), f"{d:.2f} m")
        painter.restore()

    def _draw_scale_bar(self, painter: QPainter, bs: float):
        if self._map_w <= 0:
            return
        px_per_m = bs / self._resolution * self._zoom
        meters = 1.0
        for cand in (0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50):
            meters = cand
            if px_per_m * cand >= 70:
                break
        length = px_per_m * meters
        x0, y0 = 16, self.height() - 22
        painter.save()
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(0, 0, 0, 130))
        painter.drawRoundedRect(QRectF(x0 - 8, y0 - 18, length + 16 + 38, 30), 6, 6)
        pen = QPen(QColor(255, 255, 255, 230), 2)
        painter.setPen(pen)
        painter.drawLine(QLineF(x0, y0, x0 + length, y0))
        painter.drawLine(QLineF(x0, y0 - 5, x0, y0 + 5))
        painter.drawLine(QLineF(x0 + length, y0 - 5, x0 + length, y0 + 5))
        f = QFont()
        f.setPixelSize(12)
        painter.setFont(f)
        label = f"{meters:g} m"
        painter.drawText(QPointF(x0 + length + 8, y0 + 4), label)
        painter.restore()

    def _draw_minimap(self, painter: QPainter):
        if self._pixmap is None or self._zoom < 1.6:
            return
        mw = 150
        mh = max(40, int(mw * self._map_h / max(1, self._map_w)))
        mh = min(mh, 150)
        x0, y0 = self.width() - mw - 12, 12
        painter.save()
        painter.setOpacity(0.92)
        painter.fillRect(x0 - 3, y0 - 3, mw + 6, mh + 6, QColor(13, 17, 23, 220))
        painter.drawPixmap(x0, y0, mw, mh, self._pixmap)
        bs = self._base_scale() * self._zoom
        # visible rectangle in map-pixel coordinates
        vx0 = -self._pan_x / bs
        vy0 = -self._pan_y / bs
        vx1 = (self.width() - self._pan_x) / bs
        vy1 = (self.height() - self._pan_y) / bs
        sx, sy = mw / self._map_w, mh / self._map_h
        painter.setPen(QPen(QColor(88, 166, 255), 1.5))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(QRectF(x0 + max(0, vx0) * sx, y0 + max(0, vy0) * sy,
                                (min(self._map_w, vx1) - max(0, vx0)) * sx,
                                (min(self._map_h, vy1) - max(0, vy0)) * sy))
        rx, ry = self._w2px(self.robot_x, self.robot_y)
        b0 = self._base_scale()
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(QColor(248, 81, 73)))
        painter.drawEllipse(QPointF(x0 + rx / b0 * sx, y0 + ry / b0 * sy), 3, 3)
        painter.restore()

    def _draw_scan(self, painter: QPainter):
        if not self._scan_pts:
            return
        painter.save()
        pen = QPen(C_SCAN, max(1.5, 2 / self._zoom))
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(pen)
        dot_r = max(1, 2 / self._zoom)
        for sx, sy in self._scan_pts:
            px, py = self._w2px(sx, sy)
            painter.drawEllipse(QPointF(px, py), dot_r, dot_r)
        painter.restore()

    def _draw_nav_path(self, painter: QPainter):
        """
        Vẽ đường Nav2 path giữa các waypoints.
        FIX 5: thêm painter.save/restore để tránh ảnh hưởng painter
                state sang các hàm vẽ phía sau.
        """
        if len(self._nav_path) < 2:
            return

        painter.save()   # FIX 5

        # Đường liền màu xanh lá mờ
        pen = QPen(C_NAV_PATH, max(2, 3 / self._zoom))
        pen.setStyle(Qt.PenStyle.SolidLine)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        # FIX 1: QPainterPath đã được import ở đầu file
        path = QPainterPath()
        x0, y0 = self._w2px(*self._nav_path[0])
        path.moveTo(x0, y0)
        for wx, wy in self._nav_path[1:]:
            px, py = self._w2px(wx, wy)
            path.lineTo(px, py)
        painter.drawPath(path)

        # Chấm nhỏ tại mỗi điểm (mỗi 5 điểm 1 chấm để không quá dày)
        dot_r = max(1.5, 2.5 / self._zoom)
        painter.setBrush(QBrush(QColor(63, 185, 80, 120)))
        painter.setPen(Qt.PenStyle.NoPen)
        for wx, wy in self._nav_path[::5]:
            px, py = self._w2px(wx, wy)
            painter.drawEllipse(QPointF(px, py), dot_r, dot_r)

        painter.restore()  # FIX 5

    def _draw_waypoints(self, painter: QPainter):
        painter.save()
        for idx, (wx, wy, label) in enumerate(self._waypoints):
            px, py = self._w2px(wx, wy)
            r = max(7, int(10 / self._zoom))

            color = (C_GOAL_END  if label == "E"
                     else C_GOAL_DONE if idx == 0
                     else C_GOAL_CURR)
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(QColor(255, 255, 255, 200),
                                max(1, 1.5 / self._zoom)))
            painter.drawEllipse(QPointF(px, py), r, r)

            fnt = QFont()
            fnt.setPixelSize(max(7, int(9 / self._zoom)))
            fnt.setWeight(QFont.Weight.Bold)
            painter.setFont(fnt)
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(
                int(px - r), int(py - r), r * 2, r * 2,
                Qt.AlignmentFlag.AlignCenter,
                label,
            )
        painter.restore()

    def _draw_pose_estimate(self, painter: QPainter):
        if self._pose_est is None:
            return

        painter.save()
        ex, ey, eyaw = self._pose_est
        epx, epy = self._w2px(ex, ey)

        r_unc = max(20, int(28 / self._zoom))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(QColor(50, 220, 120, 80),
                            max(1, 1.5 / self._zoom),
                            Qt.PenStyle.DashLine))
        painter.drawEllipse(QPointF(epx, epy), r_unc, r_unc)

        r_dot = max(5, int(7 / self._zoom))
        painter.setBrush(QBrush(C_POSE_EST))
        painter.setPen(QPen(QColor(0, 0, 0, 160), max(1, 1.5 / self._zoom)))
        painter.drawEllipse(QPointF(epx, epy), r_dot, r_dot)

        arrow_len = r_unc + max(12, int(18 / self._zoom))
        ax = epx + arrow_len * math.cos(eyaw)
        ay = epy - arrow_len * math.sin(eyaw)

        pen_arrow = QPen(C_POSE_EST, max(3, int(4 / self._zoom)))
        pen_arrow.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(pen_arrow)
        painter.drawLine(QPointF(epx, epy), QPointF(ax, ay))
        painter.restore()

    def _draw_robot(self, painter: QPainter, bs: float):
        rx, ry = self._w2px(self.robot_x, self.robot_y)
        ppm    = bs / self._resolution
        rl     = self._robot_len * ppm
        rw     = self._robot_wid * ppm
        cxo    = self._robot_cx * ppm

        painter.save()
        painter.translate(rx, ry)
        painter.rotate(-math.degrees(self.robot_yaw))

        # ── Thân chính — bo góc ──────────────────────────────────────
        # FIX 2: QRectF đã import ở đầu file, không cần import inline
        if self._safety:
            from agv_hmi.core import robot_profile as _RP
            for kind, col in (("slow", QColor(240, 136, 62)), ("stop", QColor(248, 81, 73))):
                x0, x1, y0, y1 = _RP.zone_rect(self._safety, kind)
                painter.setPen(QPen(col, max(1.0, 1.5 / self._zoom), Qt.PenStyle.DashLine))
                fill = QColor(col); fill.setAlpha(30)
                painter.setBrush(QBrush(fill))
                # painter y points to the robot's right: base-frame y (left +) is flipped
                painter.drawRect(QRectF(x0 * ppm, -y1 * ppm, (x1 - x0) * ppm, (y1 - y0) * ppm))
        body = QRectF(cxo - rl / 2, -rw / 2, rl, rw)
        painter.setBrush(QBrush(QColor(55, 138, 221, 210)))
        painter.setPen(QPen(QColor(255, 255, 255, 200),
                            max(1, 1.5 / self._zoom)))
        painter.drawRoundedRect(body, rw * 0.18, rw * 0.18)

        # ── 4 bánh xe ────────────────────────────────────────────────
        wheel_w     = max(2, rw * 0.18)
        wheel_h     = max(3, rl * 0.22)
        wheel_color = QColor(30, 30, 40, 220)
        painter.setBrush(QBrush(wheel_color))
        painter.setPen(Qt.PenStyle.NoPen)
        for wx_off, wy_off in [
            (cxo + rl * 0.30,  rw * 0.50),
            (cxo + rl * 0.30, -rw * 0.50),
            (cxo - rl * 0.30,  rw * 0.50),
            (cxo - rl * 0.30, -rw * 0.50),
        ]:
            painter.drawRoundedRect(
                QRectF(wx_off - wheel_h / 2,
                       wy_off - wheel_w / 2,
                       wheel_h, wheel_w),
                2, 2,
            )

        # ── Mũi tên hướng ────────────────────────────────────────────
        arrow_len = int(cxo + rl / 2 + max(8, 12 / self._zoom))
        pen_arrow = QPen(QColor(248, 81, 73), max(2, 3 / self._zoom))
        pen_arrow.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(pen_arrow)
        painter.drawLine(QPointF(0, 0), QPointF(arrow_len, 0))

        head = max(4, int(6 / self._zoom))
        tip  = arrow_len
        pts  = [QPoint(tip, 0),
                QPoint(tip - head, -head // 2),
                QPoint(tip - head,  head // 2)]
        painter.setBrush(QBrush(QColor(248, 81, 73)))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawPolygon(QPolygon(pts))

        painter.restore()

    def _draw_pose_mode_banner(self, painter: QPainter):
        banner_h = 32
        painter.fillRect(0, 0, self.width(), banner_h, QColor(40, 20, 0, 200))
        painter.setPen(QPen(C_POSE_EST, 2))
        painter.drawLine(0, banner_h - 1, self.width(), banner_h - 1)

        fnt = QFont()
        fnt.setPixelSize(13)
        fnt.setWeight(QFont.Weight.Medium)
        painter.setFont(fnt)
        painter.setPen(QPen(C_POSE_EST))
        painter.drawText(
            0, 0, self.width(), banner_h,
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
            "  ✛  Chế độ 2D Pose Estimate — Click vị trí robot, kéo để set hướng",
        )

    # ── Coordinate helpers ──────────────────────────────────────────

    def _base_scale(self) -> float:
        if self._map_w <= 0 or self._map_h <= 0:
            return 1.0
        return min(
            self.width()  / self._map_w,
            self.height() / self._map_h,
        )

    def _reset_view(self):
        if self._map_w <= 0 or self._map_h <= 0:
            return
        s = self._base_scale()
        self._zoom  = 1.0
        self._pan_x = (self.width()  - self._map_w * s) / 2
        self._pan_y = (self.height() - self._map_h * s) / 2

    def _w2px(self, wx: float, wy: float) -> tuple[float, float]:
        bs = self._base_scale()
        px = (wx - self._origin_x) / self._resolution * bs
        py = (self._map_h - (wy - self._origin_y) / self._resolution) * bs
        return px, py

    def _widget_to_world(self, sx: float, sy: float) -> tuple[float, float]:
        s = self._base_scale() * self._zoom
        if s <= 0:
            return 0.0, 0.0
        mx = (sx - self._pan_x) / s
        my = (sy - self._pan_y) / s
        wx = mx * self._resolution + self._origin_x
        wy = (self._map_h - my) * self._resolution + self._origin_y
        return wx, wy