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
from PyQt6.QtCore import Qt, QPoint, QPointF, QRectF, pyqtSignal
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

MODE_GOAL = "goal"
MODE_POSE = "pose"


class MapWidget(QWidget):
    goal_selected    = pyqtSignal(float, float)
    pose_estimate_set = pyqtSignal(float, float, float)

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
            Qt.CursorShape.CrossCursor if mode == MODE_POSE
            else Qt.CursorShape.ArrowCursor
        )
        self.update()

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
        img  = np.zeros((h, w, 3), dtype=np.uint8)
        img[data < 0]  = [40,  44,  52]
        img[data == 0] = [200, 200, 195]
        img[data > 50] = [20,  20,  20]

        # OccupancyGrid row 0 là y thấp nhất → flip để hiển thị đúng.
        img = np.ascontiguousarray(np.flipud(img))

        qimg = QImage(img.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()
        self._pixmap = QPixmap.fromImage(qimg)

        if reset_view or first_map:
            self._reset_view()

        self.update()

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
        self._map_w = w
        self._map_h = h

        ptr = raw.constBits()
        ptr.setsize(raw.sizeInBytes())
        arr = np.frombuffer(ptr, dtype=np.uint8).reshape(
            (h, raw.bytesPerLine()))[:, :w].copy()

        if negate:
            arr = 255 - arr

        # .pgm do MapSaver lưu đã flip trước khi ghi → không flip thêm.
        rgb = np.stack([arr, arr, arr], axis=2)
        rgb = np.ascontiguousarray(rgb)

        qimg = QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()
        self._pixmap = QPixmap.fromImage(qimg)

        self._reset_view()
        self.update()

        print(f"[MapWidget] Loaded: {pgm} ({w}x{h})")

    def clear_map(self):
        self._pixmap     = None
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
        self.update()

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
            if self._mode == MODE_POSE:
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

        self._draw_scan(painter)
        self._draw_nav_path(painter)      # vẽ path trước waypoints để WP nằm trên
        self._draw_waypoints(painter)
        self._draw_pose_estimate(painter)
        self._draw_robot(painter, bs)

        painter.restore()

        if self._mode == MODE_POSE:
            self._draw_pose_mode_banner(painter)

    # ── Draw helpers ────────────────────────────────────────────────

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
        rl     = ROBOT_L * ppm
        rw     = ROBOT_W * ppm

        painter.save()
        painter.translate(rx, ry)
        painter.rotate(-math.degrees(self.robot_yaw))

        # ── Thân chính — bo góc ──────────────────────────────────────
        # FIX 2: QRectF đã import ở đầu file, không cần import inline
        body = QRectF(-rl / 2, -rw / 2, rl, rw)
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
            ( rl * 0.30,  rw * 0.50),
            ( rl * 0.30, -rw * 0.50),
            (-rl * 0.30,  rw * 0.50),
            (-rl * 0.30, -rw * 0.50),
        ]:
            painter.drawRoundedRect(
                QRectF(wx_off - wheel_h / 2,
                       wy_off - wheel_w / 2,
                       wheel_h, wheel_w),
                2, 2,
            )

        # ── Mũi tên hướng ────────────────────────────────────────────
        arrow_len = int(rl / 2 + max(8, 12 / self._zoom))
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