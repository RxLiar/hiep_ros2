"""
joystick_widget.py — Virtual joystick overlay
- Vẽ lên trên map widget (góc trái dưới)
- Mouse press/move/release → tính linear.x và angular.z
- Emit velocity_signal(linear, angular) → gửi /cmd_vel
- Không tự publish — emit signal để parent wiring
"""
import math
from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QRadialGradient
from PyQt6.QtCore import Qt, QPointF, QTimer, pyqtSignal


class JoystickWidget(QWidget):
    """
    Virtual joystick tròn.
    Kích thước cố định 150×150 px.
    Đặt overlapping góc trái dưới MapWidget bằng QStackedLayout hoặc
    absolute position trong parent.
    """
    velocity_signal = pyqtSignal(float, float)   # linear, angular

    RADIUS   = 60    # bán kính vùng joystick (px)
    KNOB_R   = 22    # bán kính núm
    MAX_LIN  = 1.0   # m/s tối đa (nhân với max_linear từ velocity panel)
    MAX_ANG  = 1.5   # rad/s tối đa

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(150, 150)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
        self.setMouseTracking(True)

        self._center  = QPointF(75, 75)
        self._knob    = QPointF(75, 75)   # vị trí hiện tại của núm
        self._pressed = False

        # Publish timer: 20Hz khi đang nhấn
        self._timer = QTimer()
        self._timer.setInterval(50)
        self._timer.timeout.connect(self._emit_velocity)

        # Multiplier từ velocity panel (1.0 mặc định)
        self._max_linear  = 1.0
        self._max_angular = 1.5

    # ── Public ──────────────────────────────────────────────────────

    def set_max_velocity(self, linear: float, angular: float):
        """Gọi mỗi khi user thay đổi velocity setup panel."""
        self._max_linear  = max(0.01, linear)
        self._max_angular = max(0.01, angular)

    # ── Mouse events ────────────────────────────────────────────────

    def mousePressEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton:
            self._pressed = True
            self._update_knob(ev.position())
            self._timer.start()

    def mouseMoveEvent(self, ev):
        if self._pressed:
            self._update_knob(ev.position())

    def mouseReleaseEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton:
            self._pressed = False
            self._knob = QPointF(self._center)
            self._timer.stop()
            self.velocity_signal.emit(0.0, 0.0)   # Stop
            self.update()

    def _update_knob(self, pos: QPointF):
        dx = pos.x() - self._center.x()
        dy = pos.y() - self._center.y()
        dist = math.hypot(dx, dy)
        if dist > self.RADIUS:
            dx = dx / dist * self.RADIUS
            dy = dy / dist * self.RADIUS
        self._knob = QPointF(self._center.x() + dx,
                             self._center.y() + dy)
        self.update()

    # ── Emit velocity ────────────────────────────────────────────────

    def _emit_velocity(self):
        dx = self._knob.x() - self._center.x()   # +right = angular -
        dy = self._knob.y() - self._center.y()   # +down  = backward

        linear  =  (-dy / self.RADIUS) * self._max_linear
        angular =  (-dx / self.RADIUS) * self._max_angular

        # Clamp
        linear  = max(-self._max_linear,  min(self._max_linear,  linear))
        angular = max(-self._max_angular, min(self._max_angular, angular))

        self.velocity_signal.emit(linear, angular)

    # ── Paint ────────────────────────────────────────────────────────

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        cx, cy = self._center.x(), self._center.y()
        R = self.RADIUS

        # Background circle (semi-transparent)
        p.setBrush(QBrush(QColor(13, 17, 23, 180)))
        p.setPen(QPen(QColor(88, 166, 255, 120), 1.5))
        p.drawEllipse(QPointF(cx, cy), R + 8, R + 8)

        # Cross hair
        p.setPen(QPen(QColor(88, 166, 255, 60), 1))
        p.drawLine(QPointF(cx - R, cy), QPointF(cx + R, cy))
        p.drawLine(QPointF(cx, cy - R), QPointF(cx, cy + R))

        # Direction rings
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor(88, 166, 255, 40), 1))
        p.drawEllipse(QPointF(cx, cy), R * 0.5, R * 0.5)

        # Knob
        kx, ky = self._knob.x(), self._knob.y()
        grad = QRadialGradient(kx - 4, ky - 4, self.KNOB_R * 2)
        if self._pressed:
            grad.setColorAt(0, QColor(88, 166, 255, 230))
            grad.setColorAt(1, QColor(17, 88, 199, 200))
        else:
            grad.setColorAt(0, QColor(60, 80, 110, 200))
            grad.setColorAt(1, QColor(30, 40, 60, 180))

        p.setBrush(QBrush(grad))
        p.setPen(QPen(QColor(88, 166, 255, 180), 1.5))
        p.drawEllipse(QPointF(kx, ky), self.KNOB_R, self.KNOB_R)

        # Arrow indicator (hướng đang nhấn)
        if self._pressed:
            dx = kx - cx; dy = ky - cy
            dist = math.hypot(dx, dy)
            if dist > 5:
                p.setPen(QPen(QColor(255, 255, 255, 150), 1.5))
                p.drawLine(QPointF(cx, cy), QPointF(kx, ky))