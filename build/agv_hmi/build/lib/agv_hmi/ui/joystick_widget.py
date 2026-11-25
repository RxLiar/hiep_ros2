"""
joystick_widget.py — Virtual joystick overlay
- Vẽ lên trên map widget (góc trái dưới)
- Mouse press/move/release → tính linear.x và angular.z
- Emit velocity_signal(linear, angular) → gửi /cmd_vel
- Không tự publish — emit signal để parent wiring
"""
import math
import weakref

from PyQt6.QtWidgets import QWidget, QToolTip, QPushButton, QMessageBox
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QRadialGradient, QRegion
from PyQt6.QtCore import Qt, QPointF, QRect, QTimer, QVariantAnimation, QEvent, pyqtSignal

from agv_hmi.core import prefs
from agv_hmi.ui import anim
from agv_hmi.ui.i18n import tr
from agv_hmi.ui.safety_override import OVERRIDE

_INSTANCES: "weakref.WeakSet" = weakref.WeakSet()


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

    BTN_H = 26       # height of the Engineer "safety off" button above the stick

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

        # 4.7.0: the joystick can be folded into a small chip (bottom-left) to free the map
        self._chip_c = QPointF(17, 133)
        self._chip_r = 13.0
        self._t = 1.0                     # 1 = joystick shown, 0 = folded into the chip
        self._collapsed = False
        self._t_anim = QVariantAnimation(self)
        self._t_anim.setEasingCurve(anim.EASE_INOUT)
        self._t_anim.valueChanged.connect(self._on_t)
        self._t_anim.finished.connect(self._after_t)
        _INSTANCES.add(self)

        # 4.8.0: Engineer only - switch the safety zones off while teleoperating (shared by all joysticks)
        self._engineer = False
        self._top = 0
        self._btn = QPushButton(self)
        self._btn.setCheckable(True)
        self._btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._btn.setGeometry(0, 0, 150, self.BTN_H)
        self._btn.hide()
        self._btn.toggled.connect(self._on_override_toggled)
        OVERRIDE.changed.connect(self._sync_override)
        self._style_btn(False)

        if prefs.get("ui_joystick_collapsed"):
            self._apply_collapsed(True, animate=False)

    # ── Public ──────────────────────────────────────────────────────

    def set_max_velocity(self, linear: float, angular: float):
        """Gọi mỗi khi user thay đổi velocity setup panel."""
        self._max_linear  = max(0.01, linear)
        self._max_angular = max(0.01, angular)

    def set_engineer(self, engineer: bool):
        """Show the 'safety off' button (Engineer role only) and make room for it above the stick."""
        engineer = bool(engineer)
        if engineer == self._engineer:
            return
        self._engineer = engineer
        self._top = (self.BTN_H + 4) if engineer else 0
        self.setFixedSize(150, 150 + self._top)
        self._center = QPointF(75, 75 + self._top)
        self._knob = QPointF(self._center)
        self._chip_c = QPointF(17, 133 + self._top)
        self._btn.setVisible(engineer)
        if not engineer:
            OVERRIDE.set(False)
        self._after_t()
        self.update()

    def _style_btn(self, on: bool):
        self._btn.setText(tr("joy_safety_off") if on else tr("joy_safety_on"))
        self._btn.setToolTip(tr("joy_safety_tip"))
        if on:
            self._btn.setStyleSheet("QPushButton{background:#F85149;color:white;font-weight:700;border:1px solid #FFB3AE;"
                                    "border-radius:6px;font-size:11px;}")
        else:
            self._btn.setStyleSheet("QPushButton{background:rgba(13,17,23,215);color:#3FB950;border:1px solid #3FB950;"
                                    "border-radius:6px;font-size:11px;}QPushButton:hover{background:rgba(63,185,80,60);}")

    def _on_override_toggled(self, checked: bool):
        if checked and not OVERRIDE.confirmed:
            ans = QMessageBox.warning(self.window(), tr("joy_safety_title"), tr("joy_safety_confirm"),
                                      QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                      QMessageBox.StandardButton.No)
            if ans != QMessageBox.StandardButton.Yes:
                self._btn.blockSignals(True); self._btn.setChecked(False); self._btn.blockSignals(False)
                return
            OVERRIDE.confirmed = True
        OVERRIDE.set(checked)

    def _sync_override(self, on: bool):
        """Another joystick (or MainWindow, e.g. a mission started) changed the shared switch."""
        self._btn.blockSignals(True); self._btn.setChecked(on); self._btn.blockSignals(False)
        self._style_btn(on)

    def retranslate(self):
        self._style_btn(OVERRIDE.is_on())

    def is_collapsed(self) -> bool:
        return self._collapsed

    def toggle_collapsed(self):
        """Fold / unfold every joystick (they share one preference)."""
        target = not self._collapsed
        prefs.set("ui_joystick_collapsed", target)
        for j in list(_INSTANCES):
            j._apply_collapsed(target, animate=True)

    def _apply_collapsed(self, collapsed: bool, animate: bool = True):
        if collapsed == self._collapsed and self._t_anim.state() == QVariantAnimation.State.Stopped:
            return
        self._collapsed = collapsed
        if collapsed and self._pressed:
            self._pressed = False
            self._knob = QPointF(self._center)
            self._timer.stop()
            self.velocity_signal.emit(0.0, 0.0)
        if not collapsed:
            self.clearMask()
        end = 0.0 if collapsed else 1.0
        self._t_anim.stop()
        d = anim.dur(anim.DUR_MED) if animate else 0
        if d == 0:
            self._t = end
            self._after_t()
            self.update()
        else:
            self._t_anim.setDuration(d)
            self._t_anim.setStartValue(self._t)
            self._t_anim.setEndValue(end)
            self._t_anim.start()

    def _on_t(self, v):
        self._t = float(v)
        self.update()

    def _after_t(self):
        if self._collapsed:   # only the chip (and the Engineer button) may catch the mouse, the map below stays usable
            r = int(self._chip_r) + 3
            region = QRegion(QRect(int(self._chip_c.x()) - r, int(self._chip_c.y()) - r,
                                   2 * r, 2 * r), QRegion.RegionType.Ellipse)
            if self._engineer:
                region = region.united(QRegion(QRect(0, 0, 150, self.BTN_H)))
            self.setMask(region)
        else:
            self.clearMask()
        self.update()

    def _in_chip(self, pos) -> bool:
        return math.hypot(pos.x() - self._chip_c.x(), pos.y() - self._chip_c.y()) <= self._chip_r + 3

    def event(self, ev):
        if ev.type() == QEvent.Type.ToolTip:
            if self._in_chip(ev.pos()):
                QToolTip.showText(ev.globalPos(),
                                  tr("joy_show") if self._collapsed else tr("joy_hide"), self)
            else:
                QToolTip.hideText()
            return True
        return super().event(ev)

    # ── Mouse events ────────────────────────────────────────────────

    def mousePressEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton and self._in_chip(ev.position()):
            self.toggle_collapsed()
            return
        if self._collapsed or self._t < 0.99:
            return
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

        if self._t > 0.01:
            p.save()
            p.setOpacity(self._t)
            sc = 0.25 + 0.75 * self._t
            p.translate(self._chip_c)
            p.scale(sc, sc)
            p.translate(-self._chip_c)
            self._paint_stick(p, cx, cy, R)
            p.restore()
        self._paint_chip(p)

    def _paint_chip(self, p):
        c = self._chip_c
        p.setBrush(QBrush(QColor(13, 17, 23, 215)))
        p.setPen(QPen(QColor(88, 166, 255, 170), 1.4))
        p.drawEllipse(c, self._chip_r, self._chip_r)
        pen = QPen(QColor(150, 200, 255, 230), 1.6)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        p.setPen(pen)
        if self._t > 0.5:                      # open: "minimise" dash
            p.drawLine(QPointF(c.x() - 4.5, c.y()), QPointF(c.x() + 4.5, c.y()))
        else:                                   # folded: mini joystick (ring + knob)
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(c, 6.5, 6.5)
            p.setBrush(QBrush(QColor(150, 200, 255, 230)))
            p.drawEllipse(c, 2.2, 2.2)

    def _paint_stick(self, p, cx, cy, R):

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