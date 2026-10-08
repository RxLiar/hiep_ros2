"""vehicle_diagram.py - top view of the AGV with its 8 sensors, 4 belts and 2 bumpers."""
from PyQt6.QtCore import Qt, QRectF, QPointF
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont
from PyQt6.QtWidgets import QWidget

_ON, _OFF = QColor("#3FB950"), QColor("#484F58")
_ALERT = QColor("#F85149")


class VehicleDiagram(QWidget):
    """Layout: 4 belts side by side along the length of the AGV; each belt has a
    front and a rear end sensor (ids 2k-1, 2k). Bumpers sit on the two long sides."""

    def __init__(self):
        super().__init__()
        self._sensors = [False] * 8
        self._cargo = [False] * 4
        self._bumper = {"left": False, "right": False}
        self.setMinimumSize(320, 130)

    def set_sensor(self, sensor_id: int, on: bool):
        if 1 <= sensor_id <= 8:
            self._sensors[sensor_id - 1] = bool(on); self.update()

    def set_cargo(self, belt_id: int, has: bool):
        if 1 <= belt_id <= 4:
            self._cargo[belt_id - 1] = bool(has); self.update()

    def set_bumper(self, side: str, triggered: bool):
        if side in self._bumper:
            self._bumper[side] = bool(triggered); self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        body = QRectF(w * 0.06, h * 0.22, w * 0.88, h * 0.56)
        p.setPen(QPen(QColor("#30363D"), 2)); p.setBrush(QBrush(QColor("#161B22")))
        p.drawRoundedRect(body, 14, 14)
        # direction arrow (front = right)
        p.setPen(QPen(QColor("#58A6FF"), 2))
        p.drawLine(QPointF(w * 0.94, h * 0.5), QPointF(w * 0.985, h * 0.5))
        p.drawLine(QPointF(w * 0.985, h * 0.5), QPointF(w * 0.965, h * 0.44))
        p.drawLine(QPointF(w * 0.985, h * 0.5), QPointF(w * 0.965, h * 0.56))
        bw = body.width() / 4
        f = QFont(); f.setPixelSize(11); p.setFont(f)
        for k in range(4):
            belt = QRectF(body.left() + k * bw + 6, body.top() + 14, bw - 12, body.height() - 28)
            p.setPen(QPen(QColor("#30363D"), 1))
            p.setBrush(QBrush(QColor("#1F6F3F") if self._cargo[k] else QColor("#0D1117")))
            p.drawRoundedRect(belt, 6, 6)
            p.setPen(QColor("#C9D1D9"))
            p.drawText(belt, Qt.AlignmentFlag.AlignCenter, f"B{k + 1}")
            for j, x in enumerate((belt.left() + 8, belt.right() - 8)):
                on = self._sensors[2 * k + j]
                p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(_ON if on else _OFF))
                p.drawEllipse(QPointF(x, belt.top() + 10), 5, 5)
        for side, y in (("left", body.top() - 7), ("right", body.bottom() + 1)):
            trig = self._bumper[side]
            p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(_ALERT if trig else QColor("#30363D")))
            p.drawRoundedRect(QRectF(body.left() + 20, y, body.width() - 40, 6), 3, 3)
        p.end()
