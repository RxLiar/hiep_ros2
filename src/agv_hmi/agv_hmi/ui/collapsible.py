"""collapsible.py - toolbars / panels that hide with a slide and come back on click (4.7.0).

* ``CollapsiblePanel``  - wraps a side panel; a slim handle on its inner edge hides / shows it.
* ``attach_group_collapse`` / ``make_groups_collapsible`` - click a group-box title to fold it.
* ``collapse_panels``   - collapse / restore every panel inside a page (focus mode).

Collapsed state is remembered in ``~/.agv_hmi/hmi_prefs.json`` (keys ``ui_panels`` / ``ui_groups``).
"""
from __future__ import annotations

from PyQt6.QtCore import QEvent, QObject, QPointF, QRectF, Qt, QTimer, QVariantAnimation
from PyQt6.QtGui import QPainter, QPen
from PyQt6.QtWidgets import QAbstractButton, QGroupBox, QHBoxLayout, QSizePolicy, QWidget

from agv_hmi.core import prefs
from agv_hmi.ui import anim
from agv_hmi.ui.i18n import tr

_BIG = 16777215


def _saved(pref_key: str, key: str) -> bool:
    try:
        return bool((prefs.get(pref_key) or {}).get(key, False))
    except Exception:
        return False


def _save(pref_key: str, key: str, value: bool) -> None:
    if not key:
        return
    try:
        data = dict(prefs.get(pref_key) or {})
        data[key] = bool(value)
        prefs.set(pref_key, data)
    except Exception:
        pass


# ── Edge handle ───────────────────────────────────────────────────────

class _EdgeHandle(QAbstractButton):
    """Thin strip with a chevron pill. Always visible, even when the panel is hidden."""

    WIDTH = 16

    def __init__(self, side: str, parent=None):
        super().__init__(parent)
        self._side = side                        # side of the SCREEN the panel lives on
        self._open_dir = 1 if side == "right" else -1
        self._hover = 0.0
        self._flip = 0.0                         # 0 = panel open, 1 = panel hidden
        self.setFixedWidth(self.WIDTH)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._hover_anim = QVariantAnimation(self)
        self._hover_anim.setDuration(anim.DUR_FAST)
        self._hover_anim.valueChanged.connect(self._on_hover)
        self._flip_anim = QVariantAnimation(self)
        self._flip_anim.setEasingCurve(anim.EASE_OUT)
        self._flip_anim.valueChanged.connect(self._on_flip)

    def _on_hover(self, v):
        self._hover = float(v)
        self.update()

    def _on_flip(self, v):
        self._flip = float(v)
        self.update()

    def set_collapsed(self, collapsed: bool, animate: bool = True):
        end = 1.0 if collapsed else 0.0
        self._flip_anim.stop()
        d = anim.dur(anim.DUR_MED) if animate else 0
        if d == 0:
            self._flip = end
            self.update()
            return
        self._flip_anim.setDuration(d)
        self._flip_anim.setStartValue(self._flip)
        self._flip_anim.setEndValue(end)
        self._flip_anim.start()

    def _go(self, target: float):
        self._hover_anim.stop()
        d = anim.dur(anim.DUR_FAST)
        if d == 0:
            self._hover = target
            self.update()
            return
        self._hover_anim.setDuration(d)
        self._hover_anim.setStartValue(self._hover)
        self._hover_anim.setEndValue(target)
        self._hover_anim.start()

    def enterEvent(self, e):
        self._go(1.0)
        super().enterEvent(e)

    def leaveEvent(self, e):
        self._go(0.0)
        super().leaveEvent(e)

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        c = anim.theme_colors()
        p.fillRect(self.rect(), anim.mix(c["bg_deep"], c["hover"], self._hover * 0.8))
        p.setPen(QPen(c["border"], 1))
        edge = 0.5 if self._side == "right" else self.width() - 0.5
        p.drawLine(QPointF(edge, 0), QPointF(edge, self.height()))

        cx, cy = self.width() / 2.0, self.height() / 2.0
        pill = QRectF(2.5, cy - 24, self.width() - 5, 48)
        fill = anim.mix(c["card"], c["accent"], self._hover * 0.30 + (0.12 if self.isDown() else 0.0))
        p.setPen(QPen(anim.mix(c["border"], c["accent"], self._hover), 1))
        p.setBrush(fill)
        p.drawRoundedRect(pill, 6, 6)

        direction = self._open_dir * (1.0 - 2.0 * self._flip)
        pen = QPen(anim.mix(c["muted"], c["accent"], self._hover), 2)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.translate(cx, cy)
        p.scale(direction, 1.0)
        p.drawPolyline([QPointF(-2.5, -6), QPointF(2.5, 0), QPointF(-2.5, 6)])


# ── Clip area ─────────────────────────────────────────────────────────

class _Clip(QWidget):
    """Fixed-width window onto the content; the content keeps its size and slides out."""

    def __init__(self, side: str, parent=None):
        super().__init__(parent)
        self._side = side
        self._content: QWidget | None = None

    def set_content(self, content: QWidget):
        self._content = content
        content.setParent(self)
        self._place()

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._place()

    def _place(self):
        c = self._content
        if c is None:
            return
        w = c.width()
        x = 0 if self._side == "left" else self.width() - w
        c.setGeometry(x, 0, w, self.height())


# ── Collapsible side panel ────────────────────────────────────────────

class CollapsiblePanel(QWidget):
    """Side panel with a hide / show handle.

    ``side`` is the side of the screen the panel sits on ("left" or "right"). ``key`` makes the
    state persistent. The panel keeps ``expanded_width`` when open and 0 when hidden (the handle
    stays so it can be opened again).
    """

    def __init__(self, content: QWidget, expanded_width: int, side: str = "right",
                 key: str = "", parent=None):
        super().__init__(parent)
        self._side = side
        self._key = key
        self._expanded_w = int(expanded_width)
        self._content = content
        self._collapsed = False

        self._clip = _Clip(side, self)
        content.setFixedWidth(self._expanded_w)
        self._clip.set_content(content)
        self._clip.setFixedWidth(self._expanded_w)

        self._handle = _EdgeHandle(side, self)
        self._handle.clicked.connect(self.toggle)
        self._update_tip()

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        if side == "right":
            lay.addWidget(self._handle)
            lay.addWidget(self._clip)
        else:
            lay.addWidget(self._clip)
            lay.addWidget(self._handle)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)

        self._anim = QVariantAnimation(self)
        self._anim.setEasingCurve(anim.EASE_INOUT)
        self._anim.valueChanged.connect(lambda v: self._clip.setFixedWidth(int(v)))
        self._anim.finished.connect(self._finish)

        if key and _saved("ui_panels", key):
            self.set_collapsed(True, animate=False, persist=False)

    # public ---------------------------------------------------------
    def is_collapsed(self) -> bool:
        return self._collapsed

    def content(self) -> QWidget:
        return self._content

    def expanded_width(self) -> int:
        return self._expanded_w

    def toggle(self):
        self.set_collapsed(not self._collapsed)

    def set_collapsed(self, collapsed: bool, animate: bool = True, persist: bool = True):
        collapsed = bool(collapsed)
        if collapsed == self._collapsed:
            return
        self._collapsed = collapsed
        self._handle.set_collapsed(collapsed, animate)
        self._update_tip()
        if not collapsed:
            self._content.show()
        target = 0 if collapsed else self._expanded_w
        self._anim.stop()
        d = anim.dur(anim.DUR_MED) if animate else 0
        if d == 0:
            self._clip.setFixedWidth(target)
            self._finish()
        else:
            self._anim.setDuration(d)
            self._anim.setStartValue(self._clip.width())
            self._anim.setEndValue(target)
            self._anim.start()
        if persist:
            _save("ui_panels", self._key, collapsed)

    def retranslate(self):
        self._update_tip()

    # internals ------------------------------------------------------
    def _finish(self):
        if self._collapsed:
            self._content.hide()
        else:
            self._content.show()

    def _update_tip(self):
        self._handle.setToolTip(tr("panel_show") if self._collapsed else tr("panel_hide"))


def collapse_panels(widget: QWidget, collapse: bool, animate: bool = True,
                    only: list | None = None) -> list:
    """Collapse (or restore) the panels inside ``widget``. Returns the panels that changed."""
    changed = []
    panels = only if only is not None else widget.findChildren(CollapsiblePanel)
    for p in panels:
        if p.is_collapsed() != collapse:
            p.set_collapsed(collapse, animate=animate, persist=False)
            changed.append(p)
    return changed


# ── Collapsible group boxes ───────────────────────────────────────────

class _Chevron(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setFixedSize(14, 14)
        self._t = 0.0                     # 0 = open (v), 1 = folded (>)

    def set_t(self, t: float):
        self._t = t
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        c = anim.theme_colors()
        p.fillRect(self.rect(), c["bg"])          # hides the box border behind the chevron
        pen = QPen(c["muted"], 1.8)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        p.setPen(pen)
        p.translate(self.width() / 2.0, self.height() / 2.0)
        p.rotate(-90.0 * self._t)
        p.drawPolyline([QPointF(-4, -2), QPointF(0, 2), QPointF(4, -2)])


class GroupCollapser(QObject):
    """Makes a QGroupBox fold when its title row is clicked."""

    def __init__(self, box: QGroupBox, key: str = ""):
        super().__init__(box)
        self.box = box
        self.key = key
        self.collapsed = False
        self._hidden: list[QWidget] = []
        self._expanded_h = 0
        self._chev = _Chevron(box)
        self._chev.show()
        self._anim = QVariantAnimation(self)
        self._anim.setEasingCurve(anim.EASE_INOUT)
        self._anim.valueChanged.connect(self._on_value)
        self._anim.finished.connect(self._finish)
        box.installEventFilter(self)
        box.setProperty("_group_collapser", True)
        self._place()
        if key and _saved("ui_groups", key):
            QTimer.singleShot(0, lambda: self.set_collapsed(True, animate=False, persist=False))

    def _header_h(self) -> int:
        return max(20, self.box.contentsRect().top())

    def _collapsed_h(self) -> int:
        return max(18, self._header_h() - 2)

    def _place(self):
        self._chev.move(self.box.width() - 26, 3)
        self._chev.raise_()

    def eventFilter(self, obj, ev):
        if obj is self.box:
            t = ev.type()
            if t in (QEvent.Type.Resize, QEvent.Type.Show):
                self._place()
            elif (t == QEvent.Type.MouseButtonRelease
                  and ev.button() == Qt.MouseButton.LeftButton
                  and ev.position().y() <= self._header_h()
                  and self.box.rect().contains(ev.position().toPoint())):
                self.toggle()
        return False

    def toggle(self):
        self.set_collapsed(not self.collapsed)

    def _children(self) -> list[QWidget]:
        return [w for w in self.box.findChildren(
            QWidget, options=Qt.FindChildOption.FindDirectChildrenOnly) if w is not self._chev]

    def set_collapsed(self, collapsed: bool, animate: bool = True, persist: bool = True):
        collapsed = bool(collapsed)
        if collapsed == self.collapsed:
            return
        self.collapsed = collapsed
        box, lay = self.box, self.box.layout()
        self._anim.stop()
        if collapsed:
            self._expanded_h = max(box.sizeHint().height(), box.height())
            self._hidden = [w for w in self._children() if w.isVisibleTo(box)]
            if lay is not None:
                lay.setEnabled(False)          # freeze children while the box shrinks
            start, end = box.height(), self._collapsed_h()
        else:
            for w in self._hidden:
                w.show()
            start, end = box.height(), self._expanded_h or box.sizeHint().height()
        d = anim.dur(anim.DUR_MED) if animate else 0
        if d == 0:
            self._on_value(end)
            self._chev.set_t(1.0 if collapsed else 0.0)
            self._finish()
        else:
            self._anim.setDuration(d)
            self._anim.setStartValue(start)
            self._anim.setEndValue(end)
            self._anim.start()
        if persist:
            _save("ui_groups", self.key, collapsed)

    def _on_value(self, v):
        h = int(v)
        self.box.setFixedHeight(h)
        span = max(1, self._expanded_h - self._collapsed_h())
        frac = (h - self._collapsed_h()) / span
        self._chev.set_t(1.0 - max(0.0, min(1.0, frac)))

    def _finish(self):
        box, lay = self.box, self.box.layout()
        if self.collapsed:
            for w in self._hidden:
                w.hide()
            self._chev.set_t(1.0)
        else:
            box.setMinimumHeight(0)
            box.setMaximumHeight(_BIG)
            if lay is not None:
                lay.setEnabled(True)
                lay.activate()
            self._chev.set_t(0.0)


def attach_group_collapse(box: QGroupBox, key: str = "") -> GroupCollapser | None:
    if box is None or box.layout() is None:
        return None
    for ch in box.children():
        if isinstance(ch, GroupCollapser):
            return ch
    return GroupCollapser(box, key)


def make_groups_collapsible(container: QWidget, prefix: str) -> list[GroupCollapser]:
    """Make every top-level QGroupBox in ``container`` foldable (nested ones are left alone)."""
    out = []
    boxes = container.findChildren(QGroupBox)
    for i, g in enumerate(boxes):
        p = g.parentWidget()
        nested = False
        while p is not None and p is not container:
            if isinstance(p, QGroupBox):
                nested = True
                break
            p = p.parentWidget()
        if nested:
            continue
        c = attach_group_collapse(g, f"{prefix}.{i}")
        if c is not None:
            out.append(c)
    return out
