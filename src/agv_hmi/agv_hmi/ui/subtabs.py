"""subtabs.py - the sub-tab strip above the page area (4.10.0).

A main sidebar tab can own several pages (Home: Overview | Conveyors | Alarms | Diagnostics ...). The strip lists
the pages of the current group; it hides itself when the group has a single page. It only REQUESTS a page
(page_requested) - MainWindow does the switching, so all the usual page logic (refresh, guards) still runs.
"""
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import QWidget, QHBoxLayout, QTabBar

from agv_hmi.ui.i18n import tr
from agv_hmi.core import nav_groups as NG


class SubTabBar(QWidget):
    page_requested = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("SubTabBar")
        self._pages: list[int] = []
        self._badges: dict[int, int] = {}
        self._block = False
        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 4, 10, 0)
        lay.setSpacing(0)
        self._bar = QTabBar()
        self._bar.setDrawBase(False)
        self._bar.setExpanding(False)
        self._bar.setCursor(Qt.CursorShape.PointingHandCursor)
        self._bar.setStyleSheet(
            "QTabBar::tab{padding:7px 16px;margin-right:2px;background:transparent;border:none;"
            "border-bottom:2px solid transparent;color:#8B949E;font-size:13px;}"
            "QTabBar::tab:hover{color:#C9D1D9;}"
            "QTabBar::tab:selected{color:#58A6FF;font-weight:700;border-bottom:2px solid #58A6FF;}")
        self._bar.currentChanged.connect(self._on_current_changed)
        lay.addWidget(self._bar)
        lay.addStretch(1)
        self.setVisible(False)

    # -- public ----------------------------------------------------------------
    def set_group(self, pages: list[int], current: int) -> None:
        """Show the pages of a group; hide the strip when there is nothing to choose from."""
        if pages != self._pages:
            self._pages = list(pages)
            self._block = True
            while self._bar.count():
                self._bar.removeTab(0)
            for p in self._pages:
                self._bar.addTab(self._title(p))
                self._bar.setTabData(self._bar.count() - 1, p)
            self._block = False
        self.set_current(current)
        self.setVisible(len(self._pages) > 1)

    def set_current(self, idx: int) -> None:
        if idx in self._pages:
            self._block = True
            self._bar.setCurrentIndex(self._pages.index(idx))
            self._block = False

    def set_badge(self, idx: int, count: int) -> None:
        """A number after the title (e.g. active alarms). 0 removes it."""
        self._badges[idx] = int(count)
        if idx in self._pages:
            self._bar.setTabText(self._pages.index(idx), self._title(idx))

    def retranslate(self) -> None:
        for i, p in enumerate(self._pages):
            self._bar.setTabText(i, self._title(p))

    # -- internals ---------------------------------------------------------------
    def _title(self, idx: int) -> str:
        t = tr(NG.TITLE_KEY.get(idx, "page_home"))
        n = self._badges.get(idx, 0)
        return f"{t} ({n})" if n else t

    def _on_current_changed(self, i: int) -> None:
        if self._block or not (0 <= i < len(self._pages)):
            return
        self.page_requested.emit(self._pages[i])
