"""
map_library_page.py — Thư viện bản đồ (Engineer only)

Danh sách map từ ~/maps/*.yaml, mới nhất lên đầu.
Mỗi item: Thumbnail + Tên + Thời gian + nút Dùng trong Nav + Xoá.
Thumbnail: snapshot QPixmap từ MapWidget lúc lưu map.
"""
import os
from datetime import datetime

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QScrollArea,
    QFrame, QMessageBox
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QPixmap

from agv_hmi.ui.i18n import tr
from agv_hmi.ui.map_widget import MapWidget

MAPS_DIR   = os.path.expanduser("~/maps")
THUMB_DIR  = os.path.expanduser("~/.agv_hmi/thumbnails")


def _btn(t, obj="", h=30, enabled=True):
    b = QPushButton(t)
    if obj: b.setObjectName(obj)
    b.setFixedHeight(h)
    b.setEnabled(enabled)
    return b


class _MapCard(QFrame):
    """Card cho 1 map file."""
    use_clicked = pyqtSignal(str)   # path
    del_clicked = pyqtSignal(str)   # path

    def __init__(self, yaml_path: str):
        super().__init__()
        self._path = yaml_path
        self.setObjectName("Card")
        self.setStyleSheet(
            "QFrame#Card{background:#161B22;border:1px solid #30363D;"
            "border-radius:10px;}"
            "QFrame#Card:hover{border-color:#58A6FF;}"
        )
        self._build()

    def _build(self):
        lay = QHBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(14)

        # Thumbnail
        thumb_lbl = QLabel()
        thumb_lbl.setFixedSize(80, 60)
        thumb_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        thumb_lbl.setStyleSheet(
            "background:#0D1117;border:1px solid #21262D;border-radius:6px;")

        thumb_path = _MapCard._thumb_path(self._path)
        if os.path.exists(thumb_path):
            pix = QPixmap(thumb_path).scaled(
                80, 60,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation)
            thumb_lbl.setPixmap(pix)
        else:
            thumb_lbl.setText("🗺")
            thumb_lbl.setStyleSheet(
                thumb_lbl.styleSheet() + "font-size:28px;color:#30363D;")
        lay.addWidget(thumb_lbl)

        # Info
        info = QVBoxLayout()
        info.setSpacing(4)

        name = os.path.basename(self._path)
        name_lbl = QLabel(name)
        name_lbl.setStyleSheet(
            "font-size:13px;font-weight:600;color:#E6EDF3;background:transparent;")
        info.addWidget(name_lbl)

        try:
            mtime = os.stat(self._path).st_mtime
            dt = datetime.fromtimestamp(mtime).strftime("%d/%m/%Y  %H:%M")
        except Exception:
            dt = "—"
        time_lbl = QLabel(f"{tr('maplib_created')} {dt}")
        time_lbl.setStyleSheet(
            "font-size:11px;color:#8B949E;background:transparent;")
        info.addWidget(time_lbl)
        lay.addLayout(info)
        lay.addStretch()

        # Buttons
        btn_col = QVBoxLayout()
        btn_col.setSpacing(6)

        use_btn = _btn(tr("maplib_use_nav"), "BtnPrimary", h=28)
        use_btn.clicked.connect(lambda: self.use_clicked.emit(self._path))

        del_btn = _btn(tr("maplib_delete"), "BtnDanger", h=28)
        del_btn.clicked.connect(lambda: self.del_clicked.emit(self._path))

        btn_col.addWidget(use_btn)
        btn_col.addWidget(del_btn)
        lay.addLayout(btn_col)

    @staticmethod
    def _thumb_path(yaml_path: str) -> str:
        base = os.path.splitext(os.path.basename(yaml_path))[0]
        return os.path.join(THUMB_DIR, f"{base}.png")


class MapLibraryPage(QWidget):
    map_selected = pyqtSignal(str)   # yaml_path → NavigationPage

    def __init__(self):
        super().__init__()
        os.makedirs(MAPS_DIR,  exist_ok=True)
        os.makedirs(THUMB_DIR, exist_ok=True)
        self._build()
        self.refresh()

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(14)

        # Header
        hdr = QHBoxLayout()
        title = QLabel(tr("maplib_title"))
        title.setStyleSheet(
            "font-size:18px;font-weight:700;color:#E6EDF3;background:transparent;")
        hdr.addWidget(title)
        hdr.addStretch()
        ref_btn = _btn(tr("maplib_refresh"), h=32)
        ref_btn.clicked.connect(self.refresh)
        hdr.addWidget(ref_btn)
        root.addLayout(hdr)

        # Scrollable cards
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setStyleSheet(
            "QScrollArea{border:none;background:transparent;}")

        self._cards_widget = QWidget()
        self._cards_widget.setStyleSheet("background:transparent;")
        self._cards_lay = QVBoxLayout(self._cards_widget)
        self._cards_lay.setContentsMargins(0, 0, 4, 0)
        self._cards_lay.setSpacing(12)
        self._cards_lay.addStretch()
        self._scroll.setWidget(self._cards_widget)
        root.addWidget(self._scroll)

        # Empty hint
        self._empty_lbl = QLabel("Chưa có map nào. Thực hiện Mapping để tạo map.")
        self._empty_lbl.setObjectName("EmptyHint")
        self._empty_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.addWidget(self._empty_lbl)

    def refresh(self):
        """Reload danh sách từ ~/maps."""
        # Xoá cards cũ (giữ stretch cuối)
        while self._cards_lay.count() > 1:
            item = self._cards_lay.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        # Liệt kê yaml files, sort mới nhất đầu
        try:
            files = [
                os.path.join(MAPS_DIR, f)
                for f in os.listdir(MAPS_DIR)
                if f.endswith(".yaml")
            ]
            files.sort(key=os.path.getmtime, reverse=True)
        except Exception:
            files = []

        self._empty_lbl.setVisible(not files)

        for path in files:
            card = _MapCard(path)
            card.use_clicked.connect(self.map_selected)
            card.del_clicked.connect(self._on_delete)
            self._cards_lay.insertWidget(
                self._cards_lay.count() - 1, card)

    def _on_delete(self, path: str):
        name = os.path.basename(path)
        reply = QMessageBox.question(
            self,
            tr("delete_map"),          # FIX: bỏ arg thừa ""
            f"Xoá map '{name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            os.remove(path)
            pgm = path.replace(".yaml", ".pgm")
            if os.path.exists(pgm):
                os.remove(pgm)
            thumb = _MapCard._thumb_path(path)
            if os.path.exists(thumb):
                os.remove(thumb)
        except Exception as e:
            QMessageBox.warning(self, "Lỗi", str(e))
        self.refresh()

    @staticmethod
    def save_thumbnail(yaml_path: str, map_widget: MapWidget):
        """
        Chụp snapshot từ MapWidget và lưu vào THUMB_DIR.
        Gọi ngay sau khi map_saver.save() thành công.
        """
        os.makedirs(THUMB_DIR, exist_ok=True)
        base  = os.path.splitext(os.path.basename(yaml_path))[0]
        thumb = os.path.join(THUMB_DIR, f"{base}.png")
        try:
            pixmap = map_widget.grab()
            pixmap.save(thumb, "PNG")
        except Exception as e:
            print(f"[MapLibrary] thumbnail save failed: {e}")

    def retranslate(self):
        self.refresh()