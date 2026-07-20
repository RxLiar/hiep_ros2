"""
settings_page.py — Trang cài đặt (Engineer only)
- Đổi mật khẩu kỹ sư (hash SHA-256, lưu ~/.agv_hmi/config.json)
- Chọn theme: System / Light / Dark — apply ngay, không cần restart
- Placeholder section mở rộng sau
"""
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QLineEdit, QPushButton, QFrame, QButtonGroup
)
from PyQt6.QtCore import Qt

from agv_hmi.ui.i18n import tr
from agv_hmi.ui.login_dialog import (
    check_engineer_password, set_engineer_password
)
from agv_hmi.ui import theme_manager as TM


def _btn(t, obj="", h=34):
    b = QPushButton(t)
    if obj: b.setObjectName(obj)
    b.setFixedHeight(h)
    return b


class _ThemeOptionButton(QPushButton):
    """Nút lựa chọn theme dạng segmented control (System/Light/Dark)."""

    def __init__(self, icon: str, label: str, value: str):
        super().__init__(f"{icon}  {label}")
        self.value = value
        self.setObjectName("ThemeOptBtn")
        self.setCheckable(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)


class SettingsPage(QWidget):
    """
    theme_mgr: instance của ThemeManager (truyền từ main.py qua MainWindow).
    Nếu None, phần Theme vẫn hiển thị nhưng không apply được (safe-guard).
    """

    def __init__(self, theme_mgr=None):
        super().__init__()
        self._theme_mgr = theme_mgr
        self._build()

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(32, 28, 32, 28)
        root.setSpacing(20)

        # Title
        t = QLabel(tr("settings_title"))
        t.setStyleSheet("font-size:18px;font-weight:700;color:#E6EDF3;")
        root.addWidget(t)

        # ── Theme selector ────────────────────────────────────────
        theme_box = QGroupBox(tr("settings_theme_title"))
        theme_lay = QVBoxLayout(theme_box)
        theme_lay.setSpacing(10)

        hint = QLabel(tr("settings_theme_hint"))
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#8B949E;font-size:11px;")
        theme_lay.addWidget(hint)

        opt_row = QHBoxLayout()
        opt_row.setSpacing(8)

        self._theme_group = QButtonGroup(self)
        self._theme_group.setExclusive(True)

        self._opt_system = _ThemeOptionButton("💻", tr("theme_system"), TM.THEME_SYSTEM)
        self._opt_light  = _ThemeOptionButton("☀️", tr("theme_light"),  TM.THEME_LIGHT)
        self._opt_dark   = _ThemeOptionButton("🌙", tr("theme_dark"),   TM.THEME_DARK)

        for b in (self._opt_system, self._opt_light, self._opt_dark):
            self._theme_group.addButton(b)
            b.clicked.connect(self._on_theme_clicked)
            opt_row.addWidget(b)

        theme_lay.addLayout(opt_row)
        root.addWidget(theme_box)

        # Sync trạng thái nút với preference hiện tại
        self._sync_theme_buttons()

        # ── Password change ──────────────────────────────────────
        pw_box = QGroupBox(tr("settings_pw_title"))
        pw_lay = QVBoxLayout(pw_box)
        pw_lay.setSpacing(10)

        def _pw_row(label_key: str) -> QLineEdit:
            lbl = QLabel(tr(label_key))
            lbl.setStyleSheet("color:#8B949E;font-size:12px;")
            edit = QLineEdit()
            edit.setEchoMode(QLineEdit.EchoMode.Password)
            edit.setPlaceholderText("••••••")
            pw_lay.addWidget(lbl)
            pw_lay.addWidget(edit)
            return edit

        self._pw_old  = _pw_row("settings_pw_old")
        self._pw_new  = _pw_row("settings_pw_new")
        self._pw_conf = _pw_row("settings_pw_conf")

        self._pw_msg = QLabel("")
        self._pw_msg.setVisible(False)
        pw_lay.addWidget(self._pw_msg)

        save_btn = _btn(tr("settings_pw_save"), "BtnPrimary")
        save_btn.clicked.connect(self._save_password)
        pw_lay.addWidget(save_btn)

        root.addWidget(pw_box)

        # ── Future placeholder ────────────────────────────────────
        future_box = QGroupBox(tr("settings_future"))
        fl = QVBoxLayout(future_box)
        hint2 = QLabel("—")
        hint2.setStyleSheet("color:#484F58;font-size:12px;")
        fl.addWidget(hint2)
        root.addWidget(future_box)

        root.addStretch()

    # ── Theme logic ──────────────────────────────────────────────

    def _sync_theme_buttons(self):
        """Đánh dấu nút đang được chọn theo preference đã lưu."""
        current = self._theme_mgr.current_pref() if self._theme_mgr else TM.get_theme_pref()
        mapping = {
            TM.THEME_SYSTEM: self._opt_system,
            TM.THEME_LIGHT:  self._opt_light,
            TM.THEME_DARK:   self._opt_dark,
        }
        btn = mapping.get(current, self._opt_system)
        btn.setChecked(True)

    def _on_theme_clicked(self):
        sender = self.sender()
        if not isinstance(sender, _ThemeOptionButton):
            return

        if self._theme_mgr is not None:
            self._theme_mgr.set_pref(sender.value)
        else:
            # Safe-guard: chỉ lưu pref, không apply runtime được
            TM.set_theme_pref(sender.value)

    # ── Password logic ───────────────────────────────────────────

    def _save_password(self):
        old  = self._pw_old.text()
        new  = self._pw_new.text()
        conf = self._pw_conf.text()

        if not check_engineer_password(old):
            self._show_msg(tr("settings_pw_wrong"), ok=False)
            return
        if new != conf:
            self._show_msg(tr("settings_pw_nomatch"), ok=False)
            return
        if not new:
            self._show_msg("Mật khẩu không được trống!", ok=False)
            return

        set_engineer_password(new)
        self._pw_old.clear(); self._pw_new.clear(); self._pw_conf.clear()
        self._show_msg(tr("settings_pw_ok"), ok=True)

    def _show_msg(self, msg: str, ok: bool):
        color = "#3FB950" if ok else "#F85149"
        self._pw_msg.setText(msg)
        self._pw_msg.setStyleSheet(f"color:{color};font-size:12px;")
        self._pw_msg.setVisible(True)

    def retranslate(self):
        self._opt_system.setText(f"💻  {tr('theme_system')}")
        self._opt_light.setText(f"☀️  {tr('theme_light')}")
        self._opt_dark.setText(f"🌙  {tr('theme_dark')}")