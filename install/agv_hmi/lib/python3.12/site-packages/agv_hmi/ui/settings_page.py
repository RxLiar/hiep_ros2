"""Settings page.

- Language: available to Operator and Engineer.
- Theme: available to Operator and Engineer.
- Engineer password: visible only to Engineer.
"""
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QLineEdit,
    QPushButton,
    QButtonGroup,
    QScrollArea,
)
from PyQt6.QtCore import Qt, pyqtSignal

from agv_hmi.ui.i18n import tr
from agv_hmi.ui.language_selector import LanguageSelector
from agv_hmi.ui.login_dialog import (
    check_engineer_password,
    set_engineer_password,
)
from agv_hmi.ui import theme_manager as TM


def _btn(text: str, object_name: str = "", height: int = 34):
    button = QPushButton(text)
    if object_name:
        button.setObjectName(object_name)
    button.setFixedHeight(height)
    return button


class _ThemeOptionButton(QPushButton):
    def __init__(self, icon: str, label: str, value: str):
        super().__init__(f"{icon}  {label}")
        self.icon_text = icon
        self.value = value
        self.setObjectName("ThemeOptBtn")
        self.setCheckable(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)


class SettingsPage(QWidget):
    language_changed = pyqtSignal(str)

    def __init__(self, role: str = "operator", theme_mgr=None):
        super().__init__()
        self._role = role
        self._theme_mgr = theme_mgr
        self._build()

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(14)

        self._title = QLabel(tr("settings_title"))
        self._title.setStyleSheet(
            "font-size:18px;font-weight:700;color:#E6EDF3;"
        )
        root.addWidget(self._title)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setObjectName("Panel")
        scroll.setStyleSheet("QScrollArea{border:none;background:transparent;}")

        inner = QWidget()
        inner.setStyleSheet("background:transparent;")
        content = QVBoxLayout(inner)
        content.setContentsMargins(4, 4, 8, 8)
        content.setSpacing(16)

        # Language
        self._language_box = QGroupBox(tr("settings_language_title"))
        language_layout = QVBoxLayout(self._language_box)
        language_layout.setSpacing(10)

        self._language_hint = QLabel(tr("settings_language_hint"))
        self._language_hint.setWordWrap(True)
        self._language_hint.setStyleSheet("color:#8B949E;font-size:11px;")
        language_layout.addWidget(self._language_hint)

        self._language_selector = LanguageSelector(compact=False)
        self._language_selector.language_changed.connect(
            self._on_language_changed
        )
        language_layout.addWidget(self._language_selector)
        content.addWidget(self._language_box)

        # Theme
        self._theme_box = QGroupBox(tr("settings_theme_title"))
        theme_layout = QVBoxLayout(self._theme_box)
        theme_layout.setSpacing(10)

        self._theme_hint = QLabel(tr("settings_theme_hint"))
        self._theme_hint.setWordWrap(True)
        self._theme_hint.setStyleSheet("color:#8B949E;font-size:11px;")
        theme_layout.addWidget(self._theme_hint)

        option_row = QHBoxLayout()
        option_row.setSpacing(8)

        self._theme_group = QButtonGroup(self)
        self._theme_group.setExclusive(True)

        self._opt_system = _ThemeOptionButton(
            "💻", tr("theme_system"), TM.THEME_SYSTEM
        )
        self._opt_light = _ThemeOptionButton(
            "☀️", tr("theme_light"), TM.THEME_LIGHT
        )
        self._opt_dark = _ThemeOptionButton(
            "🌙", tr("theme_dark"), TM.THEME_DARK
        )

        for button in (self._opt_system, self._opt_light, self._opt_dark):
            self._theme_group.addButton(button)
            button.clicked.connect(self._on_theme_clicked)
            option_row.addWidget(button)

        theme_layout.addLayout(option_row)
        content.addWidget(self._theme_box)
        self._sync_theme_buttons()

        # Engineer password
        self._password_box = QGroupBox(tr("settings_pw_title"))
        password_layout = QVBoxLayout(self._password_box)
        password_layout.setSpacing(10)

        self._pw_old_label, self._pw_old = self._make_password_row(
            password_layout, "settings_pw_old"
        )
        self._pw_new_label, self._pw_new = self._make_password_row(
            password_layout, "settings_pw_new"
        )
        self._pw_conf_label, self._pw_conf = self._make_password_row(
            password_layout, "settings_pw_conf"
        )

        self._pw_msg = QLabel("")
        self._pw_msg.setVisible(False)
        password_layout.addWidget(self._pw_msg)

        self._save_password_button = _btn(
            tr("settings_pw_save"), "BtnPrimary"
        )
        self._save_password_button.clicked.connect(self._save_password)
        password_layout.addWidget(self._save_password_button)

        self._password_box.setVisible(self._role == "engineer")
        content.addWidget(self._password_box)

        self._future_box = QGroupBox(tr("settings_future"))
        future_layout = QVBoxLayout(self._future_box)
        placeholder = QLabel("—")
        placeholder.setStyleSheet("color:#484F58;font-size:12px;")
        future_layout.addWidget(placeholder)
        content.addWidget(self._future_box)

        content.addStretch()
        scroll.setWidget(inner)
        root.addWidget(scroll)

    @staticmethod
    def _make_password_row(layout: QVBoxLayout, label_key: str):
        label = QLabel(tr(label_key))
        label.setStyleSheet("color:#8B949E;font-size:12px;")
        edit = QLineEdit()
        edit.setEchoMode(QLineEdit.EchoMode.Password)
        edit.setPlaceholderText("••••••")
        layout.addWidget(label)
        layout.addWidget(edit)
        return label, edit

    def _on_language_changed(self, code: str):
        self.language_changed.emit(code)

    def _sync_theme_buttons(self):
        current = (
            self._theme_mgr.current_pref()
            if self._theme_mgr
            else TM.get_theme_pref()
        )
        mapping = {
            TM.THEME_SYSTEM: self._opt_system,
            TM.THEME_LIGHT: self._opt_light,
            TM.THEME_DARK: self._opt_dark,
        }
        mapping.get(current, self._opt_system).setChecked(True)

    def _on_theme_clicked(self):
        sender = self.sender()
        if not isinstance(sender, _ThemeOptionButton):
            return
        if self._theme_mgr is not None:
            self._theme_mgr.set_pref(sender.value)
        else:
            TM.set_theme_pref(sender.value)

    def _save_password(self):
        old_password = self._pw_old.text()
        new_password = self._pw_new.text()
        confirm_password = self._pw_conf.text()

        if not check_engineer_password(old_password):
            self._show_msg(tr("settings_pw_wrong"), ok=False)
            return
        if new_password != confirm_password:
            self._show_msg(tr("settings_pw_nomatch"), ok=False)
            return
        if not new_password:
            self._show_msg(tr("settings_pw_empty"), ok=False)
            return

        set_engineer_password(new_password)
        self._pw_old.clear()
        self._pw_new.clear()
        self._pw_conf.clear()
        self._show_msg(tr("settings_pw_ok"), ok=True)

    def _show_msg(self, message: str, ok: bool):
        color = "#3FB950" if ok else "#F85149"
        self._pw_msg.setText(message)
        self._pw_msg.setStyleSheet(f"color:{color};font-size:12px;")
        self._pw_msg.setVisible(True)

    def retranslate(self):
        self._title.setText(tr("settings_title"))

        self._language_box.setTitle(tr("settings_language_title"))
        self._language_hint.setText(tr("settings_language_hint"))
        self._language_selector.retranslate()

        self._theme_box.setTitle(tr("settings_theme_title"))
        self._theme_hint.setText(tr("settings_theme_hint"))
        self._opt_system.setText(f"💻  {tr('theme_system')}")
        self._opt_light.setText(f"☀️  {tr('theme_light')}")
        self._opt_dark.setText(f"🌙  {tr('theme_dark')}")

        self._password_box.setTitle(tr("settings_pw_title"))
        self._pw_old_label.setText(tr("settings_pw_old"))
        self._pw_new_label.setText(tr("settings_pw_new"))
        self._pw_conf_label.setText(tr("settings_pw_conf"))
        self._save_password_button.setText(tr("settings_pw_save"))
        self._future_box.setTitle(tr("settings_future"))
