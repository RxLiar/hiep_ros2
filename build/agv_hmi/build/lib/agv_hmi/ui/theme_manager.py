"""
theme_manager.py — Quản lý Light/Dark theme

- Detect system theme (Linux: gsettings/dconf, fallback dark).
- Lưu preference vào ~/.agv_hmi/config.json (key "theme": "system"|"light"|"dark").
- apply_theme() load đúng .qss và setStyleSheet() cho QApplication ngay
  lập tức — không cần restart app.
- theme_changed signal để các widget custom-paint (vd MapWidget) tự
  refresh màu nếu cần.
"""
import json
import os
import subprocess

from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QApplication

CONFIG_PATH = os.path.expanduser("~/.agv_hmi/config.json")

UI_DIR = os.path.join(os.path.dirname(__file__))
QSS_DARK_PATH  = os.path.join(UI_DIR, "style_dark.qss")
QSS_LIGHT_PATH = os.path.join(UI_DIR, "style_light.qss")

THEME_SYSTEM = "system"
THEME_LIGHT  = "light"
THEME_DARK   = "dark"

VALID_THEMES = (THEME_SYSTEM, THEME_LIGHT, THEME_DARK)


def _load_config() -> dict:
    try:
        with open(CONFIG_PATH) as f:
            return json.load(f)
    except Exception:
        return {}


def _save_config(data: dict):
    os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
    with open(CONFIG_PATH, "w") as f:
        json.dump(data, f, indent=2)


def get_theme_pref() -> str:
    """Đọc preference đã lưu. Mặc định 'system' nếu chưa từng set."""
    pref = _load_config().get("theme", THEME_SYSTEM)
    return pref if pref in VALID_THEMES else THEME_SYSTEM


def set_theme_pref(theme: str):
    if theme not in VALID_THEMES:
        return
    cfg = _load_config()
    cfg["theme"] = theme
    _save_config(cfg)


def detect_system_is_dark() -> bool:
    """
    Detect system theme trên Linux (GNOME/KDE qua gsettings).
    Fallback: dark (an toàn cho môi trường industrial HMI thường tối).
    """
    try:
        out = subprocess.run(
            ["gsettings", "get", "org.gnome.desktop.interface", "color-scheme"],
            capture_output=True, text=True, timeout=1.0,
        )
        val = out.stdout.strip().strip("'").lower()
        if "light" in val:
            return False
        if "dark" in val:
            return True
    except Exception:
        pass

    try:
        out = subprocess.run(
            ["gsettings", "get", "org.gnome.desktop.interface", "gtk-theme"],
            capture_output=True, text=True, timeout=1.0,
        )
        val = out.stdout.strip().strip("'").lower()
        if "light" in val:
            return False
        if "dark" in val:
            return True
    except Exception:
        pass

    # Không detect được → mặc định dark (an toàn, phù hợp HMI nhà máy)
    return True


def resolve_effective_theme(pref: str) -> str:
    """Quy đổi 'system' sang 'light'/'dark' thực tế."""
    if pref == THEME_SYSTEM:
        return THEME_DARK if detect_system_is_dark() else THEME_LIGHT
    return pref


class ThemeManager(QObject):
    """
    Singleton-style manager. Tạo 1 instance trong main.py,
    truyền xuống SettingsPage để đổi theme runtime.
    """
    theme_changed = pyqtSignal(str)   # emit "light" hoặc "dark" (effective)

    def __init__(self, app: QApplication):
        super().__init__()
        self._app = app
        self._pref = get_theme_pref()
        self._effective = resolve_effective_theme(self._pref)

    def current_pref(self) -> str:
        return self._pref

    def current_effective(self) -> str:
        return self._effective

    def apply_initial(self):
        """Gọi 1 lần lúc khởi động app, trước khi show window."""
        self._apply_qss(self._effective)

    def set_pref(self, theme: str):
        """Đổi theme — gọi từ SettingsPage. Apply ngay, không cần restart."""
        if theme not in VALID_THEMES:
            return
        self._pref = theme
        set_theme_pref(theme)

        new_effective = resolve_effective_theme(theme)
        if new_effective != self._effective or theme == THEME_SYSTEM:
            self._effective = new_effective
            self._apply_qss(self._effective)
            self.theme_changed.emit(self._effective)

    def _apply_qss(self, effective: str):
        path = QSS_DARK_PATH if effective == THEME_DARK else QSS_LIGHT_PATH
        if not os.path.exists(path):
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                self._app.setStyleSheet(f.read())
        except Exception as e:
            print(f"[ThemeManager] Không load được {path}: {e}")