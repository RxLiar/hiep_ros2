"""
login_dialog.py — Màn hình login

Fix:
- Thêm text "VN / EN / KR" cạnh lá cờ trong language button.
- Thêm label "Ngôn ngữ / Language:" trước dãy nút chọn ngôn ngữ.
- Window controls: nút nhỏ hơn, ký tự không bị cắt.
"""
import hashlib
import json
import os

from PyQt6.QtWidgets import (
    QDialog, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QFrame,
    QButtonGroup
)
from PyQt6.QtCore import Qt, QSize, pyqtSignal
from PyQt6.QtGui import QMouseEvent

from agv_hmi.ui.i18n import tr, set_lang, get_lang, LANGUAGES
from agv_hmi.ui.language_selector import make_flag_icon

CONFIG_PATH = os.path.expanduser("~/.agv_hmi/config.json")


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


def get_engineer_password() -> str:
    return _load_config().get(
        "engineer_pw_hash",
        hashlib.sha256(b"9999").hexdigest()
    )


def set_engineer_password(new_pw: str):
    cfg = _load_config()
    cfg["engineer_pw_hash"] = hashlib.sha256(new_pw.encode()).hexdigest()
    _save_config(cfg)


def check_engineer_password(pw: str) -> bool:
    return hashlib.sha256(pw.encode()).hexdigest() == get_engineer_password()


# ── Role card ─────────────────────────────────────────────────────────

class _RoleCard(QFrame):
    clicked = pyqtSignal()

    def __init__(self, icon: str, title_key: str, desc_key: str, accent: str):
        super().__init__()
        self._accent    = accent
        self._title_key = title_key
        self._desc_key  = desc_key
        self.setFixedSize(240, 170)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self._apply_style(False)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(8)

        ic = QLabel(icon)
        ic.setFixedSize(48, 48)
        ic.setStyleSheet(
            f"font-size:24px;background:{accent}25;"
            "border-radius:10px;qproperty-alignment:AlignCenter;")

        self._tl = QLabel(tr(title_key))
        self._tl.setStyleSheet(
            "font-size:15px;font-weight:700;color:#E6EDF3;")

        self._dl = QLabel(tr(desc_key))
        self._dl.setStyleSheet("font-size:11px;color:#8B949E;")
        self._dl.setWordWrap(True)

        for w in (ic, self._tl, self._dl):
            lay.addWidget(w)
        lay.addStretch()

    def _apply_style(self, sel: bool):
        color = self._accent if sel else "#30363D"
        bg    = f"{self._accent}15" if sel else "#161B22"
        self.setStyleSheet(
            f"QFrame{{background:{bg};"
            f"border:2px solid {color};"
            "border-radius:12px;}")

    def set_selected(self, v: bool):
        self._apply_style(v)

    def retranslate(self):
        self._tl.setText(tr(self._title_key))
        self._dl.setText(tr(self._desc_key))

    def mousePressEvent(self, e: QMouseEvent):
        self.clicked.emit()


# ── Language button với cờ + text ────────────────────────────────────

class _FlagLangButton(QPushButton):
    """
    Nút chọn ngôn ngữ: painted flag icon + text code (VN / EN / KR).
    """
    # Tên ngắn hiển thị cạnh cờ
    _SHORT = {"vi": "VN", "en": "EN", "ko": "KR"}

    def __init__(self, code: str, label: str):
        super().__init__()
        self.code  = code
        self.label = label

        self.setCheckable(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip(label)

        # Painted flag icon
        icon = make_flag_icon(code, width=32, height=22)
        self.setIcon(icon)
        self.setIconSize(QSize(32, 22))

        # Text ngắn cạnh cờ
        short = self._SHORT.get(code, code.upper())
        self.setText(f"  {short}")

        # Kích thước cố định vừa đủ
        self.setFixedSize(80, 36)

        self._apply_style(False)

    def _apply_style(self, checked: bool):
        if checked:
            self.setStyleSheet(
                "QPushButton{"
                "background:#1C2D40;"
                "border:2px solid #58A6FF;"
                "border-radius:8px;"
                "color:#58A6FF;"
                "font-size:12px;"
                "font-weight:700;"
                "text-align:left;"
                "padding-left:4px;"
                "}"
            )
        else:
            self.setStyleSheet(
                "QPushButton{"
                "background:#161B22;"
                "border:1px solid #30363D;"
                "border-radius:8px;"
                "color:#8B949E;"
                "font-size:12px;"
                "font-weight:600;"
                "text-align:left;"
                "padding-left:4px;"
                "}"
                "QPushButton:hover{"
                "border-color:#58A6FF;"
                "color:#E6EDF3;"
                "background:#1A2030;"
                "}"
            )

    def setChecked(self, v: bool):
        super().setChecked(v)
        self._apply_style(v)


# ── Login Dialog ──────────────────────────────────────────────────────

class LoginDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint | Qt.WindowType.Dialog)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setFixedSize(620, 560)

        self._role     = "operator"
        self._drag_pos = None

        self._build()
        self._select_role("operator")
        self._select_lang(get_lang())

    # ── Build ─────────────────────────────────────────────────────────

    def _build(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        card = QFrame()
        card.setStyleSheet(
            "QFrame{"
            "background:#0D1117;"
            "border:1px solid #30363D;"
            "border-radius:18px;"
            "}"
        )
        outer.addWidget(card)

        lay = QVBoxLayout(card)
        lay.setContentsMargins(44, 32, 44, 36)
        lay.setSpacing(0)

        # ── Titlebar ─────────────────────────────────────────────────
        tb = QHBoxLayout()
        tb.setContentsMargins(0, 0, 0, 0)
        tb.setSpacing(6)

        logo_ic = QLabel("🤖")
        logo_ic.setFixedSize(36, 36)
        logo_ic.setStyleSheet(
            "background:#185FA5;border-radius:8px;"
            "font-size:20px;qproperty-alignment:AlignCenter;")

        app_lbl = QLabel(tr("app_name"))
        app_lbl.setStyleSheet(
            "font-size:13px;font-weight:700;color:#E6EDF3;")

        tb.addWidget(logo_ic)
        tb.addSpacing(8)
        tb.addWidget(app_lbl)
        tb.addStretch()

        # Window controls — nút nhỏ gọn, text không bị cắt
        for sym, tip, bg, hover, slot in [
            ("—",  "Thu nhỏ",  "#21262D", "#30363D", self._minimize),
            ("⬜", "Phóng to", "#21262D", "#30363D", self._maximize),
            ("✕",  "Đóng",     "#6B0000", "#B22222", self.reject),
        ]:
            b = QPushButton(sym)
            b.setToolTip(tip)
            b.setFixedSize(26, 26)
            b.setStyleSheet(
                f"QPushButton{{"
                f"  background:{bg};"
                "  border:none;"
                "  border-radius:5px;"
                "  color:#E6EDF3;"
                "  font-size:12px;"
                "  font-weight:600;"
                "  qproperty-flat:true;"
                "}}"
                f"QPushButton:hover{{"
                f"  background:{hover};"
                "  color:#FFFFFF;"
                "}}"
            )
            b.clicked.connect(slot)
            tb.addWidget(b)

        lay.addLayout(tb)
        lay.addSpacing(20)

        # ── Language selector ─────────────────────────────────────────
        lang_row = QHBoxLayout()
        lang_row.setContentsMargins(0, 0, 0, 0)
        lang_row.setSpacing(8)

        # Label chú thích
        lang_lbl = QLabel("🌐  Language:")
        lang_lbl.setStyleSheet(
            "color:#8B949E;"
            "font-size:12px;"
            "font-weight:600;"
            "background:transparent;"
        )
        lang_row.addWidget(lang_lbl)
        lang_row.addSpacing(4)

        # Các nút cờ + text
        self._lang_btns: list[_FlagLangButton] = []
        self._lang_group = QButtonGroup(self)
        self._lang_group.setExclusive(True)

        for meta in LANGUAGES:
            b = _FlagLangButton(meta["code"], meta["label"])
            self._lang_group.addButton(b)
            b.clicked.connect(
                lambda _, c=meta["code"]: self._select_lang(c))
            lang_row.addWidget(b)
            self._lang_btns.append(b)

        lang_row.addStretch()
        lay.addLayout(lang_row)
        lay.addSpacing(20)

        # ── Subtitle ──────────────────────────────────────────────────
        self._sub = QLabel(tr("login_select_mode"))
        self._sub.setStyleSheet("color:#8B949E;font-size:12px;")
        lay.addWidget(self._sub)
        lay.addSpacing(14)

        # ── Role cards ────────────────────────────────────────────────
        cards_row = QHBoxLayout()
        cards_row.setSpacing(16)
        self._op_card  = _RoleCard(
            "👷", "login_operator", "login_op_desc", "#3FB950")
        self._eng_card = _RoleCard(
            "⚙️", "login_engineer", "login_eng_desc", "#58A6FF")
        self._op_card.clicked.connect(
            lambda: self._select_role("operator"))
        self._eng_card.clicked.connect(
            lambda: self._select_role("engineer"))
        cards_row.addWidget(self._op_card)
        cards_row.addWidget(self._eng_card)
        lay.addLayout(cards_row)
        lay.addSpacing(18)

        # ── Password ──────────────────────────────────────────────────
        self._pw_frame = QFrame()
        self._pw_frame.setVisible(False)
        pwl = QVBoxLayout(self._pw_frame)
        pwl.setContentsMargins(0, 0, 0, 0)
        pwl.setSpacing(6)

        self._pw_lbl = QLabel(tr("login_password"))
        self._pw_lbl.setStyleSheet("color:#8B949E;font-size:12px;")

        self._pw_edit = QLineEdit()
        self._pw_edit.setEchoMode(QLineEdit.EchoMode.Password)
        self._pw_edit.setPlaceholderText("••••••")
        self._pw_edit.setStyleSheet(
            "QLineEdit{"
            "background:#161B22;"
            "border:1px solid #30363D;"
            "border-radius:8px;"
            "padding:8px 14px;"
            "font-size:15px;"
            "color:#E6EDF3;"
            "letter-spacing:3px;"
            "}"
            "QLineEdit:focus{border-color:#58A6FF;}"
        )
        self._pw_edit.returnPressed.connect(self._do_login)

        self._pw_err = QLabel("")
        self._pw_err.setStyleSheet("color:#F85149;font-size:11px;")
        self._pw_err.setVisible(False)

        pwl.addWidget(self._pw_lbl)
        pwl.addWidget(self._pw_edit)
        pwl.addWidget(self._pw_err)
        lay.addWidget(self._pw_frame)

        lay.addStretch()

        # ── Enter button ──────────────────────────────────────────────
        self._enter_btn = QPushButton(tr("login_enter"))
        self._enter_btn.setFixedHeight(44)
        self._enter_btn.setStyleSheet(
            "QPushButton{"
            "background:#1158C7;"
            "color:white;"
            "border:none;"
            "border-radius:10px;"
            "font-size:14px;"
            "font-weight:700;"
            "}"
            "QPushButton:hover{background:#388BFD;}"
            "QPushButton:disabled{background:#1C2D40;color:#484F58;}"
        )
        self._enter_btn.clicked.connect(self._do_login)
        lay.addWidget(self._enter_btn)

    # ── Logic ─────────────────────────────────────────────────────────

    def _select_role(self, role: str):
        self._role = role
        self._op_card.set_selected(role == "operator")
        self._eng_card.set_selected(role == "engineer")
        self._pw_frame.setVisible(role == "engineer")
        self._pw_err.setVisible(False)
        self._pw_edit.clear()
        if role == "engineer":
            self._pw_edit.setFocus()

    def _select_lang(self, code: str):
        set_lang(code)
        for b in self._lang_btns:
            b.setChecked(b.code == code)
        self._retranslate()

    def _retranslate(self):
        self._sub.setText(tr("login_select_mode"))
        self._pw_lbl.setText(tr("login_password"))
        self._enter_btn.setText(tr("login_enter"))
        self._op_card.retranslate()
        self._eng_card.retranslate()

    def _do_login(self):
        if self._role == "engineer":
            if not check_engineer_password(self._pw_edit.text()):
                self._pw_err.setText(tr("login_wrong_pw"))
                self._pw_err.setVisible(True)
                self._pw_edit.selectAll()
                return
        self.accept()

    def result_role(self) -> str:
        return self._role

    # ── Window drag ───────────────────────────────────────────────────

    def mousePressEvent(self, e: QMouseEvent):
        if e.button() == Qt.MouseButton.LeftButton:
            self._drag_pos = (
                e.globalPosition().toPoint()
                - self.frameGeometry().topLeft()
            )

    def mouseMoveEvent(self, e: QMouseEvent):
        if self._drag_pos and e.buttons() == Qt.MouseButton.LeftButton:
            self.move(e.globalPosition().toPoint() - self._drag_pos)

    def mouseReleaseEvent(self, e: QMouseEvent):
        self._drag_pos = None

    def _minimize(self):
        self.showMinimized()

    def _maximize(self):
        if self.isMaximized():
            self.showNormal()
        else:
            self.showMaximized()

