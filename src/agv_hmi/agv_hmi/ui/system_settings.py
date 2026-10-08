"""system_settings.py - HMI system options (pre-flight, sound, tower light, gamepad, touch, backup)."""
import os

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (QGroupBox, QVBoxLayout, QHBoxLayout, QCheckBox, QLabel, QSpinBox,
                             QDoubleSpinBox, QPushButton, QFormLayout, QFileDialog, QMessageBox, QWidget)

from agv_hmi.core import prefs, backup
from agv_hmi.ui.i18n import tr


class SystemSettingsBox(QGroupBox):
    touch_mode_changed = pyqtSignal(bool)

    def __init__(self, role: str = "operator"):
        super().__init__()
        self._role = role
        v = QVBoxLayout(self)

        def chk(key):
            c = QCheckBox(); c.setChecked(bool(prefs.get(key)))
            c.toggled.connect(lambda on, k=key: prefs.set(k, bool(on)))
            return c

        self._c_pre, self._c_snd, self._c_tower = chk("preflight_enabled"), chk("sound_enabled"), chk("tower_enabled")
        self._c_touch = chk("operator_kiosk")
        self._c_touch.toggled.connect(self.touch_mode_changed)
        self._c_joy = chk("joy_enabled")
        for c in (self._c_pre, self._c_snd, self._c_touch):
            v.addWidget(c)

        # tower light masks (engineer only: they depend on the PLC wiring)
        self._tower_w = QWidget(); tv = QVBoxLayout(self._tower_w); tv.setContentsMargins(0, 0, 0, 0)
        tv.addWidget(self._c_tower)
        self._tower_lbl = QLabel(); self._tower_lbl.setWordWrap(True); self._tower_lbl.setStyleSheet("color:#8B949E;")
        tv.addWidget(self._tower_lbl)
        row = QHBoxLayout()
        for st in ("idle", "running", "paused", "error"):
            sp = QSpinBox(); sp.setRange(0, 15); sp.setValue(int(prefs.get(f"tower_mask_{st}")))
            sp.valueChanged.connect(lambda val, s=st: prefs.set(f"tower_mask_{s}", int(val)))
            row.addWidget(sp)
        tv.addLayout(row)
        v.addWidget(self._tower_w); self._tower_w.setVisible(role == "engineer")

        # gamepad
        self._joy_lbl = QLabel(); self._joy_lbl.setStyleSheet("font-weight:600;margin-top:6px;")
        v.addWidget(self._c_joy); v.addWidget(self._joy_lbl)
        form = QFormLayout()
        self._f = {}
        for key, rng, step, dec in (("joy_deadman_button", (0, 20), 1, 0), ("joy_lin_axis", (0, 10), 1, 0),
                                    ("joy_ang_axis", (0, 10), 1, 0), ("joy_max_lin", (0.05, 1.5), 0.05, 2),
                                    ("joy_max_ang", (0.1, 2.0), 0.1, 2)):
            sp = QDoubleSpinBox(); sp.setRange(*rng); sp.setSingleStep(step); sp.setDecimals(dec)
            sp.setValue(float(prefs.get(key)))
            sp.valueChanged.connect(lambda val, k=key, d=dec: prefs.set(k, int(val) if d == 0 else float(val)))
            lbl = QLabel(); self._f[key] = lbl; form.addRow(lbl, sp)
        v.addLayout(form)

        # backup / restore
        br = QHBoxLayout()
        self._b_backup, self._b_restore = QPushButton(), QPushButton()
        self._b_backup.clicked.connect(self._backup); self._b_restore.clicked.connect(self._restore)
        br.addWidget(self._b_backup); br.addWidget(self._b_restore)
        v.addLayout(br)
        self.retranslate()

    def _backup(self):
        path, _ = QFileDialog.getSaveFileName(self, tr("ss_backup"),
                                              os.path.expanduser("~/" + backup.default_name()), "Zip (*.zip)")
        if path:
            n = backup.make_backup(path)
            QMessageBox.information(self, tr("ss_backup"), tr("ss_backup_done", n, path))

    def _restore(self):
        path, _ = QFileDialog.getOpenFileName(self, tr("ss_restore"), os.path.expanduser("~"), "Zip (*.zip)")
        if not path:
            return
        if QMessageBox.question(self, tr("ss_restore"), tr("ss_restore_confirm")) != QMessageBox.StandardButton.Yes:
            return
        try:
            n = backup.restore_backup(path)
        except Exception as exc:
            QMessageBox.warning(self, tr("ss_restore"), str(exc)); return
        QMessageBox.information(self, tr("ss_restore"), tr("ss_restore_done", n))

    def retranslate(self):
        self.setTitle(tr("ss_title"))
        self._c_pre.setText(tr("ss_preflight")); self._c_snd.setText(tr("ss_sound"))
        self._c_tower.setText(tr("ss_tower")); self._tower_lbl.setText(tr("ss_tower_masks"))
        self._c_touch.setText(tr("ss_touch")); self._c_joy.setText(tr("ss_joy")); self._joy_lbl.setText("")
        for key, tk in (("joy_deadman_button", "ss_joy_btn"), ("joy_lin_axis", "ss_joy_lin"),
                        ("joy_ang_axis", "ss_joy_ang"), ("joy_max_lin", "ss_joy_vmax"), ("joy_max_ang", "ss_joy_wmax")):
            self._f[key].setText(tr(tk))
        self._b_backup.setText(tr("ss_backup")); self._b_restore.setText(tr("ss_restore"))
