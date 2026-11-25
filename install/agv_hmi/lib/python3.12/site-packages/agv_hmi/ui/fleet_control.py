"""fleet_control.py - turn the fleet on and connect robots from the HMI, without typing ros2 commands (4.11.0).

Two services, each with its own start / stop button, status light and log:

* Robot side  : Fleet Agent + domain_bridge of THIS robot   (agv_fleet_domains/robot_fleet_side.launch.py)
* Fleet Server: aggregates every robot into /fleet/snapshot (agv_fleet_domains/fleet_server_domain20.launch.py)

Only the Engineer can start / stop / edit; the Operator sees the status and the logs.
"""
from __future__ import annotations

import subprocess
import time

from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QFrame, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QPlainTextEdit, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

from agv_hmi.core import fleet_config as FC
from agv_hmi.ui.collapsible import attach_group_collapse
from agv_hmi.ui.i18n import tr
from agv_hmi.ui.process_manager import ManagedLaunch

_LEVEL_STYLE = {
    "ok": "border:1px solid #2EA043;background:#1B3629;color:#3FB950;",
    "warn": "border:1px solid #9E6A03;background:#2E2000;color:#E3B341;",
    "bad": "border:1px solid #8B0000;background:#3D1515;color:#F85149;",
    "info": "border:1px solid #30363D;background:#161B22;color:#8B949E;",
}


def _banner(label: QLabel, level: str, text: str) -> None:
    label.setText(text)
    label.setStyleSheet(f"padding:6px 10px;border-radius:6px;font-size:11px;{_LEVEL_STYLE.get(level, _LEVEL_STYLE['info'])}")


def _run_ros2(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    return p.returncode, p.stdout


class _PkgThread(QThread):
    done = pyqtSignal(dict)

    def run(self):
        try:
            self.done.emit(FC.check_packages(_run_ros2))
        except Exception:
            self.done.emit({k: False for k in FC.REQUIRED_PACKAGES})


class FleetControlPanel(QGroupBox):
    robot_id_changed = pyqtSignal(str)          # -> the HMI ROS node (addresses /fleet/map_context)
    status_changed = pyqtSignal(str, str)       # (level, short text) -> title-bar chip

    def __init__(self, parent=None):
        super().__init__(parent)
        self._cfg = FC.load()
        self._engineer = False
        self._pkgs: dict[str, bool] = {}
        self._pkg_thread: _PkgThread | None = None
        self._robots: list[dict] = []
        self._snapshot_t: float | None = None
        self._ext_cache = {"agent": False, "server": False}
        self._tick_n = 0
        self._agent: ManagedLaunch | None = None
        self._server: ManagedLaunch | None = None
        self._build()
        self._rebuild_launchers()
        self._fill_form(self._cfg)
        self.retranslate()
        self.check_packages()
        self._timer = QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._tick)
        self._timer.start()
        attach_group_collapse(self, "fleet.control")

    # -- UI --------------------------------------------------------------------------
    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setSpacing(8)

        self._state_lbl = QLabel(); self._state_lbl.setWordWrap(True)
        root.addWidget(self._state_lbl)

        pk = QHBoxLayout()
        self._pkg_lbl = QLabel(); self._pkg_lbl.setWordWrap(True)
        self._pkg_btn = QPushButton(); self._pkg_btn.clicked.connect(self.check_packages)
        pk.addWidget(self._pkg_lbl, 1); pk.addWidget(self._pkg_btn)
        root.addLayout(pk)
        self._pkg_hint = QLabel(); self._pkg_hint.setWordWrap(True)
        self._pkg_hint.setStyleSheet("color:#E3B341;font-size:11px;")
        self._pkg_hint.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        root.addWidget(self._pkg_hint)

        cols = QHBoxLayout()
        cols.setSpacing(10)
        cols.addWidget(self._build_robot_frame(), 1)
        cols.addWidget(self._build_server_frame(), 1)
        root.addLayout(cols)

        self._dom_lbl = QLabel(); self._dom_lbl.setWordWrap(True)
        root.addWidget(self._dom_lbl)
        self._msg_lbl = QLabel(); self._msg_lbl.setWordWrap(True)
        self._msg_lbl.setStyleSheet("color:#F85149;font-size:11px;")
        root.addWidget(self._msg_lbl)

        lh = QHBoxLayout()
        self._log_title = QLabel()
        self._log_combo = QComboBox()
        self._log_combo.addItem("", "agent"); self._log_combo.addItem("", "server")
        self._log_combo.currentIndexChanged.connect(lambda _i: self._refresh_log(force=True))
        lh.addWidget(self._log_title); lh.addWidget(self._log_combo); lh.addStretch(1)
        root.addLayout(lh)
        self._log = QPlainTextEdit(); self._log.setReadOnly(True); self._log.setMaximumHeight(120)
        self._log.setStyleSheet("font-family:monospace;font-size:11px;")
        root.addWidget(self._log)
        self._last_log = ""

    def _frame(self) -> tuple[QFrame, QVBoxLayout]:
        fr = QFrame()
        fr.setFrameShape(QFrame.Shape.StyledPanel)
        lay = QVBoxLayout(fr)
        return fr, lay

    def _build_robot_frame(self) -> QFrame:
        fr, lay = self._frame()
        self._robot_title = QLabel(); self._robot_title.setStyleSheet("font-weight:700;")
        lay.addWidget(self._robot_title)
        form = QFormLayout()
        self._f_id, self._f_name = QLineEdit(), QLineEdit()
        self._f_id.setMaxLength(32)
        self._f_rdom, self._f_fdom = QSpinBox(), QSpinBox()
        for sp in (self._f_rdom, self._f_fdom):
            sp.setRange(0, FC.MAX_DOMAIN)
        self._f_ns = QLineEdit()
        self._f_pub = QDoubleSpinBox(); self._f_pub.setRange(0.5, 20.0); self._f_pub.setSingleStep(0.5)
        self._lbl = {k: QLabel() for k in ("id", "name", "rdom", "fdom", "ns", "pub")}
        for k, w in (("id", self._f_id), ("name", self._f_name), ("rdom", self._f_rdom), ("fdom", self._f_fdom),
                     ("ns", self._f_ns), ("pub", self._f_pub)):
            form.addRow(self._lbl[k], w)
        lay.addLayout(form)
        self._f_auto_agent = QCheckBox()
        lay.addWidget(self._f_auto_agent)
        row = QHBoxLayout()
        self._agent_led = QLabel("●"); self._agent_state = QLabel()
        self._agent_btn = QPushButton(); self._agent_btn.setFixedHeight(34)
        self._agent_btn.clicked.connect(self._on_agent_clicked)
        row.addWidget(self._agent_led); row.addWidget(self._agent_state, 1); row.addWidget(self._agent_btn)
        lay.addLayout(row)
        lay.addStretch(1)
        return fr

    def _build_server_frame(self) -> QFrame:
        fr, lay = self._frame()
        self._server_title = QLabel(); self._server_title.setStyleSheet("font-weight:700;")
        lay.addWidget(self._server_title)
        form = QFormLayout()
        self._f_shz = QDoubleSpinBox(); self._f_shz.setRange(0.5, 20.0); self._f_shz.setSingleStep(0.5)
        self._f_warn = QDoubleSpinBox(); self._f_warn.setRange(0.2, 60.0); self._f_warn.setSingleStep(0.1)
        self._f_off = QDoubleSpinBox(); self._f_off.setRange(0.3, 120.0); self._f_off.setSingleStep(0.1)
        self._slbl = {k: QLabel() for k in ("shz", "warn", "off")}
        for k, w in (("shz", self._f_shz), ("warn", self._f_warn), ("off", self._f_off)):
            form.addRow(self._slbl[k], w)
        lay.addLayout(form)
        self._f_auto_server = QCheckBox()
        lay.addWidget(self._f_auto_server)
        self._server_hint = QLabel(); self._server_hint.setWordWrap(True)
        self._server_hint.setStyleSheet("color:#8B949E;font-size:11px;")
        lay.addWidget(self._server_hint)
        row = QHBoxLayout()
        self._server_led = QLabel("●"); self._server_state = QLabel()
        self._server_btn = QPushButton(); self._server_btn.setFixedHeight(34)
        self._server_btn.clicked.connect(self._on_server_clicked)
        row.addWidget(self._server_led); row.addWidget(self._server_state, 1); row.addWidget(self._server_btn)
        lay.addLayout(row)
        lay.addStretch(1)
        return fr

    # -- settings <-> form -------------------------------------------------------------
    def _fill_form(self, c: FC.FleetConfig) -> None:
        self._f_id.setText(c.robot_id); self._f_name.setText(c.robot_name)
        self._f_rdom.setValue(c.robot_domain); self._f_fdom.setValue(c.fleet_domain)
        self._f_ns.setText(c.source_namespace); self._f_pub.setValue(c.publish_hz)
        self._f_shz.setValue(c.snapshot_hz); self._f_warn.setValue(c.warning_timeout_sec)
        self._f_off.setValue(c.offline_timeout_sec)
        self._f_auto_agent.setChecked(c.auto_connect); self._f_auto_server.setChecked(c.auto_server)

    def _cfg_from_form(self) -> FC.FleetConfig:
        return FC.FleetConfig.from_dict({
            "robot_id": self._f_id.text(), "robot_name": self._f_name.text(),
            "robot_domain": self._f_rdom.value(), "fleet_domain": self._f_fdom.value(),
            "source_namespace": self._f_ns.text(), "publish_hz": self._f_pub.value(),
            "snapshot_hz": self._f_shz.value(), "warning_timeout_sec": self._f_warn.value(),
            "offline_timeout_sec": self._f_off.value(),
            "auto_connect": self._f_auto_agent.isChecked(), "auto_server": self._f_auto_server.isChecked()})

    def _rebuild_launchers(self) -> None:
        """The agent command contains the robot id, so every setting change builds a fresh launcher (an old
        one that is still running keeps being tracked until it is stopped)."""
        c = self._cfg
        if self._agent is not None and self._agent.is_running():
            return                                   # never replace a running launcher: stop it first
        self._agent = ManagedLaunch(
            "Fleet Agent", FC.agent_cmd(c), log_path=f"{FC.LOG_DIR}/fleet_agent.log",
            signature=f"ros2 launch agv_fleet_domains {FC.GENERIC_LAUNCH} robot_id:={c.robot_id} ")
        if self._server is None or not self._server.is_running():
            self._server = ManagedLaunch(
                "Fleet Server", FC.server_cmd(c), log_path=f"{FC.LOG_DIR}/fleet_server.log",
                signature=f"ros2 launch agv_fleet_domains {FC.SERVER_LAUNCH}")

    # -- public ------------------------------------------------------------------------
    def set_engineer(self, on: bool) -> None:
        self._engineer = bool(on)
        self._apply_enabled()

    def set_snapshot(self, robots: list) -> None:
        self._robots = [r for r in robots if isinstance(r, dict)]
        self._snapshot_t = time.monotonic()

    def auto_start(self) -> None:
        """Called once after the HMI started: bring up what the Engineer asked to start automatically."""
        if self._cfg.auto_server and not self._server.is_active():
            self._start_server(quiet=True)
        if self._cfg.auto_connect and not self._agent.is_active():
            self._start_agent(quiet=True)

    def check_packages(self) -> None:
        if self._pkg_thread is not None and self._pkg_thread.isRunning():
            return
        self._pkg_lbl.setText(tr("fc_pkg_checking"))
        self._pkg_thread = _PkgThread(self)
        self._pkg_thread.done.connect(self._on_pkgs)
        self._pkg_thread.start()

    # -- actions -----------------------------------------------------------------------
    def _precheck(self, cfg: FC.FleetConfig) -> bool:
        errs = FC.validate(cfg)
        if errs:
            key, args = errs[0]
            self._msg_lbl.setText("❌ " + tr(key, *args))
            return False
        missing = [k for k, ok in self._pkgs.items() if not ok]
        if missing:
            self._msg_lbl.setText("❌ " + tr("fc_err_pkg", ", ".join(missing)))
            return False
        self._msg_lbl.setText("")
        return True

    def _save_cfg(self, cfg: FC.FleetConfig) -> None:
        self._cfg = cfg
        FC.save(cfg)
        FC.remember_robot_id(cfg.robot_id)
        self.robot_id_changed.emit(cfg.robot_id)

    def _on_agent_clicked(self) -> None:
        if self._agent.is_active():
            self._agent.stop()
            self._ext_cache["agent"] = False
            self._tick()
            return
        self._start_agent()

    def _start_agent(self, quiet: bool = False) -> None:
        cfg = self._cfg_from_form() if not quiet else self._cfg
        if not self._precheck(cfg):
            return
        self._save_cfg(cfg)
        self._rebuild_launchers()
        ok, msg = self._agent.start()
        if not ok:
            self._msg_lbl.setText("❌ " + msg)
        self._log_combo.setCurrentIndex(0)
        self._tick()

    def _on_server_clicked(self) -> None:
        if self._server.is_active():
            self._server.stop()
            self._ext_cache["server"] = False
            self._tick()
            return
        self._start_server()

    def _start_server(self, quiet: bool = False) -> None:
        cfg = self._cfg_from_form() if not quiet else self._cfg
        if not self._precheck(cfg):
            return
        self._cfg = cfg
        FC.save(cfg)
        self._server = ManagedLaunch("Fleet Server", FC.server_cmd(cfg), log_path=f"{FC.LOG_DIR}/fleet_server.log",
                                     signature=f"ros2 launch agv_fleet_domains {FC.SERVER_LAUNCH}")
        ok, msg = self._server.start()
        if not ok:
            self._msg_lbl.setText("❌ " + msg)
        self._log_combo.setCurrentIndex(1)
        self._tick()

    # -- periodic refresh ------------------------------------------------------------------
    def _on_pkgs(self, result: dict) -> None:
        self._pkgs = dict(result)
        parts = [("✅ " if ok else "❌ ") + name for name, ok in self._pkgs.items()]
        self._pkg_lbl.setText(tr("fc_pkg") + ":  " + "   ".join(parts))
        hints = []
        if not self._pkgs.get("agv_fleet") or not self._pkgs.get("agv_fleet_domains"):
            hints.append(tr("fc_pkg_fix_build"))
        if not self._pkgs.get("domain_bridge"):
            hints.append(tr("fc_pkg_fix_apt"))
        self._pkg_hint.setText("\n".join(hints))
        self._pkg_hint.setVisible(bool(hints))

    def _active(self, which: str, launcher: ManagedLaunch) -> bool:
        if launcher.is_running():
            return True
        if self._tick_n % 3 == 0:                      # looking for a leftover process costs a pgrep: every 3 s
            self._ext_cache[which] = bool(launcher.external_pids())
        return self._ext_cache[which]

    def _tick(self) -> None:
        self._tick_n += 1
        agent_on = self._active("agent", self._agent)
        server_on = self._active("server", self._server)
        self._paint_service("agent", self._agent, agent_on, self._agent_led, self._agent_state, self._agent_btn,
                            tr("fc_btn_disconnect"), tr("fc_btn_connect"))
        self._paint_service("server", self._server, server_on, self._server_led, self._server_state, self._server_btn,
                            tr("fc_btn_server_off"), tr("fc_btn_server_on"))
        age = None if self._snapshot_t is None else time.monotonic() - self._snapshot_t
        level, key, args = FC.explain_state(agent_on, server_on, age, self._robots, self._cfg.robot_id)
        text = tr(key, *args)
        _banner(self._state_lbl, level, text)
        hint_level, hint_key, hint_args = FC.domain_hints(self._cfg_from_form(), FC.hmi_domain())[0]
        _banner(self._dom_lbl, hint_level, tr(hint_key, *hint_args))
        self._refresh_log()
        self.status_changed.emit(level, self._chip_text(key, args, agent_on, server_on))

    def _paint_service(self, which, launcher, on, led, state, btn, off_text, on_text) -> None:
        died = launcher.proc is not None and launcher.proc.poll() is not None
        if died:
            led.setStyleSheet("color:#F85149;font-size:16px;")
            state.setText(tr("fc_run_died", launcher.proc.returncode))
        elif on:
            ext = not launcher.is_running()
            led.setStyleSheet("color:#3FB950;font-size:16px;")
            state.setText(tr("fc_run_ext") if ext else tr("fc_run_on"))
        else:
            led.setStyleSheet("color:#8B949E;font-size:16px;")
            state.setText(tr("fc_run_off"))
        btn.setText(off_text if on else on_text)

    def _chip_text(self, key: str, args: tuple, agent_on: bool, server_on: bool) -> str:
        if key == "fc_st_connected":
            return tr("chip_fleet_ok", args[1])
        if key == "fc_st_snapshot":
            return tr("chip_fleet_ok", args[0])
        if key == "fc_st_nothing":
            return tr("chip_fleet_off")
        return tr("chip_fleet_wait")

    def _refresh_log(self, force: bool = False) -> None:
        launcher = self._agent if self._log_combo.currentData() == "agent" else self._server
        text = launcher.tail(80) or tr("fc_log_empty")
        if text != self._last_log or force:
            self._last_log = text
            self._log.setPlainText(text)
            sb = self._log.verticalScrollBar()
            sb.setValue(sb.maximum())

    def _apply_enabled(self) -> None:
        on = self._engineer
        for w in (self._f_id, self._f_name, self._f_rdom, self._f_fdom, self._f_ns, self._f_pub, self._f_shz,
                  self._f_warn, self._f_off, self._f_auto_agent, self._f_auto_server, self._agent_btn,
                  self._server_btn, self._pkg_btn):
            w.setEnabled(on)

    # -- i18n --------------------------------------------------------------------------------
    def retranslate(self) -> None:
        self.setTitle(tr("fc_title"))
        self._pkg_btn.setText(tr("fc_pkg_check"))
        if self._pkgs:
            self._on_pkgs(self._pkgs)
        self._robot_title.setText(tr("fc_robot_box")); self._server_title.setText(tr("fc_server_box"))
        for k, key in (("id", "fc_robot_id"), ("name", "fc_robot_name"), ("rdom", "fc_robot_domain"),
                       ("fdom", "fc_fleet_domain"), ("ns", "fc_namespace"), ("pub", "fc_publish_hz")):
            self._lbl[k].setText(tr(key))
        for k, key in (("shz", "fc_snapshot_hz"), ("warn", "fc_warn_to"), ("off", "fc_off_to")):
            self._slbl[k].setText(tr(key))
        self._f_auto_agent.setText(tr("fc_auto_connect")); self._f_auto_server.setText(tr("fc_auto_server"))
        self._server_hint.setText(tr("fc_server_hint"))
        self._log_title.setText(tr("fc_log"))
        self._log_combo.setItemText(0, tr("fc_log_agent")); self._log_combo.setItemText(1, tr("fc_log_server"))
        self._apply_enabled()
        self._tick()
