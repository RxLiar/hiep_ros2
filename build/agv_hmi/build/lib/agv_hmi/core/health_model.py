"""health_model.py - one place that knows whether the robot is ready to run.

The MainWindow feeds it from the existing ROS signals; the pre-flight dialog,
dashboard and alarm rules only read from it.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field


@dataclass
class Check:
    key: str
    label_key: str          # i18n key
    ok: bool
    detail: str = ""
    blocking: bool = True   # False = warning only


class HealthModel:
    def __init__(self):
        self.rates: dict[str, float] = {}          # topic -> Hz
        self.plc_online: bool | None = None
        self._plc_t = 0.0
        self.plc_ever = False
        self.driver_ever = False
        self.estop: bool = False
        self.driver: dict = {}
        self._driver_t = 0.0
        self.battery_pct: int | None = None
        self.amcl_sigma_m: float | None = None     # sqrt(max(var_x, var_y))
        self.pose_seen_t = 0.0
        self.ros_online: bool = False
        self.safety_state: str | None = None
        self._safety_t = 0.0
        self.safety_ever = False

    # -- feeders --------------------------------------------------------------
    def set_driver_payload(self, payload: str):
        try:
            d = json.loads(payload or "{}")
            if isinstance(d, dict):
                self.driver = d
                self._driver_t = time.monotonic()
                self.driver_ever = True
        except ValueError:
            pass

    def set_plc(self, online: bool):
        self.plc_online = bool(online)
        self.plc_ever = True
        self._plc_t = time.monotonic()

    def plc_alive(self) -> bool:
        return self.plc_online is True and time.monotonic() - self._plc_t < 5.0

    def driver_fresh(self) -> bool:
        return time.monotonic() - self._driver_t < 3.0

    def set_safety(self, state: str):
        self.safety_state = state
        self._safety_t = time.monotonic()
        self.safety_ever = True

    def safety_fresh(self) -> bool:
        return time.monotonic() - self._safety_t < 3.0

    def set_amcl_cov(self, sigma_m: float):
        self.amcl_sigma_m = sigma_m
        self.pose_seen_t = time.monotonic()

    # -- evaluation -----------------------------------------------------------
    def checks(self, min_scan_hz: float = 3.0, max_sigma_m: float = 0.35,
               min_battery_pct: int = 15) -> list[Check]:
        out: list[Check] = []
        out.append(Check("ros", "pf_ros", self.ros_online))
        scan = self.rates.get("/scan", 0.0)
        out.append(Check("lidar", "pf_lidar", scan >= min_scan_hz, f"{scan:.1f} Hz"))
        # PLC / e-stop and the KEYA driver only count once they have been seen,
        # so the ESP32 test model (no PLC, no KEYA) is not blocked by them.
        if self.plc_ever:
            out.append(Check("estop", "pf_estop", self.plc_alive() and not self.estop,
                             "E-STOP" if self.estop else ("" if self.plc_alive() else "PLC offline")))
        if self.driver_ever:
            if self.driver_fresh():
                d = self.driver
                fault = [int(x) for x in (d.get("fault") or [0, 0])]
                out.append(Check("driver", "pf_driver", bool(d.get("online")) and not any(fault),
                                 "" if d.get("online") else "offline"))
            else:
                out.append(Check("driver", "pf_driver", False, "no /keya_driver_status"))
        if self.safety_ever:
            ok = self.safety_fresh() and self.safety_state != "no_scan"
            out.append(Check("safety", "pf_safety", ok, "" if ok else ("no_scan" if self.safety_fresh() else "no /safety_state")))
        sig = self.amcl_sigma_m
        fresh = time.monotonic() - self.pose_seen_t < 5.0
        out.append(Check("loc", "pf_loc", fresh and sig is not None and sig <= max_sigma_m,
                         "no pose" if not fresh or sig is None else f"σ={sig:.2f} m"))
        if self.battery_pct is not None:
            out.append(Check("battery", "pf_battery", self.battery_pct >= min_battery_pct,
                             f"{self.battery_pct}%", blocking=False))
        return out
