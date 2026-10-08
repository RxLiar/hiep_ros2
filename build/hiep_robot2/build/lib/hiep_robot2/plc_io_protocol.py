"""ASCII protocol for the Mega2560 PLC I/O controller.

Host -> PLC
-----------
CV,<seq>,<id>,<mode>,<speed_pct>,<duration_ms>\n
    mode: R=receive, S=send, X=stop
LED,<seq>,<mask>\n
STOPALL,<seq>\n

PLC -> Host
-----------
ID,AGV_PLC_MEGA2560,<fw_version>\n
PSTAT,<seq>,<uptime_ms>,<sensor_mask>,<bumper_mask>,<emg>,<start>,<stop>,
      <cargo_mask>,<running_mask>,<fault_mask>,<led_mask>,<m1>,<m2>,<m3>,<m4>\n

Masks are decimal integers. Bit 0 is item 1.
Mode codes: 0=idle, 1=receive, 2=send, 3=fault.
"""

from dataclasses import dataclass


from typing import Optional


PLC_DEVICE_ID = "AGV_PLC_MEGA2560"
MODE_IDLE = 0
MODE_RECEIVE = 1
MODE_SEND = 2
MODE_FAULT = 3

_MODE_NAMES = {
    MODE_IDLE: "idle",
    MODE_RECEIVE: "receive",
    MODE_SEND: "send",
    MODE_FAULT: "fault",
}


@dataclass(frozen=True)
class PlcStatus:
    sequence: int
    uptime_ms: int
    sensor_mask: int
    bumper_mask: int
    emergency_stop: bool
    start_button: bool
    stop_button: bool
    cargo_mask: int
    running_mask: int
    fault_mask: int
    led_mask: int
    modes: tuple[int, int, int, int]

    def sensor(self, sensor_id: int) -> bool:
        if not 1 <= sensor_id <= 8:
            raise ValueError("sensor_id must be 1..8")
        return bool(self.sensor_mask & (1 << (sensor_id - 1)))

    def bumper(self, side: str) -> bool:
        side = side.lower().strip()
        if side == "left":
            return bool(self.bumper_mask & 0x01)
        if side == "right":
            return bool(self.bumper_mask & 0x02)
        raise ValueError("side must be 'left' or 'right'")

    def cargo(self, conveyor_id: int) -> bool:
        if not 1 <= conveyor_id <= 4:
            raise ValueError("conveyor_id must be 1..4")
        return bool(self.cargo_mask & (1 << (conveyor_id - 1)))

    def running(self, conveyor_id: int) -> bool:
        if not 1 <= conveyor_id <= 4:
            raise ValueError("conveyor_id must be 1..4")
        return bool(self.running_mask & (1 << (conveyor_id - 1)))

    def fault(self, conveyor_id: int) -> bool:
        if not 1 <= conveyor_id <= 4:
            raise ValueError("conveyor_id must be 1..4")
        return bool(self.fault_mask & (1 << (conveyor_id - 1)))

    def mode_name(self, conveyor_id: int) -> str:
        if not 1 <= conveyor_id <= 4:
            raise ValueError("conveyor_id must be 1..4")
        return _MODE_NAMES.get(self.modes[conveyor_id - 1], "unknown")


def _line(text: str) -> bytes:
    return (text + "\n").encode("ascii")


def encode_conveyor_command(
    sequence: int,
    conveyor_id: int,
    mode: str,
    speed_percent: int,
    duration_seconds: float,
) -> bytes:
    if not 1 <= conveyor_id <= 4:
        raise ValueError("conveyor_id must be 1..4")

    mode_key = mode.lower().strip()
    mode_code = {
        "receive": "R",
        "send": "S",
        "stop": "X",
    }.get(mode_key)
    if mode_code is None:
        raise ValueError("mode must be receive, send, or stop")

    speed = max(0, min(100, int(speed_percent)))
    duration_ms = max(0, int(round(float(duration_seconds) * 1000.0)))
    if mode_code == "X":
        speed = 0
        duration_ms = 0

    return _line(f"CV,{int(sequence)},{conveyor_id},{mode_code},{speed},{duration_ms}")


def encode_light_command(sequence: int, mask: int) -> bytes:
    return _line(f"LED,{int(sequence)},{int(mask) & 0x0F}")


def encode_stop_all(sequence: int) -> bytes:
    return _line(f"STOPALL,{int(sequence)}")


def parse_device_identity(line: str) -> Optional[tuple[str, str]]:
    fields = line.strip().split(",")
    if len(fields) != 3 or fields[0] != "ID":
        return None
    return fields[1], fields[2]


def parse_plc_status(line: str) -> Optional[PlcStatus]:
    fields = line.strip().split(",")
    if len(fields) != 16 or fields[0] != "PSTAT":
        return None

    try:
        values = [int(value) for value in fields[1:]]
    except ValueError:
        return None

    return PlcStatus(
        sequence=values[0],
        uptime_ms=values[1],
        sensor_mask=values[2] & 0xFF,
        bumper_mask=values[3] & 0x03,
        emergency_stop=bool(values[4]),
        start_button=bool(values[5]),
        stop_button=bool(values[6]),
        cargo_mask=values[7] & 0x0F,
        running_mask=values[8] & 0x0F,
        fault_mask=values[9] & 0x0F,
        led_mask=values[10] & 0x0F,
        modes=(values[11], values[12], values[13], values[14]),
    )
