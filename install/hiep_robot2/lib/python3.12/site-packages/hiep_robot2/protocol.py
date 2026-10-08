"""ASCII serial protocol shared by ROS 2 hardware bridge and ESP32.

ROS -> ESP32:
    CMD,<seq>,<left_mm_s>,<right_mm_s>\n

ESP32 -> ROS:
    TEL,<seq>,<mcu_ms>,<enc_l>,<enc_r>,<vel_l_mm_s>,<vel_r_mm_s>\n

The first version intentionally stays human-readable so it is easy to debug
with a serial terminal. A binary/checksummed protocol can replace this later
without changing the ROS API.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Telemetry:
    sequence: int
    mcu_time_ms: int
    encoder_left: int
    encoder_right: int
    velocity_left_mps: float
    velocity_right_mps: float


@dataclass(frozen=True)
class DeviceIdentity:
    device_id: str
    firmware_version: str


def parse_device_identity(line: str) -> DeviceIdentity:
    """Parse an ESP32 identity line: ID,<device_id>,<firmware_version>."""
    parts = line.strip().split(",")
    if len(parts) != 3 or parts[0] != "ID":
        raise ValueError("unsupported identity frame")
    return DeviceIdentity(device_id=parts[1], firmware_version=parts[2])


def encode_velocity_command(sequence: int, left_mps: float, right_mps: float) -> bytes:
    """Encode wheel velocity targets using integer mm/s."""
    left_mm_s = int(round(left_mps * 1000.0))
    right_mm_s = int(round(right_mps * 1000.0))
    return f"CMD,{sequence},{left_mm_s},{right_mm_s}\n".encode("ascii")


def parse_telemetry(line: str) -> Telemetry:
    """Parse one ESP32 telemetry line.

    Raises ValueError when the frame is malformed. Keeping validation here
    keeps serial parsing separate from ROS logic.
    """
    parts = line.strip().split(",")
    if len(parts) != 7 or parts[0] != "TEL":
        raise ValueError("unsupported frame")

    return Telemetry(
        sequence=int(parts[1]),
        mcu_time_ms=int(parts[2]),
        encoder_left=int(parts[3]),
        encoder_right=int(parts[4]),
        velocity_left_mps=int(parts[5]) / 1000.0,
        velocity_right_mps=int(parts[6]) / 1000.0,
    )
