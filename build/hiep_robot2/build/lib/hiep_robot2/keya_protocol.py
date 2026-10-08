"""KEYA KYDAS serial protocol helpers.

Implemented from the KYDAS4850-1E V1.8 serial command documentation:
  * Fixed 12-byte binary frames.
  * Control frame starts with 0xE0.
  * Query frame/response starts with 0xED.
  * Unsolicited heartbeat starts with 0xEE.
  * Speed setpoint range -10000..+10000 corresponds to
    -rated_speed_rpm..+rated_speed_rpm.

This module intentionally contains no ROS code.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct

FRAME_LEN = 12
CONTROL_START = 0xE0
QUERY_START = 0xED
HEARTBEAT_START = 0xEE

QUERY_CONTROL_STATUS = 0x00
QUERY_SPEED = 0x02
QUERY_CURRENT = 0x03
QUERY_VOLTAGE = 0x05
QUERY_TEMPERATURE = 0x06
QUERY_FAULT = 0x07
QUERY_ENCODER = 0x08
QUERY_VERSION = 0x09


@dataclass(frozen=True)
class KeyaEncoder:
    motor1_count: int
    motor2_count: int


@dataclass(frozen=True)
class KeyaSpeed:
    motor1_rpm: int
    motor2_rpm: int


@dataclass(frozen=True)
class KeyaFault:
    motor1_code: int
    motor2_code: int


def _clamp_command(value: int) -> int:
    return max(-10000, min(10000, int(value)))


def build_speed_frame(motor1_command: int, motor2_command: int, enable_mask: int = 0x03) -> bytes:
    """Build the documented 12-byte E0 control frame.

    enable_mask:
      0x00 = disable both
      0x01 = enable motor 1
      0x02 = enable motor 2
      0x03 = enable both
    """
    if enable_mask not in (0x00, 0x01, 0x02, 0x03):
        raise ValueError("enable_mask must be 0x00..0x03")

    m1 = _clamp_command(motor1_command)
    m2 = _clamp_command(motor2_command)
    return bytes((CONTROL_START, enable_mask, 0x00, 0x00)) + struct.pack(">ii", m1, m2)


def build_disable_frame() -> bytes:
    return build_speed_frame(0, 0, enable_mask=0x00)


def build_query_frame(query_code: int) -> bytes:
    if not 0 <= int(query_code) <= 0xFF:
        raise ValueError("query_code must fit in one byte")
    return bytes((QUERY_START, int(query_code))) + bytes(10)


def speed_rpm_to_command(rpm: float, rated_speed_rpm: float) -> int:
    if rated_speed_rpm <= 0.0:
        raise ValueError("rated_speed_rpm must be > 0")
    return _clamp_command(round((float(rpm) / float(rated_speed_rpm)) * 10000.0))


def command_to_speed_rpm(command: int, rated_speed_rpm: float) -> float:
    return (_clamp_command(command) / 10000.0) * float(rated_speed_rpm)


def parse_encoder_response(frame: bytes) -> KeyaEncoder:
    _validate_query_response(frame, QUERY_ENCODER)
    m1, m2 = struct.unpack(">ii", frame[2:10])
    return KeyaEncoder(m1, m2)


def parse_speed_response(frame: bytes) -> KeyaSpeed:
    _validate_query_response(frame, QUERY_SPEED)
    m1, m2 = struct.unpack(">hh", frame[2:6])
    return KeyaSpeed(m1, m2)


def parse_fault_response(frame: bytes) -> KeyaFault:
    _validate_query_response(frame, QUERY_FAULT)
    # Manual examples encode each motor fault in two bytes.
    m1, m2 = struct.unpack(">HH", frame[2:6])
    return KeyaFault(m1, m2)


def parse_voltage_response(frame: bytes) -> int:
    _validate_query_response(frame, QUERY_VOLTAGE)
    return int(frame[2])


def parse_temperature_response(frame: bytes) -> int:
    _validate_query_response(frame, QUERY_TEMPERATURE)
    # Manual example: ED 06 00 22 ... => 0x22 = 34 C.
    return int(frame[3])


def _validate_query_response(frame: bytes, expected_code: int) -> None:
    if len(frame) != FRAME_LEN:
        raise ValueError(f"KEYA frame must be {FRAME_LEN} bytes")
    if frame[0] != QUERY_START:
        raise ValueError("not a KEYA query response")
    if frame[1] != expected_code:
        raise ValueError(f"unexpected query response code 0x{frame[1]:02X}")
