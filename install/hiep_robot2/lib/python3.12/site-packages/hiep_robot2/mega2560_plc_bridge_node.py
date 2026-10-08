#!/usr/bin/env python3

import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

import serial
import serial.tools.list_ports

from .plc_io_protocol import (
    PLC_DEVICE_ID,
    PlcStatus,
    encode_conveyor_command,
    encode_light_command,
    encode_stop_all,
    parse_device_identity,
    parse_plc_status,
)


class Mega2560PlcBridgeNode(Node):
    """ROS 2 <-> Mega2560 PLC bridge for conveyor and robot I/O.

    PLC responsibilities:
      * 4 conveyors.
      * 2 sensors per conveyor (8 sensors total).
      * left/right bumper inputs.
      * EMG, START, STOP inputs.
      * 4 signal-light outputs (2 per side).

    This node is deliberately independent from the traction backend. It can run
    together with either esp32_bridge_node or keya_driver_bridge_node.
    """

    def __init__(self) -> None:
        super().__init__("mega2560_plc_bridge")

        # Serial / discovery.
        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.05)
        self.declare_parameter("auto_detect_serial", True)
        self.declare_parameter("probe_timeout", 1.5)
        self.declare_parameter("expected_device_id", PLC_DEVICE_ID)
        self.declare_parameter("preferred_usb_serial", "")
        self.declare_parameter("reconnect_period", 2.0)
        self.declare_parameter("telemetry_timeout", 0.8)

        # ROS topics.
        self.declare_parameter("conveyor_cmd_topic", "/conveyor_cmd")
        self.declare_parameter("light_cmd_topic", "/light_cmd")
        self.declare_parameter("sensor_states_topic", "/sensor_states")
        self.declare_parameter("conveyor_cargo_topic", "/conveyor_cargo")
        self.declare_parameter("conveyor_status_topic", "/conveyor_status")
        self.declare_parameter("bumper_states_topic", "/bumper_states")
        self.declare_parameter("emergency_stop_topic", "/emergency_stop")
        self.declare_parameter("start_button_topic", "/start_button")
        self.declare_parameter("stop_button_topic", "/stop_button")
        self.declare_parameter("plc_status_topic", "/plc_status")
        self.declare_parameter("plc_connection_topic", "/plc_connection_status")
        self.declare_parameter("signal_light_states_topic", "/signal_light_states")

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout").value)
        self.auto_detect_serial = bool(self.get_parameter("auto_detect_serial").value)
        self.probe_timeout = float(self.get_parameter("probe_timeout").value)
        self.expected_device_id = str(self.get_parameter("expected_device_id").value)
        self.preferred_usb_serial = str(self.get_parameter("preferred_usb_serial").value).strip()
        self.reconnect_period = float(self.get_parameter("reconnect_period").value)
        self.telemetry_timeout = float(self.get_parameter("telemetry_timeout").value)

        self.conveyor_cmd_topic = str(self.get_parameter("conveyor_cmd_topic").value)
        self.light_cmd_topic = str(self.get_parameter("light_cmd_topic").value)
        self.sensor_states_topic = str(self.get_parameter("sensor_states_topic").value)
        self.conveyor_cargo_topic = str(self.get_parameter("conveyor_cargo_topic").value)
        self.conveyor_status_topic = str(self.get_parameter("conveyor_status_topic").value)
        self.bumper_states_topic = str(self.get_parameter("bumper_states_topic").value)
        self.emergency_stop_topic = str(self.get_parameter("emergency_stop_topic").value)
        self.start_button_topic = str(self.get_parameter("start_button_topic").value)
        self.stop_button_topic = str(self.get_parameter("stop_button_topic").value)
        self.plc_status_topic = str(self.get_parameter("plc_status_topic").value)
        self.plc_connection_topic = str(self.get_parameter("plc_connection_topic").value)
        self.signal_light_states_topic = str(self.get_parameter("signal_light_states_topic").value)

        # ROS input.
        self.create_subscription(String, self.conveyor_cmd_topic, self._conveyor_cmd_callback, 20)
        self.create_subscription(String, self.light_cmd_topic, self._light_cmd_callback, 10)

        # ROS output. Existing HMI already consumes conveyor_cargo,
        # sensor_states and bumper_states.
        self.sensor_pub = self.create_publisher(String, self.sensor_states_topic, 20)
        self.cargo_pub = self.create_publisher(String, self.conveyor_cargo_topic, 10)
        self.conveyor_status_pub = self.create_publisher(String, self.conveyor_status_topic, 10)
        self.bumper_pub = self.create_publisher(String, self.bumper_states_topic, 10)
        self.emg_pub = self.create_publisher(Bool, self.emergency_stop_topic, 10)
        self.start_pub = self.create_publisher(Bool, self.start_button_topic, 10)
        self.stop_pub = self.create_publisher(Bool, self.stop_button_topic, 10)
        self.plc_status_pub = self.create_publisher(String, self.plc_status_topic, 10)
        self.connection_pub = self.create_publisher(String, self.plc_connection_topic, 10)
        self.light_states_pub = self.create_publisher(String, self.signal_light_states_topic, 10)

        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._connect_lock = threading.Lock()
        self._active_port: Optional[str] = None
        self._last_connect_attempt = 0.0
        self._last_status_monotonic: Optional[float] = None
        self._running = True
        self._was_online: Optional[bool] = None
        self._tx_sequence = 0

        self._last_status: Optional[PlcStatus] = None
        self._refresh_tick = 0

        self._reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self._reader_thread.start()
        self.create_timer(0.2, self._health_timer_callback)

        self.get_logger().info(
            "Mega2560 PLC bridge ready: "
            f"conveyors=4, sensors=8, serial={'AUTO' if self.auto_detect_serial else self.serial_port}, "
            f"{self.baudrate} baud"
        )

    # ------------------------------------------------------------------
    # ROS -> PLC
    # ------------------------------------------------------------------
    def _next_sequence(self) -> int:
        self._tx_sequence = (self._tx_sequence + 1) & 0x7FFFFFFF
        return self._tx_sequence

    def _conveyor_cmd_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warning(f"Invalid /conveyor_cmd JSON: {exc}")
            return

        # Existing HMI has placeholder mission commands {"type":"buzzer"}
        # and {"type":"io"}. They are intentionally ignored by this v0.3
        # bridge rather than being interpreted as conveyor movement.
        if "type" in payload and "conveyor_id" not in payload:
            self.get_logger().info(f"Ignoring unsupported HMI auxiliary command: {payload.get('type')}")
            return

        try:
            conveyor_id = int(payload["conveyor_id"])
            mode = str(payload.get("mode", "stop")).lower().strip()
            speed = int(payload.get("speed", 0))
            duration = float(payload.get("duration", 0.0))
            frame = encode_conveyor_command(
                self._next_sequence(), conveyor_id, mode, speed, duration
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid conveyor command {payload!r}: {exc}")
            return

        if self._write_serial(frame):
            self.get_logger().info(
                f"PLC conveyor {conveyor_id}: mode={mode}, speed={max(0, min(100, speed))}%, "
                f"duration={max(0.0, duration):.1f}s"
            )

    def _light_cmd_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if "mask" in payload:
                mask = int(payload["mask"]) & 0x0F
            else:
                mask = 0
                names = ("left_1", "left_2", "right_1", "right_2")
                for bit, name in enumerate(names):
                    if bool(payload.get(name, False)):
                        mask |= 1 << bit
            self._write_serial(encode_light_command(self._next_sequence(), mask))
        except Exception as exc:
            self.get_logger().warning(f"Invalid /light_cmd payload: {exc}")

    # ------------------------------------------------------------------
    # Serial discovery / connection
    # ------------------------------------------------------------------
    def _candidate_ports(self):
        ports = list(serial.tools.list_ports.comports())

        def score(port):
            serial_number = (port.serial_number or "").strip()
            text = f"{port.description or ''} {port.manufacturer or ''} {port.product or ''} {port.hwid or ''}".lower()
            value = 0
            if self.preferred_usb_serial and serial_number == self.preferred_usb_serial:
                value += 1000
            if "mega" in text or "arduino" in text:
                value += 200
            if any(key in text for key in ("ch340", "ch341", "wch", "atmega16u2")):
                value += 100
            if "usb" in text and "serial" in text:
                value += 25
            return value

        return sorted(ports, key=score, reverse=True)

    def _probe_port(self, device: str) -> Optional[serial.Serial]:
        """Passively listen for the PLC identity/status protocol."""
        candidate = None
        try:
            candidate = serial.Serial(
                device,
                self.baudrate,
                timeout=self.serial_timeout,
                write_timeout=0.2,
                exclusive=True,  # never fight another bridge for the same tty
            )
            candidate.reset_input_buffer()
            deadline = time.monotonic() + self.probe_timeout
            while self._running and time.monotonic() < deadline:
                raw = candidate.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                identity = parse_device_identity(line)
                if identity and identity[0] == self.expected_device_id:
                    self.get_logger().info(
                        f"Detected {identity[0]} firmware {identity[1]} on {device}"
                    )
                    return candidate
                if parse_plc_status(line) is not None:
                    self.get_logger().info(f"Detected Mega2560 PLC status protocol on {device}")
                    return candidate
        except (serial.SerialException, OSError):
            pass

        if candidate is not None:
            try:
                candidate.close()
            except Exception:
                pass
        return None

    def _try_connect(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_connect_attempt) < self.reconnect_period:
            return
        if not self._connect_lock.acquire(blocking=False):
            return

        try:
            self._last_connect_attempt = now
            with self._serial_lock:
                if self._serial is not None and self._serial.is_open:
                    return

            if self.auto_detect_serial:
                ports = self._candidate_ports()
                if not ports:
                    return
                for port in ports:
                    if not self._running:
                        return
                    candidate = self._probe_port(port.device)
                    if candidate is None:
                        continue
                    with self._serial_lock:
                        self._serial = candidate
                        self._active_port = port.device
                    self._last_status_monotonic = time.monotonic()
                    self.get_logger().info(f"Connected to Mega2560 PLC on {port.device}")
                    return
            else:
                candidate = serial.Serial(
                    self.serial_port,
                    self.baudrate,
                    timeout=self.serial_timeout,
                    write_timeout=0.2,
                    exclusive=True,
                )
                with self._serial_lock:
                    self._serial = candidate
                    self._active_port = self.serial_port
                self._last_status_monotonic = time.monotonic()
                self.get_logger().info(f"Connected to Mega2560 PLC on {self.serial_port}")
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f"PLC serial unavailable: {exc}")
        finally:
            self._connect_lock.release()

    def _disconnect(self, reason: str = "") -> None:
        with self._serial_lock:
            ser = self._serial
            self._serial = None
            self._active_port = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        self._last_status_monotonic = None
        if reason:
            self.get_logger().warning(reason)

    def _write_serial(self, frame: bytes) -> bool:
        with self._serial_lock:
            ser = self._serial
            if ser is None or not ser.is_open:
                return False
            try:
                ser.write(frame)
                return True
            except (serial.SerialException, OSError) as exc:
                reason = f"PLC serial write failed: {exc}"
        self._disconnect(reason)
        return False

    def _serial_reader_loop(self) -> None:
        while self._running:
            with self._serial_lock:
                ser = self._serial

            if ser is None or not ser.is_open:
                self._try_connect()
                time.sleep(0.05)
                continue

            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                status = parse_plc_status(line)
                if status is not None:
                    self._last_status_monotonic = time.monotonic()
                    self._handle_status(status)
            except (serial.SerialException, OSError) as exc:
                self._disconnect(f"PLC serial read failed: {exc}")
            except Exception as exc:
                self.get_logger().warning(f"PLC serial parse error: {exc}")

    # ------------------------------------------------------------------
    # PLC -> ROS
    # ------------------------------------------------------------------
    def _publish_json(self, publisher, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        publisher.publish(msg)

    def _handle_status(self, status: PlcStatus) -> None:
        previous = self._last_status
        self._last_status = status

        # Publish changed sensor states (and all states on first packet).
        for sensor_id in range(1, 9):
            state = status.sensor(sensor_id)
            if previous is None or state != previous.sensor(sensor_id):
                self._publish_json(
                    self.sensor_pub,
                    {"sensor_id": sensor_id, "state": state},
                )

        # Each conveyor maps to two sensor IDs:
        # conveyor 1 -> sensors 1,2; conveyor 2 -> 3,4; etc.
        for conveyor_id in range(1, 5):
            cargo = status.cargo(conveyor_id)
            if previous is None or cargo != previous.cargo(conveyor_id):
                self._publish_json(
                    self.cargo_pub,
                    {"belt_id": conveyor_id, "has_cargo": cargo},
                )

            changed = previous is None
            if previous is not None:
                changed = any((
                    status.running(conveyor_id) != previous.running(conveyor_id),
                    status.fault(conveyor_id) != previous.fault(conveyor_id),
                    status.mode_name(conveyor_id) != previous.mode_name(conveyor_id),
                ))
            if changed:
                self._publish_json(
                    self.conveyor_status_pub,
                    {
                        "conveyor_id": conveyor_id,
                        "mode": status.mode_name(conveyor_id),
                        "running": status.running(conveyor_id),
                        "fault": status.fault(conveyor_id),
                        "sensor_a": status.sensor((conveyor_id - 1) * 2 + 1),
                        "sensor_b": status.sensor((conveyor_id - 1) * 2 + 2),
                        "has_cargo": cargo,
                    },
                )

        for side in ("left", "right"):
            state = status.bumper(side)
            if previous is None or state != previous.bumper(side):
                self._publish_json(
                    self.bumper_pub,
                    {"side": side, "triggered": state},
                )

        # Self-healing refresh. The HMI subscribes BEST_EFFORT / depth 1 and may
        # start after this bridge, so change-only publishing would leave its
        # sensor/cargo/bumper widgets stale (and a burst of 8 sensors on one
        # topic would be truncated to the last one). Re-publish ONE item per
        # PSTAT frame (10 Hz): every sensor is refreshed every ~0.8 s.
        self._refresh_tick = (self._refresh_tick + 1) & 0x7FFFFFFF
        refresh_sensor = (self._refresh_tick % 8) + 1
        self._publish_json(
            self.sensor_pub,
            {"sensor_id": refresh_sensor, "state": status.sensor(refresh_sensor)},
        )
        refresh_belt = (self._refresh_tick % 4) + 1
        self._publish_json(
            self.cargo_pub,
            {"belt_id": refresh_belt, "has_cargo": status.cargo(refresh_belt)},
        )
        refresh_side = ("left", "right")[self._refresh_tick % 2]
        self._publish_json(
            self.bumper_pub,
            {"side": refresh_side, "triggered": status.bumper(refresh_side)},
        )

        # E-stop is published on EVERY frame (10 Hz): it doubles as a heartbeat
        # for the KEYA interlock and for HMIs that start late.
        self.emg_pub.publish(Bool(data=status.emergency_stop))
        if previous is None or status.start_button != previous.start_button:
            self.start_pub.publish(Bool(data=status.start_button))
        if previous is None or status.stop_button != previous.stop_button:
            self.stop_pub.publish(Bool(data=status.stop_button))

        if previous is None or status.led_mask != previous.led_mask:
            self._publish_json(
                self.light_states_pub,
                {
                    "left_1": bool(status.led_mask & 0x01),
                    "left_2": bool(status.led_mask & 0x02),
                    "right_1": bool(status.led_mask & 0x04),
                    "right_2": bool(status.led_mask & 0x08),
                    "mask": status.led_mask,
                },
            )

        self._publish_json(
            self.plc_status_pub,
            {
                "online": True,
                "port": self._active_port,
                "uptime_ms": status.uptime_ms,
                "sensor_mask": status.sensor_mask,
                "bumper_mask": status.bumper_mask,
                "emergency_stop": status.emergency_stop,
                "start_button": status.start_button,
                "stop_button": status.stop_button,
                "cargo_mask": status.cargo_mask,
                "running_mask": status.running_mask,
                "fault_mask": status.fault_mask,
                "led_mask": status.led_mask,
            },
        )

    def _health_timer_callback(self) -> None:
        now = time.monotonic()
        online = (
            self._serial is not None
            and self._last_status_monotonic is not None
            and (now - self._last_status_monotonic) <= self.telemetry_timeout
        )

        # Publish on change AND once per second, so an HMI started later learns
        # the PLC state without waiting for the next transition.
        self._health_tick = (getattr(self, "_health_tick", 0) + 1) % 5
        if online != self._was_online or self._health_tick == 0:
            msg = String(data="online" if online else "offline")
            self.connection_pub.publish(msg)
        if online != self._was_online:
            self._was_online = online
            if online:
                self.get_logger().info("Mega2560 PLC online")
            else:
                self.get_logger().warning("Mega2560 PLC offline")

        if self._serial is not None and not online and self._last_status_monotonic is not None:
            if (now - self._last_status_monotonic) > max(2.0, self.telemetry_timeout * 2.0):
                self._disconnect("Mega2560 PLC telemetry timeout; reconnecting")

    def destroy_node(self):
        self._running = False
        # Best effort: stop conveyors before closing serial.
        try:
            self._write_serial(encode_stop_all(self._next_sequence()))
        except Exception:
            pass
        self._disconnect()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Mega2560PlcBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
