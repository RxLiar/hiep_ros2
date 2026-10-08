#!/usr/bin/env python3

import json
import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

import serial
import serial.tools.list_ports

from .protocol import (
    Telemetry,
    encode_velocity_command,
    parse_device_identity,
    parse_telemetry,
)


class HardwareBridgeNode(Node):
    """ROS 2 <-> ESP32 hardware bridge for a differential-drive base.

    Responsibilities:
      * Receive /cmd_vel and convert it to left/right wheel velocity targets.
      * Send velocity targets to ESP32 at a fixed rate.
      * Receive encoder telemetry from ESP32.
      * Publish raw wheel odometry on /wheel/odom.
      * Publish basic hardware/connection status for the HMI.

    Deliberately NOT responsible for:
      * Motor PID or PWM generation (ESP32 does that).
      * odom -> base_footprint TF (robot_localization EKF does that).
      * Navigation, SLAM, AMCL, or lidar processing.
    """

    def __init__(self) -> None:
        super().__init__("hardware_bridge")

        # ---- Serial parameters ----
        # serial_port="auto" + auto_detect_serial=true means no ttyUSB/ttyACM
        # number is hard-coded. The bridge passively probes serial devices and
        # only accepts a port that emits our ESP32 ID/TEL protocol.
        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.05)
        self.declare_parameter("auto_detect_serial", True)
        self.declare_parameter("probe_timeout", 1.5)
        self.declare_parameter("expected_device_id", "AGV_ESP32")
        self.declare_parameter("preferred_usb_serial", "")
        self.declare_parameter("reconnect_period", 2.0)
        self.declare_parameter("telemetry_timeout", 0.5)

        # ---- Command/safety parameters ----
        self.declare_parameter("command_rate", 50.0)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("max_wheel_speed", 0.6)

        # ---- Robot geometry / encoder ----
        self.declare_parameter("wheel_radius", 0.024)
        self.declare_parameter("wheel_separation", 0.20)
        self.declare_parameter("encoder_ticks_per_rev", 2970.0)
        self.declare_parameter("left_encoder_sign", 1.0)
        self.declare_parameter("right_encoder_sign", 1.0)

        # ---- Topic / frame names ----
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout").value)
        self.auto_detect_serial = bool(self.get_parameter("auto_detect_serial").value)
        self.probe_timeout = float(self.get_parameter("probe_timeout").value)
        self.expected_device_id = str(self.get_parameter("expected_device_id").value)
        self.preferred_usb_serial = str(self.get_parameter("preferred_usb_serial").value).strip()
        self.reconnect_period = float(self.get_parameter("reconnect_period").value)
        self.telemetry_timeout = float(self.get_parameter("telemetry_timeout").value)

        self.command_rate = float(self.get_parameter("command_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_wheel_speed = float(self.get_parameter("max_wheel_speed").value)

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.ticks_per_rev = float(self.get_parameter("encoder_ticks_per_rev").value)
        self.left_encoder_sign = float(self.get_parameter("left_encoder_sign").value)
        self.right_encoder_sign = float(self.get_parameter("right_encoder_sign").value)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.wheel_odom_topic = str(self.get_parameter("wheel_odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        if self.command_rate <= 0.0:
            raise ValueError("command_rate must be > 0")
        if self.wheel_separation <= 0.0 or self.wheel_radius <= 0.0 or self.ticks_per_rev <= 0.0:
            raise ValueError("wheel geometry and encoder_ticks_per_rev must be > 0")

        # ---- ROS interfaces ----
        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.wheel_odom_topic, 20)
        self.encoder_pub = self.create_publisher(String, "/encoder_data", 10)
        self.connection_pub = self.create_publisher(String, "/connection_status", 10)
        self.robot_status_pub = self.create_publisher(String, "/robot_status", 10)
        self.moving_pub = self.create_publisher(Bool, "/agv_status", 10)

        # ---- Command state ----
        self._target_vx = 0.0
        self._target_wz = 0.0
        self._last_cmd_monotonic: Optional[float] = None
        self._tx_sequence = 0
        self._state_lock = threading.Lock()

        # ---- Serial state ----
        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._connect_lock = threading.Lock()
        self._active_serial_port: Optional[str] = None
        self._running = True
        self._last_connect_attempt = 0.0
        self._last_telemetry_monotonic: Optional[float] = None
        self._was_online = False

        # ---- Odometry state ----
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._prev_encoder_left: Optional[int] = None
        self._prev_encoder_right: Optional[int] = None
        self._last_odom_ros_time = None
        self._last_mcu_time_ms: Optional[int] = None

        # Connection is non-fatal. The reader thread discovers/reconnects in
        # the background so the ROS executor never blocks while USB ports scan.
        self._reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self._reader_thread.start()

        self.create_timer(1.0 / self.command_rate, self._command_timer_callback)
        self.create_timer(0.2, self._health_timer_callback)

        self.get_logger().info(
            f"Hardware bridge ready: cmd={self.cmd_vel_topic}, odom={self.wheel_odom_topic}, "
            f"serial={'AUTO' if self.auto_detect_serial else self.serial_port}, {self.baudrate} baud"
        )

    # ------------------------------------------------------------------
    # ROS command path
    # ------------------------------------------------------------------
    def _cmd_vel_callback(self, msg: Twist) -> None:
        vx = max(-self.max_linear_speed, min(self.max_linear_speed, float(msg.linear.x)))
        wz = max(-self.max_angular_speed, min(self.max_angular_speed, float(msg.angular.z)))

        with self._state_lock:
            self._target_vx = vx
            self._target_wz = wz
            self._last_cmd_monotonic = time.monotonic()

    def _command_timer_callback(self) -> None:
        now = time.monotonic()

        with self._state_lock:
            stale = (
                self._last_cmd_monotonic is None
                or (now - self._last_cmd_monotonic) > self.command_timeout
            )
            vx = 0.0 if stale else self._target_vx
            wz = 0.0 if stale else self._target_wz

        left = vx - 0.5 * self.wheel_separation * wz
        right = vx + 0.5 * self.wheel_separation * wz
        left, right = self._limit_wheel_speeds(left, right)

        self._tx_sequence = (self._tx_sequence + 1) & 0x7FFFFFFF
        frame = encode_velocity_command(self._tx_sequence, left, right)
        self._write_serial(frame)

    def _limit_wheel_speeds(self, left: float, right: float) -> tuple[float, float]:
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_speed and peak > 0.0:
            scale = self.max_wheel_speed / peak
            left *= scale
            right *= scale
        return left, right

    # ------------------------------------------------------------------
    # Serial connection
    # ------------------------------------------------------------------
    def _candidate_serial_ports(self):
        """Return serial ports in a useful probe order.

        Detection is intentionally passive: we do not transmit probe bytes to
        unknown devices (important when KEYA/lidar USB adapters are present).
        ESP32 firmware identifies itself by emitting ID/TEL frames.
        """
        ports = list(serial.tools.list_ports.comports())

        def score(port):
            score_value = 0
            serial_number = (port.serial_number or "").strip()
            text = f"{port.description or ''} {port.manufacturer or ''} {port.product or ''} {port.hwid or ''}".lower()

            if self.preferred_usb_serial and serial_number == self.preferred_usb_serial:
                score_value += 1000
            if "esp32" in text or "espressif" in text:
                score_value += 200
            if any(key in text for key in ("cp210", "silicon labs", "ch340", "ch341", "wch")):
                score_value += 100
            if "usb" in text and "serial" in text:
                score_value += 25
            return score_value

        return sorted(ports, key=score, reverse=True)

    def _probe_serial_port(self, device: str) -> Optional[serial.Serial]:
        """Open one port and passively verify that it is our ESP32.

        No command is written while probing. This prevents auto-discovery from
        accidentally sending bytes to a motor driver, lidar, or other serial
        device connected to the same computer.
        """
        try:
            candidate = serial.Serial(
                port=device,
                baudrate=self.baudrate,
                timeout=self.serial_timeout,
                write_timeout=self.serial_timeout,
                exclusive=True,  # never fight another bridge for the same tty
            )
        except (serial.SerialException, OSError):
            return None

        deadline = time.monotonic() + self.probe_timeout
        try:
            while self._running and time.monotonic() < deadline:
                raw = candidate.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue

                # New firmware emits a positive identity frame.
                try:
                    identity = parse_device_identity(line)
                    if identity.device_id == self.expected_device_id:
                        self.get_logger().info(
                            f"Detected {identity.device_id} firmware {identity.firmware_version} on {device}"
                        )
                        return candidate
                except (ValueError, TypeError):
                    pass

                # Backward-compatible fallback: valid AGV telemetry uniquely
                # identifies firmware v0.2.1, which did not emit an ID frame.
                try:
                    parse_telemetry(line)
                    self.get_logger().info(f"Detected ESP32 telemetry protocol on {device}")
                    return candidate
                except (ValueError, TypeError):
                    pass
        except (serial.SerialException, OSError):
            pass

        try:
            candidate.close()
        except Exception:
            pass
        return None

    def _try_connect(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_connect_attempt) < self.reconnect_period:
            return

        # Only one thread may perform discovery at a time.
        if not self._connect_lock.acquire(blocking=False):
            return

        try:
            self._last_connect_attempt = now

            with self._serial_lock:
                if self._serial is not None and self._serial.is_open:
                    return

            if self.auto_detect_serial:
                ports = self._candidate_serial_ports()
                if not ports:
                    self.get_logger().warning("No serial devices found while searching for ESP32")
                    return

                port_names = ", ".join(p.device for p in ports)
                self.get_logger().info(f"Searching for ESP32 on: {port_names}")

                for port_info in ports:
                    if not self._running:
                        return
                    candidate = self._probe_serial_port(port_info.device)
                    if candidate is None:
                        continue

                    with self._serial_lock:
                        self._serial = candidate
                        self._active_serial_port = port_info.device
                    self._last_telemetry_monotonic = time.monotonic()
                    self._reset_encoder_baseline()
                    self.get_logger().info(f"Connected to ESP32 on {port_info.device}")
                    return

                self.get_logger().warning("ESP32 not found on any available serial port")
                return

            # Manual/fixed-port mode remains available for diagnostics.
            try:
                candidate = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baudrate,
                    timeout=self.serial_timeout,
                    write_timeout=self.serial_timeout,
                    exclusive=True,
                )
                with self._serial_lock:
                    self._serial = candidate
                    self._active_serial_port = self.serial_port
                self._reset_encoder_baseline()
                self.get_logger().info(f"Connected to ESP32 on {self.serial_port}")
            except (serial.SerialException, OSError) as exc:
                with self._serial_lock:
                    self._serial = None
                    self._active_serial_port = None
                self.get_logger().warning(f"ESP32 serial unavailable on {self.serial_port}: {exc}")
        finally:
            self._connect_lock.release()

    def _disconnect_serial(self, reason: str) -> None:
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
            self._serial = None
            self._active_serial_port = None
        self.get_logger().warning(reason)

    def _write_serial(self, payload: bytes) -> None:
        if self._serial is None or not self._serial.is_open:
            # Background reader owns discovery/reconnect. Never block a ROS
            # command callback while scanning USB devices.
            return

        try:
            with self._serial_lock:
                if self._serial is not None and self._serial.is_open:
                    self._serial.write(payload)
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as exc:
            self._disconnect_serial(f"Serial write failed: {exc}")

    def _serial_reader_loop(self) -> None:
        while self._running and rclpy.ok():
            if self._serial is None or not self._serial.is_open:
                self._try_connect()
                time.sleep(0.05)
                continue

            try:
                # Read outside the lock: pyserial allows one reader and one
                # writer thread, and holding the lock during readline() (up to
                # serial_timeout) used to stall the 50 Hz command writer.
                with self._serial_lock:
                    ser = self._serial
                if ser is None or not ser.is_open:
                    continue
                raw = ser.readline()

                if not raw:
                    continue

                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue

                try:
                    telemetry = parse_telemetry(line)
                except (ValueError, TypeError):
                    self.get_logger().debug(f"Ignoring serial frame: {line}")
                    continue

                self._last_telemetry_monotonic = time.monotonic()
                self._process_telemetry(telemetry)

            except (serial.SerialException, OSError) as exc:
                self._disconnect_serial(f"Serial read failed: {exc}")
                time.sleep(0.1)
            except Exception as exc:
                self.get_logger().error(f"Unexpected serial reader error: {exc}")
                time.sleep(0.05)

    # ------------------------------------------------------------------
    # Telemetry / odometry
    # ------------------------------------------------------------------
    def _reset_encoder_baseline(self) -> None:
        """Forget the previous encoder reading (new session or MCU reboot).

        The ESP32 restarts its tick counters at 0 after a reset. Without this
        the first delta after a reconnect would be a huge bogus jump.
        """
        self._prev_encoder_left = None
        self._prev_encoder_right = None
        self._last_odom_ros_time = None
        self._last_mcu_time_ms = None

    def _process_telemetry(self, telemetry: Telemetry) -> None:
        if (
            self._last_mcu_time_ms is not None
            and telemetry.mcu_time_ms < self._last_mcu_time_ms
        ):
            self.get_logger().warning("ESP32 rebooted (uptime went backwards); resetting odometry baseline")
            self._reset_encoder_baseline()
        self._last_mcu_time_ms = telemetry.mcu_time_ms

        raw_msg = String()
        raw_msg.data = f"{telemetry.encoder_left},{telemetry.encoder_right}"
        self.encoder_pub.publish(raw_msg)

        left_ticks = int(round(telemetry.encoder_left * self.left_encoder_sign))
        right_ticks = int(round(telemetry.encoder_right * self.right_encoder_sign))

        now_ros = self.get_clock().now()
        if self._prev_encoder_left is None or self._prev_encoder_right is None:
            self._prev_encoder_left = left_ticks
            self._prev_encoder_right = right_ticks
            self._last_odom_ros_time = now_ros
            return

        dt = (now_ros - self._last_odom_ros_time).nanoseconds * 1e-9
        self._last_odom_ros_time = now_ros
        if dt <= 0.0 or dt > 1.0:
            self._prev_encoder_left = left_ticks
            self._prev_encoder_right = right_ticks
            return

        delta_left = left_ticks - self._prev_encoder_left
        delta_right = right_ticks - self._prev_encoder_right
        self._prev_encoder_left = left_ticks
        self._prev_encoder_right = right_ticks

        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_left = delta_left * meters_per_tick
        d_right = delta_right * meters_per_tick

        ds = 0.5 * (d_left + d_right)
        d_yaw = (d_right - d_left) / self.wheel_separation

        # Midpoint integration is more accurate than using the final yaw only.
        heading_mid = self._yaw + 0.5 * d_yaw
        self._x += ds * math.cos(heading_mid)
        self._y += ds * math.sin(heading_mid)
        self._yaw = self._normalize_angle(self._yaw + d_yaw)

        measured_left = telemetry.velocity_left_mps * self.left_encoder_sign
        measured_right = telemetry.velocity_right_mps * self.right_encoder_sign
        vx = 0.5 * (measured_left + measured_right)
        wz = (measured_right - measured_left) / self.wheel_separation

        msg = Odometry()
        msg.header.stamp = now_ros.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.z = math.sin(self._yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self._yaw * 0.5)

        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz

        # Conservative starter covariances. Tune after real-world measurement.
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[7] = 0.05
        msg.twist.covariance[35] = 0.05

        self.odom_pub.publish(msg)

        moving = Bool()
        moving.data = abs(vx) > 0.01 or abs(wz) > 0.03
        self.moving_pub.publish(moving)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    # ------------------------------------------------------------------
    # Health status for HMI
    # ------------------------------------------------------------------
    def _health_timer_callback(self) -> None:
        now = time.monotonic()
        serial_open = self._serial is not None and self._serial.is_open
        telemetry_fresh = (
            self._last_telemetry_monotonic is not None
            and (now - self._last_telemetry_monotonic) <= self.telemetry_timeout
        )
        online = bool(serial_open and telemetry_fresh)

        if online != self._was_online:
            self._was_online = online
            self.get_logger().info("ESP32 online" if online else "ESP32 offline")

        conn = String()
        conn.data = "online" if online else "offline"
        self.connection_pub.publish(conn)

        status = String()
        if online:
            status.data = json.dumps({"level": "ok", "message": ""})
        elif serial_open:
            status.data = json.dumps({"level": "warn", "message": "ESP32 telemetry timeout"})
        else:
            status.data = json.dumps({"level": "error", "message": "ESP32 serial disconnected"})
        self.robot_status_pub.publish(status)

        # Reconnect/discovery is handled by the serial reader thread.

    def destroy_node(self) -> bool:
        self._running = False

        # Best effort stop before closing serial. ESP32 watchdog is the real safety layer.
        try:
            self._tx_sequence = (self._tx_sequence + 1) & 0x7FFFFFFF
            self._write_serial(encode_velocity_command(self._tx_sequence, 0.0, 0.0))
        except Exception:
            pass

        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=0.5)

        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
                self._active_serial_port = None

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
