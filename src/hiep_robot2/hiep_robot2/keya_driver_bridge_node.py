#!/usr/bin/env python3

"""ROS 2 bridge for direct serial control of a KEYA KYDAS servo driver.

ROS API intentionally matches the ESP32 backend:
    subscribe: /cmd_vel
    publish:   /wheel/odom, /connection_status, /robot_status, /agv_status

The node does NOT publish odom->base_footprint TF. robot_localization/EKF should own it.
"""

from __future__ import annotations

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
from std_srvs.srv import SetBool

import serial

from .keya_protocol import (
    FRAME_LEN,
    HEARTBEAT_START,
    QUERY_ENCODER,
    QUERY_FAULT,
    QUERY_SPEED,
    QUERY_START,
    QUERY_TEMPERATURE,
    QUERY_VOLTAGE,
    build_disable_frame,
    build_query_frame,
    build_speed_frame,
    parse_encoder_response,
    parse_fault_response,
    parse_speed_response,
    parse_temperature_response,
    parse_voltage_response,
    speed_rpm_to_command,
)


class KeyaDriverBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("keya_driver_bridge")

        # Serial / transport. The published KYDAS4850-1E manual documents RS232.
        self.declare_parameter("serial_port", "/dev/keya_driver")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.03)
        self.declare_parameter("reconnect_period", 2.0)
        self.declare_parameter("telemetry_timeout", 0.7)
        self.declare_parameter("transport", "rs232")   # rs232 | rs485 (USB-RS485 adapter)
        # Some USB-RS485 adapters echo every transmitted byte back to the RX line.
        # Our 12-byte query frames start with 0xED, exactly like a driver response,
        # so an echo would be parsed as a fake "encoder = 0,0" reply.
        self.declare_parameter("rs485_echo_filter", False)

        # Command safety.
        self.declare_parameter("command_rate", 40.0)  # 25 ms; manual says >20 ms and <500 ms.
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("max_wheel_speed", 0.6)
        self.declare_parameter("arm_on_start", False)
        # Software e-stop interlock from the PLC bridge (Bool). "" disables it.
        self.declare_parameter("estop_topic", "/emergency_stop")
        # Reject encoder deltas implying a wheel speed above this (driver power
        # cycle / counter reset). The bridge re-syncs instead of teleporting.
        self.declare_parameter("encoder_jump_speed_limit", 2.0)

        # Robot/motor geometry.
        self.declare_parameter("wheel_radius", 0.10)
        self.declare_parameter("wheel_separation", 0.50)
        self.declare_parameter("gear_ratio", 1.0)  # motor rev / wheel rev
        self.declare_parameter("rated_speed_rpm", 1500.0)
        self.declare_parameter("encoder_counts_per_motor_rev", 10000.0)
        self.declare_parameter("motor1_command_sign", 1.0)
        self.declare_parameter("motor2_command_sign", 1.0)
        self.declare_parameter("motor1_encoder_sign", 1.0)
        self.declare_parameter("motor2_encoder_sign", 1.0)

        # Polling. Encoder is the odometry source; the others are lower rate diagnostics.
        self.declare_parameter("encoder_poll_rate", 20.0)
        self.declare_parameter("diagnostic_poll_rate", 1.0)

        # ROS names.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        gp = lambda name: self.get_parameter(name).value
        self.serial_port = str(gp("serial_port"))
        self.baudrate = int(gp("baudrate"))
        self.serial_timeout = float(gp("serial_timeout"))
        self.reconnect_period = float(gp("reconnect_period"))
        self.telemetry_timeout = float(gp("telemetry_timeout"))
        self.transport = str(gp("transport")).lower().strip()
        self.rs485_echo_filter = bool(gp("rs485_echo_filter"))
        self.estop_topic = str(gp("estop_topic")).strip()
        self.encoder_jump_speed_limit = float(gp("encoder_jump_speed_limit"))

        self.command_rate = float(gp("command_rate"))
        self.command_timeout = float(gp("command_timeout"))
        self.max_linear_speed = float(gp("max_linear_speed"))
        self.max_angular_speed = float(gp("max_angular_speed"))
        self.max_wheel_speed = float(gp("max_wheel_speed"))
        self._armed = bool(gp("arm_on_start"))

        self.wheel_radius = float(gp("wheel_radius"))
        self.wheel_separation = float(gp("wheel_separation"))
        self.gear_ratio = float(gp("gear_ratio"))
        self.rated_speed_rpm = float(gp("rated_speed_rpm"))
        self.encoder_counts_per_motor_rev = float(gp("encoder_counts_per_motor_rev"))
        self.motor1_command_sign = float(gp("motor1_command_sign"))
        self.motor2_command_sign = float(gp("motor2_command_sign"))
        self.motor1_encoder_sign = float(gp("motor1_encoder_sign"))
        self.motor2_encoder_sign = float(gp("motor2_encoder_sign"))

        self.encoder_poll_rate = float(gp("encoder_poll_rate"))
        self.diagnostic_poll_rate = float(gp("diagnostic_poll_rate"))
        self.cmd_vel_topic = str(gp("cmd_vel_topic"))
        self.wheel_odom_topic = str(gp("wheel_odom_topic"))
        self.odom_frame = str(gp("odom_frame"))
        self.base_frame = str(gp("base_frame"))

        if self.transport not in ("rs232", "rs485"):
            raise ValueError("transport must be 'rs232' or 'rs485'")
        if min(self.command_rate, self.wheel_radius, self.wheel_separation, self.gear_ratio,
               self.rated_speed_rpm, self.encoder_counts_per_motor_rev, self.encoder_poll_rate) <= 0.0:
            raise ValueError("robot geometry, speed, encoder and rate parameters must be > 0")

        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.wheel_odom_topic, 20)
        self.connection_pub = self.create_publisher(String, "/connection_status", 10)
        self.robot_status_pub = self.create_publisher(String, "/robot_status", 10)
        self.moving_pub = self.create_publisher(Bool, "/agv_status", 10)
        self.encoder_pub = self.create_publisher(String, "/encoder_data", 10)
        self.driver_diag_pub = self.create_publisher(String, "/keya_driver_status", 10)
        self.enable_srv = self.create_service(SetBool, "/motor_enable", self._enable_callback)
        if self.estop_topic:
            self.create_subscription(Bool, self.estop_topic, self._estop_callback, 10)

        self._state_lock = threading.Lock()
        self._serial_lock = threading.Lock()
        self._serial: Optional[serial.Serial] = None
        self._running = True
        self._last_connect_attempt = 0.0
        self._last_rx_monotonic: Optional[float] = None
        self._last_cmd_monotonic: Optional[float] = None
        self._target_vx = 0.0
        self._target_wz = 0.0
        self._last_fault_1 = 0
        self._last_fault_2 = 0
        self._last_voltage: Optional[int] = None
        self._last_temperature: Optional[int] = None
        self._last_speed_1 = 0
        self._last_speed_2 = 0
        self._diag_query_index = 0
        self._estop_active = False

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._prev_enc1: Optional[int] = None
        self._prev_enc2: Optional[int] = None
        self._last_odom_ros_time = None

        self._try_connect(force=True)
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.create_timer(1.0 / self.command_rate, self._command_timer)
        self.create_timer(1.0 / self.encoder_poll_rate, self._encoder_poll_timer)
        if self.diagnostic_poll_rate > 0.0:
            self.create_timer(1.0 / self.diagnostic_poll_rate, self._diagnostic_poll_timer)
        self.create_timer(0.2, self._health_timer)

        self.get_logger().info(
            f"KEYA bridge ready on {self.serial_port} @ {self.baudrate}; "
            f"armed={self._armed}; cmd={self.cmd_vel_topic}; odom={self.wheel_odom_topic}"
        )
        if self.transport == "rs485":
            self.get_logger().warning(
                "transport=rs485: the documented RS232 12-byte frames are sent unchanged. "
                "Confirm in your exact KYDAS manual that the RS485 port uses the same protocol, "
                "baud rate and slave addressing, and use an adapter with automatic direction control."
            )
        self.get_logger().warning(
            "Verify wheel radius, gear ratio, rated RPM, motor direction and encoder direction before placing the robot on the floor."
        )

    # ---------------- ROS command path ----------------
    def _cmd_vel_callback(self, msg: Twist) -> None:
        vx = max(-self.max_linear_speed, min(self.max_linear_speed, float(msg.linear.x)))
        wz = max(-self.max_angular_speed, min(self.max_angular_speed, float(msg.angular.z)))
        with self._state_lock:
            self._target_vx = vx
            self._target_wz = wz
            self._last_cmd_monotonic = time.monotonic()

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data and self._estop_active:
            response.success = False
            response.message = "Emergency stop is active - release it before enabling motors"
            self._publish_robot_status("error", response.message)
            return response

        self._armed = bool(request.data)
        if not self._armed:
            self._write_frame(build_disable_frame())
        response.success = True
        response.message = "KEYA motor output enabled" if self._armed else "KEYA motor output disabled"
        self._publish_robot_status("ok", response.message)
        return response

    def _estop_callback(self, msg: Bool) -> None:
        """Software interlock only. The real e-stop must cut driver power in hardware.

        Pressing e-stop disables the output AND latches it off: releasing the
        button does NOT re-enable motion, an operator must call /motor_enable again.
        """
        active = bool(msg.data)
        if active and not self._estop_active:
            self._estop_active = True
            self._armed = False
            self._write_frame(build_disable_frame())
            self.get_logger().warning("Emergency stop active: KEYA output disabled")
            self._publish_robot_status("error", "Emergency stop active - motors disabled")
        elif not active and self._estop_active:
            self._estop_active = False
            self.get_logger().info("Emergency stop released: motors stay disabled until /motor_enable")

    def _command_timer(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            stale = self._last_cmd_monotonic is None or (now - self._last_cmd_monotonic) > self.command_timeout
            vx = 0.0 if stale else self._target_vx
            wz = 0.0 if stale else self._target_wz

        if not self._armed:
            self._write_frame(build_disable_frame())
            return

        left_mps = vx - 0.5 * self.wheel_separation * wz
        right_mps = vx + 0.5 * self.wheel_separation * wz
        left_mps, right_mps = self._limit_wheels(left_mps, right_mps)

        m1_rpm = self._wheel_mps_to_motor_rpm(left_mps) * self.motor1_command_sign
        m2_rpm = self._wheel_mps_to_motor_rpm(right_mps) * self.motor2_command_sign
        c1 = speed_rpm_to_command(m1_rpm, self.rated_speed_rpm)
        c2 = speed_rpm_to_command(m2_rpm, self.rated_speed_rpm)
        self._write_frame(build_speed_frame(c1, c2, enable_mask=0x03))

    def _limit_wheels(self, left: float, right: float) -> tuple[float, float]:
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_speed and peak > 0.0:
            scale = self.max_wheel_speed / peak
            left *= scale
            right *= scale
        return left, right

    def _wheel_mps_to_motor_rpm(self, wheel_mps: float) -> float:
        wheel_rps = wheel_mps / (2.0 * math.pi * self.wheel_radius)
        return wheel_rps * 60.0 * self.gear_ratio

    # ---------------- Serial ----------------
    def _try_connect(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_connect_attempt) < self.reconnect_period:
            return
        self._last_connect_attempt = now
        with self._serial_lock:
            if self._serial is not None and self._serial.is_open:
                return
            try:
                self._serial = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.serial_timeout,
                    write_timeout=self.serial_timeout,
                    exclusive=True,
                )
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
                self._reset_encoder_baseline()
                self.get_logger().info(f"Connected to KEYA driver on {self.serial_port} ({self.transport})")
            except (serial.SerialException, OSError) as exc:
                self._serial = None
                self.get_logger().warning(f"KEYA serial unavailable: {exc}")

    def _disconnect(self, reason: str) -> None:
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
            self._serial = None
        self.get_logger().warning(reason)

    def _write_frame(self, frame: bytes) -> None:
        if self._serial is None or not self._serial.is_open:
            self._try_connect()
            return
        try:
            with self._serial_lock:
                if self._serial is not None and self._serial.is_open:
                    self._serial.write(frame)
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as exc:
            self._disconnect(f"KEYA serial write failed: {exc}")

    def _read_exact_after_start(self, ser: serial.Serial, start_byte: int) -> Optional[bytes]:
        data = bytearray((start_byte,))
        deadline = time.monotonic() + max(0.05, self.serial_timeout * 4.0)
        while len(data) < FRAME_LEN and time.monotonic() < deadline:
            chunk = ser.read(FRAME_LEN - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data) if len(data) == FRAME_LEN else None

    def _is_echo(self, frame: bytes) -> bool:
        """True for our own query frame (ED <code> + 10 zero bytes) echoed by an RS485 adapter."""
        return (
            self.rs485_echo_filter
            and frame[0] == QUERY_START
            and not any(frame[2:])
        )

    def _reader_loop(self) -> None:
        while self._running and rclpy.ok():
            if self._serial is None or not self._serial.is_open:
                self._try_connect()
                time.sleep(0.05)
                continue
            try:
                # Read outside the lock: holding it for up to serial_timeout made
                # the 40 Hz command writer wait and jitter.
                with self._serial_lock:
                    ser = self._serial
                if ser is None or not ser.is_open:
                    continue
                first = ser.read(1)
                if not first:
                    continue
                start = first[0]
                if start not in (QUERY_START, HEARTBEAT_START):
                    continue
                frame = self._read_exact_after_start(ser, start)
                if frame is None or self._is_echo(frame):
                    continue
                self._last_rx_monotonic = time.monotonic()
                self._handle_frame(frame)
            except (serial.SerialException, OSError) as exc:
                self._disconnect(f"KEYA serial read failed: {exc}")
                time.sleep(0.1)
            except Exception as exc:
                self.get_logger().error(f"KEYA reader error: {exc}")
                time.sleep(0.05)

    # ---------------- Poll/query handling ----------------
    def _encoder_poll_timer(self) -> None:
        self._write_frame(build_query_frame(QUERY_ENCODER))

    def _diagnostic_poll_timer(self) -> None:
        queries = (QUERY_SPEED, QUERY_FAULT, QUERY_VOLTAGE, QUERY_TEMPERATURE)
        self._write_frame(build_query_frame(queries[self._diag_query_index % len(queries)]))
        self._diag_query_index += 1

    def _handle_frame(self, frame: bytes) -> None:
        if frame[0] == HEARTBEAT_START:
            # Heartbeat is useful for connection health even if we do not depend on its
            # model-specific field layout for odometry.
            return
        if frame[0] != QUERY_START:
            return
        code = frame[1]
        try:
            if code == QUERY_ENCODER:
                enc = parse_encoder_response(frame)
                self._process_encoder(enc.motor1_count, enc.motor2_count)
            elif code == QUERY_SPEED:
                speed = parse_speed_response(frame)
                self._last_speed_1, self._last_speed_2 = speed.motor1_rpm, speed.motor2_rpm
            elif code == QUERY_FAULT:
                fault = parse_fault_response(frame)
                self._last_fault_1, self._last_fault_2 = fault.motor1_code, fault.motor2_code
                if fault.motor1_code or fault.motor2_code:
                    self._publish_robot_status(
                        "error", f"KEYA fault: motor1=0x{fault.motor1_code:04X}, motor2=0x{fault.motor2_code:04X}"
                    )
            elif code == QUERY_VOLTAGE:
                self._last_voltage = parse_voltage_response(frame)
            elif code == QUERY_TEMPERATURE:
                self._last_temperature = parse_temperature_response(frame)
        except ValueError as exc:
            self.get_logger().warning(f"Invalid KEYA response: {exc}")

    # ---------------- Odometry ----------------
    def _reset_encoder_baseline(self) -> None:
        self._prev_enc1 = None
        self._prev_enc2 = None
        self._last_odom_ros_time = None

    def _process_encoder(self, raw1: int, raw2: int) -> None:
        e1 = int(round(raw1 * self.motor1_encoder_sign))
        e2 = int(round(raw2 * self.motor2_encoder_sign))

        raw_msg = String()
        raw_msg.data = f"{e1},{e2}"
        self.encoder_pub.publish(raw_msg)

        now_ros = self.get_clock().now()
        if self._prev_enc1 is None or self._prev_enc2 is None:
            self._prev_enc1, self._prev_enc2 = e1, e2
            self._last_odom_ros_time = now_ros
            return

        dt = (now_ros - self._last_odom_ros_time).nanoseconds * 1e-9
        self._last_odom_ros_time = now_ros
        if dt <= 0.0 or dt > 1.0:
            self._prev_enc1, self._prev_enc2 = e1, e2
            return

        # int32 rollover-safe deltas.
        d1_count = self._int32_delta(e1, self._prev_enc1)
        d2_count = self._int32_delta(e2, self._prev_enc2)
        self._prev_enc1, self._prev_enc2 = e1, e2

        meters_per_motor_count = (
            2.0 * math.pi * self.wheel_radius
        ) / (self.encoder_counts_per_motor_rev * self.gear_ratio)
        d_left = d1_count * meters_per_motor_count
        d_right = d2_count * meters_per_motor_count

        if self.encoder_jump_speed_limit > 0.0 and max(abs(d_left), abs(d_right)) / dt > self.encoder_jump_speed_limit:
            self.get_logger().warning(
                f"Encoder jump ignored (dL={d_left:.3f} m, dR={d_right:.3f} m in {dt * 1000:.0f} ms): "
                "driver restarted or counter reset?"
            )
            return

        ds = 0.5 * (d_left + d_right)
        d_yaw = (d_right - d_left) / self.wheel_separation
        heading_mid = self._yaw + 0.5 * d_yaw
        self._x += ds * math.cos(heading_mid)
        self._y += ds * math.sin(heading_mid)
        self._yaw = math.atan2(math.sin(self._yaw + d_yaw), math.cos(self._yaw + d_yaw))

        msg = Odometry()
        msg.header.stamp = now_ros.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.z = math.sin(self._yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self._yaw * 0.5)
        msg.twist.twist.linear.x = ds / dt
        msg.twist.twist.angular.z = d_yaw / dt

        # Without covariances robot_localization treats the odometry as perfect
        # and the EKF can diverge. Same starter values as the ESP32 bridge.
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[7] = 0.05
        msg.twist.covariance[35] = 0.05
        self.odom_pub.publish(msg)

        moving = Bool()
        moving.data = abs(msg.twist.twist.linear.x) > 0.01 or abs(msg.twist.twist.angular.z) > 0.02
        self.moving_pub.publish(moving)

    @staticmethod
    def _int32_delta(current: int, previous: int) -> int:
        diff = (int(current) - int(previous)) & 0xFFFFFFFF
        if diff & 0x80000000:
            diff -= 0x100000000
        return diff

    # ---------------- Health/status ----------------
    def _health_timer(self) -> None:
        online = (
            self._serial is not None
            and self._serial.is_open
            and self._last_rx_monotonic is not None
            and (time.monotonic() - self._last_rx_monotonic) <= self.telemetry_timeout
        )
        msg = String()
        msg.data = "online" if online else "offline"
        self.connection_pub.publish(msg)

        diag = String()
        diag.data = json.dumps({
            "online": online,
            "armed": self._armed,
            "estop": self._estop_active,
            "speed_rpm": [self._last_speed_1, self._last_speed_2],
            "fault": [self._last_fault_1, self._last_fault_2],
            "voltage_v": self._last_voltage,
            "temperature_c": self._last_temperature,
        }, separators=(",", ":"))
        self.driver_diag_pub.publish(diag)

        if self._estop_active:
            self._publish_robot_status("error", "Emergency stop active - motors disabled")
        elif not online:
            self._publish_robot_status("warn", "KEYA driver communication offline")
        elif not self._armed:
            self._publish_robot_status("warn", "KEYA motor output disabled")
        elif self._last_fault_1 or self._last_fault_2:
            self._publish_robot_status(
                "error", f"KEYA fault: motor1=0x{self._last_fault_1:04X}, motor2=0x{self._last_fault_2:04X}"
            )
        else:
            self._publish_robot_status("ok", "")

    def _publish_robot_status(self, level: str, message: str) -> None:
        msg = String()
        msg.data = json.dumps({"level": level, "message": message}, separators=(",", ":"))
        self.robot_status_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        self._armed = False
        try:
            self._write_frame(build_disable_frame())
        except Exception:
            pass
        if hasattr(self, "_reader_thread") and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=0.5)
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyaDriverBridgeNode()
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
