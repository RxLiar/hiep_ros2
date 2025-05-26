#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading
import serial.tools.list_ports

def find_esp32_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if ("CP210" in port.description or "CH340" in port.description or "Silicon" in port.description):
            return port.device
    return None

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm_node')

        self.port = find_esp32_port()
        self.baudrate = 115200

        if not self.port:
            self.get_logger().error("Không tìm thấy ESP32 trên bất kỳ cổng USB nào.")
            raise RuntimeError("ESP32 not found")

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"Không mở được cổng serial {self.port}: {e}")
            raise

        # ROS2 Communication
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.encoder_pub = self.create_publisher(String, 'encoder_data', 10)

        # Thread to read from serial
        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

        self.get_logger().info('SerialCommNode started')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        L = 0.2  # khoảng cách giữa hai bánh xe

        # Chuyển đổi sang tốc độ cho động cơ trái và phải
        v_l = int((v - w * L / 2.0) / 0.5 * 255)
        v_r = int((v + w * L / 2.0) / 0.5 * 255)

        command = f"{v_l},{v_r}\n"
        try:
            self.ser.write(command.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Lỗi gửi serial: {e}")

    def read_from_serial(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    msg = String()
                    msg.data = line
                    self.encoder_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
