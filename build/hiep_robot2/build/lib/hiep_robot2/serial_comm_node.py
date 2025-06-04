#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import serial
import threading
import serial.tools.list_ports
import math
import tf_transformations
from tf2_ros import TransformBroadcaster
import time

def find_esp32_port():
    """Tìm cổng ESP32 tự động"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if ("CP210" in port.description or "CH340" in port.description or 
            "Silicon" in port.description or "USB-SERIAL" in port.description):
            return port.device
    return None

class SerialCommNode(Node): 
    def __init__(self):
        super().__init__('serial_comm_node')

        # ----- Thông số vật lý xe (có thể đưa vào parameters) -----
        self.declare_parameter('pulses_per_rev', 2970.0)
        self.declare_parameter('wheel_radius', 0.024)  # 0.048/2 = 0.024m
        self.declare_parameter('wheel_base', 0.20)
        self.declare_parameter('max_wheel_speed', 0.5)  # m/s
        
        self.pulses_per_rev = self.get_parameter('pulses_per_rev').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value

        # ----- Kết nối ESP32 qua Serial -----
        self.port = find_esp32_port()
        self.baudrate = 115200
        self.serial_timeout = 0.1  # Tăng timeout để ổn định hơn
        
        if not self.port:
            self.get_logger().error("Không tìm thấy ESP32 trên bất kỳ cổng USB nào.")
            raise RuntimeError("ESP32 not found")

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.serial_timeout)
            time.sleep(2)  # Đợi ESP32 khởi động
            self.ser.flushInput()  # Xóa buffer cũ
        except serial.SerialException as e:
            self.get_logger().error(f"Không mở được cổng serial {self.port}: {e}")
            raise

        self.get_logger().info(f"SerialCommNode đã kết nối ESP32 tại {self.port}")

        # ----- ROS2 Publisher / Subscriber -----
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.encoder_pub = self.create_publisher(String, 'encoder_data', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ----- Biến lưu trạng thái Odometry -----
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_ticks_l = None
        self.prev_ticks_r = None
        self.last_time = self.get_clock().now()

        # ----- Bộ lọc nhiễu và validation -----
        self.max_tick_jump = 1000  # Giới hạn nhảy tick để lọc nhiễu
        self.consecutive_errors = 0
        self.max_consecutive_errors = 10

        # ----- Luồng đọc Serial -----
        self.running = True
        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

        # ----- Timer để check connection -----
        self.create_timer(5.0, self.check_connection)

    def cmd_vel_callback(self, msg: Twist):
        """Nhận lệnh /cmd_vel và gửi xuống ESP32"""
        try:
            v = msg.linear.x
            w = msg.angular.z
            
            # Tính vận tốc cho từng bánh
            v_l = v - w * self.wheel_base / 2.0
            v_r = v + w * self.wheel_base / 2.0
            
            # Chuyển sang PWM (0-255)
            pwm_l = int(v_l / self.max_wheel_speed * 255.0)
            pwm_r = int(v_r / self.max_wheel_speed * 255.0)
            
            # Giới hạn PWM
            pwm_l = max(min(pwm_l, 255), -255)
            pwm_r = max(min(pwm_r, 255), -255)

            command = f"{pwm_l},{pwm_r}\n"
            
            if self.ser and self.ser.is_open:
                self.ser.write(command.encode('utf-8'))
                
        except Exception as e:
            self.get_logger().error(f"Lỗi gửi lệnh: {e}")

    def validate_encoder_data(self, ticks_l, ticks_r):
        """Kiểm tra tính hợp lệ của dữ liệu encoder"""
        if self.prev_ticks_l is None:
            return True
            
        delta_l = abs(ticks_l - self.prev_ticks_l)
        delta_r = abs(ticks_r - self.prev_ticks_r)
        
        # Kiểm tra nhảy tick quá lớn (có thể do nhiễu)
        if delta_l > self.max_tick_jump or delta_r > self.max_tick_jump:
            self.get_logger().warn(f"Tick jump detected: L={delta_l}, R={delta_r}")
            return False
            
        return True

    def read_from_serial(self):
        """Thread đọc dữ liệu từ ESP32"""
        while rclpy.ok() and self.running:
            try:
                if not self.ser or not self.ser.is_open:
                    time.sleep(0.1)
                    continue
                    
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if not line:
                        continue

                    # Publish raw encoder data
                    msg_raw = String()
                    msg_raw.data = line
                    self.encoder_pub.publish(msg_raw)

                    # Parse encoder data
                    if not self.parse_and_process_encoder(line):
                        self.consecutive_errors += 1
                        if self.consecutive_errors > self.max_consecutive_errors:
                            self.get_logger().error("Quá nhiều lỗi liên tiếp, check ESP32!")
                    else:
                        self.consecutive_errors = 0

                time.sleep(0.001)  # Giảm CPU usage

            except Exception as e:
                self.get_logger().error(f"Lỗi đọc serial: {e}")
                time.sleep(0.1)

    def parse_and_process_encoder(self, line):
        """Parse và xử lý dữ liệu encoder"""
        try:
            parts = line.split(',')
            if len(parts) != 2:
                return False
                
            ticks_l = int(parts[0])
            ticks_r = int(parts[1])
            
            # Validation
            if not self.validate_encoder_data(ticks_l, ticks_r):
                return False

            # Lần đầu tiên - chỉ khởi tạo
            if self.prev_ticks_l is None:
                self.prev_ticks_l = ticks_l
                self.prev_ticks_r = ticks_r
                self.last_time = self.get_clock().now()
                return True

            # Tính toán odometry
            self.calculate_odometry(ticks_l, ticks_r)
            return True
            
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f"Parse error: {e}")
            return False

    def calculate_odometry(self, ticks_l, ticks_r):
        """Tính toán và publish odometry"""
        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nanoseconds * 1e-9
        
        if dt <= 0:
            return
            
        # Tính delta ticks
        delta_l = ticks_l - self.prev_ticks_l
        delta_r = ticks_r - self.prev_ticks_r
        
        # Update previous values
        self.prev_ticks_l = ticks_l
        self.prev_ticks_r = ticks_r
        self.last_time = curr_time

        # Chuyển đổi ticks sang khoảng cách (m)
        circumference = 2.0 * math.pi * self.wheel_radius
        d_l = (delta_l / self.pulses_per_rev) * circumference
        d_r = (delta_r / self.pulses_per_rev) * circumference

        # Tính toán differential kinematics
        dc = (d_r + d_l) / 2.0  # Khoảng cách trung tâm
        dtheta = (d_r - d_l) / self.wheel_base  # Thay đổi góc

        # Cập nhật pose
        self.theta += dtheta
        self.x += dc * math.cos(self.theta)
        self.y += dc * math.sin(self.theta)

        # Tính vận tốc
        v_linear = dc / dt
        v_angular = dtheta / dt

        # Tạo Odometry message
        self.publish_odometry(curr_time, v_linear, v_angular)
        
        # Broadcast TF
        self.broadcast_tf(curr_time)

    def publish_odometry(self, timestamp, v_linear, v_angular):
        """Publish odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Velocity
        odom_msg.twist.twist.linear.x = v_linear
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = v_angular

        # Covariance matrices (có thể điều chỉnh theo độ chính xác thực tế)
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[35] = 0.02  # yaw
        odom_msg.twist.covariance[0] = 0.01  # vx
        odom_msg.twist.covariance[35] = 0.02 # vyaw

        self.odom_pub.publish(odom_msg)

    def broadcast_tf(self, timestamp):
        """Broadcast TF transform"""
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def check_connection(self):
        """Kiểm tra kết nối ESP32 định kỳ"""
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Mất kết nối với ESP32, đang thử kết nối lại...")
            self.reconnect_esp32()

    def reconnect_esp32(self):
        """Thử kết nối lại ESP32"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                
            new_port = find_esp32_port()
            if new_port:
                self.ser = serial.Serial(new_port, self.baudrate, timeout=self.serial_timeout)
                time.sleep(1)
                self.ser.flushInput()
                self.get_logger().info(f"Đã kết nối lại ESP32 tại {new_port}")
            else:
                self.get_logger().error("Không tìm thấy ESP32")
                
        except Exception as e:
            self.get_logger().error(f"Lỗi kết nối lại ESP32: {e}")

    def reset_odometry(self):
        """Reset odometry về gốc tọa độ"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.get_logger().info("Đã reset odometry")

    def destroy_node(self):
        """Cleanup khi tắt node"""
        self.running = False
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()