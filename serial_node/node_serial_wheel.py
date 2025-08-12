#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import serial
import time

class CmdVelSerialNode(Node):
    def __init__(self):
        super().__init__('agent_serial_cmd_vel')

        try:
            self.ser = serial.Serial('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', 115200, timeout=1)  # Ganti port sesuai dengan device Anda
            self.get_logger().info('Serial port opened successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.wheel1_pub = self.create_publisher(Float64MultiArray, 'wheel1_controller/commands', 10)
        self.wheel2_pub = self.create_publisher(Float64MultiArray, 'wheel2_controller/commands', 10)
        self.wheel3_pub = self.create_publisher(Float64MultiArray, 'wheel3_controller/commands', 10)

        # Timer untuk membaca serial setiap 100 ms
        self.timer = self.create_timer(0.1, self.read_serial_data)
    
    def cmd_vel_callback(self, msg: Twist):
        lin_x = msg.linear.x
        lin_y = msg.linear.y
        lin_z = msg.linear.z
        ang_x = msg.angular.x
        ang_y = msg.angular.y
        ang_z = msg.angular.z

        serial_data = f"{lin_x:.2f},{lin_y:.2f},{lin_z:.2f},{ang_x:.2f},{ang_y:.2f},{ang_z:.2f}\r\n"

        # self.get_logger().info(f'Sending: {serial_data.strip()}')

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(serial_data.encode())
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to send data: {e}')
        else:
            self.get_logger().warn('Serial port is not open.')
            
    def read_serial_data(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                data = self.ser.readline().decode().strip()
                values = data.split(",")  # Hasilnya list string

                # Convert ke float atau int sesuai kebutuhan
                linear_x = float(values[0])
                linear_y = float(values[1])
                angular_z = float(values[2])
                V1 = float(values[3])*10
                V2 = float(values[4])*10
                V3 = float(values[5])*10

                if data:
                    self.get_logger().info(f"Received from serial: {data}")
                    msg1 = Float64MultiArray()
                    msg2 = Float64MultiArray()
                    msg3 = Float64MultiArray()
                    
                    # Example commands (adjust as needed)
                    msg1.data = [float(V1)]  # Wheel 1 speed
                    msg2.data = [float(V2)] # Wheel 2 speed
                    msg3.data = [float(V3)]  # Wheel 3 speed

                    self.wheel1_pub.publish(msg1)
                    self.wheel2_pub.publish(msg2)
                    self.wheel3_pub.publish(msg3)
            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()