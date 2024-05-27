# cmd_vel_to_serial_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import serial

class CmdVelToSerialNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')
        self.ser = serial.Serial("/dev/ttyACM0", baudrate=9600)
        self.subscription = self.create_subscription(
            Float32MultiArray, 'cmd_vel', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        send_data = bytes([int(d * 100) for d in msg.data])  # データをバイト配列に変換
        self.ser.write(send_data)
        self.get_logger().info(f"Sent to Serial: {list(send_data)}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
