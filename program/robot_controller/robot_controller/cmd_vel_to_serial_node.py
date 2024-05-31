import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import serial
import struct

class CmdVelToSerialNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')
        self.ser = serial.Serial("/dev/ttyACM0", baudrate=9600)
        self.subscription = self.create_subscription(
            Float32MultiArray, 'cmd_vel', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

        # 速度、方向、角度を整数に変換
        speed = int(msg.data[0])
        direction = int(msg.data[1])
        angle = int(msg.data[2])

        # 整数値をバイト配列に変換
        send_data = struct.pack('BBB', speed, direction, angle)
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
