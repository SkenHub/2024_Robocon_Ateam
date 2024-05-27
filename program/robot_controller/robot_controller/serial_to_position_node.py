# serial_to_position_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import serial

class SerialToPositionNode(Node):
    def __init__(self):
        super().__init__('serial_to_position_node')
        self.ser = serial.Serial("/dev/ttyACM0", baudrate=9600)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_position', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting >= 12:  # 12バイトデータを期待
            received_data = list(self.ser.read(12))
            position_data = [d / 100.0 for d in received_data]  # データをスケールバック
            msg = Float32MultiArray(data=position_data)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Received from Serial and Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialToPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
