import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialSend(Node):
    def __init__(self):
        super().__init__('serial_send')
        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
        self.sub = self.create_subscription(String, 'web_socket_pub', self.callback, 10)

    def callback(self, sub_msg):
        data_list = [int(item) for item in sub_msg.data.split(',')]
        bucket_mode = 0  # 初期化

        if data_list[5]:  # バケット上昇 R2
            bucket_mode = 2
        if data_list[6]:  # バケット下降 L2
            bucket_mode = 4
        if data_list[7]:  # バケット途中停止 R1
            bucket_mode = 8

        # データ送信
        send_data = bytearray(6)
        send_data[0] = 0xA5
        send_data[1] = 0xA5
        send_data[2] = data_list[0]
        send_data[3] = data_list[1]
        send_data[4] = data_list[2]
        send_data[5] = (data_list[8] << 5) | (data_list[3] << 4) | bucket_mode | data_list[4]

        self.ser.write(send_data)
        self.get_logger().info(f'Initial data sent: {list(send_data)}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialSend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
