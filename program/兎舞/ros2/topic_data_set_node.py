import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

class TopicDataSetNode(Node):
    def __init__(self):
        super().__init__('topic_data_set_node')
        self.serial_data = []

        # Subscriber
        self.serial_sub = self.create_subscription(Int32MultiArray, 'serial_pub', self.serial_callback, 10)
        self.websocket_sub = self.create_subscription(String, 'web_socket_pub', self.websocket_callback, 10)

        # Publisher
        self.serial_pub = self.create_publisher(Int32MultiArray, 'serial_sub', 10)

    def serial_callback(self, msg):
        self.serial_data = list(msg.data)
        #self.get_logger().info(f"Received from serial_pub: {self.serial_data}")

    def websocket_callback(self, msg):
        data_str = msg.data
        #self.get_logger().info(f"Received from web_socket_pub: {data_str}")

        # Process the string data as needed, here we simply convert it to a list of integers
        processed_data = self.process_data(data_str)
        
        # Prepare message to send
        send_msg = Int32MultiArray()
        send_msg.data = processed_data
        self.serial_pub.publish(send_msg)

    def process_data(self, data_str):
        try:
            # 文字列データをカンマで分割して整数のリストに変換
            data_list = [int(item) for item in data_str.split(',')]

            # スティック情報を1バイトに収める
            processed_data = [
                data_list[0],  # rx
                data_list[1],  # ry
                data_list[2],  # lx
                data_list[3]   # ly
            ]

            # ボタン情報をビット演算でまとめる
            # data_byte_2 (2バイト目) の設定
            data_byte_2 = 0
            data_byte_2 |= data_list[4] << 0  # cross
            data_byte_2 |= data_list[5] << 1  # circle
            data_byte_2 |= data_list[6] << 2  # square
            data_byte_2 |= data_list[7] << 3  # triangle
            data_byte_2 |= data_list[8] << 4  # l1
            data_byte_2 |= data_list[9] << 5  # r1
            data_byte_2 |= data_list[10] << 6 # l2
            data_byte_2 |= data_list[11] << 7 # r2

            # data_byte_3 (3バイト目) の設定
            data_byte_3 = 0
            data_byte_3 |= data_list[12] << 0  # create
            data_byte_3 |= data_list[13] << 1  # option
            data_byte_3 |= data_list[14] << 2  # l3
            data_byte_3 |= data_list[15] << 3  # r3
            data_byte_3 |= data_list[16] << 4  # up
            data_byte_3 |= data_list[17] << 5  # down
            data_byte_3 |= data_list[18] << 6  # left
            data_byte_3 |= data_list[19] << 7  # right

            # processed_data にボタン情報を追加
            processed_data.append(data_byte_2)
            processed_data.append(data_byte_3)

            print(processed_data)
            return processed_data

        except ValueError as e:
            #self.get_logger().error(f"Invalid data format received from web_socket_pub: {e}")
            return []

def main(args=None):
    rclpy.init(args=args)
    node = TopicDataSetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
