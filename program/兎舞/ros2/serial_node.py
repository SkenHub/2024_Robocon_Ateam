import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

class SerialTest(Node):
    def __init__(self):
        super().__init__('serial_pub_sub')
        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
        self.pub = self.create_publisher(Int32MultiArray, 'serial_pub', 10)
        self.sub = self.create_subscription(Int32MultiArray, 'serial_sub', self.callback, 10)

        while True:
            #byte = self.ser.read(1)]
            byte = 0
            if byte == b'\xA5':
                second_byte = self.ser.read(1)
                if second_byte == b'\xA5':
                    while self.ser.in_waiting < 10:
                        pass
                    data = self.ser.read(10)
                    received_data = list(data)
                    #print(f"Received data: {received_data}")
                    msg = Int32MultiArray()
                    msg.data = received_data
                    self.pub.publish(msg)

    def callback(self, sub_msg):
        self.get_logger().info(f"Received from serial_sub: {sub_msg.data}")
        send_data = bytearray(12)
        send_data[0] = 0xA5
        send_data[1] = 0xA5
        if len(sub_msg.data) == 6:
            for i in range(6):
                send_data[i + 2] = sub_msg.data[i]
        else:
            self.get_logger().error("Received data length is not 9 bytes.")
        self.ser.write(send_data)

def main(args=None):
    rclpy.init(args=args)
    node = SerialTest()
    rclpy.spin(node)
    node.ser.close()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
