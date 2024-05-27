import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import json
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.subscription = self.create_subscription(
            String,
            'web_socket_pub',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'cmd_vel', 10)

        # 地点の座標を設定
        self.locations = {
            '1': [1500, 1500, 0],  # [x, y, theta]
            '2': [5000, 3000, 0],
            '3': [5000, 3000, 0]
        }
        self.current_position = [0, 0, 0]  # 初期位置 [x, y, theta]

    def listener_callback(self, msg):
        # 受け取ったデータをパース
        try:
            data = json.loads(msg.data)
            point = data.get('point')

            if point in self.locations:
                target = self.locations[point]
                self.move_to_target(target)
            else:
                self.get_logger().info(f"Unknown point: {point}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {str(e)}")

    def move_to_target(self, target):
        # 目標地点までの移動計算
        x, y, theta = target
        dx = x - self.current_position[0]
        dy = y - self.current_position[1]
        direction = math.atan2(dy, dx)
        distance = math.sqrt(dx**2 + dy**2)

        # 移動指示をパブリッシュ
        msg = Float32MultiArray()
        msg.data = [distance, direction, theta]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Moving to {target} with command: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
