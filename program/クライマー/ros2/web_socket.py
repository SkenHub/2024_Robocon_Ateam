import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse
import uvicorn
import os
from socket import SO_REUSEADDR, SOL_SOCKET, socket

# ipアドレス、ポートの指定
ipadress_ = '172.20.10.2'
port_ = 8000

# HTMLファイルのパスを指定
path = '/home/hori/climber_ws/src/communication_pkg/communication_pkg/R2_UI.html'

# FastAPIのインスタンスを作成
app = FastAPI()

# HTMLファイルが存在するか確認し、読み込み
if not os.path.exists(path):
    raise FileNotFoundError(f'File not found: {path}')

with open(path, 'r') as f:
    html = f.read()

# ROS 2 ノードの定義
class MobileController(Node):
    def __init__(self):
        super().__init__('mobile_controller')
        self.send_data = ''

        # パブリッシャーを作成
        self.pub = self.create_publisher(String, 'web_socket_pub', 10)
        
        # サブスクリプションを作成し、コールバック関数を設定
        self.sub = self.create_subscription(String, 'web_socket_sub', self.callback, 10)

        # FastAPIルートの定義
        @app.get("/")
        async def get():
            return HTMLResponse(html)
        
        # WebSocketエンドポイントの定義
        @app.websocket('/ws')
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            try:
                while True:
                    # クライアントからのデータを受信
                    receive_data = await websocket.receive_text()
                    print(receive_data)
                    
                    # 受信したデータをROSトピックにパブリッシュ
                    msg = String()
                    msg.data = receive_data
                    self.pub.publish(msg)
                    # サブスクライブしたデータをクライアントに送信
                    await websocket.send_text(self.send_data)
            except Exception as e:
                print(f'WebSocket error: {str(e)}')
            finally:
                print('WebSocket disconnected')

    # サブスクリプションのコールバック関数
    def callback(self, sub_msg):
        self.send_data = sub_msg.data

# ROS 2ノードを実行する関数
def run_ros2():
    rclpy.init()
    node = MobileController()
    rclpy.spin(node)
    rclpy.shutdown()

# FastAPIサーバーを実行する関数
# FastAPIサーバーを実行する関数
def run_fastapi():
    # Uvicornの設定を作成
    config = uvicorn.Config(app, host=ipadress_, port=port_, log_level="info")

    # Uvicornサーバーを作成
    server = uvicorn.Server(config)

    # FastAPIサーバーを起動
    server.run()


# メイン関数
def main():
    # ROS 2のspinをメインスレッドで実行
    ros2_thread = threading.Thread(target=run_ros2)
    ros2_thread.start()

    # FastAPIサーバーを別のスレッドで実行
    fastapi_thread = threading.Thread(target=run_fastapi)
    fastapi_thread.start()

    # 両方のスレッドが終了するのを待つ
    ros2_thread.join()
    fastapi_thread.join()

if __name__ == '__main__':
    main()
