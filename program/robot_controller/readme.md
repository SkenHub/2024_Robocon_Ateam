# robot_controller

## 概要
`robot_controller`は、ロボットの移動を制御するためのROS2パッケージです。このパッケージは、WebSocket経由でコントローラーから指示を受け取り、指定された地点にロボットを移動させます。また、ロボットの自己位置をマイコンから受信し、ROS2トピックに公開します。

## システム構成
### ノード構成
- `web_socket_node`：WebSocket通信を介してコントローラーから指示を受け取り、ROS2トピックにパブリッシュします。
- `controller_node`：WebSocketノードからの指示を受け取り、ロボットを指定された地点に移動させるための速度と方向を計算し、シリアル通信ノードに送信します。
- `cmd_vel_to_serial_node`：`controller_node`からの移動指示を受け取り、シリアル通信を介してマイコンに送信します。
- `serial_to_position_node`：マイコンから自己位置データを受信し、ROS2トピックにパブリッシュします。

## 通信の概要
### トピック
- `/web_socket_pub` (`std_msgs/String`)：WebSocketノードからの指示をパブリッシュ。
- `/cmd_vel` (`std_msgs/Float32MultiArray`)：`controller_node`が計算した移動指示をパブリッシュ。
- `/robot_position` (`std_msgs/Float32MultiArray`)：`serial_to_position_node`が受信した自己位置をパブリッシュ。

### メッセージの内容
- `/web_socket_pub`：JSON形式の文字列。例：`{"point": "1"}`。
- `/cmd_vel`：速度と方向。例：`[1.0, 0.5]`。
- `/robot_position`：ロボットの自己位置（x, y, θ）。例：`[1500.0, 1500.0, 0.0]`。

## ノードの役割
### `web_socket_node`
- WebSocket通信を介してコントローラーからの指示を受け取る。
- 受け取った指示を`/web_socket_pub`トピックにパブリッシュ。

### `controller_node`
- `web_socket_node`からの指示を受け取り、指定された地点に移動するための速度と方向を計算。
- 計算結果を`/cmd_vel`トピックにパブリッシュ。

### `cmd_vel_to_serial_node`
- `/cmd_vel`トピックから移動指示を受け取り、シリアル通信でマイコンに送信。

### `serial_to_position_node`
- マイコンから自己位置データをシリアル通信で受信。
- 受信したデータを`/robot_position`トピックにパブリッシュ。

## セットアップとビルド
1. ROS2ワークスペースの作成
    ```bash
    mkdir -p ~/ros2_robocon_ws/src
    cd ~/ros2_robocon_ws/
    colcon build
    source install/setup.bash
    ```

2. パッケージの作成
    ```bash
    cd ~/ros2_robocon_ws/src
    ros2 pkg create --build-type ament_python robot_controller
    ```

3. 必要なファイルの配置
    ```bash
    cd ~/ros2_robocon_ws/src/robot_controller/robot_controller
    ```

    `controller_node.py`、`cmd_vel_to_serial_node.py`、`serial_to_position_node.py`をこのディレクトリにコピーします。

4. `setup.py`の編集
    ```python
    from setuptools import setup

    package_name = 'robot_controller'

    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        py_modules=[
            'robot_controller.controller_node',
            'robot_controller.cmd_vel_to_serial_node',
            'robot_controller.serial_to_position_node'
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='your_name',
        maintainer_email='your_email@example.com',
        description='ROS2 package for robot control',
        license='Apache License 2.0',
        entry_points={
            'console_scripts': [
                'controller_node = robot_controller.controller_node:main',
                'cmd_vel_to_serial_node = robot_controller.cmd_vel_to_serial_node:main',
                'serial_to_position_node = robot_controller.serial_to_position_node:main'
            ],
        },
    )
    ```

5. パッケージのビルド
    ```bash
    cd ~/ros2_robocon_ws/
    colcon build
    source install/setup.bash
    ```

6. ノードの実行
    ```bash
    ros2 run robot_controller controller_node
    ros2 run robot_controller cmd_vel_to_serial_node
    ros2 run robot_controller serial_to_position_node
    ```

## 注意事項
- シリアルポートの設定や接続には注意が必要です。正しいポートを指定してください。
- WebSocketのIPアドレスとポートは環境に応じて変更してください。

## ライセンス
Apache License 2.0

---



