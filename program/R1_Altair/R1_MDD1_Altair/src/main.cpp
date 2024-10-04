#include "mbed.h"
#include "MotorDriver.h"

DigitalOut led(LED1);
BufferedSerial pc_serial(USBTX, USBRX, 115200); // PCと通信するシリアルポート
CAN can(PB_12, PB_13, (int)1e6);                // CAN通信ピン設定、通信速度1Mbps
uint32_t id_vel = 0x300;                        // Vx, Vy, ωのCANメッセージID
uint32_t id_cmd = 0x100;                        // 指示番号とモードのCANメッセージID
uint32_t id_pos = 0x101;                        // X, Y, θのCANメッセージID
uint32_t id_act = 0x102;                        // 動作番号のCANメッセージID
uint32_t id_sw = 0x105;                         // リミットスイッチのCANメッセージID

uint8_t command_number = 0; // 指示番号
uint8_t mode = 0;           // モード
int16_t Vx = 0;
int16_t Vy = 0;
int16_t omega = 0;
uint8_t data[8];                     // CANで送信するデータ
uint8_t data_cmd[2];                 // CANで送信するデータ
const char HEADER[2] = {0xA5, 0xA5}; // ヘッダーマーカー

int16_t X = 0;             // 現在位置X
int16_t Y = 0;             // 現在位置Y
int16_t theta = 0;         // 現在の角度θ
uint8_t action_number = 0; // 動作番号
uint8_t d1 = 0;
uint8_t d2 = 0;
uint8_t d3 = 0;
uint8_t a = 0;
uint8_t b = 0;
uint8_t flag1 = 0;
uint8_t flag2 = 0;

MotorDriver motor1(PA_8, PA_11);
MotorDriver motor2(PB_14, PB_15);

bool receive_serial_data()
{
    char header[2] = {0, 0}; // ヘッダー読み取り用バッファ
    char serial_data[8];     // データ読み取り用バッファ（指示番号, モード, Vx, Vy, ω）

    // ヘッダーが見つかるまで待機
    while (true)
    {
        // 1バイト読み取り
        char byte_in;
        if (pc_serial.read(&byte_in, 1))
        {
            header[0] = header[1]; // シフトして過去のバイトを保存
            header[1] = byte_in;   // 新しいバイトを保存

            // ヘッダーが一致するか確認
            if (header[0] == HEADER[0] && header[1] == HEADER[1])
            {
                break; // ヘッダーが見つかったのでループを抜ける
            }
        }
    }

    // データを読み取る
    if (pc_serial.read(serial_data, 8) == 8)
    { // 8バイトのデータを読み取り
        // データを変数に格納
        command_number = serial_data[0];                // 指示番号
        mode = serial_data[1];                          // モード
        Vx = (serial_data[2] << 8) | serial_data[3];    // X方向の速度
        Vy = (serial_data[4] << 8) | serial_data[5];    // Y方向の速度
        omega = (serial_data[6] << 8) | serial_data[7]; // 回転速度

        return true; // 正常にデータを受信
    }
    else
    {
        return false; // データが正しく受信されなかった場合
    }
}

void send_data_to_pc()
{
    int16_t x_mm = static_cast<int16_t>(X);          // X座標をmm単位で送信
    int16_t y_mm = static_cast<int16_t>(Y);          // Y座標をmm単位で送信
    int16_t theta_deg = static_cast<int16_t>(theta); // θを度単位で送信

    // PCに送信するためのデータ配列を準備
    char send_data[10];
    send_data[0] = HEADER[0];        // ヘッダー
    send_data[1] = HEADER[1];        // ヘッダー
    send_data[2] = action_number;    // 動作番号
    send_data[3] = mode;             // モード
    send_data[4] = (x_mm >> 8);      // Xの上位バイト
    send_data[5] = x_mm & 0xFF;      // Xの下位バイト
    send_data[6] = (y_mm >> 8);      // Yの上位バイト
    send_data[7] = y_mm & 0xFF;      // Yの下位バイト
    send_data[8] = (theta_deg >> 8); // thetaの上位バイト
    send_data[9] = theta_deg & 0xFF; // thetaの下位バイト

    // シリアルポートを通してデータを送信
    pc_serial.write(send_data, sizeof(send_data));
}

void check_can_message()
{
    CANMessage msg;
    if (can.read(msg))
    {
        if (msg.id == id_pos)
        {
            // id_pos メッセージの受信
            X = (msg.data[0] << 8) | msg.data[1];     // X = 上位バイト << 8 | 下位バイト
            Y = (msg.data[2] << 8) | msg.data[3];     // Y = 上位バイト << 8 | 下位バイト
            theta = (msg.data[4] << 8) | msg.data[5]; // θ = 上位バイト << 8 | 下位バイト
        }
        else if (msg.id == id_act)
        {
            action_number = msg.data[0]; // 動作番号
        }
        else if (msg.id == id_sw)
        {
            d1 = msg.data[0];
            d2 = msg.data[1];
            d3 = msg.data[2];
            led = 1;
        }
        else if (msg.id == id_cmd)
        {
            a = msg.data[0];
        }
        else if (msg.id == id_vel)
        {
            b = msg.data[0];
        }
        else
        {
            led = 0;
        }
    }
    else
    {
        led = 0; // メッセージを受信できなかった場合
    }
}

void control_motor(uint8_t cmd)
{
    switch (cmd)
    {
    case 1: // 電磁弁1(R100)オン
        led = 1;
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;

    case 2: // 電磁弁2(巻き取り)をオン
        led = 0;
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;

    case 3: // リミット３に押されるまで前進，押されたら停止＆電磁弁3(加速) 電磁弁4(ロック)をオン
        led = 1;
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;

    case 4: // 電磁弁2(巻き取り) 電磁弁3(加速)をオフ
        led = 1;
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;

    case 5: // モータ1(パワウィンド)を回転、リミット1(スライダ)に当たると停止
        if (flag1 == 0)
        {
            if (d1 == 0)
            {
                motor1.setSpeed(40);
                motor2.setSpeed(0);
                led = 0;
                break;
            }
            else
            {
                motor1.setSpeed(0);
                motor2.setSpeed(0);
                flag1 = 1;
                led = 0;
                break;
            }
        }
        else
        {
            motor1.setSpeed(0);
            motor2.setSpeed(0);
            led = 0;
            break;
        }
    case 6: // 電磁弁4(ロック)をオフ
        led = 1;
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;

    case 7: // // 電磁弁2(巻き取り)をオン
        led = 0;
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;
    case 8: // モーターを一瞬逆回転
        if (flag1 == 0)
        {

            motor1.setSpeed(-40);
            motor2.setSpeed(0);
            led = 0;
            auto now1 = HighResClock::now();
            static auto pre1 = now1;
            if (now1 - pre1 > 800ms)
            {
                flag1 = 1;
                break;
            }
            else
            {
                flag1 = 0;
                pre1 = now1;
                break;
            }
        }
        else
        {
            motor1.setSpeed(0);
            motor2.setSpeed(0);
            led = 0;
            break;
        }

    case 9: // 手動で装填
        led = 1;
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;

    case 10: // リミット３に押されるまで前進，押されたら停止＆電磁弁3(加速) 電磁弁4(ロック)をオン
        led = 0;
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;

    case 11: // すべてOFF
        led = 1;
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;

    case 20:
        led = 1;
        motor1.setSpeed(-40);
        motor2.setSpeed(0);
        break;

    default:
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;
    }
}

int main()
{
    while (true)
    {
        control_motor(command_number);

        // シリアルデータを受信する
        bool data_received = receive_serial_data();

        // データをバイト配列に格納（上位バイトを先に）
        data[0] = (Vx >> 8) & 0xFF;    // Vxの上位バイト
        data[1] = Vx & 0xFF;           // Vxの下位バイト
        data[2] = (Vy >> 8) & 0xFF;    // Vyの上位バイト
        data[3] = Vy & 0xFF;           // Vyの下位バイト
        data[4] = (omega >> 8) & 0xFF; // omegaの上位バイト
        data[5] = omega & 0xFF;        // omegaの下位バイト
        data[6] = 0;
        data[7] = 0;

        data_cmd[0] = command_number;
        data_cmd[1] = mode;

        // CANメッセージをチェックして処理
        check_can_message();

        // 1msごとにCAN通信を行う
        auto now = HighResClock::now();
        static auto pre = now;
        if (now - pre > 1ms)
        {
            if (data_received)
            {
                // CANメッセージの送信
                CANMessage msg_vel{id_vel, data, 8};
                can.write(msg_vel);
                CANMessage msg_cmd{id_cmd, data_cmd, 2};
                can.write(msg_cmd);
            }
            // PCにデータを送信
            send_data_to_pc();

            // 前回の実行時間を更新
            pre = now;
        }
    }
}
