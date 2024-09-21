#include "mbed.h"

DigitalOut led(LED1);
DigitalOut solenoid1(PB_6); // 電磁弁1制御ピン
DigitalOut solenoid2(PB_7); // 電磁弁2制御ピン
DigitalOut solenoid3(PB_8); // 電磁弁3制御ピン
DigitalOut solenoid4(PB_9); // 電磁弁4制御ピン

CAN can(PB_12, PB_13, (int)1e6); // CAN通信ピン設定、通信速度1Mbps
uint32_t id_cmd = 0x100;         // 指示番号とモードのCANメッセージID
uint32_t id_vel = 0x300;         // Vx, Vy, ωのCANメッセージID
uint32_t id_pos = 0x101;         // X, Y, θのCANメッセージID
uint32_t id_act = 0x102;         // 動作番号のCANメッセージID
uint32_t id_sw = 0x105;          // リミットスイッチのCANメッセージID

uint8_t command_number = 0; // 指示番号
uint8_t mode = 0;           // モード
uint16_t Vx = 0;
int16_t Vy = 0;
int16_t omega = 0;
uint16_t X = 0;            // 現在位置X
uint16_t Y = 0;            // 現在位置Y
int16_t theta = 0;         // 現在の角度θ
uint8_t action_number = 0; // 動作番号
uint8_t d1 = 0;
uint8_t d2 = 0;
uint8_t d3 = 0;
uint8_t a = 0;
uint8_t b = 0;

void control_solenoid(uint8_t cmd)
{
    switch (cmd)
    {
    case 1:
        action_number = 1;
        solenoid1 = 0;
        solenoid2 = 1;
        led = 1;
        break;

    case 2:
        action_number = 2;
        solenoid1 = 1;
        solenoid2 = 1;
        led = 0;
        break;

    case 3:
        action_number = 3;
        solenoid1 = 1;
        solenoid2 = 0;
        led = 1;
        break;

    case 4:
        action_number = 4;
        solenoid1 = 1;
        solenoid2 = 0;
        led = 0;
        break;

    case 5:
        action_number = 5;
        solenoid1 = 0;
        solenoid2 = 0;
        led = 1;
        break;

    case 6:
        action_number = 6;
        solenoid1 = 0;
        solenoid2 = 0;
        led = 0;
        break;

    case 7:
        action_number = 7;
        solenoid1 = 0;
        solenoid2 = 0;
        led = 1;
        break;

    case 8:
        action_number = 8;
        solenoid1 = 0;
        solenoid2 = 1;
        led = 0;
        break;

    case 9:
        action_number = 9;
        solenoid1 = 1;
        solenoid2 = 1;
        led = 1;
        break;

    default:
        solenoid1 = 0;
        solenoid2 = 0;
        break;
    }
}

void check_can_message()
{
    CANMessage msg;

    if (can.read(msg))
    {
        if (msg.id == id_cmd)
        {
            // id_cmd メッセージの受信
            command_number = msg.data[0]; // 指示番号
            mode = msg.data[1];           // モード
        }
        else if (msg.id == id_vel)
        {
            // id_vel メッセージの受信
            Vx = (msg.data[0] << 8) | msg.data[1];    // Vx = 上位バイト << 8 | 下位バイト
            Vy = (msg.data[2] << 8) | msg.data[3];    // Vy = 上位バイト << 8 | 下位バイト
            omega = (msg.data[4] << 8) | msg.data[5]; // ω = 上位バイト << 8 | 下位バイト
        }
        else if (msg.id == id_pos)
        {
            // id_pos メッセージの受信
            X = (msg.data[0] << 8) | msg.data[1];     // X = 上位バイト << 8 | 下位バイト
            Y = (msg.data[2] << 8) | msg.data[3];     // Y = 上位バイト << 8 | 下位バイト
            theta = (msg.data[4] << 8) | msg.data[5]; // θ = 上位バイト << 8 | 下位バイト
        }
        else if (msg.id == id_sw)
        {
            d1 = msg.data[0];
            d2 = msg.data[1];
            d3 = msg.data[2];
        }
        else if (msg.id == id_act)
        {
            a = msg.data[0];
        }
        else
        {
            led = 0; // 未知のIDのためLEDをOFF
        }
    }
    else
    {
        // メッセージを受信できなかった場合
        led = 0; // 受信失敗のためLEDをOFF
    }
}

int main()
{
    while (true)
    {
        check_can_message(); // CANメッセージをチェックして処理
        control_solenoid(command_number);

        // 1msごとにCAN通信を行う
        auto now = HighResClock::now();
        static auto pre = now;
        if (now - pre > 2ms)
        {
            {
                // 動作番号をCANで送信
                uint8_t data[1] = {action_number};
                can.write(CANMessage(id_act, data, 1));
                // 前回の実行時間を更新
                pre = now;
            }
        }
    }
}
