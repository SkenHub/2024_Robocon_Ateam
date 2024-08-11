#include <mbed.h>
#include "pid.hpp"
#include "encoder.hpp"

#define M_PI 3.14159
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f


struct PS3_data_{
	int Up,Down,Right,Left,Triangle,Cross,Circle,Square,L1,L2,R1,R2,Start,Select;
	int LxPad,LyPad,RxPad,RyPad;
};

PS3_data_ PS3_data;

BufferedSerial pc(USBTX, USBRX);
BufferedSerial ps3(PA_9, PA_10);

CAN can_1{PA_11, PA_12, (int)1e6},can_2{PB_12, PB_13, (int)1e6};
DigitalOut led{LED1};
Pid pid;
uint8_t data_6020[8], cybergear[2][8],ps3_data[8];

double deg_6020,target_6020;

uint32_t cyber_id[2] = {0x7F, 0x7E}, out_id[2], master_id = 0x55;

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void motor_turn(void)
{
    CANMessage msg_6020{0x1ff, reinterpret_cast<char *>(data_6020), sizeof(data_6020)};
    CANMessage cybergear_1{out_id[0], reinterpret_cast<char *>(cybergear[0]), sizeof(cybergear[0])};
    CANMessage cybergear_2{out_id[1], reinterpret_cast<char *>(cybergear[1]), sizeof(cybergear[1])};
    cybergear_1.format = CANExtended;
    cybergear_2.format = CANExtended;

    can_1.write(msg_6020);
    can_2.write(cybergear_1);
    can_2.write(cybergear_2);
}

void motor_read(void)
{
    CANMessage msg_6020;
    CANMessage cybergear_1;
    CANMessage cybergear_2;

    can_1.read(msg_6020);
    can_2.read(cybergear_1);
    can_2.read(cybergear_2);

    deg_6020 = (float)((msg_6020.data[0] << 8) | (msg_6020.data[1] & 0xff)) / 8192 * 360;
}

void reset_motor()
{
    out_id[0] = (4 << 24) | (master_id << 8) | (cyber_id[0]);
    out_id[1] = (4 << 24) | (master_id << 8) | (cyber_id[1]);
    for (uint8_t i = 0; i < 8; i++)
    {
        cybergear[i % 2][i] = 0;
    }
    motor_turn();
}

void enable_motor()
{
    out_id[0] = (3 << 24) | (master_id << 8) | (cyber_id[0]);
    out_id[1] = (3 << 24) | (master_id << 8) | (cyber_id[1]);
    for (uint8_t i = 0; i < 8; i++)
    {
        cybergear[i % 2][i] = 0;
    }
    motor_turn();
}

void set_zero()
{
    out_id[0] = (6 << 24) | (master_id << 8) | (cyber_id[0]);
    out_id[1] = (6 << 24) | (master_id << 8) | (cyber_id[1]);
    for (uint8_t i = 0; i < 8; i++)
    {
        cybergear[i % 2][i] = 0;
    }
    motor_turn();
}

void cyber_data_set(double torque, double target_deg, double speed, double Kp, double Kd, int cyber_number)
{
    uint16_t torque_uint = float_to_uint(torque, T_MIN, T_MAX, 16);
    if (cyber_number == 1)
    {
        out_id[0] = (1 << 24) | (torque_uint << 8) | (cyber_id[0]);
        cybergear[0][0] = float_to_uint(target_deg, P_MIN, P_MAX, 16) >> 8;
        cybergear[0][1] = float_to_uint(target_deg, P_MIN, P_MAX, 16) & 0xFF;
        cybergear[0][2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
        cybergear[0][3] = float_to_uint(speed, V_MIN, V_MAX, 16) & 0xFF;
        cybergear[0][4] = float_to_uint(Kp, KP_MIN, KP_MAX, 16) >> 8;
        cybergear[0][5] = float_to_uint(Kp, KP_MIN, KP_MAX, 16) & 0xFF;
        cybergear[0][6] = float_to_uint(Kd, KD_MIN, KD_MAX, 16) >> 8;
        cybergear[0][7] = float_to_uint(Kd, KD_MIN, KD_MAX, 16) & 0xFF;
    }
    else if (cyber_number == 2)
    {
        out_id[1] = (1 << 24) | (torque_uint << 8) | (cyber_id[0]);
        cybergear[1][0] = float_to_uint(target_deg, P_MIN, P_MAX, 16) >> 8;
        cybergear[1][1] = float_to_uint(target_deg, P_MIN, P_MAX, 16) & 0xFF;
        cybergear[1][2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
        cybergear[1][3] = float_to_uint(speed, V_MIN, V_MAX, 16) & 0xFF;
        cybergear[1][4] = float_to_uint(Kp, KP_MIN, KP_MAX, 16) >> 8;
        cybergear[1][5] = float_to_uint(Kp, KP_MIN, KP_MAX, 16) & 0xFF;
        cybergear[1][6] = float_to_uint(Kd, KD_MIN, KD_MAX, 16) >> 8;
        cybergear[1][7] = float_to_uint(Kd, KD_MIN, KD_MAX, 16) & 0xFF;
    }
    motor_turn();
}

void PS3(void){
    if(ps3_data[0] == 0x80){
		PS3_data.Up 		= ((ps3_data[2]&0x03) == 0x01)? true : false;
		PS3_data.Down 		= ((ps3_data[2]&0x03) == 0x02)? true : false;
		PS3_data.Right 	= ((ps3_data[2]&0x0C) == 0x04)? true : false;
		PS3_data.Left 		= ((ps3_data[2]&0x0C) == 0x08)? true : false;
		PS3_data.Triangle 	= (ps3_data[2]&0x10)? true : false;
		PS3_data.Cross 	= (ps3_data[2]&0x20)? true : false;
		PS3_data.Circle 	= (ps3_data[2]&0x40)? true : false;
		PS3_data.Square 	= (ps3_data[1]&0x01)? true : false;
		PS3_data.L1 		= (ps3_data[1]&0x02)? true : false;
		PS3_data.L2 		= (ps3_data[1]&0x04)? true : false;
		PS3_data.R1 		= (ps3_data[1]&0x08)? true : false;
		PS3_data.R2 		= (ps3_data[1]&0x10)? true : false;
		PS3_data.Start 	= ((ps3_data[2]&0x03) == 0x03)? true : false;
		PS3_data.Select 	= ((ps3_data[2]&0x0C) == 0x0C)? true : false;
		PS3_data.LxPad 	= ps3_data[3]-64;
		PS3_data.LyPad 	= -1*(ps3_data[4]-64);
		PS3_data.RxPad 	= ps3_data[5]-64;
		PS3_data.RyPad 	= -1*(ps3_data[6]-64);
	}else{
		PS3_data.Up 		= 0;
		PS3_data.Down 		= 0;
		PS3_data.Right 	= 0;
		PS3_data.Left 		= 0;
		PS3_data.Triangle 	= 0;
		PS3_data.Cross 	= 0;
		PS3_data.Circle 	= 0;
		PS3_data.Square 	= 0;
		PS3_data.L1 		= 0;
		PS3_data.L2 		= 0;
		PS3_data.R1 		= 0;
		PS3_data.R2 		= 0;
		PS3_data.Start 	= 0;
		PS3_data.Select 	= 0;
		PS3_data.LxPad 	= 0;
		PS3_data.LyPad 	= 0;
		PS3_data.RxPad 	= 0;
		PS3_data.RyPad 	= 0;
	}
}

int main()
{
    double torque[2] = {1.0,1.0}, target_deg[2] = {0,0}, speed[2] = {40,40}, KP[2] = {500,500}, KD[2] = {1.0,1.0};

    pid.setGain(180, 0.4, 0.05);

    reset_motor();
    enable_motor();
    set_zero();

    while (true)
    {
        ps3.read(&ps3_data, 8);
        PS3();

        if(PS3_data.R2){
            target_deg[0] = 40;
            target_deg[1] = -40;
            speed[0] = 200;
            speed[1] = 200;
            target_6020 = 200;
        }else if(PS3_data.L2){
            target_deg[0] = -2;
            target_deg[1] = -2;
            speed[0] = 40;
            speed[1] = 40;
            target_6020 = 110;
        }else if(PS3_data.R1){
            target_deg[0] = 0;
            target_deg[1] = 0;
            speed[0] = 40;
            speed[1] = 40;
            target_6020 = 100;
        }

        data_6020[0] = ((short)(pid.control(target_6020, deg_6020, 1)) >> 8);
        data_6020[1] = (short)(pid.control(target_6020, deg_6020, 1));
        cyber_data_set(torque[0], target_deg[0], speed[0], KP[0], KD[0], 1);
        cyber_data_set(torque[1], target_deg[1], speed[1], KP[1], KD[1], 2);
    }
}
