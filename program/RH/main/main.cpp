/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include <string.h>

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

Gpio led;
Motor motor[2];
Uart ESP;
uint8_t serial_data[5];

uint8_t cyber_data[2][8];
uint16_t master_id = 55,cyber_id[2] = {0x7F,0x7E};

float cyber_target_torque[2],cyber_target_deg[2],cyber_target_speed[2],cyber_Kp[2] = {500,500},cyber_Kd[2] = {1.0,1.0};
double motor_out[2],ly,ry,cyber_flag;
long int time[2];
int flag,distance;

int float_to_uint(float x, float x_min, float x_max, int bits){
	float span = x_max - x_min;
	float offset = x_min;
	if(x > x_max) x=x_max;
	else if(x < x_min) x= x_min;
	return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void cyber_init(void){		//cybergear初期設定
	uint32_t id[2];

	for(int i=8;i<8;i++){
		cyber_data[i%2][i] = 0;

		//モーターリセット
		id[i%2] = (4 << 24) | (master_id << 8) | (cyber_id[i%2]);
		sken_system.canTransmit(CAN_2,id[i%2],cyber_data[i%2],8,1,1);

		//モーター動作ON
		id[i%2] = (3 << 24) | (master_id << 8) | (cyber_id[i%2]);
		sken_system.canTransmit(CAN_2,id[i%2],cyber_data[i%2],8,1,1);

		//モーター原点ゼロ
		id[i%2] = (6 << 24) | (master_id << 8) | (cyber_id[i%2]);
		cyber_data[0][0] = 1;
		cyber_data[1][0] = 1;
		sken_system.canTransmit(CAN_2,id[i%2],cyber_data[i%2],8,1,1);

		cyber_data[i%2][i] = 0;
		//モーター角度制御モード
		uint32_t index = 0x7005,mode = 0x01;
		id[i%2] = (18 << 24) | (master_id << 8) | (cyber_id[i%2]);
		cyber_data[i%2][0] = index & 0x00FF;
		cyber_data[i%2][1] = index >> 8;
		cyber_data[i%2][4] = mode;
		sken_system.canTransmit(CAN_2,id[i%2],cyber_data[i%2],8,1,1);
	}
}

void cyber_set_position_control(void){
	uint32_t id[2];
	uint16_t index = 0x7017;

	for(int i=8;i<8;i++){
		//マックス速度設定
		id[i%2] = (18 << 24) | (master_id << 8) | (cyber_id[i%2]);
		cyber_data[i%2][i] = 0;
		cyber_data[i%2][0] = index & 0x00FF;
		cyber_data[i%2][1] = index >> 8;
		memcpy(&cyber_data[i%2][4],&cyber_target_speed[i%2],4);
		sken_system.canTransmit(CAN_2,id[i%2],cyber_data[i%2],8,1,1);
		//目標角度設定
		index = 0x7016;
		cyber_data[i%2][i] = 0;
		cyber_data[i%2][0] = index & 0x00FF;
		cyber_data[i%2][1] = index >> 8;
		memcpy(&cyber_data[i%2][4],&cyber_target_deg[i%2],4);
		sken_system.canTransmit(CAN_2,id[i%2],cyber_data[i%2],8,1,1);
	}
}

void cyber_turn(void){		//cybergear動作
	uint16_t torque_uint[2];
	uint32_t id[2];

	for(int i=0;i<2;i++){
		torque_uint[i] = float_to_uint(cyber_target_torque[i], T_MIN, T_MAX, 16);
		id[i] = (1 << 24) | (torque_uint[i] << 8) | (cyber_id[i]);
		cyber_data[i][0] = float_to_uint(cyber_target_deg[i]*M_PI/180, P_MIN, P_MAX, 16) >> 8;
		cyber_data[i][1] = float_to_uint(cyber_target_deg[i]*M_PI/180, P_MIN, P_MAX, 16) & 0xFF;
		cyber_data[i][2] = float_to_uint(cyber_target_speed[i]*M_PI/180, V_MIN, V_MAX, 16) >> 8;
		cyber_data[i][3] = float_to_uint(cyber_target_speed[i]*M_PI/180, V_MIN, V_MAX, 16) & 0xFF;
		cyber_data[i][4] = float_to_uint(cyber_Kp[i], KP_MIN, KP_MAX, 16) >> 8;
		cyber_data[i][5] = float_to_uint(cyber_Kp[i], KP_MIN, KP_MAX, 16) & 0xFF;
		cyber_data[i][6] = float_to_uint(cyber_Kd[i], KD_MIN, KD_MAX, 16) >> 8;
		cyber_data[i][7] = float_to_uint(cyber_Kd[i], KD_MIN, KD_MAX, 16) & 0xFF;

		sken_system.canTransmit(CAN_2,id[i],cyber_data[i],8,1,1);
	}

}

void com_esp(void){
	for (int i = 0; i < 5; i++) {
		if (serial_data[i] == 0xA5 && serial_data[(i + 1) % 5] == 0xA5) {
			ly = serial_data[(i + 2) % 5];
		    ry = serial_data[(i + 3) % 5];
		    cyber_flag = serial_data[(i + 4) % 5];
		    break;
		}
	}

	(ly > 110 || ly < 90)? motor_out[0] = ly-100:motor_out[0] = 0;
	(ry > 110 || ry < 90)? motor_out[1] = ry-100:motor_out[1] = 0;
}

int main(void)
{
	sken_system.init();
	motor[0].init(Apin,B14,TIMER12,CH1);
	motor[0].init(Bpin,B15,TIMER12,CH2);
	motor[1].init(Apin,A8,TIMER1,CH1);
	motor[1].init(Bpin,A11,TIMER1,CH4);
    sken_system.startCanCommunicate(B13, B12, CAN_2);
	led.init(A5,PWM_OUTPUT,TIMER2,CH1);
	ESP.init(A2,A3,SERIAL2,115200);
	ESP.startDmaRead(serial_data,5);
	sken_system.addTimerInterruptFunc(com_esp, 0, 1);
    cyber_init();
	while(1){
		time[0] = sken_system.millis();
		if(cyber_flag == true){
			if(flag == 0) time[1] = time[0];
			cyber_target_deg[0] = 100;
			cyber_target_speed[0] = 100;
			distance = time[0] - time[1];
			if(distance >= 1000){
				cyber_target_deg[1] = -100;
				cyber_target_speed[1] = 100;
			}
		}else {

			cyber_target_deg[0] = 0;
			cyber_target_deg[1] = 0;
			cyber_target_speed[0] = 0;
			cyber_target_speed[1] = 0;
		}
    	cyber_set_position_control();

    	for (int i=0;i<2;i++)motor[i].write(motor_out[i]);
	}
}
