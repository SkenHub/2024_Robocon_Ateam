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
#include <cmath>

Uart PC;
CanData can_data;
uint8_t RoboMas_data[8],PC_receive[6],PC_data[4];
Encoder encoder;
Encoder_data encoder_data;
Gpio limit[2];
Pid pid[3];

const double O_diameter = 59,M_diameter = 101.5; //オムニ直径、メカナム直径
const double M_distance = 324.9,R_O_radius = 209.08,R_M_radius = 186.5; //ロボット メカナム間距離、オムニ旋回半径、メカナム旋回半径
double motor_out[4],motor_target[3],motor_now[4];	//モーター関連
double R_speed_x = 0,R_speed_y = 0,R_speed_z = 0; //ロボット進行速度、旋回速度
int bucket_mode = 0,stop = 0,team_color = 0,max_speed[2] = {0,1},max_deg,flag = 180;

void asi(double lx,double ly,double rx){//足回り lx(mm/s),ly(mm/s),rx(rad/s)
	//移動速度、旋回速度->それぞれの回転速度
	motor_target[0] = -(lx*max_speed[1]/(O_diameter*M_PI)) + (R_O_radius*rx)/(O_diameter*M_PI);
	motor_target[1] = (lx/(M_diameter*M_PI))*(R_O_radius/R_M_radius) - ly/(M_diameter*M_PI) + (M_distance/2+(R_O_radius+R_M_radius)/2)*rx/(M_diameter*M_PI);
	motor_target[2] = (lx/(M_diameter*M_PI))*(R_O_radius/R_M_radius) + ly/(M_diameter*M_PI) - (M_distance/2+(R_O_radius+R_M_radius)/2)*-rx/(M_diameter*M_PI);
}

void RoboMas_turn(void){
	//ロボマスデータ送信
	for (int i=0;i<4;i++){
		RoboMas_data[i*2] = ((short)motor_out[i] >> 8);
		RoboMas_data[(i*2)+1] = (short)motor_out[i];
	}

	sken_system.canTransmit(CAN_2,0x200,RoboMas_data,8,0,1);

	//ロボマスデータ受信
	switch (can_data.rx_stdid){
		case 0x201:
			motor_now[0] = (double)(int16_t(can_data.rx_data[2] << 8) | (can_data.rx_data[3] & 0xff)) / (36*60);
			break;
		case 0x202:
			motor_now[1] = (double)(int16_t(can_data.rx_data[2] << 8) | (can_data.rx_data[3] & 0xff)) / (36*60);
			break;
		case 0x203:
			motor_now[2] = (double)(int16_t(can_data.rx_data[2] << 8) | (can_data.rx_data[3] & 0xff)) / (36*60);
			break;
	}
	motor_now[3] = encoder_data.deg + flag;
}


void com_PC(void){// PC_シリアル通信
	for (int i = 0; i < 6; i++) {
		if (PC_receive[i] == 0xA5 && PC_receive[(i + 1) % 6] == 0xA5) {
			PC_data[0] = PC_receive[(i + 2) % 6];
	        PC_data[1] = PC_receive[(i + 3) % 6];
	        PC_data[2] = PC_receive[(i + 4) % 6];
	        PC_data[3] = PC_receive[(i + 5) % 6];
	        break;
	     }
	}
}

void data_set(){//データ管理
	(PC_data[0] > 110 || PC_data[0] < 90)? R_speed_z = (PC_data[0]-100)*max_deg:R_speed_z = 0;
	(PC_data[1] > 110 || PC_data[1] < 90)? R_speed_x = (PC_data[1]-100)*max_speed[0]:R_speed_x = 0;
	(PC_data[2] > 110 || PC_data[2] < 90)? R_speed_y = (PC_data[2]-100)*max_speed[0]:R_speed_y = 0;

	stop = (PC_data[3]&0x01)? true:false; //true:ストップ
	if((PC_data[3]&0x02)){//上昇
		bucket_mode = 1;
	}else if(PC_data[3]&0x04){ //下降
		bucket_mode = 2;
	}else if(PC_data[3]&0x08){ //途中
		bucket_mode = 3;
	}
	team_color = (PC_data[3]&0x10)? true:false;
	max_speed[0] = (PC_data[3]&0x20)? 15:10;
	if(stop){
		max_speed[0] = 0;
		max_deg = 0;
	}else max_deg = 1.8;
	(bucket_mode == 2)? max_speed[1] = 1.3:max_speed[1] = 1;
}

void main_interrupt(void){//メイン割り込み
	data_set();
	if(bucket_mode == 1 && motor_now[3] < 120){ //上昇
		motor_out[3] = 10000;
	}else if(bucket_mode == 2 && motor_now[3] > 0){ //下降
		motor_out[3] = -3000;
		if(!limit[0].read()) encoder.reset();
	}else if(bucket_mode == 3 && motor_now[3] < 2){ //途中
		motor_out[3] = 6000;
	}else if(bucket_mode == 3 && motor_now[3] > 45){ //途中
		motor_out[3] = -6000;
	}else if(!limit[1].read() && limit[0].read()){ //手動
		motor_out[3] = -8000;
	}else {
		motor_out[3] = 0;
	}
	asi(R_speed_x,R_speed_y,R_speed_z*M_PI/180);
	encoder.interrupt(&encoder_data);
	for(int i=0;i<3;i++){
		if(R_speed_x == 0 && R_speed_y == 0 && R_speed_z == 0) pid[i].reset();
		motor_out[i] = pid[i].control(motor_target[i],motor_now[i],1);
	}
	if(!limit[0].read()) flag = 0;
	RoboMas_turn();
}

int main(void)
{
	sken_system.init();

	encoder.init(A0,A1,TIMER5,1); //バケット

	pid[0].setGain(2500,100,1);
	pid[1].setGain(2500,100,1);
	pid[2].setGain(2500,100,1);

	limit[0].init(B15,INPUT_PULLUP); //バケットリミット
	limit[1].init(C13,INPUT_PULLUP); //付属スイッチ

	sken_system.startCanCommunicate(B13,B12,CAN_2);
	sken_system.addCanRceiveInterruptFunc(CAN_2, &can_data);

	PC.init(A2,A3,SERIAL2,115200);
	PC.startDmaRead(PC_receive,6);

	sken_system.addTimerInterruptFunc(com_PC,0,1);
	sken_system.addTimerInterruptFunc(main_interrupt,1,1);
	while(1){
	}
	return 0;
}
