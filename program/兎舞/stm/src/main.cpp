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
#include "wt901c.h"
#include <cmath>

Uart PC;
CanData can_data;
uint8_t RoboMas_data[8],PC_receive[8],PC_data[10];
Encoder encoder[3];
Encoder_data encoder_data[3];
Gpio limit;
WT901C wit;
Read_Data wit_data;
Pid pid[4];

const double L[3] = {312.46,312.46,312.46}; // 計測輪の中心からロボット中心までの距離（ミリメートル）
const double theta[3] = {54.0 * M_PI / 180.0,129.41 * M_PI / 180.0,(360-129.41) * M_PI / 180.0}; // 計測輪の取り付け角度（度）
const double O_diameter = 76,M_diameter = 101.5; //オムニ直径、メカナム直径
const double M_distance = 324.9,R_O_diameter = 209.08,R_M_diameter = 186.5; //ロボット メカナム間距離、オムニ旋回半径、メカナム旋回半径

double R_O_x = 0.0,R_O_y = 0.0,R_O_theta = 0.0,R_G_theta = 0.0; // ロボットの位置と向き（ラジアン）　（オドメトリ）
double motor_out[4],motor_target[4],motor_now[4],motor_torque;	//モーター関連
double motor_deg_[5] = {0,0,0,0,0}; //ロボマスデバック調整用
double R_speed_x = 0,R_speed_y = 0,R_speed_z = 0; //ロボット進行速度、旋回速度
long double time[2];

void calculate_position(double deg1, double deg2, double deg3) {//オドメトリ v1(deg),v2(deg),v3(deg)
	static double R_r[3],X,Y;

	R_r[0] = (deg1*M_PI/180)*59/2;
	R_r[1] = (deg2*M_PI/180)*59/2;
	R_r[2] = -(deg3*M_PI/180)*59/2;
    // 計測輪の移動距離->ロボット移動距離
	R_O_theta = ((R_r[0]/L[0]) + (R_r[1]/L[1]) + (R_r[2]/L[2]))/3;
	X = ((R_r[0]-(R_O_theta*L[0])) / cos(theta[0]) + (R_r[1]-(R_O_theta*L[1])) / cos(theta[1]) + (R_r[2]-(R_O_theta*L[2])) / cos(theta[2]))/3;
	Y = ((R_r[0]-(R_O_theta*L[0])) / sin(theta[0]) + (R_r[1]-(R_O_theta*L[1])) / sin(theta[1]) + (R_r[2]-(R_O_theta*L[2])) / sin(theta[2]))/3;
	R_O_x = X*cos(R_O_theta) + Y*sin(R_O_theta);
	R_O_y = Y*cos(R_O_theta) + X*sin(R_O_theta);
}

void asi(double lx,double ly,double rx,double r_x,double o_y,double m_y){//足回り lx(mm/s),ly(mm/s),rx(rad/s),r_x(メカナム間距離),o_y(オムニ旋回半径),m_y(メカナム旋回半径)
	//移動速度、旋回速度->それぞれの回転速度
	motor_target[0] = -lx/(O_diameter*M_PI) + (o_y*rx)/(O_diameter*M_PI);
	motor_target[1] = lx/(M_diameter*M_PI)*sqrt(2) + ly/(M_diameter*M_PI) + (r_x/2+(o_y+m_y)/2)*rx/(M_diameter*M_PI);
	motor_target[2] = lx/(M_diameter*M_PI)*sqrt(2) + ly/(M_diameter*M_PI) - (r_x/2+(o_y+m_y)/2)*-rx/(M_diameter*M_PI);
}

void RoboMas_turn(void){
	int flag = 0;
	//ロボマスデータ送信
	for (int i=0;i<4;i++){
		RoboMas_data[i*2] = ((short)motor_out[i] >> 8);
		RoboMas_data[(i*2)+1] = (short)motor_out[i];
	}

	sken_system.canTransmit(CAN_1,0x200,RoboMas_data,8,0,1);

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
		case 0x204:
			motor_deg_[0] = (double)((can_data.rx_data[0] << 8) | (can_data.rx_data[1] & 0xff)) / (8191) * 360;
			motor_torque = (double)(int16_t(can_data.rx_data[4] << 8) | (can_data.rx_data[5] & 0xff)) / (36*60);
			break;
	}
	//ロボマス受信機械角度->絶対角度
	motor_deg_[2]=motor_deg_[0]-motor_deg_[1];
	if(motor_deg_[2] > 180){
		motor_deg_[2] -= 360;
	}else if(motor_deg_[2] < -180){
		motor_deg_[2] += 360;
	}

	motor_deg_[4] += motor_deg_[2] / 36;

	if(flag == 0 && can_data.rx_stdid == 0x204){
		motor_deg_[3] = motor_deg_[4];
		flag = 1;
	}

	motor_now[3] = motor_deg_[4] - motor_deg_[3];

	motor_deg_[1] = motor_deg_[0];
}

void RoboMas_deg_reset(void){
	int flag = 0;
	if(flag == 0 && can_data.rx_stdid == 0x204){
		motor_deg_[3] = motor_deg_[4];
				flag = 1;
	}
}

void com_PC(void){// PC_シリアル通信
	uint8_t PC_send[12];
	for (int i = 0; i < 12; i++) {
		if (PC_receive[i] == 0xA5 && PC_receive[(i + 1) % 12] == 0xA5) {
			PC_data[0] = PC_receive[(i + 2) % 12];
	        PC_data[1] = PC_receive[(i + 3) % 12];
	        PC_data[2] = PC_receive[(i + 4) % 12];
	        PC_data[3] = PC_receive[(i + 5) % 12];
	        PC_data[4] = PC_receive[(i + 6) % 12];
	        PC_data[5] = PC_receive[(i + 7) % 12];
	        PC_data[6] = PC_receive[(i + 8) % 12];
	        PC_data[7] = PC_receive[(i + 9) % 12];
	        PC_data[8] = PC_receive[(i + 10) % 12];
	        PC_data[9] = PC_receive[(i + 11) % 12];
	        break;
	     }
	}
	PC_send[0] = 0xA5;
	PC_send[1] = 0xA5;
	PC_send[2] = uint8_t((R_O_theta * 1000) >> 8);
	PC_send[3] = uint8_t((R_O_theta * 1000));
	PC_send[4] = uint8_t((R_O_x * 1000) >> 8);
	PC_send[5] = uint8_t((R_O_x * 1000));
	PC_send[6] = uint8_t((R_O_y * 1000) >> 8);
	PC_send[7] = uint8_t((R_O_y * 1000));
	PC_send[8] = !limit.read();
	PC_send[9] = uint8_t(R_speed_x / 5);
	PC_send[10] = uint8_t(R_speed_y / 5);
	PC_send[11] = uint8_t(R_speed_z / 5);
	PC.write(PC_send, 12);
}

void data_set(){//データ管理
	int C_rx,C_ry,C_lx,C_ly;
	(PC_data[0]-100 < 10 && PC_data[0]-100 > -0)? C_rx = 0:C_rx = PC_data[0]-100;
	(PC_data[1]-100 < 10 && PC_data[1]-100 > -0)? C_ry = 0:C_ry = PC_data[1]-100;
	(PC_data[2]-100 < 10 && PC_data[2]-100 > -0)? C_lx = 0:C_lx = PC_data[2]-100;
	(PC_data[3]-100 < 10 && PC_data[3]-100 > -0)? C_ly = 0:C_ly = PC_data[3]-100;
	R_speed_x = C_lx * 5.0;
	R_speed_y = C_ly * 5.0;
	R_speed_z = (C_rx / 100)*90.0;
}

void main_interrupt(void){
	time[0] = sken_system.millis();
	wit.read();
	wit_data = wit.get_read_data();
	data_set();
	asi(R_speed_x,R_speed_y,R_speed_z*M_PI/180,324.9,209.48,186.5);
	for(int i=0;i<4;i++){
		encoder[i%3].interrupt(&encoder_data[i%3]);
		motor_out[i] = pid[i].control(motor_target[i],motor_now[i],1);
	}
	calculate_position(encoder_data[0].deg,encoder_data[1].deg,encoder_data[2].deg);
	R_G_theta = R_G_theta + wit_data.Gz * ((time[1]-time[0])/1000);
	time[1] = time[0];
}

int main(void)
{
	sken_system.init();

	encoder[0].init(C6,C7,TIMER3,59);
	encoder[1].init(B6,B7,TIMER4,59);
	encoder[2].init(B3,A5,TIMER2,59);

	pid[0].setGain(500,0,0);
	pid[1].setGain(500,0,0);
	pid[2].setGain(500,0,0);
	pid[3].setGain(7,0,5);

	limit.init(C13,INPUT_PULLUP);

	wit.init(C10,C11,SERIAL3,9600);

	sken_system.startCanCommunicate(A12, A11, CAN_1);
	sken_system.addCanRceiveInterruptFunc(CAN_1, &can_data);

	PC.init(A2,A3,SERIAL2,115200);
	PC.startDmaRead(PC_receive,8);

	sken_system.addTimerInterruptFunc(com_PC,0,1);
	sken_system.addTimerInterruptFunc(main_interrupt,1,1);
	while(1){
		RoboMas_turn();
	}
	return 0;
}
