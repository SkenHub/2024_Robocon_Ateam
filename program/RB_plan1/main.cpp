/**
  ******************************************************************************
  * @file    main.cpp
  * @author  tikuwa404
  * @version V0.1.1
  * @date    02-July-2024
  * @brief   RB MDD function.
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
//#include "myuart.hpp"
#include <cmath>

#define ENABLE_SBDBT_PS3

constexpr double TIRE_DIAMETER = 80; //[mm]
constexpr double TIRE_DISTANCE = 237; //[mm]
constexpr double TIRE_GEAR_RATIO = 2.8; //2.8倍に加速

//ロボマスモータのエンコーダ受信用
class ROBOMAS_Encoder_Data {
public:
	ROBOMAS_Encoder_Data() {}
	short operator[](int i) {
		return (sto[i*2]<<8)+sto[i*2+1];
	}
	void update(uint8_t *data) {
		for (int i = 0; i < 8; ++i) {
			sto[i] = data[i];
		}
	}
private:
	uint8_t sto[8] = {};
};


//Motor mtr[3];
//Encoder enc[2];
//Encoder_data enc_data[2];
//MyUart<2> sensor;

CanData can_received;
uint8_t can_send[8];
ROBOMAS_Encoder_Data enc_data[2];
union {short short_data; uint8_t uint8_data[2];} conv;
Pid mtr_pid[2];
short mtr_pow[2] = {};

PS3 ps3;
PS3_data ps3_data;

Gpio sw;

//limit switch is contained in sensor board



//return -1~1
void stick_resize(double &x, double &y) {
	using namespace std;

	double longer = (abs(x) > abs(y) ? abs(x) : abs(y));
	double angle = std::atan2(y, x);

	x = cos(angle) * longer / 64;
	y = sin(angle) * longer / 64;
}

//run by can_handler
void can_interrupt(CanRxMsgTypeDef rx) {
	if (can_received.rx_stdid == 0x201) {
		enc_data[0].update(can_received.rx_data);
	}
	if (can_received.rx_stdid == 0x202) {
		enc_data[1].update(can_received.rx_data);
	}
}

double debug[6];
short enc1[4], enc2[4];
double circle = 0;
void main_interrupt(void) {
	ps3.Getdata(&ps3_data);

	if (ps3_data.LyPad == 0) {
		if (ps3_data.Circle) {
			if (circle < 450) circle += 0.25;
			mtr_pow[0] = mtr_pid[0].control(circle, static_cast<double>(enc_data[0][1])/36.0, 1);
			mtr_pow[1] = mtr_pid[1].control(-circle, static_cast<double>(enc_data[1][1])/36.0, 1);
		} else {
			circle = 0;
			mtr_pid[0].reset();
			mtr_pid[1].reset();
			mtr_pow[0] = 0;
			mtr_pow[1] = 0;
		}
	} else {
		mtr_pow[0] = mtr_pid[0].control(ps3_data.LyPad, static_cast<double>(enc_data[0][1])/36.0, 1);
		mtr_pow[1] = mtr_pid[1].control(-ps3_data.LyPad, static_cast<double>(enc_data[1][1])/36.0, 1);
	}

	debug[0] = static_cast<double>(enc_data[0][1])/36.0;
	debug[1] = static_cast<double>(enc_data[1][1])/36.0;
	/*
	debug[0] = static_cast<double>(enc_data[0][1])/36.0/60.0*TIRE_DIAMETER*PI*TIRE_GEAR_RATIO;
	debug[1] = sw.read()?0:-500;
	debug[2] = debug[0]-debug[1];
	debug[3] = static_cast<double>(enc_data[1][1])/36.0/60.0*TIRE_DIAMETER*PI*TIRE_GEAR_RATIO;
	debug[4] = sw.read()?0:500;
	debug[5] = debug[3]-debug[4];
	*/
	for (int i = 0; i < 4; ++i) enc1[i] = enc_data[0][i];
	for (int i = 0; i < 4; ++i) enc2[i] = enc_data[1][i];

	conv.short_data = mtr_pow[0];
	can_send[0] = conv.uint8_data[0];
	can_send[1] = conv.uint8_data[1];
	conv.short_data = mtr_pow[1];
	can_send[2] = conv.uint8_data[0];
	can_send[3] = conv.uint8_data[1];
	can_send[4] = 0;
	can_send[5] = 0;
	can_send[6] = 0;
	can_send[7] = 0;
	sken_system.canTransmit(CAN_2,0x200,can_send,8);
}

int main(void)
{
	sken_system.init();

	/*
	mtr[0].init(Apin,B14,TIMER12,CH1);
	mtr[0].init(Bpin,B15,TIMER12,CH2);
	mtr[1].init(Apin,A8,TIMER1,CH1);
	mtr[1].init(Bpin,A11,TIMER1,CH4);
	mtr[2].init(Apin,A6,TIMER13,CH1);
	mtr[2].init(Bpin,A7,TIMER14,CH1);

	enc[0].init(A0,A1,TIMER5,TIRE_DIAMETER);
	enc[1].init(B3,A5,TIMER2,TIRE_DIAMETER);
	*/

	//sensor.init(USB_A,9600);

	ps3.StartRecive(); //A9, A10, SERIAL1

	sw.init(C13,INPUT_PULLUP);

	mtr_pid[0].setGain(0.1,0,0,20);
	mtr_pid[1].setGain(0.1,0,0,20);

	sken_system.startCanCommunicate(B13,B12,CAN_2);
	sken_system.addCanRceiveInterruptFunc(CAN_2,&can_received,can_interrupt);

	sken_system.addTimerInterruptFunc(main_interrupt, 0, 1);

	while (true) {}
}
