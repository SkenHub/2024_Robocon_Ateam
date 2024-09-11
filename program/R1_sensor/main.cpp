/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Tikuwa404
  * @version V0.1.1
  * @date    02-July-2024
  * @brief   R1 Sensor function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

constexpr double TIRE_DIAMETER = 60; //計測輪直径[mm]
constexpr double BODY_DIAMETER = 1027.38; //計測輪間直径[mm]

Encoder enc[3];
Encoder_data enc_data[3];
double posx, posy, post;
union {int16_t s16[3]; uint8_t u8[6];} send_data;
Gpio led;

template<typename T> void swap (T& x, T& y) {
	T tmp;
	tmp = x;
	x = y;
	y = tmp;
}

//上位バイト先
//{x, y, theta}
//stdid = 0x101
void main_interrupt(void) {
	for (int i=0; i<3; ++i) enc[i].interrupt(&enc_data[i]);
	//read and update position
	posx += -enc_data[1].volcity/1000;
	posy += (enc_data[0].volcity+enc_data[2].volcity)/2000;
	post += (enc_data[0].volcity-enc_data[2].volcity)/2000/BODY_DIAMETER/PI*360;
	if (post > 180) post -= 360;
	if (post <= -180) post += 360;

	send_data.s16[0] = static_cast<int16_t>(posx);
	send_data.s16[1] = static_cast<int16_t>(posy);
	send_data.s16[2] = static_cast<int16_t>(post);

	/*
	send_data.s16[0] = 314;
	send_data.s16[1] = 1293;
	send_data.s16[2] = 9218;
	*/

	swap(send_data.u8[0], send_data.u8[1]);
	swap(send_data.u8[2], send_data.u8[3]);
	swap(send_data.u8[4], send_data.u8[5]);

	if (sken_system.canTransmit(CAN_2, 0x101, send_data.u8, 6)) {
		led.write(HIGH);
	} else {
		led.write(LOW);
	}
}

int main(void)
{
	sken_system.init();

	led.init(C13, OUTPUT);

	//左のコネクタからから順番に
	enc[0].init(A0,A1,TIMER5,TIRE_DIAMETER); //左
	enc[1].init(B3,A5,TIMER2,TIRE_DIAMETER); //後ろ
	enc[2].init(B6,B7,TIMER4,TIRE_DIAMETER); //右

	sken_system.startCanCommunicate(B13,B12,CAN_2);

	sken_system.addTimerInterruptFunc(main_interrupt, 0, 1);

	while (true) {
	}
}
