/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Tikuwa404
  * @version V0.1.0
  * @date    23-June-2024
  * @brief   R1 Sensor function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "myuart.hpp"

constexpr double TIRE_DIAMETER = 60; //計測輪直径[mm]
constexpr double BODY_DIAMETER = 1027.38; //計測輪間直径[mm]
constexpr uint8_t SEND_DATASIZE = 12;

Encoder enc[3];
Encoder_data enc_data[3];
double posx, posy, post;
MyUart<0> uartMDD1;
union {float f[SEND_DATASIZE/4]; uint8_t u8[SEND_DATASIZE];} send_data;

void main_interrupt(void) {
	for (int i=0; i<3; ++i) enc[i].interrupt(&enc_data[i]);
	//read and update position
	posx += -enc_data[1].volcity/1000;
	posy += (enc_data[0].volcity-enc_data[2].volcity)/2000;
	post += (-enc_data[0].volcity-enc_data[1].volcity-enc_data[2].volcity-enc_data[3].volcity)/4000/BODY_DIAMETER*360;
	if (post > 180) post -= 360;
	if (post <= -180) post += 360;

	send_data.f[0] = (float)posx;
	send_data.f[1] = (float)posy;
	send_data.f[2] = (float)post;
	uartMDD1.write(send_data.u8, SEND_DATASIZE);
}

int main(void)
{
	sken_system.init();

	//左のコネクタからから順番に
	enc[0].init(A0,A1,TIMER5,TIRE_DIAMETER); //左
	enc[1].init(B3,A5,TIMER2,TIRE_DIAMETER); //後ろ
	enc[2].init(B6,B7,TIMER4,TIRE_DIAMETER); //右

	uartMDD1.init(USB_B,115200);

	sken_system.addTimerInterruptFunc(main_interrupt, 0, 1);

	while (true) {}
}
