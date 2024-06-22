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

constexpr double TIRE_DIAMETER = 1; //計測輪直径[mm]
constexpr double BODY_DIAMETER = 1; //計測輪間直径[mm]
constexpr uint8_t SEND_DATASIZE = 12;

Encoder enc[3];
Encoder_data enc_data[3];
float posx, posy, post;
MyUart<0> uartMDD1;
union {float f[SEND_DATASIZE/4]; uint8_t u8[SEND_DATASIZE];} send_data;

void main_interrupt(void) {
	for (int i=0; i<3; ++i) enc[i].interrupt(&enc_data[i]);
	//read and update position
	posx = 0;
	posy = 0;
	post = 0;

	send_data.f[0] = posx;
	send_data.f[1] = posy;
	send_data.f[2] = post;
	uartMDD1.write(send_data.u8, SEND_DATASIZE);
}

int main(void)
{
	sken_system.init();

	//左のコネクタからから順番に
	enc[0].init(A0,A1,TIMER5);
	enc[1].init(B3,A5,TIMER2);
	enc[2].init(B6,B7,TIMER4);

	uartMDD1.init(USB_B,9600);

	sken_system.addTimerInterruptFunc(main_interrupt, 0, 1);

	while (true) {}
}
