/**
  ******************************************************************************
  * @file    main.cpp
  * @author  tikuwa404
  * @version V0.0.0
  * @date    24-June-2024
  * @brief   R1 DD function. (for SW4 and sken_library)
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "myuart.hpp"

Gpio sw, led;
Gpio valve[3];

MyUart<14> uartPC;
uint8_t PC_data[14];
MyUart<12> uartMDD1;
union f3_u812_convert {
	float f[3];
	uint8_t u8[12];
} MDD1_data;
//Uart uartMDD2;
struct DataFromPC {
  float move_spd; //[m/s]
  float move_dir; //[deg]
  float rot; //[deg]
  uint8_t field; //0~1
  uint8_t action; //0~5
} received;
struct DataSTM {
  float x; //[mm]
  float y; //[mm]
  float theta; //[deg]
  uint8_t field; //0~1
  uint8_t action; //0~5
} now;

void PC_receive(void) {
	uint8_t tmp[14];
	f3_u812_convert conv;
	uartPC.read(tmp);
	for (int i = 0; i < 12; ++i) conv.u8[i] = tmp[i];
	received.move_spd = conv.f[0];
	received.move_dir = conv.f[1];
	received.rot = conv.f[2];
	received.field = tmp[12];
	received.action = tmp[13];

	now.field = received.field;
	now.action = received.action;
}

void PC_send(void) {
	uint8_t tmp[14];
	f3_u812_convert conv;
	conv.f[0] = now.x;
	conv.f[1] = now.y;
	conv.f[2] = now.theta;
	for (int i = 0; i < 12; ++i) {
		tmp[i] = conv.u8[i];
	}
	conv.u8[12] = now.field;
	conv.u8[13] = now.action;
	uartPC.write(tmp, 14);
}

void main_interrupt(void) {
	PC_receive();
	PC_send();
}

int main(void)
{
	sken_system.init();

	sw.init(C13,INPUT_PULLUP);
	led.init(A5,OUTPUT);
	valve[0].init(B6,OUTPUT);
	valve[1].init(B7,OUTPUT);
	valve[2].init(B8,OUTPUT);

	uartPC.init(USB_miniB,9600);
	uartMDD1.init(USB_A,115200);

	sken_system.addTimerInterruptFunc(main_interrupt,0,1);

	while (true) {}
}
