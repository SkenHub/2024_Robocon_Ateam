/**
  ******************************************************************************
  * @file    main.cpp
  * @author  tikuwa404
  * @version V0.1.1
  * @date    24-June-2024
  * @brief   R1 DD function. (for SW4 and sken_library)
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "myuart.hpp"

union f3_u812_convert {
	float f[3];
	uint8_t u8[12];
};

Gpio sw, led;
Gpio valve[3];

MyUart<14> uartPC;
MyUart<12> uartMDD1;
float MDD1_order[3];
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
	if (uartPC.read(tmp)) {
		f3_u812_convert conv;
		for (int i = 0; i < 12; ++i) conv.u8[i] = tmp[i];
		received.move_spd = conv.f[0];
		received.move_dir = conv.f[1];
		received.rot = conv.f[2];
		received.field = tmp[12];
		received.action = tmp[13];

		now.field = received.field;
		now.action = received.action;
	}
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
	tmp[12] = now.field;
	tmp[13] = now.action;
	uartPC.write(tmp, 14);
}

void MDD1_receive(void) {
	f3_u812_convert tmp;
	if (uartMDD1.read(tmp.u8)) {
		now.x = tmp.f[0];
		now.y = tmp.f[1];
		now.theta = tmp.f[2];
	}
}

void MDD1_send(float *order) {
	f3_u812_convert tmp;
	tmp.f[0] = order[0];
	tmp.f[1] = order[1];
	tmp.f[2] = order[2];
	uartMDD1.write(tmp.u8,12);
}

void main_interrupt(void) {
	PC_receive();
	MDD1_receive();
	PC_send();

	if (now.action == 0) {
		MDD1_order[0] = received.move_spd;
		MDD1_order[1] = received.move_dir;
		MDD1_order[2] = received.rot;
	} else if (now.action == 1) {
		MDD1_order[0] = 0;
		MDD1_order[1] = 0;
		MDD1_order[2] = 0;
		valve[0].write(HIGH);
	} else if (now.action == 255) {
		MDD1_order[0] = 0;
		MDD1_order[1] = 0;
		MDD1_order[2] = 0;
	} else if (now.action == 5) {
		MDD1_order[0] = received.move_spd;
		MDD1_order[1] = received.move_dir;
		MDD1_order[2] = received.rot;
	} else {
		MDD1_order[0] = 0;
		MDD1_order[1] = 0;
		MDD1_order[2] = 0;
		valve[1].write(HIGH);
		valve[2].write(HIGH);
	}

	MDD1_send(MDD1_order);
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

	now.x=0;
	now.y=0;
	now.theta=0;
	now.field=0;
	now.action=0;

	sken_system.addTimerInterruptFunc(main_interrupt,0,1);

	while (true) {}
}
