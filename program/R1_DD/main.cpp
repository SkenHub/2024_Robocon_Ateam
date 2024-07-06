/**
  ******************************************************************************
  * @file    main.cpp
  * @author  tikuwa404
  * @version V0.2.2
  * @date    06-July-2024
  * @brief   R1 DD function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "myuart.hpp"

#define DD_DEBUG

#ifdef DD_DEBUG
MyUart<80> uartMDD1;
double mtr_now[4];
double mtr_target[4];
int mtr_rq[4];
#else
MyUart<1> uartMDD1;
#endif
Gpio valve[3];
PinState valve_state[3] = {LOW,LOW,LOW};
//Uart uartMDD2;

void MDD1_receive(void) {
#ifdef DD_DEBUG
	union {double d[10]; int i[20]; uint8_t u8[80];} conv;
	if (uartMDD1.read(conv.u8)) {
		for (int i = 0; i < 4; ++i) mtr_now[i] = conv.d[i];
		for (int i = 0; i < 4; ++i) mtr_target[i] = conv.d[i+4];
		for (int i = 0; i < 4; ++i) mtr_rq[i] = conv.i[i+16];
	}
#else
	uint8_t tmp;
	if (uartMDD1.read(&tmp)) {
		for (int i = 0; i < 3; ++i) valve_state[i] = ((tmp&(0x01<<i))==(0x01<<i)) ? HIGH : LOW;
	}
#endif
}

void main_interrupt(void) {
	MDD1_receive();
	for (int i = 0; i < 3; ++i) valve[i].write(valve_state[i]);
}

int main(void)
{
	sken_system.init();

	valve[0].init(B6,OUTPUT);
	valve[1].init(B7,OUTPUT);
	valve[2].init(B8,OUTPUT);

	uartMDD1.init(USB_A,115200);

	sken_system.addTimerInterruptFunc(main_interrupt,0,1);

	while (true) {}
}
