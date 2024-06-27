/**
  ******************************************************************************
  * @file    main.cpp
  * @author  tikuwa404
  * @version V0.2.0
  * @date    27-June-2024
  * @brief   R1 DD function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "myuart.hpp"

MyUart<1> uartMDD1;
Gpio valve[3];
PinState valve_state[3] = {LOW,LOW,LOW};
//Uart uartMDD2;

void MDD1_receive(void) {
	uint8_t tmp;
	if (uartMDD1.read(&tmp)) {
		for (int i = 0; i < 3; ++i) valve_state[i] = ((tmp&(0x01<<i))==(0x01<<i)) ? HIGH : LOW;
	}
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
