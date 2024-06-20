/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Tikuwa404
  * @version V0.0.0
  * @date    17-June-2024
  * @brief   R1 Sensor function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

Encoder enc[3];
Encoder_data enc_data[3];
float posx, posy, post;
Uart arduino;
uint8_t send_data[16];
uint8_t mode, action;

void main_interrupt(void) {
	for (int i=0; i<3; ++i) enc[i].interrupt(&enc_data[i]);
	posx = 0;
	posy = 0;
	post = 0;

	ConvertIntFloat conv;
	send_data[0] = 0xA5;
	send_data[1] = 0xA5;
	conv.float_val = posx;
	send_data[2] = conv.uint8_val[0];
	send_data[3] = conv.uint8_val[1];
	send_data[4] = conv.uint8_val[2];
	send_data[5] = conv.uint8_val[3];
	conv.float_val = posy;
	send_data[6] = conv.uint8_val[0];
	send_data[7] = conv.uint8_val[1];
	send_data[8] = conv.uint8_val[2];
	send_data[9] = conv.uint8_val[3];
	conv.float_val = post;
	send_data[10] = conv.uint8_val[0];
	send_data[11] = conv.uint8_val[1];
	send_data[12] = conv.uint8_val[2];
	send_data[13] = conv.uint8_val[3];
	send_data[14] = mode;
	send_data[15] = action;
}

int main(void)
{
	sken_system.init();

	arduino.init();
	//arduino.startDmaRead();

	enc[0].init();
	enc[1].init();
	enc[2].init();

	sken_system.addTimerInterruptFunc(main_interrupt, 0, 1);

	while (true) {}
}
