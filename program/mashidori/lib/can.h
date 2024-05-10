/*
 * can.h
 *
 *  Created on: 2019/09/07
 *      Author: TakumaNakao
 */

#ifndef CAN_H_
#define CAN_H_

#include "stm32f4xx.h"
#include "sken_library/gpio.h"

class Can
{
public:
	Can(void);
	bool init(Pin tx_pin,Pin rx_pin,CanSelect can_select);
	inline CAN_HandleTypeDef* getCanHandle(void);
	inline void receiveInterruptCallback(void);
	HAL_StatusTypeDef transmit(uint32_t stdid,uint8_t* data_p,int data_size,int dead_time = 10);
	void addRceiveInterruptFunc(CanData* can_data);
	void addRceiveInterruptFunc(CanData* can_data, void(*function_p)(CanRxMsgTypeDef));
	void deleteRceiveInterruptFunc();
	CanRxMsgTypeDef rx_receive();
private:
	GPIO_TypeDef* tx_pin_group_;
	GPIO_TypeDef* rx_pin_group_;
	uint32_t tx_pin_number_;
	uint32_t rx_pin_number_;
	bool can_start_flag_;
	GPIO_InitTypeDef gpio_init_can_;
	CAN_HandleTypeDef can_handle_;
	CAN_FilterConfTypeDef filter_config_;
	CanRxMsgTypeDef can_rx_;
	CanTxMsgTypeDef can_tx_;
	CanData* can_data_;
	bool can_interrupt_flag_;
	bool is_func_;
	void (*can_interrupt_func_)(CanRxMsgTypeDef);
};

extern Can can_1;
extern Can can_2;

#endif /* CAN_H_ */
