/*
 * can.cpp
 *
 *  Created on: 2019/09/07
 *      Author: TakumaNakao
 */

#include "can.h"

Can can_1;
Can can_2;

extern "C" void CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(can_1.getCanHandle());
	if(HAL_CAN_Receive_IT(can_1.getCanHandle(),CAN_FIFO0) != HAL_OK){
		__HAL_CAN_ENABLE_IT(can_1.getCanHandle(),CAN_IT_FMP0);
	}
	can_1.receiveInterruptCallback();
}

extern "C" void CAN2_RX1_IRQHandler(void)
{
	HAL_CAN_IRQHandler(can_2.getCanHandle());
	if(HAL_CAN_Receive_IT(can_2.getCanHandle(),CAN_FIFO1) != HAL_OK){
		__HAL_CAN_ENABLE_IT(can_2.getCanHandle(),CAN_IT_FMP1);
	}
	can_2.receiveInterruptCallback();
}

Can::Can(void) : can_start_flag_(false),can_interrupt_flag_ (false),is_func_(false) {}

bool Can::init(Pin tx_pin,Pin rx_pin,CanSelect can_select)
{
	if(can_select == CAN_1){
		gpio_init_can_.Alternate = GPIO_AF9_CAN1;
		can_handle_.Instance = CAN1;
		can_handle_.pRxMsg = &can_rx_;
	}
	else if(can_select == CAN_2){
		gpio_init_can_.Alternate = GPIO_AF9_CAN2;
		can_handle_.Instance = CAN2;
		can_handle_.pRx1Msg = &can_rx_;
	}
	else{
		return false;
	}
	tx_pin_group_ = io_convert::PIN_GROUP[tx_pin / 16];
	rx_pin_group_ = io_convert::PIN_GROUP[rx_pin / 16];
	tx_pin_number_ = io_convert::PIN_NUM[tx_pin % 16];
	rx_pin_number_ = io_convert::PIN_NUM[rx_pin % 16];
	gpio_init_can_.Mode = GPIO_MODE_AF_PP;
	gpio_init_can_.Pull = GPIO_PULLUP;
	gpio_init_can_.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio_init_can_.Pin = tx_pin_number_;
	HAL_GPIO_Init(tx_pin_group_, &gpio_init_can_);
	gpio_init_can_.Pin = rx_pin_number_;
	HAL_GPIO_Init(rx_pin_group_, &gpio_init_can_);
	can_handle_.pTxMsg = &can_tx_;
	can_handle_.Init.Prescaler = 7;
	can_handle_.Init.Mode = CAN_MODE_NORMAL;
	can_handle_.Init.SJW = CAN_SJW_1TQ;
	can_handle_.Init.BS1 = CAN_BS1_10TQ;
	can_handle_.Init.BS2 = CAN_BS2_1TQ;
	can_handle_.Init.TTCM = DISABLE;
	can_handle_.Init.ABOM = DISABLE;
	can_handle_.Init.AWUM = DISABLE;
	can_handle_.Init.NART = DISABLE;
	can_handle_.Init.RFLM = DISABLE;
	can_handle_.Init.TXFP = DISABLE;
	HAL_CAN_Init(&can_handle_);

	if(can_select == CAN_1){
		filter_config_.FilterNumber = 0;
		filter_config_.FilterMode = CAN_FILTERMODE_IDMASK;
		filter_config_.FilterScale = CAN_FILTERSCALE_16BIT;
		filter_config_.FilterIdHigh = 0x0000;
		filter_config_.FilterIdLow = 0x0000;
		filter_config_.FilterMaskIdHigh = 0x0000;
		filter_config_.FilterMaskIdLow = 0x0000;
		filter_config_.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		filter_config_.FilterActivation = ENABLE;
		filter_config_.BankNumber = 0;
		HAL_CAN_ConfigFilter(&can_handle_, &filter_config_);
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_CAN_IRQHandler(&can_handle_);
		HAL_CAN_Receive_IT(&can_handle_,CAN_FIFO0);
	}
	else{
		filter_config_.FilterNumber = 1;
		filter_config_.FilterMode = CAN_FILTERMODE_IDMASK;
		filter_config_.FilterScale = CAN_FILTERSCALE_16BIT;
		filter_config_.FilterIdHigh = 0x0000;
		filter_config_.FilterIdLow = 0x0000;
		filter_config_.FilterMaskIdHigh = 0x0000;
		filter_config_.FilterMaskIdLow = 0x0000;
		filter_config_.FilterFIFOAssignment = CAN_FILTER_FIFO1;
		filter_config_.FilterActivation = ENABLE;
		filter_config_.BankNumber = 1;
		HAL_CAN_ConfigFilter(&can_handle_, &filter_config_);
		HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
		HAL_CAN_IRQHandler(&can_handle_);
		HAL_CAN_Receive_IT(&can_handle_,CAN_FIFO1);
	}
	can_start_flag_ = true;
	return true;
}

CAN_HandleTypeDef* Can::getCanHandle(void)
{
	return &can_handle_;
}

void Can::receiveInterruptCallback(void)
{
	if(can_interrupt_flag_){
		(*can_data_).rx_stdid = can_rx_.StdId;
		for(int i=0 ; i < 8 ; i++){
			(*can_data_).rx_data[i] = can_rx_.Data[i];
		}
		if(is_func_) {
			can_interrupt_func_(can_rx_);
		}
	}
}

HAL_StatusTypeDef Can::transmit(uint32_t stdid,uint8_t* data_p,int data_size,int dead_time)
{
	can_handle_.pTxMsg->StdId = stdid;
	can_handle_.pTxMsg->ExtId = 0x00;
	can_handle_.pTxMsg->RTR   = CAN_RTR_DATA;
	can_handle_.pTxMsg->IDE   = CAN_ID_STD;
	can_handle_.pTxMsg->DLC   = data_size;
	for(int i = 0; i < data_size; i++){
		can_handle_.pTxMsg->Data[i] = data_p[i];
	}
	return HAL_CAN_Transmit(&can_handle_,dead_time);
}

void Can::addRceiveInterruptFunc(CanData* can_data)
{
	can_data_ = can_data;
	can_interrupt_flag_ = true;
}

void Can::addRceiveInterruptFunc(CanData* can_data, void(*function_p)(CanRxMsgTypeDef))
{
	can_data_ = can_data;
	can_interrupt_flag_ = true;
	is_func_ = true;
	can_interrupt_func_ = function_p;
}

void Can::deleteRceiveInterruptFunc()
{
	can_interrupt_flag_ = false;
}

CanRxMsgTypeDef Can::rx_receive(){
	return can_rx_;
}
