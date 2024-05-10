/*
 * system.cpp
 *
 *  Created on: 2019/09/03
 *      Author: TakumaNakao
 */

#include "system.h"

System sken_system;

enum PllParameters{
	PLL_M = 8,
	PLL_N = 336,
	PLL_P = 2,
	PLL_Q = 7,
	PLLI2S_N = 258,
	PLLI2S_R = 3
};

namespace sken_system_timer_cache
{
	unsigned int timer_cache = 0;
}
bool System::instance_create_flag_ = false;

System::System(void) : interrupt_func_array_flag_(0x00) {}

extern "C" void TIM6_DAC_IRQHandler(void)
{
	TIM6->SR = 0;
	sken_system_timer_cache::timer_cache++;
}

extern "C" void TIM7_IRQHandler(void)
{
	TIM7->SR = 0;
	sken_system.timerInterruptCallback();
}

void System::timerInterruptCallback(void)
{
	static unsigned int counter = 0;
	for(int i = 0; i < 8; i++){
		if((interrupt_func_array_flag_ & (1 << i)) != 0){
			if((counter % interrupt_func_array_period_[i]) == 0){
				interruptFuncArray[i]();
			}
		}
	}
	counter++;
}

void System::allClkEnable(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();
	__TIM1_CLK_ENABLE();
	__TIM2_CLK_ENABLE();
	__TIM3_CLK_ENABLE();
	__TIM4_CLK_ENABLE();
	__TIM5_CLK_ENABLE();
	__TIM6_CLK_ENABLE();
	__TIM7_CLK_ENABLE();
	__TIM8_CLK_ENABLE();
	__TIM9_CLK_ENABLE();
	__TIM10_CLK_ENABLE();
	__TIM11_CLK_ENABLE();
	__TIM12_CLK_ENABLE();
	__TIM13_CLK_ENABLE();
	__TIM14_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_UART4_CLK_ENABLE();
	__HAL_RCC_UART5_CLK_ENABLE();
	__HAL_RCC_USART6_CLK_ENABLE();
	//__HAL_RCC_I2C1_CLK_ENABLE();
	//__HAL_RCC_I2C2_CLK_ENABLE();
	//__HAL_RCC_I2C3_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_ADC2_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_CAN2_CLK_ENABLE();
}

void System::setSysClock(void)
{
	/******************************************************************************/
	/*            PLL (clocked by HSE) used as System clock source                */
	/******************************************************************************/
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	/* Enable HSE */
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	do
	{
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;
	} while (HSEStatus == 0);

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
	{
		HSEStatus = (uint32_t)0x01;
	}
	else
	{
		HSEStatus = (uint32_t)0x00;
	}

	if (HSEStatus == (uint32_t)0x01)
	{
		/* Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz */
		RCC->APB1ENR |= RCC_APB1ENR_PWREN;
		PWR->CR |= PWR_CR_VOS;

		/* HCLK = SYSCLK / 1*/
		RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

		/* PCLK2 = HCLK / 2*/
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

		/* PCLK1 = HCLK / 4*/
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

		/* Configure the main PLL */
		RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) - 1) << 16) |
			(RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

		/* Enable the main PLL */
		RCC->CR |= RCC_CR_PLLON;

		/* Wait till the main PLL is ready */
		while ((RCC->CR & RCC_CR_PLLRDY) == 0);

		/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
		FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

		/* Select the main PLL as system clock source */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_PLL;

		/* Wait till the main PLL is used as system clock source */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	}
	else
	{ /* If HSE fails to start-up, the application will have wrong clock
	  configuration. User can add here some code to deal with this error */
		GPIOD->ODR |= 1 << 14;
	}


	/******************************************************************************/
	/*                          I2S clock configuration                           */
	/******************************************************************************/
	/* PLLI2S clock used as I2S clock source */
	//RCC->CFGR &= ~RCC_CFGR_I2SSRC;

	/* Configure PLLI2S */
	RCC->PLLI2SCFGR = (PLLI2S_N << 6) | (PLLI2S_R << 28);

	/* Enable PLLI2S */
	RCC->CR |= ((uint32_t)RCC_CR_PLLI2SON);

	/* Wait till PLLI2S is ready */
	while ((RCC->CR & RCC_CR_PLLI2SRDY) == 0);
}

void System::SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void System::basicTimerSet(void)
{
	TIM6->PSC = 168;
	TIM6->ARR = 65535;
	TIM6->CNT = 0;
	TIM6->CR1 |= TIM_CR1_CEN;
	TIM6->DIER = TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	TIM7->PSC = 999;
	TIM7->ARR = 168;
	TIM7->CNT = 0;
	TIM7->CR1 |= TIM_CR1_CEN;
	TIM7->DIER = TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM7_IRQn);
}

bool System::init(void)
{
	if(instance_create_flag_){
		return false;
	}
	instance_create_flag_ = true;
	allClkEnable();
	setSysClock();
	basicTimerSet();
	SystemCoreClockUpdate();
	HAL_Init();
	sken_system.delayMillis(10);
	return true;
}

unsigned int System::micros(void)
{
	unsigned int timer1 = sken_system_timer_cache::timer_cache;
	unsigned int cnt1 = TIM6->CNT;
	unsigned int timer2 = sken_system_timer_cache::timer_cache;
	unsigned int cnt2 = TIM6->CNT;
	if (timer1 == timer2) {
		return (timer1 << 16) + cnt1;
	}
	else {
		return (timer2 << 16) + cnt2;
	}
}

unsigned int System::millis(void)
{

	unsigned int timer1 = sken_system_timer_cache::timer_cache;
	unsigned int cnt1 = TIM6->CNT;
	unsigned int timer2 = sken_system_timer_cache::timer_cache;
	unsigned int cnt2 = TIM6->CNT;
	if (timer1 == timer2) {
		return (((timer1 << 16) + cnt1) / 1000.0) + 0.5;
	}
	else {
		return (((timer2 << 16) + cnt2) / 1000.0) + 0.5;
	}
}

void System::delayMillis(unsigned int delay_time_millis)
{
	unsigned int start_time_millis = millis();
	while((millis() - start_time_millis) <= delay_time_millis);
}

void System::delayMicros(unsigned int delay_time_micros)
{
	unsigned int start_time_micros = micros();
	while((micros() - start_time_micros) <= delay_time_micros);
}

bool System::addTimerInterruptFunc(void(*function_p)(void),int id,int period)
{
	if(id < 0 || 7 < id){
		return false;
	}
	if(period == 0){
		return false;
	}
	if(function_p == NULL){
		return false;
	}
	interruptFuncArray[id] = function_p;
	interrupt_func_array_flag_ |= 1 << id;
	interrupt_func_array_period_[id] = period;
	return true;
}

bool System::deleteTimerInterruptFunc(int id)
{
	if(id < 0 || 7 < id){
		return false;
	}
	interrupt_func_array_flag_ &= ~(1 << id);
	return true;
}

bool System::changeTimerInterruptPeriod(int id,int period)
{
	if(id < 0 || 7 < id){
		return false;
	}
	if(period == 0){
		return false;
	}
	interrupt_func_array_period_[id] = period;
	return true;
}

bool System::startCanCommunicate(Pin tx_pin,Pin rx_pin,CanSelect can_select)
{
	if(can_select == CAN_1){
		can_1.init(tx_pin,rx_pin,can_select);
	}
	else if(can_select == CAN_2){
		can_2.init(tx_pin,rx_pin,can_select);
	}
	else{
		return false;
	}
	return true;
}

bool System::canTransmit(CanSelect can_select,uint32_t stdid,uint8_t* data_p,int data_size,int dead_time)
{
	if(can_select == CAN_1){
		can_1.transmit(stdid,data_p,data_size,dead_time);
	}
	else if(can_select == CAN_2){
		can_2.transmit(stdid,data_p,data_size,dead_time);
	}
	else{
		return false;
	}
	return true;
}

bool System::addCanRceiveInterruptFunc(CanSelect can_select,CanData* can_data)
{
	if(can_select == CAN_1){
		can_1.addRceiveInterruptFunc(can_data);
		return true;
	}
	else if(can_select == CAN_2){
		can_2.addRceiveInterruptFunc(can_data);
		return true;
	}
	return false;
}

bool System::addCanRceiveInterruptFunc(CanSelect can_select,CanData* can_data,void(*function_p)(CanRxMsgTypeDef))
{
	if(can_select == CAN_1){
		can_1.addRceiveInterruptFunc(can_data,function_p);
		return true;
	}
	else if(can_select == CAN_2){
		can_2.addRceiveInterruptFunc(can_data,function_p);
		return true;
	}
	return false;
}

bool System::deleteCanRceiveInterruptFunc(CanSelect can_select)
{
	if(can_select == CAN_1){
		can_1.deleteRceiveInterruptFunc();
		return true;
	}
	else if(can_select == CAN_2){
		can_2.deleteRceiveInterruptFunc();
		return true;
	}
	return false;
}
