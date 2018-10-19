/*
 * Watchdog.cpp
 *
 *  Created on: Nov 25, 2015
 *      Author: pekka
 */


#include "Watchdog.h"
#include "PinDefinition.h"

#define TIM_GET_ITSTATUS(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->SR & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)
#define TIM_CLEAR_IT(__HANDLE__, __INTERRUPT__)     ((__HANDLE__)->SR = ~(__INTERRUPT__))
#define TIM_DISABLE(__HANDLE__) \
                        do { \
                          if (((__HANDLE__)->CCER & CCER_CCxE_MASK) == 0) \
                          { \
                            if(((__HANDLE__)->CCER & CCER_CCxNE_MASK) == 0) \
                            { \
                              (__HANDLE__)->CR1 &= ~(TIM_CR1_CEN); \
                            } \
                          } \
                        } while(0)

void Watchdog::initialize(uint16_t timeout_ms)
{
	__TIM5_CLK_ENABLE();
	TIM_Handle.Instance = TIM5;
	TIM_Handle.Init.Prescaler = (uint32_t)(SystemCoreClock/2/1000 -1);
	TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM_Handle.Init.Period = timeout_ms-1;
	TIM_Handle.Init.ClockDivision = 0;
	TIM_Handle.Init.RepetitionCounter = 0;

	HAL_NVIC_SetPriority(TIM5_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM5_IRQn);
	TIM_CLEAR_IT(TIM5,TIM_IT_UPDATE);
	HAL_TIM_Base_Init(&TIM_Handle);
	HAL_TIM_Base_Start_IT(&TIM_Handle);

}

void Watchdog::setTimer()
{


}


uint16_t Watchdog::getTimeout()
{

	return _timeout_ms;
}

void Watchdog::setTimeout(uint16_t timeout_ms)
{
	_timeout_ms=timeout_ms;
	TIM_Handle.Init.Period = timeout_ms-1;
	HAL_TIM_Base_Init(&TIM_Handle);
	//if timer was running, keep running
	if((TIM5->CR1&TIM_CR1_CEN) == TIM_CR1_CEN)
	{

		HAL_TIM_Base_Start_IT(&TIM_Handle);
	}
}
void Watchdog::pet()
{
	WD_TIMER->CNT=0;

	if((TIM5->CR1&TIM_CR1_CEN) != TIM_CR1_CEN)
	{
		HAL_TIM_Base_Init(&TIM_Handle);
		HAL_TIM_Base_Start_IT(&TIM_Handle);
	}


}

void resetAll()
{
	ledOff(1);
	config->resetEverything();
}
extern "C" void TIM5_IRQHandler(void)
{
	int i;
	if(TIM_GET_ITSTATUS(TIM5, TIM_IT_UPDATE) != RESET)
	{

		TIM_CLEAR_IT(TIM5,TIM_IT_UPDATE);
		if(config->configured)
		{
		resetAll();
		}
		TIM_DISABLE(TIM5);

	}

		return;


}






