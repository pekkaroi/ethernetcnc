/*
 *
 *
 *  Created on: 16 Jun 2015
 *      Author: pekka
 */

#include "Stepgen.h"
#include "Configuration.h"
#include "PinDefinition.h"

uint8_t Stepgen::initialize(uint8_t *data)
{
	uint8_t steppin_i = data[0];
	uint8_t dirpin_i = data[1];

	if(steppin_i >= sizeof(pins)/sizeof(PinDef) || dirpin_i >= sizeof(pins)/sizeof(PinDef))
	{
		return 1;
	}

	this->steppin = steppin_i;
	this->direction = 1;
	this->dirpin = dirpin_i;
	this->step_pulse_len = data[2];
	this->min_period = data[2]*2;
	this->delay_dir_change = data[3];
	this->delay_dir_step = data[4];


	enableBank(pins[steppin_i].bank);
	setBankOutput(pins[steppin_i].bank);

	enableBank(pins[dirpin_i].bank);
	setBankOutput(pins[dirpin_i].bank);

	this->period = 0;
	this->step_accumulator = 0;

	this->initStepGPIO(pins[steppin_i].gpio, pins[steppin_i].pin,pins[steppin_i].timer, pins[steppin_i].af_Funct);
	this->initDirGPIO(pins[dirpin_i].gpio, pins[dirpin_i].pin);

	this->initTimer(pins[steppin_i].timer,pins[steppin_i].pwm_CH,0,this->step_pulse_len);
	this->target_period = 0;
	this->target_direction = 0;
	this->setPeriod();

	return 5; //this is how many configuration bits are read

}

void Stepgen::initDirGPIO(GPIO_TypeDef *port, uint16_t pin)
{



	if(port==GPIOA)	__GPIOA_CLK_ENABLE();
	else if(port==GPIOB) __GPIOB_CLK_ENABLE();
	else if(port==GPIOC) __GPIOC_CLK_ENABLE();
	else if(port==GPIOD) __GPIOD_CLK_ENABLE();
	else if(port==GPIOE) __GPIOE_CLK_ENABLE();
	else if(port==GPIOF) __GPIOF_CLK_ENABLE();
	else if(port==GPIOG) __GPIOG_CLK_ENABLE();
	else if(port==GPIOH) __GPIOH_CLK_ENABLE();


	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = pin;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(port, &GPIO_InitStructure);
}


void Stepgen::initStepGPIO(GPIO_TypeDef *port, uint16_t pin,TIM_TypeDef *timer, uint8_t alt)
{

	if(port==GPIOA)	__GPIOA_CLK_ENABLE();
	else if(port==GPIOB) __GPIOB_CLK_ENABLE();
	else if(port==GPIOC) __GPIOC_CLK_ENABLE();
	else if(port==GPIOD) __GPIOD_CLK_ENABLE();
	else if(port==GPIOE) __GPIOE_CLK_ENABLE();
	else if(port==GPIOF) __GPIOF_CLK_ENABLE();
	else if(port==GPIOG) __GPIOG_CLK_ENABLE();
	else if(port==GPIOH) __GPIOH_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStructure;


	GPIO_InitStructure.Pin = pin;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Alternate = alt;
	HAL_GPIO_Init(port, &GPIO_InitStructure);



}
void Stepgen::initTimer(TIM_TypeDef *timer, int8_t pwm_ch,uint16_t period, uint8_t step_len)
{

	//does pin support PWM?
	if(pwm_ch<1)
		return;
	if(timer==TIM1) __TIM1_CLK_ENABLE();
    else if(timer==TIM2) __TIM2_CLK_ENABLE();
    else if(timer==TIM3) __TIM3_CLK_ENABLE();
    else if(timer==TIM4) __TIM4_CLK_ENABLE();
    else if(timer==TIM8) __TIM8_CLK_ENABLE();


	TIM_Handle.Instance = timer;
	TIM_Handle.Init.Prescaler = 49;
if(timer == TIM1 || timer == TIM8)
{
	TIM_Handle.Init.Prescaler = 99;
}

	TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM_Handle.Init.Period = period;
	TIM_Handle.Init.ClockDivision = 0;
	TIM_Handle.Init.RepetitionCounter = 0;

	HAL_TIM_Base_Stop_IT(&TIM_Handle);
	HAL_TIM_OC_Stop_IT(&TIM_Handle,TIM_CHANNEL_1);
	HAL_TIM_OC_Stop_IT(&TIM_Handle,TIM_CHANNEL_2);
	HAL_TIM_OC_Stop_IT(&TIM_Handle,TIM_CHANNEL_3);
	HAL_TIM_OC_Stop_IT(&TIM_Handle,TIM_CHANNEL_4);

	HAL_TIM_PWM_Init(&TIM_Handle);

	TIM_OC_InitTypeDef TIM_OCStruct;

	TIM_OCStruct.Pulse = step_len;
	TIM_OCStruct.OCMode = TIM_OCMODE_PWM1;

	TIM_OCStruct.OCPolarity = TIM_OCPOLARITY_HIGH;
	TIM_OCStruct.OCNPolarity = TIM_OCNPOLARITY_LOW;
	TIM_OCStruct.OCFastMode =TIM_OCFAST_DISABLE;
	TIM_OCStruct.OCIdleState =TIM_OCIDLESTATE_RESET;
	TIM_OCStruct.OCNIdleState =TIM_OCIDLESTATE_RESET;

	switch (pwm_ch)
	{
	case 1:
		HAL_TIM_PWM_ConfigChannel(&TIM_Handle, &TIM_OCStruct, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&TIM_Handle, TIM_CHANNEL_1);
		break;
	case 2:
		HAL_TIM_PWM_ConfigChannel(&TIM_Handle, &TIM_OCStruct, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&TIM_Handle, TIM_CHANNEL_2);
		break;
	case 3:
		HAL_TIM_PWM_ConfigChannel(&TIM_Handle, &TIM_OCStruct, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&TIM_Handle, TIM_CHANNEL_3);
		break;
	case 4:
		//OC4 is reserved for the timing interrupts
		break;
	default:
		break;

	}

	TIM_OCStruct.OCMode      = TIM_OCMODE_TOGGLE;                // Output compare toggling mode
	TIM_OCStruct.OCPolarity  = TIM_OCPOLARITY_LOW;               // Reverse polarity
	TIM_OCStruct.Pulse       = 65536;                       // Output Compare 1 reg value
	HAL_TIM_OC_ConfigChannel(&TIM_Handle, &TIM_OCStruct, TIM_CHANNEL_4);
	HAL_TIM_OC_Start_IT(&TIM_Handle, TIM_CHANNEL_4);

	timer->CR1 &= (uint16_t)~((uint16_t)TIM_CR1_ARPE);

	if(timer==TIM1) HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 3, 0);
	else if(timer==TIM2) HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
	else if(timer==TIM3) HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
	else if(timer==TIM4) HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
	else if(timer==TIM8) HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 3, 0);

	if(timer==TIM1) HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	else if(timer==TIM2) HAL_NVIC_EnableIRQ(TIM2_IRQn);
	else if(timer==TIM3) HAL_NVIC_EnableIRQ(TIM3_IRQn);
	else if(timer==TIM4) HAL_NVIC_EnableIRQ(TIM4_IRQn);
	else if(timer==TIM8) HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

	HAL_TIM_Base_Start_IT(&TIM_Handle);


}
void Stepgen::setDirection()
{

	//set the counter to count to right direction

	uint16_t tmp;
	//handle the direction change with appropriate delays
	tmp = pins[this->steppin].timer->CNT;


		if(tmp>this->step_pulse_len+this->delay_dir_change)
		{
			//enough time has been spent since last step. We can change dir now.
			if(this->direction>0)
				setOutput(this->dirpin);
			else
				clearOutput(this->dirpin);
			//let's 'invert' the CNT. The next step should be as far as we were from the previous step
			pins[this->steppin].timer->CNT = this->period - tmp;
			//
			if(pins[this->steppin].timer->CNT > this->period - this->delay_dir_step)
			{
				//next step would come too soon. decrease cnt.
				pins[this->steppin].timer->CNT = this->period - this->delay_dir_step;

			}

		}
		else
		{
			//we cannot change the direction right now. We need to setup an interrupt to change.
			 pins[this->steppin].timer->CNT = this->period - tmp - this->delay_dir_step - this->delay_dir_change;
			 pins[this->steppin].timer->CCR4 =  this->period -this->delay_dir_step;
			 if(TIM_Handle.Instance == pins[this->steppin].timer)
				 HAL_TIM_OC_Start_IT(&TIM_Handle, TIM_CHANNEL_4);


		}
	this->direction = this->target_direction;

}


void Stepgen::setPeriod()
{
	if(this->target_direction != this->direction)
	{
		this->setDirection();
	}

	if(this->period == 0 && this->target_period > 0)
	{
		pins[this->steppin].timer->CR1|=(TIM_CR1_CEN);
	}
	else if(this->period>0 && this->target_period==0)
	{
		pins[this->steppin].timer->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
		this->period = 0;

	}
	if(this->target_period>0)
	{
		pins[this->steppin].timer->ARR = this->target_period-1;

		if(pins[this->steppin].timer->CNT >= this->target_period)
			pins[this->steppin].timer->CNT = this->target_period-2;

		this->period = target_period;

	}



}

#define TIM_GET_ITSTATUS(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->SR & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)
#define TIM_CLEAR_IT(__HANDLE__, __INTERRUPT__)     ((__HANDLE__)->SR = ~(__INTERRUPT__))
#define TIM_DISABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->DIER &= ~(__INTERRUPT__))

extern Configuration *config;

//all timer interrupts are related to Stepgen functionality:
extern "C" void TIM4_IRQHandler()
{
	uint16_t stepgenNum = config->stepgen_timers[4];

    if (TIM_GET_ITSTATUS(TIM4, TIM_IT_UPDATE) != RESET)
    {
        TIM_CLEAR_IT(TIM4, TIM_IT_UPDATE);

        config->stepgens[stepgenNum].step_accumulator+=config->stepgens[stepgenNum].direction;

    }
    else if(TIM_GET_ITSTATUS(TIM4, TIM_IT_CC4) != RESET)
    {
    	TIM_CLEAR_IT(TIM4, TIM_IT_CC4);

    	TIM_DISABLE_IT(TIM4, TIM_IT_CC4);

    	if(config->stepgens[stepgenNum].direction == 1)
    		setOutput(config->stepgens[stepgenNum].dirpin);
    	else
    		clearOutput(config->stepgens[stepgenNum].dirpin);

    }

}

extern "C" void TIM1_UP_TIM10_IRQHandler()
{
	uint16_t stepgenNum = config->stepgen_timers[1];

    if (TIM_GET_ITSTATUS(TIM1, TIM_IT_UPDATE) != RESET)
    {
    	TIM_CLEAR_IT(TIM1, TIM_IT_UPDATE);

        config->stepgens[stepgenNum].step_accumulator+=config->stepgens[stepgenNum].direction;

    }

}

extern "C" void TIM1_CC_IRQHandler()
{
	uint16_t stepgenNum = config->stepgen_timers[1];
	if(TIM_GET_ITSTATUS(TIM1, TIM_IT_CC4) != RESET)
	{
		TIM_CLEAR_IT(TIM1, TIM_IT_CC4);
		TIM_DISABLE_IT(TIM1, TIM_IT_CC4);

		if(config->stepgens[stepgenNum].direction == 1)
	    	setOutput(config->stepgens[stepgenNum].dirpin);
	    else
	    	clearOutput(config->stepgens[stepgenNum].dirpin);
	}


}

extern "C" void TIM2_IRQHandler()
{
	uint16_t stepgenNum = config->stepgen_timers[2];

    if (TIM_GET_ITSTATUS(TIM2, TIM_IT_UPDATE) != RESET)
    {
    	TIM_CLEAR_IT(TIM2, TIM_IT_UPDATE);

        config->stepgens[stepgenNum].step_accumulator+=config->stepgens[stepgenNum].direction;

    }
    else if(TIM_GET_ITSTATUS(TIM2, TIM_IT_CC4) != RESET)
    {
    	TIM_CLEAR_IT(TIM2, TIM_IT_CC4);
    	TIM_DISABLE_IT(TIM2, TIM_IT_CC4);

		if(config->stepgens[stepgenNum].direction == 1)
	    	setOutput(config->stepgens[stepgenNum].dirpin);
	    else
	    	clearOutput(config->stepgens[stepgenNum].dirpin);
    }

}

extern "C" void TIM3_IRQHandler()
{
	uint16_t stepgenNum = config->stepgen_timers[3];

    if (TIM_GET_ITSTATUS(TIM3, TIM_IT_UPDATE) != RESET)
    {
    	TIM_CLEAR_IT(TIM3, TIM_IT_UPDATE);

        config->stepgens[stepgenNum].step_accumulator+=config->stepgens[stepgenNum].direction;
        return;

    }
    else if(TIM_GET_ITSTATUS(TIM3, TIM_IT_CC4) != RESET)
    {
    	TIM_CLEAR_IT(TIM3, TIM_IT_CC4);
    	TIM_DISABLE_IT(TIM3, TIM_IT_CC4);

		if(config->stepgens[stepgenNum].direction == 1)
	    	setOutput(config->stepgens[stepgenNum].dirpin);
	    else
	    	clearOutput(config->stepgens[stepgenNum].dirpin);
    }

}

extern "C" void TIM8_UP_TIM13_IRQHandler()
{
	uint16_t stepgenNum = config->stepgen_timers[8];

    if (TIM_GET_ITSTATUS(TIM8, TIM_IT_UPDATE) != RESET)
    {
    	TIM_CLEAR_IT(TIM8, TIM_IT_UPDATE);

        config->stepgens[stepgenNum].step_accumulator+=config->stepgens[stepgenNum].direction;

    }

}

extern "C" void TIM8_CC_IRQHandler()
{
	uint16_t stepgenNum = config->stepgen_timers[8];
	if(TIM_GET_ITSTATUS(TIM8, TIM_IT_CC4) != RESET)
	{
		TIM_CLEAR_IT(TIM8, TIM_IT_CC4);
		TIM_DISABLE_IT(TIM8, TIM_IT_CC4);

		if(config->stepgens[stepgenNum].direction == 1)
	    	setOutput(config->stepgens[stepgenNum].dirpin);
	    else
	    	clearOutput(config->stepgens[stepgenNum].dirpin);
	}


}




// ----------------------------------------------------------------------------

