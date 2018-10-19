/*
 *
 *
 *  Created on: 16 Jun 2015
 *      Author: pekka
 */

#include "Pwm.h"
#include "PinDefinition.h"
#include "Configuration.h"
#include <math.h>
#define GPIO_A 1


uint8_t Pwm::initialize(uint8_t *data)
{
	uint8_t pin_i = data[0];
	uint8_t dirPin_i = data[1];

	if(pin_i >= sizeof(pins)/sizeof(PinDef))
	{
		return 1;
	}

	this->pin = pin_i;
	this->dirPin = dirPin_i;

	enableBank(pins[pin_i].bank);
	setBankOutput(pins[pin_i].bank);

	if(dirPin_i > 0)
	{
		enableBank(pins[dirPin_i].bank);
		setBankOutput(pins[dirPin_i].bank);

	}
	this->frequency = (data[2]<<8)+data[3];
	this->initGPIO(pins[pin_i].gpio, pins[pin_i].pin,pins[pin_i].af_Funct,pins[dirPin_i].gpio, dirPin_i);
	this->initTimer(pins[pin_i].timer,pins[pin_i].pwm_CH,frequency);
	this->setDuty(0.0);

	return 4; //this is how many configuration bits are read

}

void Pwm::initGPIO(GPIO_TypeDef *port, uint16_t pin,uint32_t alt,  GPIO_TypeDef *dirPort, uint8_t dirPin_i)
{
	if(port==GPIOA)	__GPIOA_CLK_ENABLE();
	else if(port==GPIOB) __GPIOB_CLK_ENABLE();
	else if(port==GPIOC) __GPIOC_CLK_ENABLE();
	else if(port==GPIOD) __GPIOD_CLK_ENABLE();
	else if(port==GPIOE) __GPIOE_CLK_ENABLE();
	else if(port==GPIOF) __GPIOF_CLK_ENABLE();
	else if(port==GPIOG) __GPIOG_CLK_ENABLE();
	else if(port==GPIOH) __GPIOH_CLK_ENABLE();

	if(dirPort==GPIOA)	__GPIOA_CLK_ENABLE();
	else if(dirPort==GPIOB) __GPIOB_CLK_ENABLE();
	else if(dirPort==GPIOC) __GPIOC_CLK_ENABLE();
	else if(dirPort==GPIOD) __GPIOD_CLK_ENABLE();
	else if(dirPort==GPIOE) __GPIOE_CLK_ENABLE();
	else if(dirPort==GPIOF) __GPIOF_CLK_ENABLE();
	else if(dirPort==GPIOG) __GPIOG_CLK_ENABLE();
	else if(dirPort==GPIOH) __GPIOH_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStructure;


	GPIO_InitStructure.Pin = pin;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Alternate = alt;
	HAL_GPIO_Init(port, &GPIO_InitStructure);

	if(dirPin_i>0)
	{
		GPIO_InitStructure.Pin = pins[dirPin_i].pin;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(pins[dirPin_i].gpio, &GPIO_InitStructure);
	}

}

void Pwm::initTimer(TIM_TypeDef *timer, int8_t pwm_ch,uint16_t frequency)
{
	//does pin support PWM?
	if(pwm_ch<0)
		return;
	if(timer==TIM1) __TIM1_CLK_ENABLE();
    else if(timer==TIM2) __TIM2_CLK_ENABLE();
    else if(timer==TIM3) __TIM3_CLK_ENABLE();
    else if(timer==TIM4) __TIM4_CLK_ENABLE();
    else if(timer==TIM8) __TIM8_CLK_ENABLE();

	uint16_t scaler = 1;
	if(timer!=TIM1 && timer!=TIM8)
	{
		scaler=2;

	}






	//scaling of the frequency/period
	uint16_t prescaler;
	uint16_t period;
	if(frequency>3000)
	{
		prescaler = 0;
		period = (uint16_t)(SystemCoreClock  / frequency / scaler) - 1;
	}
	else if(frequency>300)
	{
		prescaler = 9;
		period = (uint16_t)(SystemCoreClock  / frequency / 9 /scaler) - 1;
	}
	else if(frequency>30)
	{
		prescaler = 99;
		period = (uint16_t)(SystemCoreClock  / frequency / 99 / scaler) - 1;
	}
	else if(frequency>3)
	{
		prescaler = 999;
		period = (uint16_t)(SystemCoreClock  / frequency / 999 / scaler) - 1;
	}
	else
	{
		//too low frequency
		frequency = 3;
		this->frequency=3;
		prescaler = 999;
		period = (uint16_t)(SystemCoreClock  / frequency / 999 / scaler) - 1;
	}


	this->period = period;


	TIM_Handle.Init.Prescaler = prescaler;
	TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM_Handle.Init.Period = period;
	TIM_Handle.Init.ClockDivision = 0;
	TIM_Handle.Init.RepetitionCounter = 0;
	TIM_Handle.Instance = timer;
	HAL_TIM_PWM_Init(&TIM_Handle);


	TIM_OC_InitTypeDef TIM_OCStruct;



	TIM_OCStruct.OCMode = TIM_OCMODE_PWM1;
	//TIM_OCStruct.OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.OCPolarity = TIM_OCPOLARITY_HIGH;
	TIM_OCStruct.OCNPolarity = TIM_OCNPOLARITY_LOW;
	TIM_OCStruct.OCFastMode =TIM_OCFAST_DISABLE;
	TIM_OCStruct.OCIdleState =TIM_OCIDLESTATE_RESET;
	TIM_OCStruct.OCNIdleState =TIM_OCIDLESTATE_RESET;

/*
	To get proper duty cycle, you have simple equation

	pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1

	where DutyCycle is in percent, between 0 and 100%

	25% duty cycle: 	pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
	50% duty cycle: 	pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
	75% duty cycle: 	pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
	100% duty cycle:	pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399

	Remember: if pulse_length is larger than TIM_Period, you will have output HIGH all the time
*/
	TIM_OCStruct.Pulse = 0;


	switch (pwm_ch)
	{
	case 1:
		//TIM_Handle.Channel = HAL_TIM_ACTIVE_CHANNEL_1;

		HAL_TIM_PWM_ConfigChannel(&TIM_Handle, &TIM_OCStruct, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&TIM_Handle, TIM_CHANNEL_1);
		//TIM_OC1PreloadConfig(timer, TIM_OCPreload_Enable);


		break;
	case 2:
		//TIM_Handle.Channel = HAL_TIM_ACTIVE_CHANNEL_2;

		HAL_TIM_PWM_ConfigChannel(&TIM_Handle, &TIM_OCStruct, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&TIM_Handle, TIM_CHANNEL_2);
		//TIM_OC2PreloadConfig(timer, TIM_OCPreload_Enable);
		break;
	case 3:
		//TIM_Handle.Channel = HAL_TIM_ACTIVE_CHANNEL_3;

		HAL_TIM_PWM_ConfigChannel(&TIM_Handle, &TIM_OCStruct, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&TIM_Handle, TIM_CHANNEL_3);
		//TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
		break;
	case 4:
		//TIM_Handle.Channel = HAL_TIM_ACTIVE_CHANNEL_4;

		HAL_TIM_PWM_ConfigChannel(&TIM_Handle, &TIM_OCStruct, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&TIM_Handle, TIM_CHANNEL_4);
		//TIM_OC4PreloadConfig(timer, TIM_OCPreload_Enable);
		break;
	default:
		break;

	}

	//TIM_ARRPreloadConfig(timer, ENABLE);
//	HAL_TIM_Cmd(timer, ENABLE);

}
void Pwm::setDuty(float duty)
{
	if(this->dirPin>0)
	{
		if(duty<0)
			clearOutput(this->dirPin);
		else
			setOutput(this->dirPin);
	}
	switch(pins[this->pin].pwm_CH)
	{
	case 1:
		pins[this->pin].timer->CCR1 = (uint16_t)((float)fabsf(duty)*(float)this->period);
		break;
	case 2:
		pins[this->pin].timer->CCR2 = (uint16_t)((float)fabsf(duty)*(float)this->period);
		break;
	case 3:
		pins[this->pin].timer->CCR3 = (uint16_t)((float)fabsf(duty)*(float)this->period);
		break;
	case 4:
		pins[this->pin].timer->CCR4 = (uint16_t)((float)fabsf(duty)*(float)this->period);
		break;
	default:
		break;
	}


}


