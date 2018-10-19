/*
 *
 *  Created on: 16 Jun 2015
 *      Author: pekka
 */

#include "Encoder.h"
#include "Configuration.h"
#include "PinDefinition.h"


uint8_t Encoder::initialize(uint8_t *data)
{
	uint8_t pin_i1 = data[0];
	uint8_t pin_i2 = data[1];

	if(pin_i1 >= sizeof(pins)/sizeof(PinDef))
	{
		return 2;
	}
	if(pin_i2 >= sizeof(pins)/sizeof(PinDef))
	{
		return 2;
	}
	this->pin1 = pin_i1;
	this->pin2 = pin_i2;


	enableBank(pins[pin_i1].bank);
	setBankInput(pins[pin_i1].bank);

	enableBank(pins[pin_i2].bank);
	setBankInput(pins[pin_i2].bank);


	if(pins[pin_i1].timer != pins[pin_i2].timer)
	{
		return 2;
	}
	this->initGPIO(pins[pin_i1].gpio, pins[pin_i1].pin,pins[pin_i1].timer,pins[pin_i1].af_Funct);
	this->initGPIO(pins[pin_i2].gpio, pins[pin_i2].pin,pins[pin_i2].timer,pins[pin_i2].af_Funct);

	this->initTimer(pins[pin_i1].timer);
	this->count = 0;
	this->oldcount = 0;


	return 2; //this is how many configuration bits are read

}

void Encoder::initGPIO(GPIO_TypeDef *port, uint16_t pin, TIM_TypeDef *timer, uint8_t alt)
{

	if(port==GPIOA) __GPIOA_CLK_ENABLE();
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
void Encoder::initTimer(TIM_TypeDef *timer)
{
	TIM_Encoder_InitTypeDef  sEncoderConfig;

	if(timer==TIM1) __TIM1_CLK_ENABLE();
	else if(timer==TIM2) __TIM2_CLK_ENABLE();
	else if(timer==TIM3) __TIM3_CLK_ENABLE();
	else if(timer==TIM4) __TIM4_CLK_ENABLE();
	else if(timer==TIM8) __TIM8_CLK_ENABLE();

	TIM_Handle.Instance = timer;
	TIM_Handle.Init.Period             = 65535;
	TIM_Handle.Init.Prescaler          = 0;
	TIM_Handle.Init.ClockDivision      = 0;
	TIM_Handle.Init.CounterMode        = TIM_COUNTERMODE_UP;
	TIM_Handle.Init.RepetitionCounter  = 0;

	sEncoderConfig.EncoderMode        = TIM_ENCODERMODE_TI12;

	sEncoderConfig.IC1Polarity        = TIM_ICPOLARITY_RISING;
	sEncoderConfig.IC1Selection       = TIM_ICSELECTION_DIRECTTI;
	sEncoderConfig.IC1Prescaler       = TIM_ICPSC_DIV1;
	sEncoderConfig.IC1Filter          = 0;

	sEncoderConfig.IC2Polarity        = TIM_ICPOLARITY_RISING;
	sEncoderConfig.IC2Selection       = TIM_ICSELECTION_DIRECTTI;
	sEncoderConfig.IC2Prescaler       = TIM_ICPSC_DIV1;
	sEncoderConfig.IC2Filter          = 0;
	HAL_TIM_Encoder_Init(&TIM_Handle, &sEncoderConfig);
	HAL_TIM_Encoder_Start(&TIM_Handle, TIM_CHANNEL_ALL);

}

int32_t Encoder::getCount()
{

	int16_t now = pins[this->pin1].timer->CNT;
	int16_t delta =  now - this->oldcount;
	this->oldcount = now;
	this->count += delta;
	return this->count;

}
