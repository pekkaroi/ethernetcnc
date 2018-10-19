/*
 * Input.cpp
 *
 *  Created on: 16 Jun 2015
 *      Author: pekka
 */

#include "Input.h"
#include "PinDefinition.h"
#include "Configuration.h"



uint8_t Input::initialize(uint8_t *data)
{
	uint8_t pin_i = data[0];

	if(pin_i >= sizeof(pins)/sizeof(PinDef))
	{
		return 1;
	}
	this->pin = pin_i;
	enableBank(pins[pin_i].bank);
	setBankInput(pins[pin_i].bank);
	this->initGPIO(pins[pin_i].gpio, pins[pin_i].pin);



	return 1; //this is how many configuration bits are read

}

void Input::initGPIO(GPIO_TypeDef *port, uint16_t pin)
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
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(port, &GPIO_InitStructure);


}


