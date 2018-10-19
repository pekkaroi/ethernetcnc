/*
 * Configuration.cpp
 *
 *  Created on: 16 Jun 2015
 *      Author: pekka
 */

#include "Configuration.h"
#include "Input.h"
#include "Output.h"
#include "Pwm.h"
#include "Encoder.h"
#include "PinDefinition.h"
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

void Configuration::initBankGpio(void)
{
	__GPIOD_CLK_ENABLE();


	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = OE_Pins[0]|OE_Pins[1]|OE_Pins[2]|OE_Pins[3]|OE_Pins[4];
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull =GPIO_NOPULL;
	HAL_GPIO_Init(OE_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = DIR_Pins[0]|DIR_Pins[1]|DIR_Pins[2]|DIR_Pins[3]|DIR_Pins[4];
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull =GPIO_NOPULL;
	HAL_GPIO_Init(DIR_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = LED_Pins[0]|LED_Pins[1];
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull =GPIO_NOPULL;
	HAL_GPIO_Init(LED_GPIO, &GPIO_InitStructure);


	OE_GPIO->BSRRL= OE_Pins[0]|OE_Pins[1]|OE_Pins[2]|OE_Pins[3]|OE_Pins[4]; //all OE high
	LED_GPIO->BSRRL = LED_Pins[0]|LED_Pins[1];

}

void Configuration::parseConfig(uint8_t *data, uint16_t len)
{
	uint16_t i,j=1;
	uint8_t *p;
	this->numberOfInputs = data[j++];
	this->numberOfOutputs = data[j++];
	this->numberOfPWMs =  data[j++];
	this->numberOfEncoders = data[j++];
	this->numberOfStepgens = data[j++];

	this->inputs = new Input[this->numberOfInputs];
	for(i=0;i<this->numberOfInputs;i++)
	{
		p = &data[j];
		j += this->inputs[i].initialize(p);
	}
	this->outputs = new Output[this->numberOfOutputs];
	for(i=0;i<this->numberOfOutputs;i++)
	{
		p = &data[j];
		j += this->outputs[i].initialize(p);
	}
	this->pwms = new Pwm[this->numberOfPWMs];
	for(i=0;i<this->numberOfPWMs;i++)
	{
		p = &data[j];
		j += this->pwms[i].initialize(p);
	}
	this->encoders = new Encoder[this->numberOfEncoders];
	for(i=0;i<this->numberOfEncoders;i++)
	{
		p = &data[j];
		j += this->encoders[i].initialize(p);
	}
	this->stepgens = new Stepgen[this->numberOfStepgens];
	for(i=0;i<this->numberOfStepgens;i++)
	{

		p = &data[j];
		j += this->stepgens[i].initialize(p);
		if(pins[this->stepgens[i].steppin].timer == TIM1)
			this->stepgen_timers[1] = i;
		else if(pins[this->stepgens[i].steppin].timer == TIM2)
			this->stepgen_timers[2] = i;
		else if(pins[this->stepgens[i].steppin].timer == TIM3)
			this->stepgen_timers[3] = i;
		else if(pins[this->stepgens[i].steppin].timer == TIM4)
			this->stepgen_timers[4] = i;
		else if(pins[this->stepgens[i].steppin].timer == TIM8)
			this->stepgen_timers[8] = i;

	}
	this->configured = 1;
}
uint16_t Configuration::readMessage(uint8_t*in)
{
	uint32_t tmp;
	uint8_t i,j=0;

	in++;//discard first byte as we already know it's data packet

	tmp = *in<<16;
	in++;
	tmp += *in<<8;
	in++;
	tmp += *in;
	in++;
	for(i=0;i<this->numberOfOutputs;i++)
	{
		this->outputs[i].value=(tmp>>i)&1;
		if((tmp>>i)&1)
			setOutput(this->outputs[i].pin);
		else
			clearOutput(this->outputs[i].pin);
	}
	float tmpf;
	for(i=0;i<this->numberOfPWMs;i++)
	{
		tmp = (*in<<24);
		in++;
		tmp = tmp + (*in<<16);
		in++;
		tmp = tmp + (*in<<8);
		in++;
		tmp = tmp + (*in);
		in++;
		memcpy(&tmpf,&tmp,4);
		if(tmpf>1.0)
		{
			tmpf = 1.0;
		}
		else if(tmpf<-1.0)
		{
			tmpf = -1.0;
		}
		this->pwms[i].setDuty(tmpf);


	}
	for(i=0;i<this->numberOfStepgens;i++)
	{
		//reading target period and target direction

		tmp = (*in<<8);
		in++;
		tmp = tmp + (*in);
		in++;

		this->stepgens[i].target_period = tmp;
		//this->stepgens[i].target_period = 4; //testing stepgen frequency
		this->stepgens[i].target_direction = *in;
		if(this->stepgens[i].target_direction==0)
		{
			this->stepgens[i].target_direction = -1;
		}
		this->stepgens[i].setPeriod();
		in++;


	}


}

void Configuration::resetEverything()
{
	//set all PWM duty cycles, stepgen step rates and outputs to zero
	uint8_t i,j=0;
	for(i=0;i<this->numberOfOutputs;i++)
	{
		clearOutput(this->outputs[i].pin);
	}
	for(i=0;i<this->numberOfPWMs;i++)
	{
		this->pwms[i].setDuty(0.0);
	}
	for(i=0;i<this->numberOfStepgens;i++)
	{
		this->stepgens[i].target_period=0;
		this->stepgens[i].setPeriod();
	}


}
uint16_t Configuration::generateMessage(uint8_t *out,uint8_t pkgId)
{
	uint16_t len=0;

	uint8_t i,j=0;
	int16_t k;
	uint32_t tmp;

	out[j++] = 0; // this is a command packet
	out[j++] = pkgId; //send back the pkgId to know which request we are replying to.
	tmp=0;
	if(this->numberOfInputs>0)
	{
		for(k=this->numberOfInputs-1;k>=0;k--)
		{
			tmp = (tmp<<1) + getInputStatus(this->inputs[k].pin);


		}
	}
	out[j++] = (tmp>>16)&0xFF;
	out[j++] = (tmp>>8)&0xFF;
	out[j++] = tmp&0xFF;


	for(i=0;i<this->numberOfEncoders;i++)
	{
		tmp = this->encoders[i].getCount();
		out[j++]=tmp>>24&0xFF;
		out[j++]=tmp>>16&0xFF;
		out[j++]=tmp>>8&0xFF;
		out[j++]=tmp&0xFF;
	}
	for(i=0;i<this->numberOfStepgens;i++)
	{

		tmp = this->stepgens[i].step_accumulator;
		out[j++]=tmp>>8&0xFF;
		out[j++]=tmp&0xFF;
		if(this->stepgens[i].direction == 1)
			tmp = ((uint32_t)((uint32_t)pins[this->stepgens[i].steppin].timer->CNT*(uint32_t)65535))/(uint32_t)this->stepgens[i].period;
		else
			tmp = ((uint32_t)((uint32_t)(this->stepgens[i].period - pins[this->stepgens[i].steppin].timer->CNT)*(uint32_t)65535))/(uint32_t)this->stepgens[i].period;

		out[j++]=tmp>>8&0xFF;
		out[j++]=tmp&0xFF;

	}




	return j;


}
