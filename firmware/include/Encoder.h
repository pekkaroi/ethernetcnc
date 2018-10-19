/*
 *
 *
 *  Created on: 16 Jun 2015
 *      Author: pekka
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

class Encoder
{
	uint8_t pin1;
	uint8_t pin2;
	int16_t oldcount;
	int32_t count;
	TIM_HandleTypeDef TIM_Handle;
	void initGPIO(GPIO_TypeDef *port, uint16_t pin, TIM_TypeDef *timer,uint8_t alt);
	void initTimer(TIM_TypeDef *timer);
public:
	uint8_t initialize(uint8_t *data);
	int32_t getCount();


};



#endif /* INPUT_H_ */
