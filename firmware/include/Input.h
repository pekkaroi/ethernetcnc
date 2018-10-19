/*
 * Input.h
 *
 *  Created on: 16 Jun 2015
 *      Author: pekka
 */

#ifndef INPUT_H_
#define INPUT_H_
#include <stdint.h>
#include "stm32f4xx_hal.h"


class Input
{
public:
	uint8_t pin;
	uint8_t value;
	void initGPIO(GPIO_TypeDef *gpio, uint16_t pin);


	uint8_t initialize(uint8_t *data);


};



#endif /* INPUT_H_ */
