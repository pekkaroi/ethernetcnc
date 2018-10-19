/*
 *
 *
 *  Created on: 16 Jun 2015
 *      Author: pekka
 */

#ifndef PWM_H_
#define PWM_H_
#include <stdint.h>
#include "stm32f4xx_hal.h"

class Pwm
{
	uint8_t pin;
	uint8_t dirPin;
	uint16_t frequency;
	uint16_t period;
	TIM_HandleTypeDef TIM_Handle;
	void initGPIO(GPIO_TypeDef *port, uint16_t pin,uint32_t alt,  GPIO_TypeDef *dirPort, uint8_t dirPin_i);
	void initTimer(TIM_TypeDef *timer, int8_t pwm_ch, uint16_t frequency);
public:
	uint8_t initialize(uint8_t *data);
	void setDuty(float duty);

};



#endif /* INPUT_H_ */
