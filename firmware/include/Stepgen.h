/*
 *
 *
 *  Created on: 16 Jun 2015
 *      Author: pekka
 */

#ifndef STEPGEN_H_
#define STEPGEN_H_
#include <stdint.h>
#include "stm32f4xx_hal.h"




class Stepgen
{
public:

	uint8_t steppin;
	uint8_t dirpin;

	uint16_t period;
	uint16_t target_period; //this comes as input

	int8_t direction;
	int8_t target_direction; //this comes as input

	uint8_t step_pulse_len; //length of the step pulse (in timer cycles)
	uint8_t min_period;
	uint8_t delay_dir_change; //delay from end of step to change of direction
	uint8_t delay_dir_step; //delay from change of direction to start of new step

	uint16_t acc_lim; //maximum allowed change in period during one cycle


	TIM_HandleTypeDef TIM_Handle;

	void initStepGPIO(GPIO_TypeDef *port, uint16_t pin, TIM_TypeDef *timer, uint8_t alt);
	void initDirGPIO(GPIO_TypeDef *port, uint16_t pin);
	void initTimer(TIM_TypeDef *timer, int8_t pwm_ch, uint16_t period,uint8_t step_len);

	uint8_t initialize(uint8_t *data);
	void setPeriod();
	void setDirection();
	volatile uint16_t step_accumulator; //number of taken full steps. Can overflow. The full location is kept only in host
};



#endif /* INPUT_H_ */
