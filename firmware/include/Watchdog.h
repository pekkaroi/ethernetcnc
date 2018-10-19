/*
 * Watchdog.h
 *
 *  Created on: Nov 25, 2015
 *      Author: pekka
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_


#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "Configuration.h"
#define WD_TIMER TIM5
extern Configuration *config;
class Watchdog
{
	uint16_t _timeout_ms;
	uint16_t fed;
	TIM_HandleTypeDef TIM_Handle;
	void setTimer();
public:
	void initialize(uint16_t timeout_ms);
	uint16_t getTimeout();
	void setTimeout(uint16_t timeout_ms);
	void pet();

};

#endif /* WATCHDOG_H_ */
