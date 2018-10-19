/*
 * Configuration.h
 *
 *  Created on: 16 Jun 2015
 *      Author: pekka
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_



#include "Input.h"
#include "Output.h"
#include "Pwm.h"
#include "Encoder.h"
#include "Stepgen.h"
#include <stdint.h>

#define getInputStatus(gpiopin) HAL_GPIO_ReadPin(pins[gpiopin].gpio, pins[gpiopin].pin)
#define setOutput(gpiopin) pins[gpiopin].gpio->BSRRL=pins[gpiopin].pin
#define clearOutput(gpiopin) pins[gpiopin].gpio->BSRRH=pins[gpiopin].pin

#define enableBank(bank) OE_GPIO->BSRRH=OE_Pins[bank-1]
#define disableBank(bank) OE_GPIO->BSRRL=OE_Pins[bank-1]

#define setBankInput(bank) DIR_GPIO->BSRRH=DIR_Pins[bank-1]
#define setBankOutput(bank) DIR_GPIO->BSRRL=DIR_Pins[bank-1]

#define ledOn(led) LED_GPIO->BSRRH=LED_Pins[led]
#define ledOff(led) LED_GPIO->BSRRL=LED_Pins[led]

class Configuration
{
	public:
	uint8_t numberOfInputs;
	uint8_t numberOfOutputs;
	uint8_t numberOfPWMs;
	uint8_t numberOfEncoders;
	uint8_t numberOfStepgens;

	uint8_t configured;


	Input *inputs;
	Output *outputs;
	Pwm *pwms;
	Encoder *encoders;
	Stepgen *stepgens;
	uint16_t stepgen_timers[10];

	void parseConfig(uint8_t *data, uint16_t len);
	void resetEverything();
	void initBankGpio(void);
	uint16_t readMessage(uint8_t *in);
	uint16_t generateMessage(uint8_t *out,uint8_t pkgId);
};



#endif /* CONFIGURATION_H_ */
