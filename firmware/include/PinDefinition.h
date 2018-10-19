/*
 * PinDefinition.h
 *
 *  Created on: 16 Jun 2015
 *      Author: pekka
 */


#ifndef PINDEFINITION_H_
#define PINDEFINITION_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>



struct PinDef {
	uint8_t bank;
	GPIO_TypeDef *gpio;

	uint16_t pin;

	uint8_t af_Funct;
	int8_t pwm_CH;
	TIM_TypeDef *timer;

};

#ifndef PCB_REV
#define PCB_REV 2
#endif

#define OE_GPIO GPIOD
#define OE_GPIO_Periph RCC_AHB1Periph_GPIOD
#define DIR_GPIO GPIOD
#define DIR_GPIO_Periph RCC_AHB1Periph_GPIOD

#define LED_GPIO GPIOD
#define LED_GPIO_Periph RCC_AHB1Periph_GPIOD

static const uint16_t DIR_Pins[] = {GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4};
static const uint16_t OE_Pins[] = {GPIO_PIN_7,GPIO_PIN_8,GPIO_PIN_9,GPIO_PIN_10,GPIO_PIN_11};
static const uint16_t LED_Pins[] = {GPIO_PIN_12, GPIO_PIN_13};

#if PCB_REV == 1
static const struct PinDef pins[] =
{
		//0
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_11,//pin
				GPIO_AF1_TIM1, //pinSource
				4, //PWM_CH
				TIM1 //Timer

		},
		//1
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_10,//pin
				GPIO_AF1_TIM1, //pinSource
				3, //PWM_CH
				TIM1 //Timer

		},
		//2
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_9,//pin
				GPIO_AF1_TIM1, //pinSource
				2, //PWM_CH
				TIM1 //Timer

		},
		//3
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_8,//pin
				GPIO_AF1_TIM1, //pinSource
				1, //PWM_CH
				TIM1 //Timer

		},
		//4
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_3,//pin
				GPIO_AF1_TIM1, //pinSource
				0, //PWM_CH
				TIM1 //Timer

		},
		//5
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_4,//pin
				GPIO_AF1_TIM1, //pinSource
				0, //PWM_CH
				TIM1 //Timer

		},
		//6
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_5,//pin
				GPIO_AF1_TIM1, //pinSource
				0, //PWM_CH
				TIM1 //Timer

		},
		//7
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_6,//pin
				GPIO_AF1_TIM1, //pinSource
				0, //PWM_CH
				TIM1 //Timer

		},

		//8
		{
				2,//Bank
				GPIOA,//Port
				GPIO_PIN_15,//pin
				GPIO_AF1_TIM2, //pinSource
				1, //PWM_CH
				TIM2 //Timer

		},
		//9
		{
				2,//Bank
				GPIOB,//Port
				GPIO_PIN_3,//pin
				GPIO_AF1_TIM2, //pinSource
				2, //PWM_CH
				TIM2 //Timer

		},
		//10
		{
				2,//Bank
				GPIOB,//Port
				GPIO_PIN_10,//pin
				GPIO_AF1_TIM2, //pinSource
				3, //PWM_CH
				TIM2 //Timer

		},
		//11
		{
				2,//Bank
				GPIOE,//Port
				GPIO_PIN_11,//pin
				GPIO_AF1_TIM2, //pinSource
				0, //PWM_CH
				TIM2 //Timer

		},
		//12
		{
				2,//Bank
				GPIOE,//Port
				GPIO_PIN_10,//pin
				GPIO_AF1_TIM2, //pinSource
				0, //PWM_CH
				TIM2 //Timer

		},
		//13
		{
				2,//Bank
				GPIOE,//Port
				GPIO_PIN_9,//pin
				GPIO_AF1_TIM2, //pinSource
				0, //PWM_CH
				TIM2 //Timer

		},
		//14
		{
				2,//Bank
				GPIOE,//Port
				GPIO_PIN_8,//pin
				GPIO_AF1_TIM2, //pinSource
				0, //PWM_CH
				TIM2 //Timer

		},
		//15
		{
				2,//Bank
				GPIOE,//Port
				GPIO_PIN_7,//pin
				GPIO_AF1_TIM2, //pinSource
				0, //PWM_CH
				TIM2 //Timer

		},
		//16
		{
				3,//Bank
				GPIOB,//Port
				GPIO_PIN_4,//pin
				GPIO_AF2_TIM3, //pinSource
				1, //PWM_CH
				TIM3 //Timer

		},
		//17
		{
				3,//Bank
				GPIOB,//Port
				GPIO_PIN_5,//pin
				GPIO_AF2_TIM3, //pinSource
				2, //PWM_CH
				TIM3 //Timer

		},
		//18
		{
				3,//Bank
				GPIOB,//Port
				GPIO_PIN_0,//pin
				GPIO_AF2_TIM3, //pinSource
				3, //PWM_CH
				TIM3 //Timer

		},
		//19
		{
				3,//Bank
				GPIOB,//Port
				GPIO_PIN_1,//pin
				GPIO_AF2_TIM3, //pinSource
				4, //PWM_CH
				TIM3 //Timer

		},
		//20
		{
				3,//Bank
				GPIOE,//Port
				GPIO_PIN_2,//pin
				GPIO_AF2_TIM3, //pinSource
				0, //PWM_CH
				TIM3 //Timer

		},
		//21
		{
				3,//Bank
				GPIOE,//Port
				GPIO_PIN_3,//pin
				GPIO_AF2_TIM3, //pinSource
				0, //PWM_CH
				TIM3 //Timer

		},
		//22
		{
				3,//Bank
				GPIOE,//Port
				GPIO_PIN_4,//pin
				GPIO_AF2_TIM3, //pinSource
				0, //PWM_CH
				TIM3 //Timer

		},
		//23
		{
				3,//Bank
				GPIOE,//Port
				GPIO_PIN_5,//pin
				GPIO_AF2_TIM3, //pinSource
				0, //PWM_CH
				TIM3 //Timer

		},
		//24
		{
				4,//Bank
				GPIOB,//Port
				GPIO_PIN_6,//pin
				GPIO_AF2_TIM4, //pinSource
				1, //PWM_CH
				TIM4 //Timer

		},
		//25
		{
				4,//Bank
				GPIOB,//Port
				GPIO_PIN_7,//pin
				GPIO_AF2_TIM4, //pinSource
				2, //PWM_CH
				TIM4 //Timer

		},
		//26
		{
				4,//Bank
				GPIOB,//Port
				GPIO_PIN_8,//pin
				GPIO_AF2_TIM4, //pinSource
				3, //PWM_CH
				TIM4 //Timer

		},
		//27
		{
				4,//Bank
				GPIOB,//Port
				GPIO_PIN_9,//pin
				GPIO_AF2_TIM4, //pinSource
				4, //PWM_CH
				TIM4 //Timer

		},
		//28
		{
				4,//Bank
				GPIOE,//Port
				GPIO_PIN_12,//pin
				GPIO_AF2_TIM4, //pinSource
				0, //PWM_CH
				TIM4 //Timer

		},
		//29
		{
				4,//Bank
				GPIOE,//Port
				GPIO_PIN_13,//pin
				GPIO_AF2_TIM4, //pinSource
				0, //PWM_CH
				TIM4 //Timer

		},
		//30
		{
				4,//Bank
				GPIOE,//Port
				GPIO_PIN_14,//pin
				GPIO_AF2_TIM4, //pinSource
				0, //PWM_CH
				TIM4 //Timer

		},
		//31
		{
				4,//Bank
				GPIOE,//Port
				GPIO_PIN_15,//pin
				GPIO_AF2_TIM4, //pinSource
				0, //PWM_CH
				TIM4 //Timer

		},
		//32
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_9,//pin
				GPIO_AF3_TIM8, //pinSource
				4, //PWM_CH
				TIM8 //Timer

		},
		//33
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_8,//pin
				GPIO_AF3_TIM8, //pinSource
				3, //PWM_CH
				TIM8 //Timer

		},
		//34
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_7,//pin
				GPIO_AF3_TIM8, //pinSource
				2, //PWM_CH
				TIM8 //Timer

		},
		//35
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_6,//pin
				GPIO_AF3_TIM8, //pinSource
				1, //PWM_CH
				TIM8 //Timer

		},
		//36
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_10,//pin
				GPIO_AF3_TIM8, //pinSource
				0, //PWM_CH
				TIM8 //Timer

		},
		//37
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_11,//pin
				GPIO_AF3_TIM8, //pinSource
				0, //PWM_CH
				TIM8 //Timer

		},
		//38
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_12,//pin
				GPIO_AF3_TIM8, //pinSource
				0, //PWM_CH
				TIM8 //Timer

		},
		//39
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_13,//pin
				GPIO_AF3_TIM8, //pinSource
				0, //PWM_CH
				TIM8 //Timer

		},

};
#endif
#if PCB_REV == 2
static const struct PinDef pins[] =
{
		//0
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_11,//pin
				GPIO_AF1_TIM1, //pinSource
				4, //PWM_CH
				TIM1 //Timer

		},
		//1
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_10,//pin
				GPIO_AF1_TIM1, //pinSource
				3, //PWM_CH
				TIM1 //Timer

		},
		//2
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_9,//pin
				GPIO_AF1_TIM1, //pinSource
				2, //PWM_CH
				TIM1 //Timer

		},
		//3
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_8,//pin
				GPIO_AF1_TIM1, //pinSource
				1, //PWM_CH
				TIM1 //Timer

		},
		//4
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_3,//pin
				GPIO_AF1_TIM1, //pinSource
				0, //PWM_CH
				TIM1 //Timer

		},
		//5
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_4,//pin
				GPIO_AF1_TIM1, //pinSource
				0, //PWM_CH
				TIM1 //Timer

		},
		//6
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_5,//pin
				GPIO_AF1_TIM1, //pinSource
				0, //PWM_CH
				TIM1 //Timer

		},
		//7
		{
				1,//Bank
				GPIOA,//Port
				GPIO_PIN_6,//pin
				GPIO_AF1_TIM1, //pinSource
				0, //PWM_CH
				TIM1 //Timer

		},

		//8
		{
				2,//Bank
				GPIOA,//Port
				GPIO_PIN_15,//pin
				GPIO_AF1_TIM2, //pinSource
				1, //PWM_CH
				TIM2 //Timer

		},
		//9
		{
				2,//Bank
				GPIOB,//Port
				GPIO_PIN_3,//pin
				GPIO_AF1_TIM2, //pinSource
				2, //PWM_CH
				TIM2 //Timer

		},
		//10
		{
				2,//Bank
				GPIOB,//Port
				GPIO_PIN_10,//pin
				GPIO_AF1_TIM2, //pinSource
				3, //PWM_CH
				TIM2 //Timer

		},
		//11
		{
				2,//Bank
				GPIOE,//Port
				GPIO_PIN_11,//pin
				GPIO_AF1_TIM2, //pinSource
				0, //PWM_CH
				TIM2 //Timer

		},
		//12
		{
				2,//Bank
				GPIOE,//Port
				GPIO_PIN_10,//pin
				GPIO_AF1_TIM2, //pinSource
				0, //PWM_CH
				TIM2 //Timer

		},
		//13
		{
				2,//Bank
				GPIOE,//Port
				GPIO_PIN_9,//pin
				GPIO_AF1_TIM2, //pinSource
				0, //PWM_CH
				TIM2 //Timer

		},
		//14
		{
				2,//Bank
				GPIOE,//Port
				GPIO_PIN_8,//pin
				GPIO_AF1_TIM2, //pinSource
				0, //PWM_CH
				TIM2 //Timer

		},
		//15
		{
				2,//Bank
				GPIOE,//Port
				GPIO_PIN_7,//pin
				GPIO_AF1_TIM2, //pinSource
				0, //PWM_CH
				TIM2 //Timer

		},
		//16
		{
				3,//Bank
				GPIOB,//Port
				GPIO_PIN_5,//pin
				GPIO_AF2_TIM3, //pinSource
				2, //PWM_CH
				TIM3 //Timer

		},
		//17
		{
				3,//Bank
				GPIOB,//Port
				GPIO_PIN_4,//pin
				GPIO_AF2_TIM3, //pinSource
				1, //PWM_CH
				TIM3 //Timer

		},
		//18
		{
				3,//Bank
				GPIOB,//Port
				GPIO_PIN_0,//pin
				GPIO_AF2_TIM3, //pinSource
				3, //PWM_CH
				TIM3 //Timer

		},
		//19
		{
				3,//Bank
				GPIOB,//Port
				GPIO_PIN_1,//pin
				GPIO_AF2_TIM3, //pinSource
				4, //PWM_CH
				TIM3 //Timer

		},
		//20
		{
				3,//Bank
				GPIOE,//Port
				GPIO_PIN_5,//pin
				GPIO_AF2_TIM3, //pinSource
				0, //PWM_CH
				TIM3 //Timer

		},
		//21
		{
				3,//Bank
				GPIOE,//Port
				GPIO_PIN_4,//pin
				GPIO_AF2_TIM3, //pinSource
				0, //PWM_CH
				TIM3 //Timer

		},
		//22
		{
				3,//Bank
				GPIOE,//Port
				GPIO_PIN_3,//pin
				GPIO_AF2_TIM3, //pinSource
				0, //PWM_CH
				TIM3 //Timer

		},
		//23
		{
				3,//Bank
				GPIOE,//Port
				GPIO_PIN_2,//pin
				GPIO_AF2_TIM3, //pinSource
				0, //PWM_CH
				TIM3 //Timer

		},
		//24
		{
				4,//Bank
				GPIOB,//Port
				GPIO_PIN_9,//pin
				GPIO_AF2_TIM4, //pinSource
				4, //PWM_CH
				TIM4 //Timer

		},
		//25
		{
				4,//Bank
				GPIOB,//Port
				GPIO_PIN_8,//pin
				GPIO_AF2_TIM4, //pinSource
				3, //PWM_CH
				TIM4 //Timer

		},
		//26
		{
				4,//Bank
				GPIOB,//Port
				GPIO_PIN_7,//pin
				GPIO_AF2_TIM4, //pinSource
				2, //PWM_CH
				TIM4 //Timer

		},
		//27
		{
				4,//Bank
				GPIOB,//Port
				GPIO_PIN_6,//pin
				GPIO_AF2_TIM4, //pinSource
				1, //PWM_CH
				TIM4 //Timer

		},
		//28
		{
				4,//Bank
				GPIOC,//Port
				GPIO_PIN_13,//pin
				GPIO_AF2_TIM4, //pinSource
				0, //PWM_CH
				TIM4 //Timer

		},
		//29
		{
				4,//Bank
				GPIOC,//Port
				GPIO_PIN_12,//pin
				GPIO_AF2_TIM4, //pinSource
				0, //PWM_CH
				TIM4 //Timer

		},
		//30
		{
				4,//Bank
				GPIOC,//Port
				GPIO_PIN_11,//pin
				GPIO_AF2_TIM4, //pinSource
				0, //PWM_CH
				TIM4 //Timer

		},
		//31
		{
				4,//Bank
				GPIOC,//Port
				GPIO_PIN_10,//pin
				GPIO_AF2_TIM4, //pinSource
				0, //PWM_CH
				TIM4 //Timer

		},
		//32
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_9,//pin
				GPIO_AF3_TIM8, //pinSource
				4, //PWM_CH
				TIM8 //Timer

		},
		//33
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_8,//pin
				GPIO_AF3_TIM8, //pinSource
				3, //PWM_CH
				TIM8 //Timer

		},
		//34
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_7,//pin
				GPIO_AF3_TIM8, //pinSource
				2, //PWM_CH
				TIM8 //Timer

		},
		//35
		{
				5,//Bank
				GPIOC,//Port
				GPIO_PIN_6,//pin
				GPIO_AF3_TIM8, //pinSource
				1, //PWM_CH
				TIM8 //Timer

		},
		//36
		{
				5,//Bank
				GPIOE,//Port
				GPIO_PIN_15,//pin
				GPIO_AF3_TIM8, //pinSource
				0, //PWM_CH
				TIM8 //Timer

		},
		//37
		{
				5,//Bank
				GPIOE,//Port
				GPIO_PIN_14,//pin
				GPIO_AF3_TIM8, //pinSource
				0, //PWM_CH
				TIM8 //Timer

		},
		//38
		{
				5,//Bank
				GPIOE,//Port
				GPIO_PIN_13,//pin
				GPIO_AF3_TIM8, //pinSource
				0, //PWM_CH
				TIM8 //Timer

		},
		//39
		{
				5,//Bank
				GPIOE,//Port
				GPIO_PIN_12,//pin
				GPIO_AF3_TIM8, //pinSource
				0, //PWM_CH
				TIM8 //Timer

		},

};
#endif

#endif /* PINDEFINITION_H_ */
