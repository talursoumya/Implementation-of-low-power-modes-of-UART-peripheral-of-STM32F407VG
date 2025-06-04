/*
 * main.h
 *
 *  Created on: Jan 28, 2025
 *      Author: talur
 */

#ifndef INC_MAIN_H_
#define INC_MAIN_H_

#include "stm32f4xx_hal.h"


#define TRUE 1
#define FALSE 0


#define SYS_CLOCK_FREQ_50_MHZ  50
#define SYS_CLOCK_FREQ_84_MHZ  84
#define	SYS_CLOCK_FREQ_120_MHZ  120


#define LED1_PORT GPIOC
#define LED2_PORT GPIOC
#define LED3_PORT GPIOB
#define LED4_PORT GPIOC


#define LED1_PIN_NO GPIO_PIN_9
#define LED2_PIN_NO GPIO_PIN_8
#define LED3_PIN_NO GPIO_PIN_8
#define LED4_PIN_NO GPIO_PIN_6

#endif /* INC_MAIN_H_ */
