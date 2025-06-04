/*
 * it.c
 *
 *  Created on: Jan 24, 2025
 *      Author: talur
 */


#include "main.h"
extern TIM_HandleTypeDef htimer6;
extern UART_HandleTypeDef huart2;

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}



void TIM6_DAC_IRQHandler(void)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	__HAL_RCC_USART2_CLK_ENABLE();
	HAL_TIM_IRQHandler(&htimer6);
	__HAL_RCC_USART2_CLK_DISABLE();
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

}

void EXIT15_10_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}
