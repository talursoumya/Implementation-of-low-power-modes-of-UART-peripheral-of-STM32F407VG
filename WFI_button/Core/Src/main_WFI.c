/*
 * main.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */
/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config_HSE(uint8_t clock_freq);
void TIMER6_Init(void);
void UART2_Init(void);
void Error_handler(void);
void GPIO_Init(void);
void GPIO_AnalogConfig(void);

TIM_HandleTypeDef htimer6;
UART_HandleTypeDef huart2;
extern uint8_t some_data[];



int main(void)
{
  HAL_Init();

  GPIO_Init();

  UART2_Init();


  TIMER6_Init();

  //SCB-.SCR |= (1<<1);
   HAL_PWR_EnableSleepOnExit();

  //lets start with fresh status register of timer to avoid any spurious timer interrupts
  TIM6->SR = 0;

  //lets start timer-polling method
  HAL_TIM_Base_Start_IT(&htimer6);

  while(1);


  return 0;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config_HSE(uint8_t clock_freq)
{
	RCC_OscInitTypeDef osc_init;

	RCC_ClkInitTypeDef clk_init;

	//uint8_t flash_latency =0;

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;

	osc_init.HSEState = RCC_HSE_ON;


	osc_init.PLL.PLLState = RCC_PLL_ON;

	osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;


switch(clock_freq)
	{
		case SYS_CLOCK_FREQ_50_MHZ:
		{
		osc_init.PLL.PLLM = 4;
		osc_init.PLL.PLLN = 50;
		osc_init.PLL.PLLP = RCC_PLLP_DIV2;
		osc_init.PLL.PLLQ = 2;
		osc_init.PLL.PLLM = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;

		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;

		clk_init.APB2CLKDivider = RCC_HCLK_DIV1;


		break;
		}


		case SYS_CLOCK_FREQ_84_MHZ:
		{
		osc_init.PLL.PLLM = 8;

		osc_init.PLL.PLLN = 84;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;
		osc_init.PLL.PLLM = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;

		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;

		clk_init.APB2CLKDivider = RCC_HCLK_DIV1;


		break;

		}


		case SYS_CLOCK_FREQ_120_MHZ:
		{
				osc_init.PLL.PLLM = 8;
				osc_init.PLL.PLLN = 120;
				osc_init.PLL.PLLP = RCC_PLLP_DIV2;
				osc_init.PLL.PLLQ = 2;
				osc_init.PLL.PLLM = 2;

				clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
				clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

				clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;

				clk_init.APB1CLKDivider = RCC_HCLK_DIV4;

				clk_init.APB2CLKDivider = RCC_HCLK_DIV2;


				break;
		}


	    default:
		return;
}

		if(HAL_RCC_OscConfig(&osc_init) != HAL_OK)
		{
			Error_handler();
		}

		if(HAL_RCC_ClockConfig(&clk_init,FLASH_LATENCY_2) != HAL_OK)
		{
			Error_handler();
		}

		//systick congiguration
		//1ms delay
		HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

		HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);


	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


void GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();

	__HAL_RCC_GPIOC_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_SLEEP_DISABLE();
#if 0
	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_5;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);


	ledgpio.Pin = GPIO_PIN_12;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);
#endif
	GPIO_InitTypeDef buttongpio;
	buttongpio.Pin = GPIO_PIN_13;
	buttongpio.Mode = GPIO_MODE_IT_FALLING;
	buttongpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC,&buttongpio);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn,15,0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);


}

void TIMER6_Init(void)
{
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 4999;
	//htimer6.Init.Period = 64000-1;
	//htimer6.Init.Period = 6400-1;
	htimer6.Init.Period = 32-1;
	if(HAL_TIM_Base_Init(&htimer6) != HAL_OK)
	{
		Error_handler();
	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htimer6)
{
	if(HAL_UART_Transmit(&huart2,(uint8_t*)some_data,(uint16_t)strlen((char*)some_data),HAL_MAX_DELAY)!= HAL_OK);
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_handler(void)
{
  while(1);
}


void UART2_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  if ( HAL_UART_Init(&huart2) != HAL_OK )
  {
    //There is a problem
    Error_handler();
  }
}

void GPIO_AnalogConfig(void)
{
	GPIO_initTypeDef GpioA,GpioC;

	uint32_t gpio_pins = GPIO_PIN_0 | GPIO_PIN_1 |GPIO_PIN_4 | GPIO_PIN_5\
			GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8  \
			GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 \
			GPIO_PIN_13 | GPIO_PIN_14|GPIO_PIN_15 ;

	GpioA.Pin = gpio_pins;
	GpioA.Mode = GPIO_MODE_ANALOG;
	HAL_Gpio_Init(GPIOA,&GpioA);

	gpio_pins = GPIO_PIN_0 | GPIO_PIN_1 |GPIO_PIN_2 |GPIO_PIN_3 |\
				GPIO_PIN_4 | GPIO_PIN_5|\
				GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | \
				GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |\
				 GPIO_PIN_14|GPIO_PIN_15 ;

	GpioC.Pin = gpio_pins;
	GpioC.Mode = GPIO_MODE_ANALOG;
	HAL_Gpio_Init(GPIOC,&GpioC);

}

void HAL_GPIO_EXIT_Callback(uint16_t GPIO_Pin)
{
	if(HAL_UART_Transmit(&huart2,(uint8_t*)some_data,(uint16_t)strlen((char*)some_data),HAL_MAX_DELAY)!= HAL_OK)
	{
		Error_handler();
	}
}
