/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
	// enable TIM2, TIM3, GPIOC RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// initialize green LED (PC9)
	GPIOC->MODER |= (1 << 18); // set green LED (PC9) to output
	GPIOC->MODER &= ~(1 << 19);
	GPIOC->OTYPER &= ~(1 << 9);
	GPIOC->OSPEEDR &= ~(1 << 18);
	GPIOC->PUPDR &= ~(1 << 18) & ~(1 << 19);
	
	// initialize orange LED (PC8)
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER &= ~(1 << 17);
	GPIOC->OTYPER &= ~(1 << 8);
	GPIOC->OSPEEDR &= ~(1 << 16);
	GPIOC->PUPDR &= ~(1 << 16) & ~(1 << 17);
	
	/*
	 * Set TIM2 frequency to 4Hz (native frequency = 8MHz)
	 *
	 * fTarget = fCLK/[(PSC+1)*ARR]
	 * 4 = 8000000/[(PSC+1)*ARR]
	 * choose PSC+1 = 2000, ARR = 1000 -> 8000000/[(1999+1)*ARR] = 8000000/[2000*1000] = 8000000/2000000 = 4
	 */
	TIM2->ARR &= 0x00000000; // reset ARR to all zeros
	TIM2->PSC |= 0x7CF; // 0x7CF = 1999 in hex, set prescaler for TIM2 to 1999
	TIM2->ARR |= 0x3E8; // 0x3E8 = 1000 in hex, set ARR for TIM2 to 1000
	TIM2->DIER |= (1 << 0); // enable UIE bit (update interrupt enable)
	TIM2->CR1 &= ~(1 << 1); // enable UEV
	TIM2->CR1 |= (1 << 4); // set to downcounter
	TIM2->CR1 |= (1 << 2); // force update only on counter overflow/underflow
	TIM2->CR1 |= (1 << 0); // set clock enable bit to start timer after setup
	
	NVIC_EnableIRQ(TIM2_IRQn); // enable interrupt request for TIM2
	//NVIC_SetPriority(TIM2_IRQn,1); // set to high priority arbitrarily

	// initial output state
	GPIOC->ODR |= (1 << 9);
	GPIOC->ODR &= ~(1 << 8);
  while(1)
  {
  }
}

void TIM2_IRQHandler(void)
{
	// flip LED states
	GPIOC->ODR ^= (1 << 9);
	GPIOC->ODR ^= (1 << 8);
	TIM2->SR &= ~(1 << 0); // clear interrupt request flag in TIM2 status register
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
