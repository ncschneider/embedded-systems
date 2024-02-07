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
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

	// initialize green and red LEDs
	RCC->AHBENR |= (1 << 19) | (1 << 17); // enable RCC clock for GPIOC and GPIOA
	GPIOC->MODER |= (1 << 12); // set red LED (PC6) to output
	GPIOC->MODER &= ~(1 << 13);
	GPIOC->MODER |= (1 << 18); // set green LED (PC9) to output
	GPIOC->MODER &= ~(1 << 19);
	GPIOC->OTYPER &= ~(1 << 6) & ~(1 << 9); // set LEDs to push-pull output
	GPIOC->OSPEEDR &= ~(1 << 12) & ~(1 << 18); // set LEDs to low speed
	GPIOC->PUPDR &= ~(1 << 12) & ~(1 << 13) & ~(1 << 18) & ~(1 << 19); // no pull-up/pull-down on LEDs
	
	// initialize Blue LED (PC7)
	GPIOC->MODER |= (1 << 14); // set blue LED (PC7) to output
	GPIOC->MODER &= ~(1 << 15);
	GPIOC->OTYPER &= ~(1 << 7); // push-pull output
	GPIOC->OSPEEDR &= ~(1 << 14); // low speed
	GPIOC->PUPDR &= ~(1 << 14) & ~(1 << 15); // no pull-up/pull-down
	
	// initialize Orange LED (PC8)
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER &= ~(1 << 17);
	GPIOC->OTYPER &= ~(1 << 8);
	GPIOC->OSPEEDR &= ~(1 << 16);
	GPIOC->PUPDR &= ~(1 << 16) & ~(1 << 17);
	
	// initialize user button
	GPIOA->MODER &= ~(1 << 0) & ~(1 << 1); // set GPIOA pin 0 to input
	GPIOA->OSPEEDR &= ~(1 << 0) & ~(1 << 1); // set switch to low speed
	GPIOA->PUPDR |= (1 << 1); // set switch to pull-down resistor
	GPIOA->PUPDR &= ~(1 << 0);
	
	// set up EXTI0 interrupt for user button
	EXTI->IMR |= (1 << 0); // setup EXTI0
	EXTI->RTSR |= (1 << 0); // rising edge trigger
	
	RCC->APB2ENR |= (1 << 0); // enable RCC for SYSCFG
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // connect EXTI0 to PA0
	
	NVIC_EnableIRQ(EXTI0_1_IRQn); // enable EXTI0
	NVIC_SetPriority(EXTI0_1_IRQn, 1); // set to high priority
	//NVIC_SetPriority(EXTI0_1_IRQn, 3); // set EXTI to low priority
	NVIC_SetPriority(SysTick_IRQn, 2); // set SysTick to medium priority


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	GPIOC->ODR &= ~(1 << 7); // turn off blue LED
	GPIOC->ODR |= (1 << 9); // turn on green LED
  while (1)
  {
    GPIOC->ODR |= (1 << 6); // toggle red LED
		HAL_Delay(400);
		GPIOC->ODR &= ~(1 << 6);
		HAL_Delay(400);
  }
  /* USER CODE END 3 */
}

// user button interrupt handler
void EXTI0_1_IRQHandler(void) {
	GPIOC->ODR ^= (1 << 8);
	GPIOC->ODR ^= (1 << 9);
	for(volatile int i = 0; i < 1500000; i++) {} // add arbitrary delay to make interrupt perform badly
	GPIOC->ODR ^= (1 << 8);
	GPIOC->ODR ^= (1 << 9);
	EXTI->PR |= (1 << 0);
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
