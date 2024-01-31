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
int main(void) {
int redLedOn = 1; // flag to check which state the program is in. Value is 1 when the red LED is lit, and 0 when it is not
uint32_t debouncer = 0;
HAL_Init(); // Reset of all peripherals, init the Flash and Systick
SystemClock_Config(); //Configure the system clock
/* This example uses HAL library calls to control
the GPIOC peripheral. You’ll be redoing this code
with hardware register access. */
RCC->AHBENR |= (1 << 19) | (1 << 17); // enable RCC clock for GPIOC and GPIOA
GPIOA->MODER &= ~(1 << 0) & ~(1 << 1); // set GPIOA pin 0 to input
GPIOC->MODER |= (1 << 12); // set red LED (PC6) to output
GPIOC->MODER |= (1 << 14); // set blue LED (PC7) to output
GPIOC->OTYPER &= ~(1 << 6) & ~(1 << 7); // set LEDs to push-pull output
GPIOA->OSPEEDR &= ~(1 << 0) & ~(1 << 1); // set switch to low speed
GPIOC->OSPEEDR &= ~(1 << 12) & ~(1 << 14); // set LEDs to low speed
GPIOA->PUPDR |= (1 << 1); // set switch to pull-down resistor
GPIOA->PUPDR &= ~(1 << 0);
GPIOC->PUPDR &= ~(1 << 12) & ~(1 << 13) & ~(1 << 14) & ~(1 << 15); // no pull-up/pull-down on LEDs
// Set up a configuration struct to pass to the initialization function
GPIOC->ODR |= (1 << 6); // set red LED on
GPIOC->ODR &= ~(1 << 7); // turn off blue LED
while (1) {
HAL_Delay(5); // wait 5 ms for debouncer
debouncer = (debouncer << 1); // shift debouncer bits
// Toggle the output state of both PC8 and PC9 on switch input
if(GPIOA->IDR & 0x01) { // if port A bit 0 is high (button pressed)
	debouncer |= 0x01; // set lowest bit of debouncer
}
if (debouncer == 0xFFFFFFFF) {
// This code triggers repeatedly when button is steady high!
}
if (debouncer == 0x00000000) {
// This code triggers repeatedly when button is steady low!
}
if (debouncer == 0x7FFFFFFF) { // trigger only once on high press
	if(redLedOn == 1) { // if red LED is on when button is pressed
		GPIOC->ODR |= (1 << 7); // turn on blue LED
		GPIOC->ODR &= ~(1 << 6); // turn off red LED
		redLedOn = 0; // change flag
	}
	else { // if red LED is off when button is pressed
		GPIOC->ODR |= (1 << 6); // turn on red LED
		GPIOC->ODR &= ~(1 << 7); // turn off blue LED
		redLedOn = 1; // change flag
	}
}
}
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
