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

#define THRESHOLD_1 63
#define THRESHOLD_2 127
#define THRESHOLD_3 191
#define THRESHOLD_4 255

uint8_t adcVal = 0;
uint8_t counter = 0;
const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	
	// initialize clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
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
	
	// initialize Blue LED (PC7)
	GPIOC->MODER |= (1 << 14);
	GPIOC->MODER &= ~(1 << 15);
	GPIOC->OTYPER &= ~(1 << 7); // push-pull output
	GPIOC->OSPEEDR &= ~(1 << 14); // low speed
	GPIOC->PUPDR &= ~(1 << 14) & ~(1 << 15); // no pull-up/pull-down
	
	// initialize Red LED (PC6)
	GPIOC->MODER |= (1 << 12);
	GPIOC->MODER &= ~(1 << 13);
	GPIOC->OTYPER &= ~(1 << 6); // push-pull output
	GPIOC->OSPEEDR &= ~(1 << 12); // low speed
	GPIOC->PUPDR &= ~(1 << 12) & ~(1 << 13); // no pull-up/pull-down
	
	// set PC0 to analog mode (ADC_10)
	GPIOC->MODER |= (1 << 0) | (1 << 1);
	
	// setup ADC: 8bit resolution, continuous conversion, software trigger
	ADC1->CFGR1 |= (1 << 4); // resolution
	ADC1->CFGR1 &= ~(1 << 3); // resolution
	ADC1->CFGR1 |= (1 << 13); // continuous
	ADC1->CFGR1 &= ~(1 << 10) & ~(1 << 11); // hardware trigger disabled
	
	// select ADC channel 10 for conversion
	ADC1->CHSELR |= (1 << 10);
	
	// calibrate ADC and wait for calibration to complete
	ADC1->CR |= (1 << 31);
	while(ADC1->CR & (1 << 31)) {}
		
	// enable ADC and wait for confirmation that it is ready
	ADC1->CR |= (1 << 0);
	while((ADC1->ISR & (1 << 0)) == 0) {}
		
	// set PA4 to analog mode (DAC1)
	GPIOA->MODER |= (1 << 8) | (1 << 9);
		
	// set DAC channel 1 to software trigger
	DAC1->SWTRIGR |= (1 << 0);
		
	// enable DAC channel 1
	DAC1->CR |= (1 << 0);
	
	// start ADC
	ADC1->CR |= (1 << 2);

  while (1)
  {
		DAC1->DHR8R1 &= 0x00;
		DAC1->DHR8R1 |= sine_table[counter];
		counter++;
		if(counter == 32) {
			counter = 0;
		}
		
		if(ADC1->ISR & (1 << 2)) {
			adcVal = ADC1->DR; // store rightmost 8 bits
		}
		
		if(adcVal >= THRESHOLD_4) {
			GPIOC->ODR |= (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9);
		}
		else if(adcVal >= THRESHOLD_3) {
			GPIOC->ODR |= (1 << 6) | (1 << 7) | (1 << 9);
			GPIOC->ODR &= ~(1 << 8);
		}
		else if(adcVal >= THRESHOLD_2) {
			GPIOC->ODR |= (1 << 6) | (1 << 9);
			GPIOC->ODR &= ~(1 << 8) & ~(1 << 7);
		}
		else if(adcVal >= THRESHOLD_1) {
			GPIOC->ODR |= (1 << 6);
			GPIOC->ODR &= ~(1 << 7) & ~(1 << 8) & ~(1 << 9);
		}
		else {
			GPIOC->ODR &= ~(1 << 6) & ~(1 << 7) & ~(1 << 8) & ~(1 << 9);
		}
		HAL_Delay(1);
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
