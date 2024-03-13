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

/* Private define ------------------------------------------------------------*/
#define GYRO_ADDR 0b1101011

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void i2c_init(uint8_t addr, uint8_t nBytes, uint8_t rw);
void i2c_write(uint8_t data);
uint8_t i2c_read(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	// initialize clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// initialize digital outputs PB14 and PC0
	GPIOB->MODER |= (1 << 28);
	GPIOB->OTYPER &= ~(1 << 14);
	GPIOC->MODER |= (1 << 0);
	GPIOC->OTYPER &= ~(1 << 0);
	
	// initialize I2C1
	// SDA: GPIOB 11 -- AF1
	// SCL: GPIOB 13 -- AF5
	GPIOB->MODER |= (1 << 23) | (1 << 27);
	GPIOB->OTYPER |= (1 << 11) | (1 << 13);
	GPIOB->AFR[1] |= (1 << 12) | (1 << 20) | (1 << 22);
	
	/* Configure I2C2 timing
	 * Prescaler = 1
	 * SCLDEL = 0x4
	 * SDADEL = 0x2
	 * SCLH = 0x0F
	 * SCLL = 0x13
	 */
	I2C2->TIMINGR |= 0x10420F13;
	
	// enable I2C2
	I2C2->CR1 |= (1 << 0);
	
	// set PC0 and PB14 high
	GPIOB->ODR |= (1 << 14);
	GPIOC->ODR |= (1 << 0);
	
  while (1)
  {
		
  }
}

void i2c_init(uint8_t addr, uint8_t nBytes, uint8_t rw) {
	I2C2->CR2 &= ~(0x3FF << 0) & ~(0xFF << 16); // clear address and number of bytes
	I2C2->CR2 |= (nBytes << 16) | (addr << 1); // set number of bytes in transaction and slave address (shifted 1 for 7-bit addresses)
	if(rw == 0) { // set or clear read/write bit depending on desired operation (0 = write, 1 = read)
		I2C2->CR2 &= ~(1 << 10);
	}
	else if(rw == 1) {
		I2C2->CR2 |= (1 << 10);
	}
}

void i2c_write(uint8_t data) {
	I2C1->CR2 |= (1 << 13); // start
}

uint8_t i2c_read(void);

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
