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
#define GYRO_ADDR 0x69
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_X 0xA8
#define OUT_Y 0xAA

/* Private variables ---------------------------------------------------------*/
uint8_t NUM_BYTES = 2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void i2c_init(uint8_t addr, uint8_t nBytes);
void i2c_write(uint8_t data);
uint8_t i2c_read(uint8_t addr);
int16_t i2c_readTwo(uint8_t addr);
void i2c_writeTwo(uint8_t addr, uint8_t data);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	uint8_t slaveData = 0x00;
	uint32_t counter = 0;
	int16_t xData = 0;
	int16_t yData = 0;
	
	// initialize clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
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
	
	// initialize digital outputs PB14 and PC0
	GPIOB->MODER |= (1 << 28);
	GPIOB->OTYPER &= ~(1 << 14);
	GPIOC->MODER |= (1 << 0);
	GPIOC->OTYPER &= ~(1 << 0);
	
	// initialize I2C2
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
	
	// initialize gyro control register for x and y axis
	i2c_init(GYRO_ADDR, NUM_BYTES);
	i2c_writeTwo(CTRL_REG1, 0x0B); // write 1 to x-axis enable, y-axis enable, and normal mode
  
	/*
	* RED: LED3 -- +Y
	* ORANGE: LED4 -- -X
	* GREEN: LED5 -- +X
	* BLUE: LED6 -- -Y
	*/
	while (1)
  {
			// collect gyro data
			xData = i2c_readTwo(OUT_X);
			yData = i2c_readTwo(OUT_Y);
			
			// compare gyro data to predefined thresholds to determine rotation directions
			if(xData > 200) {
				GPIOC->ODR |= (1 << 9);
				GPIOC->ODR &= ~(1 << 8);
			}
			else if(xData < -200) {
				GPIOC->ODR |= (1 << 8);
				GPIOC->ODR &= ~(1 << 9);
			}
			else {
				GPIOC->ODR &= ~(1 << 8) & ~(1 << 9);
			}
			
			if(yData > 200) {
				GPIOC->ODR |= (1 << 6);
				GPIOC->ODR &= ~(1 << 7);
			}
			else if(yData < -200) {
				GPIOC->ODR |= (1 << 7);
				GPIOC->ODR &= ~(1 << 6);
			}
			else {
				GPIOC->ODR &= ~(1 << 6) & ~(1 << 7);
			}
			HAL_Delay(100);
  }
}


// initialize I2C transaction
void i2c_init(uint8_t addr, uint8_t nBytes) {
	I2C2->CR2 &= ~(0xFF << 1) & ~(0xFF << 16); // clear address and number of bytes
	I2C2->CR2 |= (nBytes << 16) | (addr << 1); // set number of bytes in transaction and slave address (shifted 1 for 7-bit addresses)
}


// write one byte to the slave device
void i2c_write(uint8_t data) {
	I2C2->CR2 &= ~(1 << 10); // R/W bit set to 0 (write)
	I2C2->CR2 |= (1 << 13); // start
	while((I2C2->ISR & (1 << 1)) == 0) { // wait for TXIS to be set
		if(I2C2->ISR & (1 << 4)) { // check for NACK
		}
	}
	I2C2->TXDR = data;
	while(!(I2C2->ISR & (1 << 6))) {} // wait for transaction to complete
}


// read one byte from the slave device
uint8_t i2c_read(uint8_t addr) {
	uint8_t data;
	i2c_write(addr);
	i2c_init(GYRO_ADDR, NUM_BYTES);
	I2C2->CR2 |= (1 << 10); // set R/W bit to 1 (read)
	I2C2->CR2 |= (1 << 13); // start
	while(((I2C2->ISR >> 2) & 1) == 0) { // wait for RXNE to be set
		if(I2C2->ISR & (1 << 4)) { // check for NACK
		}
	}
	while(!(I2C2->ISR & (1 << 6))) {} // wait for transaction to complete
	data = I2C2->RXDR; // collect byte
	I2C2->CR2 |= (1 << 14); // stop
	return data;
}


// read two bytes of data, only for sequential reads without restart
int16_t i2c_readTwo(uint8_t addr) {
	uint16_t data1, data2;
	int16_t fullData;
	i2c_init(GYRO_ADDR, 1);
	i2c_write(addr);
	i2c_init(GYRO_ADDR, NUM_BYTES);
	I2C2->CR2 |= (1 << 10); // set R/W bit to 1 (read)
	I2C2->CR2 |= (1 << 13); // start
	while((I2C2->ISR & (1 << 2)) == 0) { // wait for RXNE to be set
		if(I2C2->ISR & (1 << 4)) { // check for NACK
		}
	}
	data1 = I2C2->RXDR; // collect byte 1
	while((I2C2->ISR & (1 << 2)) == 0) { // wait for RXNE to be set
		if(I2C2->ISR & (1 << 4)) { // check for NACK
		}
	}
	data2 = I2C2->RXDR; // collect byte 2
	I2C2->CR2 |= (1 << 14); // stop
	fullData = data1 | (data2 << 8); // concatenate both bytes into one string of 16 signed bits
	return fullData;
}


/*
 * write two bytes of data, usually the first is an address, the second is the data to write
 */
void i2c_writeTwo(uint8_t addr, uint8_t data) {
	I2C2->CR2 &= ~(1 << 10); // R/W bit set to 0 (write)
	I2C2->CR2 |= (1 << 13); // start
	while(((I2C2->ISR >> 1) & 1) == 0) { // wait for TXIS to be set
		if(I2C2->ISR & (1 << 4)) { // check for NACK
		}
	}
	I2C2->TXDR = addr;
	while(((I2C2->ISR >> 1) & 1) == 0) { // wait for TXIS to be set
		if(I2C2->ISR & (1 << 4)) { // check for NACK
		}
	}
	I2C2->TXDR = data;
	while(!(I2C2->ISR & (1 << 6))) {} // wait for transaction to complete
	I2C2->CR2 |= (1 << 14); // stop
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
