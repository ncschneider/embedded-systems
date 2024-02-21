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

void sendChar(char text);
char readChar();

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	char received;
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
	// Configure RCC for GPIOC and USART3
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// initialize green LED (PC9)
	GPIOC->MODER |= (1 << 18); // set green LED (PC9) to output
	GPIOC->MODER &= ~(1 << 19);
	GPIOC->OTYPER &= ~(1 << 9);
	GPIOC->OSPEEDR &= ~(1 << 18);
	GPIOC->PUPDR &= ~(1 << 18) & ~(1 << 19);
	
	// initialize orange LED (PC8)
	GPIOC->MODER |= (1 << 16); // set orange LED (PC8) to output
	GPIOC->MODER &= ~(1 << 17);
	GPIOC->OTYPER &= ~(1 << 8);
	GPIOC->OSPEEDR &= ~(1 << 16);
	GPIOC->PUPDR &= ~(1 << 16) & ~(1 << 17);
	
	// initialize Blue LED (PC7)
	GPIOC->MODER |= (1 << 14); // set blue LED (PC7) to output
	GPIOC->MODER &= ~(1 << 15);
	GPIOC->OTYPER &= ~(1 << 7); // push-pull output
	GPIOC->OSPEEDR &= ~(1 << 14); // low speed
	GPIOC->PUPDR &= ~(1 << 14) & ~(1 << 15); // no pull-up/pull-down
	
	// initialize Red LED (PC6)
	GPIOC->MODER |= (1 << 12); // set red LED (PC6) to output
	GPIOC->MODER &= ~(1 << 13);
	GPIOC->OTYPER &= ~(1 << 6); // push-pull output
	GPIOC->OSPEEDR &= ~(1 << 12); // low speed
	GPIOC->PUPDR &= ~(1 << 12) & ~(1 << 13); // no pull-up/pull-down
	
	// Configure USART3 on PC4/PC5
	GPIOC->MODER |= (1 << 9) | (1 << 11); // set PC4/5 to alternate function mode
	GPIOC->MODER &= ~(1 << 8) & ~(1 << 10);
	GPIOC->OTYPER &= ~(1 << 4) & ~(1 << 5); // push-pull output
	GPIOC->OSPEEDR &= ~(1 << 8) & ~(1 << 10); // low speed
	GPIOC->PUPDR &= ~(1 << 8) & ~(1 << 9) & ~(1 << 10) & ~(1 << 11); // no pull-up/pull-down
	GPIOC->AFR[0] |= (1 << 16) | (1 << 20); // select AF1 (0b0001) on PC4 and PC5
	
	// USART baud = 115200 b/s, set BRR to 69 -> 0x45 from 8MHz clock, 8M/69 = 115942 b/s, error = 0.64%
	USART3->BRR |= 0x45; // 115200 baud
	
	// enable TX/RX on USART3
	USART3->CR1 |= (1 << 2) | (1 << 3); // enable TX/RX
	USART3->CR1 |= (1 << 0); // enable USART
	
  while (1)
  {
		received = readChar();
		if(received == 'r') {
			GPIOC->ODR |= (1 << 6);
			GPIOC->ODR &= ~(1 << 7) & ~(1 << 8) & ~(1 << 9);
		}
		else if(received == 'b') {
			GPIOC->ODR |= (1 << 7);
			GPIOC->ODR &= ~(1 << 6) & ~(1 << 8) & ~(1 << 9);
		}
		else if(received == 'o') {
			GPIOC->ODR |= (1 << 8);
			GPIOC->ODR &= ~(1 << 7) & ~(1 << 6) & ~(1 << 9);
		}
		else if(received == 'g') {
			GPIOC->ODR |= (1 << 9);
			GPIOC->ODR &= ~(1 << 7) & ~(1 << 8) & ~(1 << 6);
		}
		else {
			sendChar('E');
			sendChar('R');
			sendChar('R');
			sendChar('O');
			sendChar('R');
			sendChar('\n');
			sendChar('\r');
			GPIOC->ODR &= ~(1 << 6) & ~(1 << 7) & ~(1 << 8) & ~(1 << 9);

		}
  }
}

char readChar() {
	while(!(USART3->ISR & (1 << 5))) {} // wait for received data to be ready
	char text = USART3->RDR & 0xFF; // store 8 bits into character variable
	return text;
}

// Send one character
void sendChar(char text) {
	while(!(USART3->ISR & (1 << 7))) {} // wait for transmit register to report empty
	USART3->TDR = text; // transmit character
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
