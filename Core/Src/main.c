/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
char formated_text[20];
float data[6], temp;
uint8_t test;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  uint8_t status = 0;
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
//  if(vd6283tx_init())
//	  status = 1;
//  else
//	  status = 0;
  /*data[0] = vd6283tx_get_als_ch1();
  data[1] = vd6283tx_get_als_ch2();
  data[2] = vd6283tx_get_als_ch3();
  data[3] = vd6283tx_get_als_ch4();
  data[4] = vd6283tx_get_als_ch5();*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(vd6283tx_get_interrupt() == 0x00)
//	  {
//		  data[0] = vd6283tx_get_als_ch1();
//	  	  data[1] = vd6283tx_get_als_ch2();
//	  	  data[2] = vd6283tx_get_als_ch3();
//	  	  data[3] = vd6283tx_get_als_ch4();
//	  	  data[4] = vd6283tx_get_als_ch5();
//	  	  data[5] = vd6283tx_get_als_ch6();
//	  	  vd6283tx_clear_interrupt();
//	  }
//	  memset(formated_text, '\0', sizeof(formated_text));
//	  sprintf(formated_text, "%0.0f,%0.0f,%0.0f,%0.0f,%0.0f,%0.0f\r\n", data[0],data[1],data[2],data[3],data[4],data[5]);
//	  USART2_PutBuffer((uint8_t*)formated_text, strlen(formated_text));
//	  LL_mDelay(1000);
//	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 5);//0degree
//	  LL_mDelay(3000);
//	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 15);//
//	  LL_mDelay(3000);
//	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 25);
//	  LL_mDelay(3000);
	  Servo_Control(0);//0degree????????
	  HAL_Delay(3000);
	  Servo_Control(45); //45degree????????
	  HAL_Delay(3000);
	  Servo_Control(90); //90degree????????
	  HAL_Delay(3000);
	  Servo_Control(180); //180degree????????
	  HAL_Delay(3000);
	  Servo_Control(135); //135degree????????
	  HAL_Delay(3000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(64000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */

void Servo_Control(uint16_t angle)
{
   float temp;
   temp =(1.0 / 9.0) * angle + 5.0;  //占空比�?? = 1/9 * 角度 + 5
   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t )temp);//修改占空�????????
   //htim3.Instance->CCR1 = temp;//修改占空�????????
}
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
