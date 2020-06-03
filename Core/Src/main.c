/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "i2c-lcd.h"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "EEPROM.h"

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
CAN_TxHeaderTypeDef		Tx_Header;
CAN_RxHeaderTypeDef 	Rx_Header;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t			z=0, data, EEPROM_data;
char 			buffer_i2c[100];
char 			usart_Tx_buffer[100];
extern uint8_t	Eror_Code;
extern float 	dc;
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
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
  Charger_Mode = 0;
  Eror_Code = 0;
  CHARGER_ON_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  EEPROM_isDeviceReady(0XA0);
  EEPROM_WriteData(10, 15);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(Charger_Mode==1) 		Display_ChargeMode();
	  else if (Charger_Mode==2)	Display_ProtectionMode();
	  else						Display_StanbyMode();

	  HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 195;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance==ADC1)
	{
		ADC_VoutN = ADC_value[0];
		ADC_VoutP = ADC_value[1];
		ADC_Iin = ADC_value[2];
		ADC_temp1 = ADC_value[3];
		ADC_temp2 = ADC_value[4];
		ADC_VinN = ADC_value[5];
		ADC_VinP = ADC_value[6];
		ADC_Iout = ADC_value[7];
	}
}

void CHARGER_ON_Init(void)
{
	SSD1306_Init();
	HAL_Delay(1000);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_UpdateScreen();

	HAL_GPIO_WritePin(GPIOC, Buzzer_Pin,1);
	HAL_Delay(300);
	HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);
	HAL_Delay(300);
	HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);
	HAL_Delay(100);

	CAN_Setting();

	SSD1306_GotoXY (15,10);
	SSD1306_Puts ("GEN-I Charger", &Font_7x10, 1);
	SSD1306_GotoXY (50, 30);
	SSD1306_Puts ("V1.0", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
	SSD1306_Fill (0);

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_value, 8);
	HAL_Delay(1000);
//	Charger_Mode=1;
}

void Display_StanbyMode(void){
	SSD1306_Fill (0);
	SSD1306_GotoXY (20,30);
	SSD1306_Puts ("STANBY-Mode", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
}

void Display_ProtectionMode(void){
	SSD1306_Fill (0);
	SSD1306_GotoXY (20,10);
	SSD1306_Puts ("FAULT Protect", &Font_7x10, 1);

	sprintf(buffer_i2c, "Eror =%2d", Eror_Code);
	SSD1306_GotoXY (20,30);
	SSD1306_Puts (buffer_i2c, &Font_7x10, 1);

	SSD1306_UpdateScreen(); //display
}

void Display_ChargeMode(void){
	SSD1306_Fill (0);

	sprintf(buffer_i2c, "Charger - RUN");
	SSD1306_GotoXY (12,0);
	SSD1306_Puts (buffer_i2c, &Font_7x10, 1);

	sprintf(buffer_i2c, "D = %4.1f | %2d| %2.0f\r\n", dc, Batt_SOC.m_uint16t, BPack_Temp);
//	sprintf(buffer_i2c, "D = %4.1f | %4d   \r\n", dc, EEPROM_ReadData(10));
	SSD1306_GotoXY (5,13);
	SSD1306_Puts (buffer_i2c, &Font_7x10, 1);

	//sprintf(usart_Tx_buffer,"Test USART %d\r\n",(unsigned int)i);
	HAL_UART_Transmit_IT(&huart3, (uint8_t *)buffer_i2c, strlen(buffer_i2c));
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)buffer_i2c, strlen(buffer_i2c));

	HAL_Delay(10);

	sprintf(buffer_i2c, "T = %4.1f | %4.1f", Temp_T1, Temp_T2);
	SSD1306_GotoXY (5,23);
	SSD1306_Puts (buffer_i2c, &Font_7x10, 1);

	sprintf(buffer_i2c, "V = %4.0f | %4.2f", ADC_VoltageResult, Voltage_Charger);
//	sprintf(buffer_i2c, "V = %4.0f | %4.0f", ADC_Average_VoutN, ADC_Average_VoutP);
	SSD1306_GotoXY (5,33);
	SSD1306_Puts (buffer_i2c, &Font_7x10, 1);

	sprintf(buffer_i2c, "A = %4.0f | %4.2f", ADC_Average_Iout, Current_Charger);
	//(float)Batt_current.m_uint16t/100);
	SSD1306_GotoXY (5,43);
	SSD1306_Puts (buffer_i2c, &Font_7x10, 1);

	sprintf(buffer_i2c, "E =%2d--%2d ", Eror_Code, LastEror_code);
	SSD1306_GotoXY (5,53);
	SSD1306_Puts (buffer_i2c, &Font_7x10, 1);

	sprintf(buffer_i2c, "| %lx", UNIQUE_Code);
	SSD1306_GotoXY (68,53);
	SSD1306_Puts (buffer_i2c, &Font_7x10, 1);

	SSD1306_UpdateScreen(); //display
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
