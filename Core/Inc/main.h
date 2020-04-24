/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Buzzer_Pin GPIO_PIN_15
#define Buzzer_GPIO_Port GPIOC
#define Led1_Pin GPIO_PIN_14
#define Led1_GPIO_Port GPIOB
#define Led2_Pin GPIO_PIN_15
#define Led2_GPIO_Port GPIOB
#define Led3_Pin GPIO_PIN_6
#define Led3_GPIO_Port GPIOC
#define Button2_Pin GPIO_PIN_7
#define Button2_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

void CHARGER_ON_Init(void);
void Display_StanbyMode(void);
void Display_ChargeMode(void);
void Display_ProtectionMode(void);

uint16_t 	ADC_value[5];
uint16_t 	ADC_current;
uint16_t 	ADC_voltagen;
uint16_t 	ADC_voltagep;
uint16_t 	ADC_temp1;
uint16_t 	ADC_temp2;
uint8_t		Eror_Code,
			LastEror_code;
uint32_t	UNIQUE_Code;

int			ADC_SUM_I,
			ADC_SUM_Vn,
			ADC_SUM_Vp;

uint16_t	ADC_Array_I[100],
			ADC_Array_Vn[100],
			ADC_Array_Vp[100];

float		ADC_Average_I,
			ADC_Average_Vn,
			ADC_Average_Vp,
			Current_Charger,
			OFFSET_Calibration,
			OFFSET_CurrentSense,
			Voltage_Charger,
			ADC_VoltageResult;

float		Res_T1,
			Res_T2,
			Temp_T1,
			Temp_T2;

float 		duty;

uint8_t		Charger_Mode;

uint8_t		Flag_ChargerOverVoltage,
			Flag_ChargerUnderVoltage,
			Flag_ChargerOverTemperature,
			Flag_ChargerUnderTemperature,
			Flag_ChargerShortCircuit,
			Flag_ChargerOverCurrent,
			Flag_ChargerLostCommunication;

uint8_t 	flag_trip_overtemperature,
			flag_trip_undertemperature,
			flag_trip_SOCOverDischarge,
			flag_trip_SOCOverCharge,			//di tiada kan..!
			flag_trip_undervoltage,
			flag_trip_overvoltage,
			flag_trip_overcurrentdischarge,
			flag_trip_overcurrentcharge,
			flag_trip_shortcircuit,
			flag_trip_systemfailure,
			flag_trip_unbalance,
			flag_Derating,
			LastFlag_OverTemperature;


uint8_t		charge_state,
			discharge_state,
			sleep_state,
			Communication_Flag,
			Handshaking;

float 		BP_Voltage,
			BP_Capacity,
			BP_Current,
			BP_Cycle,
			BP_Temp,
			BP_SOC,
			BP_SOH;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
