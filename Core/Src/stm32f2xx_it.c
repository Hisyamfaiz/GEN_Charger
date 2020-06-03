/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f2xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f2xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "math.h"
#include "tim.h"
#include "Control_Init.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#define maxdata 100
/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */
void Eror_CodeCheck(void);
void Constant_Voltage(void);
void Constant_Current(void);
void Fault_Check(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

uint8_t		i, Temp_delay_calc, L=0, SS=0;
uint16_t	Tbuzz=999;
float		OFFSET_CurrentSense;
float		TripTime_OverCurrent,
			Count_TripTime;

uint8_t		SetProtection_ShortCircuit = 20;//Setting current protection
uint8_t		SetProtection_OverCurrent = 7;	//Setting current protection
uint8_t		SetProtection_OverVoltage = 65;	//Setting voltage protection
uint8_t		SetProtection_Temp2 = 60; 	//Setting inductor Temperature protection
uint8_t		SetProtection_Temp1 = 60;	//Setting Mosfet & Diode Temperature protection

void Clear_ProtectionFlag(void);

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F2xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f2xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles CAN1 SCE interrupt.
  */
void CAN1_SCE_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_SCE_IRQn 0 */

  /* USER CODE END CAN1_SCE_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_SCE_IRQn 1 */

  /* USER CODE END CAN1_SCE_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

	CAN_Rx_Process();	//can receive handle

	// *********************** Sensing Process (ADC average) ******************************

	ADC_SUM_Iin = ADC_SUM_Iin - ADC_Array_Iin[i];		//delete old data
	ADC_SUM_VinN = ADC_SUM_VinN - ADC_Array_VinN[i];
	ADC_SUM_VinP = ADC_SUM_VinP - ADC_Array_VinP[i];
	ADC_SUM_Iout = ADC_SUM_Iout - ADC_Array_Iout[i];
	ADC_SUM_VoutN = ADC_SUM_VoutN - ADC_Array_VoutN[i];
	ADC_SUM_VoutP = ADC_SUM_VoutP - ADC_Array_VoutP[i];

	ADC_Array_Iin[i] = ADC_Iin;				//save data from ADC read
	ADC_Array_VinN[i] = ADC_VinN;
	ADC_Array_VinP[i] = ADC_VinP;
	ADC_Array_Iout[i] = ADC_Iout;
	ADC_Array_VoutN[i] = ADC_VoutN;
	ADC_Array_VoutP[i] = ADC_VoutP;

	ADC_SUM_Iin = ADC_SUM_Iin + ADC_Array_Iin[i];		//summing data and add new data
	ADC_SUM_VinN = ADC_SUM_VinN + ADC_Array_VinN[i];
	ADC_SUM_VinP = ADC_SUM_VinP + ADC_Array_VinP[i];
	ADC_SUM_Iout = ADC_SUM_Iout + ADC_Array_Iout[i];
	ADC_SUM_VoutN = ADC_SUM_VoutN + ADC_Array_VoutN[i];
	ADC_SUM_VoutP = ADC_SUM_VoutP + ADC_Array_VoutP[i];

	ADC_Average_Iin = (float) ADC_SUM_Iin / maxdata;	//calculate average data
	ADC_Average_VinN = (float) ADC_SUM_VinN / maxdata;
	ADC_Average_VinP = (float) ADC_SUM_VinP / maxdata;
	ADC_Average_Iout = (float) ADC_SUM_Iout / maxdata;
	ADC_Average_VoutN = (float) ADC_SUM_VoutN / maxdata;
	ADC_Average_VoutP = (float) ADC_SUM_VoutP / maxdata;

	i++;
	i = i % maxdata;

	//Current value calculation and calibration
	Current_Charger = 0.0125*ADC_Average_Iout - 24.845 - OFFSET_CurrentSense;
	OFFSET_Calibration = 0.0125*ADC_Average_Iout - 24.845;
	if (Current_Charger<=0)
		Current_Charger = 0;

	//Voltage value calculation and calibration
	ADC_VoltageResult = fabs (ADC_Average_VoutN - ADC_Average_VoutP);
	Voltage_Charger = ADC_VoltageResult*0.0275-0.018;
	if(Voltage_Charger <= 0)
		Voltage_Charger = 0;

	Temp_delay_calc++;

	if(Temp_delay_calc >= 100)
	{
		Temp_delay_calc = 0;
		Res_T1 = ADC_temp1*10000/(3900-ADC_temp1); 	// 10000 => R1 , 3900 => Vcc dalam nilai digital
		Temp_T1 = -24.05*log(Res_T1) + 246.41;			//1 / a + b (Ln RT / R25) + c b (Ln RT / R25)2
		Res_T2 = ADC_temp2*10000/(3900-ADC_temp2);
		Temp_T2 = -24.05*log(Res_T2) + 246.41;			//1 / a + b (Ln RT / R25) + c b (Ln RT / R25)2
	}

	// *********************** end of Sensing Process (ADC average) ******************************


	// ***********************Charge or standby State ******************************

	if (Charger_Mode == 1){	//charge mode

		if(	flag_trip_overvoltage == 1		||
			flag_trip_overtemperature == 1	||
			flag_trip_undertemperature == 1	||
			flag_trip_overcurrentcharge == 1||
			flag_trip_SOCOverCharge == 1	||
			flag_trip_shortcircuit == 1		||
			flag_trip_systemfailure == 1	||
			Flag_ChargerShortCircuit == 1	||
			Flag_ChargerOverCurrent == 1	||
			Flag_ChargerOverTemperature == 1||
			Flag_ChargerOverVoltage == 1	)
			{
				duty=0;
				htim1.Instance->CCR1=duty*TIM1->ARR;
				Charger_Mode = 2;
			}

		Fault_Check();
		htim1.Instance->CCR1=duty*TIM1->ARR;
		if(duty>=0.9)
			duty=0;

		if(Batt_SOC.m_uint16t>70){
			Constant_Voltage();
		}

		if(Batt_SOC.m_uint16t<=70){
			Constant_Current();
		}

		//Clearing Charger Decrease rating flag
		if (flag_Derating == 1 && Temp_T1<=(SetProtection_Temp1-15) && Temp_T2<=(SetProtection_Temp2-25)){
			flag_Derating = 0;
		}

		L=0; Tbuzz=999;
	}

	if(Charger_Mode == 0){	//standby mode
		duty=0;
		htim1.Instance->CCR1=duty*TIM1->ARR;
		Clear_ProtectionFlag();
		Eror_Code = 0;
		OFFSET_CurrentSense = OFFSET_Calibration;
	}

	if(Charger_Mode == 2){	//Protection mode
		Tbuzz=Tbuzz+1;
		if (Tbuzz==1000 && L<=5){
			HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);
			HAL_GPIO_TogglePin(GPIOB, Led1_Pin);
			Tbuzz=0; L+=1;
		}

		if (HAL_GPIO_ReadPin(GPIOC, Button2_Pin)==1){
			HAL_GPIO_TogglePin(GPIOC, Led3_Pin);
			HAL_GPIO_WritePin(GPIOC, Buzzer_Pin, 0);
			Clear_ProtectionFlag();
			dc=0; Charger_Mode =1;
		}

		//Clearing Charger Over Temperature
		if (Flag_ChargerOverTemperature == 1 && Temp_T1<=(SetProtection_Temp1-10) && Temp_T2<=(SetProtection_Temp2-10) && L>5){
			Flag_ChargerOverTemperature = 0;
			dc=0; Charger_Mode =1;
		}

		//Clearing Battery Over Temperature
		if (flag_trip_overtemperature == 0 && LastFlag_OverTemperature == 1){
			dc=0; Charger_Mode =1;
		}
	}

	Eror_CodeCheck();

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
//	CAN_Tx_Process();
	SS+=1;



  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
//	CAN_Rx_Process();	//can receive handle

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 Stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void Eror_CodeCheck(void)
{
	if (flag_trip_shortcircuit==1)
		Eror_Code=1;	//Battery Pack short circuit

	else if (flag_trip_overcurrentcharge==1)
		Eror_Code=3;	//Battery Pack over current charge

	else if (flag_trip_overtemperature==1){
		Eror_Code=4;	//Battery Pack over temperature
		LastFlag_OverTemperature = 1;
	}

	else if (flag_trip_undertemperature==1)
		Eror_Code=5;	//Battery Pack under temperature

	else if (flag_trip_unbalance==1)
		Eror_Code=8;	//Battery Pack unbalance

	else if (flag_trip_undervoltage==1)
		Eror_Code=9;	//Battery Pack under voltage

	else if (flag_trip_overvoltage==1)
		Eror_Code=10;	//Battery Pack over voltage

	else if (flag_trip_systemfailure==1)
			Eror_Code=12;	//Battery Pack system failure


	else if (Flag_ChargerUnderVoltage==1)
		Eror_Code=13;	//Charger Under Voltage

//	else if (Flag_ChargerOverTemperature==1)
//		Eror_Code=14;	//Charger Over Temperature

	else if (Flag_ChargerUnderTemperature==1)
		Eror_Code=15;	//Charger Under Temperature

	else if (Flag_ChargerShortCircuit==1)
		Eror_Code=16;	//Charger Short Circuit

	else if (Flag_ChargerOverCurrent==1)
		Eror_Code=17;	//Charger Over Current

	else if (Flag_ChargerOverVoltage==1)
		Eror_Code=18;	//Charger Over Current
	else if(Flag_ChargerLostCommunication==1)
		Eror_Code=19;
//	else
//		Eror_Code=0;
}

void Clear_ProtectionFlag(void)
{
	flag_trip_overvoltage = 0;
	flag_trip_overtemperature = 0;
	flag_trip_undertemperature = 0;
	flag_trip_overcurrentcharge = 0;
	flag_trip_SOCOverCharge = 0;
	flag_trip_shortcircuit = 0;
	flag_trip_systemfailure = 0;
	Flag_ChargerOverCurrent = 0;
	Flag_ChargerOverTemperature = 0;
	Flag_ChargerOverVoltage = 0;
	Flag_ChargerLostCommunication = 0;
}

void Fault_Check(void)
{
	if(Current_Charger >= SetProtection_ShortCircuit){
		Flag_ChargerShortCircuit=1;
		HAL_GPIO_WritePin(GPIOC, Buzzer_Pin,1);
	}

	else if((SetProtection_OverCurrent - Current_Charger)<=0 && Flag_ChargerOverCurrent==0 ){
		Eror_Code=17;
		TripTime_OverCurrent = 1.8/(((Current_Charger/SetProtection_OverCurrent)*(Current_Charger/SetProtection_OverCurrent))-1);
		Count_TripTime += 0.001;

		if(Count_TripTime >= TripTime_OverCurrent){
			Flag_ChargerOverCurrent=1;
		}
	}

	// Decrease Charger Rating check
	// Temp1 = mosfet & diode
	// Temp2 = inductor
	else if ( Temp_T1 >= (SetProtection_Temp1-15)  || Temp_T2 >= (SetProtection_Temp2-15)){
		flag_Derating = 1;
		Eror_Code = 14;
		if(Temp_T1 >= SetProtection_Temp1 || Temp_T2 >= SetProtection_Temp2){
			Flag_ChargerOverTemperature = 1;
			Charger_Mode = 2;
		}
	}

	else if(Voltage_Charger >= SetProtection_OverVoltage){
		Flag_ChargerOverVoltage=1;
	}

	else {
		if (Eror_Code != 0) LastEror_code = Eror_Code;
		Eror_Code = 0;
		TripTime_OverCurrent = 0;
		Count_TripTime -= 0.001;
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
