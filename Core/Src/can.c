/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"

extern CAN_TxHeaderTypeDef	Tx_Header;
extern CAN_RxHeaderTypeDef 	Rx_Header;

uint8_t		Tx_data[8];
uint32_t	TxMailbox;
uint8_t		Rx_data[8];
uint8_t		identified=0;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void CAN_Setting(void)
{
/* Configure the CAN Filter */
	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	//sFilterConfig.FilterIdHigh = 0x00B0 << 5;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	//sFilterConfig.FilterMaskIdHigh = 0x00B0 << 5;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) Error_Handler();

	/* Start the CAN peripheral */
	if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();

	/* Configure Transmission process */
	Tx_Header.TransmitGlobalTime = DISABLE;
	Tx_Header.RTR = CAN_RTR_DATA;
	Tx_Header.IDE = CAN_ID_STD;

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN_Tx_Process(void)
{
  Tx_Header.ExtId = 0x7B1FFFFF;
  Tx_data[0] = 1;
  Tx_data[1] = 2;
  Tx_data[2] = 3;
  Tx_data[3] = 4;

  Tx_Header.DLC = 8;
  if(HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) Error_Handler();
}

void CAN_Rx_Process(void)
{
if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Rx_Header, Rx_data)== HAL_OK){
	HAL_GPIO_TogglePin(GPIOB, Led2_Pin);
	Communication_Flag = 1;
}

		if(Handshaking==0){
			if(identified == 0){
				Tx_Header.StdId = 0x1B2;
				Tx_data[0] = 0x01;
				Tx_Header.DLC = 8;
				if(HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) Error_Handler();
			}
		// CAN ID receive (Handshaking)
			if(Rx_Header.ExtId>>20==0x0E0){
				if(Rx_data[6]==0x55 && identified==0){
					Tx_Header.StdId = 0x0E2;
					Tx_data[0] = 1;
					Tx_data[1] = 2;
					Tx_data[2] = 3;
					Tx_data[3] = 4;
					Tx_data[4] = 5;
					Tx_data[5] = 6;
					Tx_data[6] = 0x55;
					Tx_data[7] = 8;

					Tx_Header.DLC = 8;
					if(HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) Error_Handler();
					identified = 1;
				}

				if(Rx_data[6]==0xAA && identified==1){
					Tx_data[6] = 0xAA;
					if(HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) Error_Handler();
					UNIQUE_Code = Rx_Header.ExtId & 0x000FFFFF;
					Handshaking=1;

					uint32_t Delay_Charger = 20000000;
					while(Delay_Charger>0)
					Delay_Charger--;
					Charger_Mode=1;

					Tx_Header.StdId = 0x0C1;
					Tx_data[0] = UNIQUE_Code >> 12;
					Tx_data[1] = UNIQUE_Code >> 4;
					Tx_data[2] = UNIQUE_Code << 4;
					Tx_data[3] = 0;
					Tx_data[4] = 0;
					Tx_data[5] = 0;
					Tx_data[6] = 0;
					Tx_data[7] = 0;
					if(HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) Error_Handler();
				}
			}
		}

		if(Handshaking==1){
		// CAN ID receive #1 (0x7b1)
		if(Rx_Header.ExtId == (0x0B0<<20|UNIQUE_Code)){
			Batt_voltage.m_bytes[1] = Rx_data[0];
			Batt_voltage.m_bytes[0] = Rx_data[1];
			Batt_current.m_bytes[1] = Rx_data[2];
			Batt_current.m_bytes[0] = Rx_data[3];
			Batt_SOC.m_bytes[1] = Rx_data[4];
			Batt_SOC.m_bytes[0] = Rx_data[5];
			Batt_temp.m_bytes[1] = Rx_data[6];
			Batt_temp.m_bytes[0] = Rx_data[7];
		}

		// CAN ID receive #2 (0x7b2)
		if(Rx_Header.ExtId == (0x0B1<<20|UNIQUE_Code)){
			Batt_capacity.m_bytes[1] = Rx_data[0];
			Batt_capacity.m_bytes[0] = Rx_data[1];
			Batt_SOH.m_bytes[1] = Rx_data[2];
			Batt_SOH.m_bytes[0] = Rx_data[3];
			Batt_cycle.m_bytes[1] = Rx_data[4];
			Batt_cycle.m_bytes[0] = Rx_data[5];

			flag_trip_shortcircuit = Rx_data[6]&0x01;
			flag_trip_overcurrentdischarge = (Rx_data[6]>>1)&0x01;
			flag_trip_overcurrentcharge = (Rx_data[6]>>2)&0x01;
			flag_trip_overtemperature = (Rx_data[6]>>3)&0x01;
			flag_trip_undertemperature = (Rx_data[6]>>4)&0x01;
	//			flag_trip_overtemperature = (Rx_data[6]>>5)&0x01;
	//			flag_trip_undertemperature = (Rx_data[6]>>6)&0x01;
			flag_trip_unbalance = (Rx_data[6]>>7)&0x01;

			flag_trip_undervoltage = Rx_data[7]&0x01;
			flag_trip_overvoltage = (Rx_data[7]<<1)&0x01;
			flag_trip_SOCOverDischarge = (Rx_data[7]<<2)&0x01;
			flag_trip_systemfailure = (Rx_data[7]<<3)&0x01;
			charge_state = (Rx_data[7]<<4)&0x01;
			discharge_state = (Rx_data[7]<<5)&0x01;
			sleep_state = (Rx_data[7]<<6)&0x01;
		}


		// *********************** Start Cell  Voltage Data Send ******************************
		if(Rx_Header.ExtId == (0x0B4<<20|UNIQUE_Code)){
			vcell_15databyte[0].m_bytes[1] = Rx_data[0];
			vcell_15databyte[0].m_bytes[0] = Rx_data[1];
			vcell_15databyte[1].m_bytes[1] = Rx_data[2];
			vcell_15databyte[1].m_bytes[0] = Rx_data[3];
			vcell_15databyte[2].m_bytes[1] = Rx_data[4];
			vcell_15databyte[2].m_bytes[0] = Rx_data[5];
			vcell_15databyte[3].m_bytes[1] = Rx_data[6];
			vcell_15databyte[3].m_bytes[0] = Rx_data[7];
		}

		if(Rx_Header.ExtId == (0x0B5<<20|UNIQUE_Code)){
			vcell_15databyte[4].m_bytes[1] = Rx_data[0];
			vcell_15databyte[4].m_bytes[0] = Rx_data[1];
			vcell_15databyte[5].m_bytes[1] = Rx_data[2];
			vcell_15databyte[5].m_bytes[0] = Rx_data[3];
			vcell_15databyte[6].m_bytes[1] = Rx_data[4];
			vcell_15databyte[6].m_bytes[0] = Rx_data[5];
			vcell_15databyte[7].m_bytes[1] = Rx_data[6];
			vcell_15databyte[7].m_bytes[0] = Rx_data[7];
		}

		if(Rx_Header.ExtId == (0x0B6<<20|UNIQUE_Code)){
			vcell_15databyte[8].m_bytes[1] = Rx_data[0];
			vcell_15databyte[8].m_bytes[0] = Rx_data[1];
			vcell_15databyte[9].m_bytes[1] = Rx_data[2];
			vcell_15databyte[9].m_bytes[0] = Rx_data[3];
			vcell_15databyte[10].m_bytes[1] = Rx_data[4];
			vcell_15databyte[10].m_bytes[0] = Rx_data[5];
			vcell_15databyte[11].m_bytes[1] = Rx_data[6];
			vcell_15databyte[11].m_bytes[0] = Rx_data[7];
		}

		if(Rx_Header.ExtId == (0x0B7<<20|UNIQUE_Code)){
			vcell_15databyte[12].m_bytes[1] = Rx_data[0];
			vcell_15databyte[12].m_bytes[0] = Rx_data[1];
			vcell_15databyte[13].m_bytes[1] = Rx_data[2];
			vcell_15databyte[13].m_bytes[0] = Rx_data[3];
			vcell_15databyte[14].m_bytes[1] = Rx_data[4];
			vcell_15databyte[14].m_bytes[0] = Rx_data[5];
			vcell_15databyte[15].m_bytes[1] = Rx_data[6];
			vcell_15databyte[15].m_bytes[0] = Rx_data[7];
		}
		}
		// ******************************End Cell  Voltage Data Send**************************************

}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//	}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
