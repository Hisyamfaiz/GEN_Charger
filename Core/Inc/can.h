/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Setting(void);
void CAN_Tx_Process(void);
void CAN_Rx_Process(void);
void Clear_ProtectionFlag(void);

float	BPack_Temp,
		BPack_Capacity,
		BPack_Voltage,
		BPack_Current,
		BPack_SOH,
		BPack_SOC,
		BPack_cycle;

float	vcell_1,
		vcell_2,
		vcell_3,
		vcell_4,
		vcell_5,
		vcell_6,
		vcell_7,
		vcell_8,
		vcell_9,
		vcell_10,
		vcell_11,
		vcell_12,
		vcell_13,
		vcell_14,
		vcell_15;

uint8_t BPack_byte6,
		BPack_byte7;

union 	uint16t_byte {
		uint16_t m_uint16t;
		uint8_t  m_bytes[sizeof(float)];};

union uint16t_byte 	Batt_voltage,
					Batt_current,
					Batt_SOC,
					Batt_temp,
					Batt_capacity,
					Batt_cycle,
					Batt_SOH,
					vcell_15databyte[15],
					Bpack_maxvoltage,
					Bpack_maxchargecurrent,
					Bpack_maxdischargecurrent,
					Bpack_maxtemp;

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
