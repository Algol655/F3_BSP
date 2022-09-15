/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#define IS_CAN_ALL_PERIPH(PERIPH) (((PERIPH) == CAN1) || \
                                   ((PERIPH) == CAN2))
/**
  * @defgroup CAN_Error_Code_constants
  * @{
  */
#define CAN_ErrorCode_NoErr           ((uint8_t)0x00) /*!< No Error */
#define	CAN_ErrorCode_StuffErr        ((uint8_t)0x10) /*!< Stuff Error */
#define	CAN_ErrorCode_FormErr         ((uint8_t)0x20) /*!< Form Error */
#define	CAN_ErrorCode_ACKErr          ((uint8_t)0x30) /*!< Acknowledgment Error */
#define	CAN_ErrorCode_BitRecessiveErr ((uint8_t)0x40) /*!< Bit Recessive Error */
#define	CAN_ErrorCode_BitDominantErr  ((uint8_t)0x50) /*!< Bit Dominant Error */
#define	CAN_ErrorCode_CRCErr          ((uint8_t)0x60) /*!< CRC Error  */
#define	CAN_ErrorCode_SoftwareSetErr  ((uint8_t)0x70) /*!< Software Set Error */
/* Following CAN Device status */
typedef enum {
  CAND_OK   = 0,
  CAND_BUSY,
  CAND_FAIL,
}CAND_StatusTypeDef;

CAND_StatusTypeDef		can1_status;
CAN_FilterTypeDef  		sFilterConfig10, sFilterConfig11, sFilterConfig12, sFilterConfig13, sFilterConfig14, sFilterConfig15, sFilterConfig20;
CAN_RxHeaderTypeDef 	RxHeader1, RxHeader2;
CAN_TxHeaderTypeDef 	TxHeader1, TxHeader2;

uint16_t can1_local_buff_offset, can2_local_buff_offset;
#define MyNodeID 0x03U
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Config(CAN_HandleTypeDef* CanHandle);
void CAN_SetBR(CAN_HandleTypeDef* CanHandle, uint8_t br);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

