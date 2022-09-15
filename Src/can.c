/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
//#include "application/CANopen.h"
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
//uint8_t RxErrCounter = (uint8_t)(hcan1.Instance->ESR & CAN_ESR_REC >> 24);
//uint8_t TxErrCounter = (uint8_t)(hcan1.Instance->ESR & CAN_ESR_TEC >> 16);
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = CAN1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(CAN1_RX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAN1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAN1_TX_GPIO_Port, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
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
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, CAN1_RX_Pin|CAN1_TX_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*************************************************************************
 *                          MY CAN CODE
 ************************************************************************/
void CAN_Config(CAN_HandleTypeDef* CanHandle)
{
	//CanOPEN Filters
	uint32_t MyFilterId0 = 0x0600+MyNodeID;	//In FilterBank0 I accept only frames with STID of 0x0600+NodeID (SDO Rx)
	uint32_t MyFilterMask0 = 0x1FFFFFF0;	//in FilterBank0 Every Bit must match filter
	uint32_t MyFilterId1 = 0x0000;			//In FilterBank1 I accept only frames with STID of 0x0000 (NMT ID)
	uint32_t MyFilterMask1 = 0x1FFFFFFF;	//in FilterBank1 Every Bit must match filter
	uint32_t MyFilterId2 = 0x0180+MyNodeID;	//In FilterBank2 I accept only frames with STID of 0x0x80+NodeID (PDO1 Rx)
	uint32_t MyFilterMask2 = 0x1FFFFCF0;	//in FilterBank2 Every Bit must match filter
	uint32_t MyFilterId3 = 0x0700+MyNodeID;	//In FilterBank3 I accept only frames with STID of 0x0700+NodeID
	uint32_t MyFilterMask3 = 0x1FFFFFF0;	//in FilterBank3 Every Bit must match filter
	//Can Generic Filters
	uint32_t MyFilterId4 = 0x00FF0103;		//In FilterBank4 I accept only frames with EXTID of 0x00FF0103 (Dashboard On)
	uint32_t MyFilterMask4 = 0x1FFFFFFF;	//in FilterBank4 Every Bit must match filter
	uint32_t MyFilterId5 = 0x18FEBF0A;		//In FilterBank5 I accept only frames with EXTID of 0x18FEBF0A (Speed - EBC2)
	uint32_t MyFilterMask5 = 0x1FFFFFFF;	//in FilterBank5 Every Bit must match filter
	//Can 2 Filters
	uint32_t MyFilterId14 = 0x0655;			//In FilterBank14 I accept only frames with STID of 0x0655
	uint32_t MyFilterMask14 = 0x1FFFFFFF;	//in FilterBank14 Every Bit must match filter
/*	uint32_t MyFilterId15 = 0x0000;			//In FilterBank15 I accept only frames with STID of 0x00xx
	uint32_t MyFilterMask15 = 0x1FFFFF00;	//in FilterBank15 Every Bit must match filter */

//	CANopen_AppStates = INIT;

	if(CanHandle->Instance==CAN1)
	{
		/*##-2- Configure the CAN Filter ###########################################*/
		sFilterConfig10.FilterBank = 0;
		sFilterConfig10.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig10.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig10.FilterIdHigh = ((MyFilterId0 << 5) | (MyFilterId0 >> (32 - 5))) & 0xFFFF; 	//STID[10:0] & EXTID[17:13]
		sFilterConfig10.FilterIdLow = (MyFilterId0 >> (11 - 3)) & 0xFFF8; 							//EXID[12:5] & EXID[4:0] & 3 Reserved bits
		sFilterConfig10.FilterMaskIdHigh = ((MyFilterMask0 << 5) | (MyFilterMask0 >> (32 - 5))) & 0xFFFF;
		sFilterConfig10.FilterMaskIdLow = (MyFilterMask0 >> (11 - 3)) & 0xFFF8;
		sFilterConfig10.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig10.FilterActivation = ENABLE;
		sFilterConfig10.SlaveStartFilterBank = 14;

		sFilterConfig11.FilterBank = 1;
		sFilterConfig11.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig11.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig11.FilterIdHigh = ((MyFilterId1 << 5) | (MyFilterId1 >> (32 - 5))) & 0xFFFF; 	//STID[10:0] & EXTID[17:13]
		sFilterConfig11.FilterIdLow = (MyFilterId1 >> (11 - 3)) & 0xFFF8; 							//EXID[12:5] & EXID[4:0] & 3 Reserved bits
		sFilterConfig11.FilterMaskIdHigh = ((MyFilterMask1 << 5) | (MyFilterMask1 >> (32 - 5))) & 0xFFFF;
		sFilterConfig11.FilterMaskIdLow = (MyFilterMask1 >> (11 - 3)) & 0xFFF8;
		sFilterConfig11.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig11.FilterActivation = ENABLE;
		sFilterConfig11.SlaveStartFilterBank = 14;

		sFilterConfig12.FilterBank = 2;
		sFilterConfig12.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig12.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig12.FilterIdHigh = ((MyFilterId2 << 5) | (MyFilterId2 >> (32 - 5))) & 0xFFFF; 	//STID[10:0] & EXTID[17:13]
		sFilterConfig12.FilterIdLow = (MyFilterId2 >> (11 - 3)) & 0xFFF8; 							//EXID[12:5] & EXID[4:0] & 3 Reserved bits
		sFilterConfig12.FilterMaskIdHigh = ((MyFilterMask2 << 5) | (MyFilterMask2 >> (32 - 5))) & 0xFFFF;
		sFilterConfig12.FilterMaskIdLow = (MyFilterMask2 >> (11 - 3)) & 0xFFF8;
		sFilterConfig12.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig12.FilterActivation = ENABLE;
		sFilterConfig12.SlaveStartFilterBank = 14;

		sFilterConfig13.FilterBank = 3;
		sFilterConfig13.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig13.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig13.FilterIdHigh = ((MyFilterId3 << 5) | (MyFilterId3 >> (32 - 5))) & 0xFFFF; 	//STID[10:0] & EXTID[17:13]
		sFilterConfig13.FilterIdLow = (MyFilterId3 >> (11 - 3)) & 0xFFF8; 							//EXID[12:5] & EXID[4:0] & 3 Reserved bits
		sFilterConfig13.FilterMaskIdHigh = ((MyFilterMask3 << 5) | (MyFilterMask3 >> (32 - 5))) & 0xFFFF;
		sFilterConfig13.FilterMaskIdLow = (MyFilterMask3 >> (11 - 3)) & 0xFFF8;
		sFilterConfig13.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig13.FilterActivation = ENABLE;
		sFilterConfig13.SlaveStartFilterBank = 14;

		sFilterConfig14.FilterBank = 4;
		sFilterConfig14.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig14.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig14.FilterIdLow = ((MyFilterId4 << 3) | (MyFilterId4 >> (32 - 3))) & 0xFFF8; 	//STID[10:0] & EXTID[17:13]
		sFilterConfig14.FilterIdLow |= 0x0004;														//This is an EXTID !!
		sFilterConfig14.FilterIdHigh = (MyFilterId4 >> (16 - 3)) & 0xFFFF; 							//EXID[12:5] & EXID[4:0] & 3 Reserved bits
		sFilterConfig14.FilterMaskIdHigh = ((MyFilterMask4 << 3) | (MyFilterMask4 >> (32 - 3))) & 0xFFF8;
		sFilterConfig14.FilterMaskIdLow = (MyFilterMask4 >> (16 - 3)) & 0xFFFF;
		sFilterConfig14.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig14.FilterActivation = DISABLE;
		sFilterConfig14.SlaveStartFilterBank = 14;

		sFilterConfig15.FilterBank = 5;
		sFilterConfig15.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig15.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig15.FilterIdLow = ((MyFilterId5 << 3) | (MyFilterId5 >> (32 - 3))) & 0xFFF8; 	//STID[10:0] & EXTID[17:13]
		sFilterConfig15.FilterIdLow |= 0x0004;														//This is an EXTID !!
		sFilterConfig15.FilterIdHigh = (MyFilterId5 >> (16 - 3)) & 0xFFFF; 							//EXID[12:5] & 3 Reserved bits
		sFilterConfig15.FilterMaskIdHigh = ((MyFilterMask5 << 3) | (MyFilterMask5 >> (32 - 3))) & 0xFFF8;
		sFilterConfig15.FilterMaskIdLow = (MyFilterMask5 >> (16 - 3)) & 0xFFFF;
		sFilterConfig15.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig15.FilterActivation = DISABLE;
		sFilterConfig15.SlaveStartFilterBank = 14;

		if (HAL_CAN_ConfigFilter(CanHandle, &sFilterConfig10) != HAL_OK)
		{
			/* Filter configuration Error */
			Error_Handler();
		}
		if (HAL_CAN_ConfigFilter(CanHandle, &sFilterConfig11) != HAL_OK)
		{
			/* Filter configuration Error */
			Error_Handler();
		}
		if (HAL_CAN_ConfigFilter(CanHandle, &sFilterConfig12) != HAL_OK)
		{
			/* Filter configuration Error */
			Error_Handler();
		}
		if (HAL_CAN_ConfigFilter(CanHandle, &sFilterConfig13) != HAL_OK)
		{
			/* Filter configuration Error */
			Error_Handler();
		}
		if (HAL_CAN_ConfigFilter(CanHandle, &sFilterConfig14) != HAL_OK)
		{
			/* Filter configuration Error */
			Error_Handler();
		}
		if (HAL_CAN_ConfigFilter(CanHandle, &sFilterConfig15) != HAL_OK)
		{
			/* Filter configuration Error */
			Error_Handler();
		}

		/*##-3- Start the CAN peripheral ###########################################*/
		if (HAL_CAN_Start(CanHandle) != HAL_OK)
		{
			/* Start Error */
			Error_Handler();
		}

		/*##-4- Activate CAN RX notification #######################################*/
		if (HAL_CAN_ActivateNotification(CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		{
			/* Notification Error */
			Error_Handler();
		}

		/*##-4a- Activate CAN TX notification #######################################*/
		if (HAL_CAN_ActivateNotification(CanHandle, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
		{
			/* Notification Error */
			Error_Handler();
		}

		/*##-5- Configure Transmission process #####################################*/
		TxHeader1.StdId = 0x603;
		TxHeader1.ExtId = 0x603;
		TxHeader1.RTR = CAN_RTR_DATA;
		TxHeader1.IDE = CAN_ID_STD;
		TxHeader1.DLC = 8;
		TxHeader1.TransmitGlobalTime = DISABLE;

		/*##-6- Configure Reception process #####################################*/
		RxHeader1.StdId = 0x322;
		RxHeader1.ExtId = 0x01;
		RxHeader1.RTR = CAN_RTR_DATA;
		RxHeader1.IDE = CAN_ID_STD;
		RxHeader1.DLC = 8;
		RxHeader1.Timestamp = 0;
		RxHeader1.FilterMatchIndex = 0;
	}
	else if(CanHandle->Instance==CAN2)
	{
		/*##-2- Configure the CAN Filter ###########################################*/
		sFilterConfig20.FilterBank = 14;
		sFilterConfig20.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig20.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig20.FilterIdHigh = ((MyFilterId14 << 5) | (MyFilterId14 >> (32 - 5))) & 0xFFFF; 	//STID[10:0] & EXTID[17:13]
		sFilterConfig20.FilterIdLow = (MyFilterId14 >> (11 - 3)) & 0xFFF8; 								//EXID[12:5] & EXID[4:0] & 3 Reserved bits
		sFilterConfig20.FilterMaskIdHigh = ((MyFilterMask14 << 5) | (MyFilterMask14 >> (32 - 5))) & 0xFFFF;
		sFilterConfig20.FilterMaskIdLow = (MyFilterMask14 >> (11 - 3)) & 0xFFF8;
		sFilterConfig20.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig20.FilterActivation = ENABLE;
		sFilterConfig20.SlaveStartFilterBank = 14;

/*		sFilterConfig21.FilterBank = 15;
		sFilterConfig21.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig21.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig21.FilterIdHigh = ((MyFilterId15 << 5) | (MyFilterId15 >> (32 - 5))) & 0xFFFF; 	//STID[10:0] & EXTID[17:13]
		sFilterConfig21.FilterIdLow = (MyFilterId15 >> (11 - 3)) & 0xFFF8; 								//EXID[12:5] & EXID[4:0] & 3 Reserved bits
		sFilterConfig21.FilterMaskIdHigh = ((MyFilterMask15 << 5) | (MyFilterMask15 >> (32 - 5))) & 0xFFFF;
		sFilterConfig21.FilterMaskIdLow = (MyFilterMask15 >> (11 - 3)) & 0xFFF8;
		sFilterConfig21.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig21.FilterActivation = ENABLE;
		sFilterConfig21.SlaveStartFilterBank = 15; */

		if (HAL_CAN_ConfigFilter(CanHandle, &sFilterConfig20) != HAL_OK)
		{
			/* Filter configuration Error */
			Error_Handler();
		}

//		if (HAL_CAN_ConfigFilter(CanHandle, &sFilterConfig21) != HAL_OK)
//		{
			/* Filter configuration Error */
//			Error_Handler();
//		}

		/*##-3- Start the CAN peripheral ###########################################*/
		if (HAL_CAN_Start(CanHandle) != HAL_OK)
		{
			/* Start Error */
			Error_Handler();
		}

		/*##-4- Activate CAN RX notification #######################################*/
		if (HAL_CAN_ActivateNotification(CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		{
			/* Notification Error */
			Error_Handler();
		}

		/*##-4a- Activate CAN TX notification #######################################*/
		if (HAL_CAN_ActivateNotification(CanHandle, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
		{
			/* Notification Error */
			Error_Handler();
		}

		/*##-5- Configure Transmission process #####################################*/
		TxHeader2.StdId = 0x655;
		TxHeader2.ExtId = 0x655;
		TxHeader2.RTR = CAN_RTR_DATA;
		TxHeader2.IDE = CAN_ID_STD;
		TxHeader2.DLC = 8;
		TxHeader2.TransmitGlobalTime = DISABLE;

		/*##-6- Configure Reception process #####################################*/
		RxHeader2.StdId = 0x324;
		RxHeader2.ExtId = 0x01;
		RxHeader2.RTR = CAN_RTR_DATA;
		RxHeader2.IDE = CAN_ID_STD;
		RxHeader2.DLC = 8;
		RxHeader2.Timestamp = 0;
		RxHeader2.FilterMatchIndex = 0;
	}
}

/** @defgroup CAN Bus Set Baud Rate
 *  @brief    CAN Bus Baud Rate management functions
 *
 */
void CAN_SetBR(CAN_HandleTypeDef* CanHandle, uint8_t br)
{
	if (HAL_CAN_Stop(CanHandle) != HAL_OK)
	{
		Error_Handler();
	}
	CanHandle->Init.Mode = CAN_MODE_NORMAL;
	CanHandle->Init.SyncJumpWidth = CAN_SJW_1TQ;
	switch (br)
	{
		case 0x01:
			CanHandle->Init.Prescaler = 21;
			CanHandle->Init.TimeSeg1 = CAN_BS1_13TQ;
			CanHandle->Init.TimeSeg2 = CAN_BS2_2TQ;
			break;
		case 0x02:
			CanHandle->Init.Prescaler = 12;
			CanHandle->Init.TimeSeg1 = CAN_BS1_11TQ;
			CanHandle->Init.TimeSeg2 = CAN_BS2_2TQ;
			break;
		case 0x03:
			CanHandle->Init.Prescaler = 6;
			CanHandle->Init.TimeSeg1 = CAN_BS1_11TQ;
			CanHandle->Init.TimeSeg2 = CAN_BS2_2TQ;
			break;
		default:
			CanHandle->Init.Prescaler = 12;
			CanHandle->Init.TimeSeg1 = CAN_BS1_11TQ;
			CanHandle->Init.TimeSeg2 = CAN_BS2_2TQ;
			break;
	}
	WRITE_REG(CanHandle->Instance->BTR, (uint32_t)(CanHandle->Init.Mode        |
												CanHandle->Init.SyncJumpWidth  |
												CanHandle->Init.TimeSeg1       |
												CanHandle->Init.TimeSeg2       |
	                                            (CanHandle->Init.Prescaler - 1U)));
	if (HAL_CAN_Start(CanHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

/** @defgroup CAN_Group5 CAN Bus Error management functions
 *  @brief    CAN Bus Error management functions
 *
@verbatim
 ===============================================================================
                ##### CAN Bus Error management functions #####
 ===============================================================================
    [..] This section provides functions allowing to
      (+) Return the CANx's last error code (LEC)
      (+) Return the CANx Receive Error Counter (REC)
      (+) Return the LSB of the 9-bit CANx Transmit Error Counter(TEC).

      -@- If TEC is greater than 255, The CAN is in bus-off state.
      -@- if REC or TEC are greater than 96, an Error warning flag occurs.
      -@- if REC or TEC are greater than 127, an Error Passive Flag occurs.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the CANx's last error code (LEC).
  * @param  CANx: where x can be 1,2 or 3 to select the CAN peripheral.
  * @retval Error code:
  *          - CAN_ERRORCODE_NoErr: No Error
  *          - CAN_ERRORCODE_StuffErr: Stuff Error
  *          - CAN_ERRORCODE_FormErr: Form Error
  *          - CAN_ERRORCODE_ACKErr : Acknowledgment Error
  *          - CAN_ERRORCODE_BitRecessiveErr: Bit Recessive Error
  *          - CAN_ERRORCODE_BitDominantErr: Bit Dominant Error
  *          - CAN_ERRORCODE_CRCErr: CRC Error
  *          - CAN_ERRORCODE_SoftwareSetErr: Software Set Error
  */
uint8_t CAN_GetLastErrorCode(CAN_HandleTypeDef* CanHandle)
{
	uint8_t errorcode=0;
	
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CanHandle->Instance));
	
	/* Get the error code*/
	errorcode = (((uint8_t)CanHandle->Instance->ESR) & (uint8_t)CAN_ESR_LEC);
	
	/* Return the error code*/
	return errorcode;
}

/**
  * @brief  Returns the CANx Receive Error Counter (REC).
  * @note   In case of an error during reception, this counter is incremented
  *         by 1 or by 8 depending on the error condition as defined by the CAN
  *         standard. After every successful reception, the counter is
  *         decremented by 1 or reset to 120 if its value was higher than 128.
  *         When the counter value exceeds 127, the CAN controller enters the
  *         error passive state.
  * @param  CANx: where x can be 1,2 or 3 to select the CAN peripheral.
  * @note   CAN3 peripheral is available only for STM32F413_423xx devices
  * @retval CAN Receive Error Counter.
  */
uint8_t CAN_GetReceiveErrorCounter(CAN_HandleTypeDef* CanHandle)
{
	uint8_t counter=0;
	
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CanHandle->Instance));
	
	/* Get the Receive Error Counter*/
	counter = (uint8_t)((CanHandle->Instance->ESR & CAN_ESR_REC)>> 24);
	
	/* Return the Receive Error Counter*/
	return counter;
}

/**
  * @brief  Returns the LSB of the 9-bit CANx Transmit Error Counter(TEC).
  * @param  CANx: where x can be 1,2 or 3 to select the CAN peripheral.
  * @note   CAN3 peripheral is available only for STM32F413_423xx devices
  * @retval LSB of the 9-bit CAN Transmit Error Counter.
  */
uint8_t CAN_GetLSBTransmitErrorCounter(CAN_HandleTypeDef* CanHandle)
{
	uint8_t counter=0;
	
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CanHandle->Instance));
	
	/* Get the LSB of the 9-bit CANx Transmit Error Counter(TEC) */
	counter = (uint8_t)((CanHandle->Instance->ESR & CAN_ESR_TEC)>> 16);
	
	/* Return the LSB of the 9-bit CANx Transmit Error Counter(TEC) */
	return counter;
}
/* USER CODE END 1 */
