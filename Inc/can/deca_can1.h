/*! ----------------------------------------------------------------------------
 * @file	deca_can1.h
 * @brief	Middleware for CAN communications
 *
 * @author	Tommaso Sabatini, 2018
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CAN_DECA_CAN1_H_
#define CAN_DECA_CAN1_H_

#if defined(STM32F405xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F105xC)
	#include "stm32f1xx_hal.h"
#endif
#include "platform/port.h"

/* @fn		can1_flush_report_buff
 * @brief 	FLUSH should have higher priority than can1_port_tx_msg()
 * 			it shall be called periodically from process, which can not be locked,
 * 			i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_can1_report_buff(void);

/* @fn		flush_can1_local_buff
 * @brief 	FLUSH should have high priority
 * 			it shall be called periodically from process, which can not be locked,
 * 			i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_can1_local_buff(void);

/* @fn 		can1_port_tx_msg()
 * @brief 	put message to circular report buffer
 * 			it will be transmitted in background ASAP from flushing Thread
 * @return	HAL_BUSY - can not do it now, wait for release
 * 			HAL_ERROR- buffer overflow
 * 			HAL_OK   - scheduled for transmission
 * */
HAL_StatusTypeDef can1_port_tx_msg(uint8_t *str, int16_t len);

/* @fn 		can1_port_rx_msg()
 * @brief 	get message from CAN1 buffer and put it in the circular local buffer
 * 			it will be transmitted in background ASAP from flushing Thread
 * @return	HAL_BUSY - can not do it now, wait for release
 * 			HAL_ERROR- buffer overflow
 * 			HAL_OK   - scheduled for transmission
 * */
HAL_StatusTypeDef can1_port_rx_msg(uint8_t *str, int16_t len);

/**
  * @brief  DW_CAN1_DataTx
  *         received data to be send over CAN1 endpoint are managed in
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: CAND_OK if all operations are OK else CAND_FAIL
  */
CAND_StatusTypeDef DW_CAN1_DataTx (uint8_t* Buf, uint16_t Len);

/**
  * @brief  DW_CAN1_DataRx
  *         Data received from CAN1 device
  *@note    This is application-level function. it is necessary to protect
  * 		[Buf], [Len] from modification in the CAN driver
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval CAND_FAIL / CAND_OK
  */
CAND_StatusTypeDef DW_CAN1_DataRx (uint8_t* Buf, uint16_t Len);

void send_can1message(uint8_t *string, int16_t len);

void can1_run(void);

#endif /* CAN_DECA_CAN1_H_ */
