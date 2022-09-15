/*! ----------------------------------------------------------------------------
 * @file	deca_usart3.h
 * @brief	Middleware for USART communications
 *
 * @author	Tommaso Sabatini, 2018
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef USART_DECA_USART3_H_
#define USART_DECA_USART3_H_

#if defined(STM32F405xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F105xC)
	#include "stm32f1xx_hal.h"
#endif
#define RS485_MODE	(0)

/* @fn		usart3_flush_report_buff
 * @brief 	FLUSH should have higher priority than usart3_port_tx_msg()
 * 			it shall be called periodically from process, which usart not be locked,
 * 			i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_usart3_report_buff(void);

/* @fn		flush_usart3_local_buff
 * @brief 	FLUSH should have high priority
 * 			it shall be called periodically from process, which usart not be locked,
 * 			i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_usart3_local_buff(void);

/* @fn 		usart3_port_tx_msg()
 * @brief 	put message to circular report buffer
 * 			it will be transmitted in background ASAP from flushing Thread
 * @return	HAL_BUSY - usart not do it now, wait for release
 * 			HAL_ERROR- buffer overflow
 * 			HAL_OK   - scheduled for transmission
 * */
HAL_StatusTypeDef usart3_port_tx_msg(uint8_t *str, int16_t len);

/**
  * @brief  DW_USART3_DataTx
  *         received data to be send over USART3 endpoint are managed in
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USARTD_OK if all operations are OK else USARTD_FAIL
  */
//USARTD_StatusTypeDef DW_USART3_DataTx (uint8_t* Buf, uint16_t Len);

/**
  * @brief  DW_USART3_DataRx
  *         Data received from USART3 device
  *
  *@note
  *         This is application-level function. it is neseccary to protect
  * 		[Buf], [Len] from modification in the usb driver
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval USARTD_FAIL / USARTD_OK
  * 		the function modifies the signal "usart3_local_have_data"
  *         which indicates future processing is necessary
  */
//USARTD_StatusTypeDef DW_USART3_DataRx (uint8_t* Buf, uint16_t Len);

//USARTD_StatusTypeDef process_usart3message(int8_t* Buf);

void send_usart3message(uint8_t *string, int16_t len);

void usart3_run(void);

#endif /* USART_DECA_USART3_H_ */
