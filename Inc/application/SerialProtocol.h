/*! ----------------------------------------------------------------------------
 * @file	SerialProtocol.h
 * @brief	Serial Application Headers Collection.
 *          T.B.D.
 *
 * @author  Tommaso Sabatini, 2019
 */

#ifndef APPLICATION_SERIALPROTOCOL_H_
#define APPLICATION_SERIALPROTOCOL_H_

#include "platform/port.h"
#include "application/MEMS_app.h"

#define TMsg_EOF                	0xF0
#define TMsg_BS                 	0xF1
#define TMsg_BS_EOF             	0xF2
#define DEV_ADDR					50U		//0x32

#define MAX_ALLOWED_FRAME_ERRORS	(3)

typedef enum
{
	SerialProtocol_OK   = 0,
	SerialProtocol_CRC_KO,
	SerialProtocol_FAIL
} ProtocolStatus;

ProtocolStatus usart2_status, usart3_status;

extern void top_menu(void);

//Exported Functions
/**
 * @brief  Byte stuffing process for one byte
 * @param  Dest destination
 * @param  Source source
 * @retval Total number of bytes processed
*/
int ByteStuffCopyByte(uint8_t *Dest, uint8_t Source);

/**
 * @brief  Byte stuffing process for a Msg
 * @param  Dest destination
 * @param  Source source
 * @retval Total number of bytes processed
 */
int ByteStuffCopy(uint8_t *Dest, uint8_t *Source, uint16_t *Len);

/**
 * @brief  Compute and add checksum
 * @param  Msg pointer to the message
 * @retval None
 */
void CHK_ComputeAndAdd(uint8_t* Buff, uint16_t* len, int8_t Add_TOF);

/**
 * @brief  Unbuild a Number from an array (LSB first)
 * @param  Source source
 * @param  Len number of bytes
 * @retval Rebuild unsigned int variable
 */
uint32_t Deserialize(uint8_t *Source, uint32_t Len);

/**
 * @brief  Build the reply header
 * @param  Msg the pointer to the message to be built
 * @retval None
 */
void Build_Reply_Header(uint8_t* Buff);

/**
* @brief  Initialize the streaming header
* @param  Msg the pointer to the header to be initialized
* @retval None
*/
void Init_Streaming_Header(uint8_t* Buff);

/**
 * @brief  Handle a message from VCP
 * @param  Msg the pointer to the message to be handled
 * @retval 1 if the message is correctly handled, 0 otherwise
 */
int HandleUSB_MSG(uint8_t* Buff);

/**
 * @brief  Handle a message from USART2
 * @param  Msg the pointer to the message to be handled
 * @retval 1 if the message is correctly handled, 0 otherwise
 */
int HandleUSART2_MSG(uint8_t* Buff);

/**
 * @brief  Handle a message from USART3
 * @param  Msg the pointer to the message to be handled
 * @retval 1 if the message is correctly handled, 0 otherwise
 */
int HandleUSART3_MSG(uint8_t* Buff);

#endif /* APPLICATION_SERIALPROTOCOL_H_ */
