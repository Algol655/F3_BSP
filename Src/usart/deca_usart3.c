/*! ----------------------------------------------------------------------------
 * @file	deca_usart3.c
 * @brief	Middleware for USART communications
 *
 * @author	Tommaso Sabatini, 2018
 */

/* Includes */
#include <stddef.h>
#include "usart/deca_usart3.h"
//#include "platform/port.h"				//Da usare in alternativa a SerialProtocol.h
#include "application/SerialProtocol.h"

//USART rx data buffers
int16_t usart3_local_buff_length = 0;
uint8_t	usart3_local_buff[USARTBUFFLEN];	/**< non circular local buffer, data received from USART. */
static 	uint8_t usart3_lbuf[USARTBUFFLEN];	/**< circular local buffer, data to be transmitted in usart3_flush_report_buff() Thread. */

static circBuf_t usart3_local_buf = {.buf = usart3_lbuf,
							   	     .head= 0,
								     .tail= 0,
								     .maxLen=MSG_PAYLOAD_LEN_MAX-3};
//USART tx data buffers
int 	usart3_tx_buff_length = 0;
uint8_t usart3_tx_buff[USARTBUFFLEN];
static int 	usart3_local_have_data = 0;
static 	uint8_t usart3_rbuf[USARTBUFFLEN];	/**< circular report buffer, data to be transmitted in usart3_flush_report_buff() Thread. */
static	circBuf_t usart3_report_buf = {.buf = usart3_rbuf,
									   .head= 0,
									   .tail= 0,
									   .maxLen=USARTBUFFLEN};

static uint8_t 	usart3_ubuf[USARTBUFFLEN];	/**< used to transmit new chunk of data in single usart3 flush */
#if ((DATA_MODE==0) && (NORMAL_MODE==1))
	static uint8_t	FrameErrorsCnt = 0;
#endif
static struct
{
	HAL_LockTypeDef       Lock;		/*!< locking object                  */
}
txhandle={.Lock = HAL_UNLOCKED};

static struct
{
	HAL_LockTypeDef       Lock;		/*!< locking object                  */
}
tx1handle={.Lock = HAL_UNLOCKED};

static struct
{
	HAL_LockTypeDef       Lock;		/*!< locking object                  */
}
rxhandle={.Lock = HAL_UNLOCKED};
#if ((DATA_MODE==1) && (NORMAL_MODE==0))
static struct
{
	HAL_LockTypeDef       Lock;		/*!< locking object                  */
}
rx1handle={.Lock = HAL_UNLOCKED};
#endif
//extern HAL_StatusTypeDef usart3_port_tx_msg(uint8_t* str, uint16_t len);

#pragma GCC optimize ("O3")
/* @fn		mymemcpy
 * @brief 	Copies n characters from memory area src to memory area dest
 * 			To avoid buffer overflow, when end of src area is reached but len is > 0,
 * 			reinitialize source pointer to isrc
 * @param   dest: pointer to the destination array
 * @param   src:  pointer to the source array
 * @param   isrc: pointer to the first element of the source array
 * @param   esrc: pointer to the last element of the source array
 * @param   len: number of bytes to be copied
 *
 * @retval  This function returns a pointer to destination,	i.e. dest
 * */
/*void * mymemcpy (void *dest, const void *src, const void *isrc, const void *esrc, size_t len)
{
	char *d = dest;
	const char *s = src;
	const char *is = isrc;
	const char *es = esrc;
	while (len--)
	{
	    *d++ = *s++;
	  	if (s == es+1)
	  	{
	  		s = is;
	  	}
	}
  return dest;
} */

#pragma GCC optimize ("O3")
/* @fn		usart3_flush_report_buff
 * @brief 	FLUSH should have higher priority than usart3_port_tx_msg()
 * 			it shall be called periodically from process, which usart not be locked,
 * 			i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_usart3_report_buff(void)
{
	int i, head, tail, len, size = USARTBUFFLEN;
	uint32_t j = 0;

	__HAL_LOCK(&tx1handle);			//"return HAL_BUSY;" if locked
	head = usart3_report_buf.head;
	tail = usart3_report_buf.tail;
	len = CIRC_CNT(head, tail, size);

#if ((DATA_MODE==0) && (NORMAL_MODE==0))
	if( len > 0 )
#else
	while( len > 0 )
#endif
	{
		/* copy MAX allowed length from circular buffer to non-circular TX buffer */
//		len = MIN(USARTBUFFLEN, len);
		len = MIN(USART_DATA_MAX_PACKET_SIZE, len);

		for(i=0; i<len; i++)
		{
			usart3_ubuf[i] = usart3_report_buf.buf[tail];
			tail = (tail + 1) & (size - 1);
		}
		/* check USART3 status - ready to TX */
		while (huart3.gState == HAL_UART_STATE_BUSY_TX)
		{
			j++;
			if(j>0xffffe)
			{
				__HAL_UNLOCK(&tx1handle);
				return HAL_TIMEOUT;
			}
		}
		/* setup USART3 DMA transfer */
		if(HAL_UART_Transmit_DMA(&huart3, (uint8_t*)usart3_ubuf, len) != (uint8_t)USARTD_OK)
		{
			__HAL_UNLOCK(&tx1handle);
			return USARTD_FAIL;
		}

		len = CIRC_CNT(head, tail, size);
	}
	usart3_report_buf.tail = tail;
	__HAL_UNLOCK(&tx1handle);					//"return HAL_BUSY;" if locked

	return HAL_OK;
}

#pragma GCC optimize ("O3")
/* @fn		flush_usart3_local_buff
 * @brief 	FLUSH should have high priority
 * 			it shall be called periodically from process, which usart not be locked,
 * 			i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_usart3_local_buff(void)
{
#if ((DATA_MODE==1) && (NORMAL_MODE==0))
	int i, head, tail, len, size = USARTBUFFLEN;

	__HAL_LOCK(&rx1handle);
	head = usart3_local_buf.head;
	tail = usart3_local_buf.tail;
	len = CIRC_CNT(head, tail, size);

	if( len > 0 )
	{
		/* copy MAX allowed length from circular buffer to circular TX buffer */
		len = MIN(usart3_local_buf.maxLen, len);

		for(i=0; i<len; i++)
		{
			msg_payload[i] = usart3_local_buf.buf[tail];
			tail = (tail + 1) & (size - 1);
		}

		usart3_local_buf.tail = tail;
	}
#else
	/* copy MAX allowed length from circular buffer to non-circular TX buffer */
	memcpy(&msg_payload[0], usart3_local_buff, usart3_local_buff_length);
	memset(usart3_local_buff, 0x0, usart3_local_buff_length);
#endif
	memcpy(&msg_payload[sizeof(msg_payload)-2], (uint8_t*) &(usart3_local_buff_length), sizeof(usart3_local_buff_length));	//transmit msg length
	msg_payload[sizeof(msg_payload)-3] = d_zero;	//transmit the usart bit-rate setting or ack info
	if (usart3_local_buff_length > 994)
	{
	}
	usart3_local_buff_offset = 0;
	usart3_local_buff_length = 0;

#if ((DATA_MODE==1) && (NORMAL_MODE==0))
	__HAL_UNLOCK(&rx1handle);
#endif
	return HAL_OK;
}

#pragma GCC optimize ("O3")
/* @fn 		usart3_port_tx_msg()
 * @brief 	put message to circular report buffer
 * 			it will be transmitted in background ASAP from flushing Thread
 * @return	HAL_BUSY - usart not do it now, wait for release
 * 			HAL_ERROR- buffer overflow
 * 			HAL_OK   - scheduled for transmission
 * */
HAL_StatusTypeDef usart3_port_tx_msg(uint8_t *str, int16_t len)
{
	int head, tail, size;
	HAL_StatusTypeDef	ret;

	/* add TX msg to circular buffer and increase circular buffer length */

	__HAL_LOCK(&txhandle);	//"return HAL_BUSY;" if locked
	head = usart3_report_buf.head;
	tail = usart3_report_buf.tail;

	size = USARTBUFFLEN;

	if(CIRC_SPACE(head, tail, size) > (len))
	{
		while (len > 0)
		{
			usart3_report_buf.buf[head]= *(str++);
			head= (head+1) & (size - 1);
			len--;
		}

		usart3_report_buf.head = head;

		ret = HAL_OK;
	}
	else
	{
		/* if packet usart not fit, setup TX Buffer overflow ERROR and exit */
		ret = HAL_ERROR;
	}
	__HAL_UNLOCK(&txhandle);
    return ret;
}

#pragma GCC optimize ("O3")
/* @fn 		usart3_port_rx_msg()
 * @brief 	put message to circular local buffer
 * 			it will be transmitted in background ASAP from flushing Thread
 * @return	HAL_BUSY - usart not do it now, wait for release
 * 			HAL_ERROR- buffer overflow
 * 			HAL_OK   - scheduled for transmission
 * */
HAL_StatusTypeDef usart3_port_rx_msg(uint8_t *str, int16_t len)
{
	int head, tail, size;
	HAL_StatusTypeDef	ret;

	/* add TX msg to circular buffer and increase circular buffer length */

	__HAL_LOCK(&rxhandle);
	head = usart3_local_buf.head;
	tail = usart3_local_buf.tail;

	size = USARTBUFFLEN;

	if(CIRC_SPACE(head, tail, size) > (len))
	{
		while (len > 0)
		{
			usart3_local_buf.buf[head]= *(str++);
			head= (head+1) & (size - 1);
			len--;
		}

		usart3_local_buf.head = head;

		ret = HAL_OK;
	}
	else
	{
		/* if packet usart not fit, setup TX Buffer overflow ERROR and exit */
		ret = HAL_ERROR;
	}
	__HAL_UNLOCK(&rxhandle);
    return ret;
}

#pragma GCC optimize ("O3")
/**
  * @brief  DW_USART3_DataTx
  *         received data to be send over USART3 endpoint are managed in
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USARTD_OK if all operations are OK else USARTD_FAIL
  */
USARTD_StatusTypeDef DW_USART3_DataTx (uint8_t* Buf, uint16_t Len)
{
	if (!Link_Ok)
	{	//Empty Circular Buffer
		usart3_report_buf.head = 0;
		usart3_report_buf.tail = 0;
	}

	return usart3_port_tx_msg(Buf, Len);
}

#pragma GCC optimize ("O3")
/**
  * @brief  DW_USART3_DataRx
  *         Data received from USART3 device
  * @note   This is application-level function. it is necessary to protect
  * 		[Buf], [Len] from modification in the usart driver
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval USARTD_FAIL / USARTD_OK
  */
USARTD_StatusTypeDef DW_USART3_DataRx (uint8_t* Buf, uint16_t Len)
{
#if ((DATA_MODE==1) && (NORMAL_MODE==0))
  //If in Data Mode you plan to use some form of post-processing on the received
  //USART data (usart3_local_have_data = 1) then store the Rx data in a dedicated buffer
  //(usart3_local_buff).
  //This buffer will be used by the "process_usart3message(usart3_local_buff)" function
  //for subsequent processing
  if((usart3_local_buff_offset+Len) <= USARTBUFFLEN)
  {
	  memcpy(&usart3_local_buff[usart3_local_buff_offset], Buf, Len);
	  usart3_local_buff_length = Len + usart3_local_buff_offset;
	  usart3_local_buff_offset += Len;
	  usart3_local_have_data = 1;
  } else
  {
	  usart3_local_buff_length = USARTBUFFLEN;
	  usart3_local_buff_offset = 0;
  }
  if (!Link_Ok)
  {	  //Empty Circular Buffer
	  usart3_local_buf.head = 0;
	  usart3_local_buf.tail = 0;
	  usart3_local_buff_offset = 0;
	  usart3_local_buff_length = 0;
  }

  //If in Data Mode use circular buffer to store USART Rx Data.
  return usart3_port_rx_msg(Buf, Len);
#else
  if( (usart3_local_buff_offset+Len) <= sizeof(usart3_local_buff))
  {
	  memcpy(&usart3_local_buff[usart3_local_buff_offset], Buf, Len);
  }
  else
  {
	  usart3_local_buff_offset = 0;
	  usart3_local_buff_length = 0;
  	  return USARTD_FAIL;
  }

  usart3_local_buff_offset = 0;	//In USARTopen mode We receive 32bytes length packets over USART,
  usart3_local_buff_length = 0;	//sizeof(usart3_local_buff) = 32, so we usart zeroes the buffer data pointers
  usart3_local_have_data = 1;


  return USARTD_OK;
#endif
}

#pragma GCC optimize ("O3")
USARTD_StatusTypeDef process_usart3message(uint8_t* Buf)
{
	USARTD_StatusTypeDef result = USARTD_OK;

#if (DATA_MODE==1)		//If in Data Mode use circular buffer to store USART Rx Data.
	result = 0;			//Do nothing. No FSM in Data-Mode!!!
#else
	if (usart3app.usartlen == 0)		//To be sure that the message is only processed when the acquisition is complete
	{									//in DW_USART3_DataRx function
		result = HandleUSART3_MSG(Buf);
		if (result == (uint8_t)SerialProtocol_FAIL)		//Check for frame error
		{
			if (++FrameErrorsCnt > MAX_ALLOWED_FRAME_ERRORS)
			{									//Restart USART
				HAL_UART_ErrorCallback(&huart3);
			}
		} else
		if (result == (uint8_t)SerialProtocol_CRC_KO)	//Check for CRC error
		{
			HAL_UART_ErrorCallback(&huart3);
		} else
		{
			FrameErrorsCnt = 0;
		}
	}
#endif

	return result;
}

#pragma GCC optimize ("O3")
void send_usart3message(uint8_t *string, int16_t len)
{
	if(usart_ready(&huart3))
	{
		if(usart3_local_have_data == 0)
		{
			memcpy(&usart3_tx_buff[0], string, len);
	#if ((DATA_MODE==0) && (NORMAL_MODE==0))
			usart3_tx_buff[len] = '\r';
			usart3_tx_buff[len+1] = '\n';
			usart3_tx_buff_length = len + 2;
	#else
			usart3_tx_buff_length = len;
	#endif
	#if (RS485_MODE)	//Go to TX mode: activates the RS485 line driver by activating the RTS signal
			HAL_GPIO_WritePin(USART3_RTS_GPIO_Port, USART3_RTS_Pin, GPIO_PIN_RESET);
			usleep(5);
	#endif
			DW_USART3_DataTx(usart3_tx_buff, usart3_tx_buff_length);
			flush_usart3_report_buff();
	#if (RS485_MODE)	//Return to RX mode
			usleep(5);
			HAL_GPIO_WritePin(USART3_RTS_GPIO_Port, USART3_RTS_Pin, GPIO_PIN_SET);
	#endif
		}
	}
}

#pragma GCC optimize ("O3")
void usart3_run(void)
{
	uint16_t prev_usart3len = 0;

	if(usart3app.usartlen)
	{
	/* usart3app.usartlen is wrote in process_USART_RX_irq function in "port.c"
	 * and contain the number of frames received.
	 */
		prev_usart3len = usart3app.usartlen;
		if (DW_USART3_DataRx(usart3app.usartbuf, usart3app.usartlen) == USARTD_OK)
		{
		/*
		 * We must take into account that during the queuing of the message in the circular buffer,
		 * since the reception interrupts remain enabled, other messages can be received.
		 * So the length of the reception buffer must be reinitialized with the difference
		 * of the value it assumes at the output of the queuing function with the one it had at the input
		 * (passed as "Len" parameter)
		 */
			usart3app.usartlen = usart3app.usartlen - prev_usart3len;
		}
#if ((DATA_MODE==0) && (NORMAL_MODE==1))
		else
			usart3app.usartlen = 0;
#endif
		usart3app.usartrx_time = HAL_GetTick();	//Update the Rx timestamp
	}

	if(usart3_local_have_data == 1)
	{
     	usart3_status = process_usart3message(usart3_local_buff);
    	usart3_local_have_data = 0;
	}
}
