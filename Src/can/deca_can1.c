/*! ----------------------------------------------------------------------------
 * @file	deca_can1.c
 * @brief	Middleware for CAN communications
 *
 * @author	Tommaso Sabatini, 2018
 */

/* Includes */
#include "can/deca_can1.h"

//CAN rx data buffers
int16_t can1_local_buff_length = 0;
uint8_t can1_local_buff[CANBUFFLEN];	/**< non circular local buffer, data received from CAN. */
static 	uint8_t can1_lbuf[CANBUFFLEN];	/**< circular local buffer, data to be transmitted in can1_flush_report_buff() Thread. */

static circBuf_t can1_local_buf = {.buf = can1_lbuf,
							   	   .head= 0,
								   .tail= 0,
								   .maxLen=MSG_PAYLOAD_LEN_MAX-3};
//CAN tx data buffers
int 	can1_tx_buff_length = 0;
uint8_t can1_tx_buff[CANBUFFLEN];
static int 	can1_local_have_data = 0;
static 	uint8_t can1_rbuf[CANBUFFLEN];	/**< circular report buffer, data to be transmitted in can1_flush_report_buff() Thread. */

static circBuf_t can1_report_buf = {.buf = can1_rbuf,
							   	   	.head= 0,
									.tail= 0,
									.maxLen=CANBUFFLEN};

static uint8_t 	can1_ubuf[8];	/**< used to transmit new chunk of data in single CAN1 flush */

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
//extern HAL_StatusTypeDef can1_port_tx_msg(uint8_t* str, uint16_t len);

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
#include <stddef.h>

void * mymemcpy (void *dest, const void *src, const void *isrc, const void *esrc, size_t len)
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
}

/* @fn		can1_flush_report_buff
 * @brief 	FLUSH should have higher priority than can1_port_tx_msg()
 * 			it shall be called periodically from process, which can not be locked,
 * 			i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_can1_report_buff(void)
{
	int i, head, tail, len, size = CANBUFFLEN;
	uint32_t	TxMailbox1, j = 0;

	__HAL_LOCK(&tx1handle);
	head = can1_report_buf.head;
	tail = can1_report_buf.tail;
	len = CIRC_CNT(head, tail, size);

//	if (len==0) len=1;

#if ((DATA_MODE==0) && (NORMAL_MODE==0))
	if( len > 0 )
#else
	while( len > 0 )
#endif
	{
		/* copy MAX allowed length from circular buffer to non-circular TX buffer */
#if ((DATA_MODE==1) && (NORMAL_MODE==0))
		CAN_TxHeaderTypeDef	Header1;
		len = MIN(CANBUFFLEN, len);
//		mymemcpy((void *)&Header1, (void *)&can1_report_buf.buf[tail], (void *)&can1_report_buf.buf[0], (void *)&can1_report_buf.buf[CANBUFFLEN-1], sizeof(TxHeader1));	// Extract TxHeader from received buffer
		memcpy((void *)&Header1, (void *)&can1_report_buf.buf[tail], sizeof(TxHeader1));	// Extract TxHeader from received buffer
		tail = (tail + sizeof(TxHeader1));
#endif
		for(i=0; i<8; i++)
		{
			can1_ubuf[i] = can1_report_buf.buf[tail];
			tail = (tail + 1) & (size - 1);
		}
#if ((DATA_MODE==1) && (NORMAL_MODE==0))
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
		{
			j++;
			if(j>0xfffe)
			{
				HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0);
				HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX1);
				HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX2);
				__HAL_UNLOCK(&tx1handle);
				return HAL_TIMEOUT;
			}
		}
		j = 0;
		if (HAL_CAN_AddTxMessage(&hcan1, &Header1, can1_ubuf, &TxMailbox1) != HAL_OK)
		{
			/**< indicate CAN1 transmit error */
		}
#endif
#if ((DATA_MODE==0) && (NORMAL_MODE==1))
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, can1_ubuf, &TxMailbox1) != HAL_OK)
		{
			/**< indicate CAN1 transmit error */
		}
#endif
		/*	check CAN status - ready to TX */
		while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox1))
		{
			j++;
			if(j>0xffffe)
			{
				__HAL_UNLOCK(&tx1handle);
				return HAL_TIMEOUT;
			}
		}
		len = CIRC_CNT(head, tail, size);
	}
	can1_report_buf.tail = tail;

	__HAL_UNLOCK(&tx1handle);
	return HAL_OK;
}

/* @fn		flush_can1_local_buff
 * @brief 	FLUSH should have high priority
 * 			it shall be called periodically from process, which can not be locked,
 * 			i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_can1_local_buff(void)
{
#if ((DATA_MODE==1) && (NORMAL_MODE==0))
	int i, head, tail, len, size = CANBUFFLEN;

	__HAL_LOCK(&rx1handle);
	head = can1_local_buf.head;
	tail = can1_local_buf.tail;
	len = CIRC_CNT(head, tail, size);

	if( len > 0 )
	{
		/* copy MAX allowed length from circular buffer to circular TX buffer */
		len = MIN(can1_local_buf.maxLen, len);

		for(i=0; i<len; i++)
		{
			msg_payload[i] = can1_local_buf.buf[tail];
			tail = (tail + 1) & (size - 1);
		}

		can1_local_buf.tail = tail;
	}
#else
	/* copy MAX allowed length from circular buffer to non-circular TX buffer */
	memcpy(&msg_payload[0], can1_local_buff, can1_local_buff_length);
	memset(can1_local_buff, 0x0, can1_local_buff_length);
#endif
	memcpy(&msg_payload[sizeof(msg_payload)-2], (uint8_t*) &(can1_local_buff_length), sizeof(can1_local_buff_length));	//transmit msg length
	msg_payload[sizeof(msg_payload)-3] = d_zero;	//transmit the can bit-rate setting or ack info
	can1_local_buff_offset = 0;
	can1_local_buff_length = 0;

#if ((DATA_MODE==1) && (NORMAL_MODE==0))
	__HAL_UNLOCK(&rx1handle);
#endif
	return HAL_OK;
}

/* @fn 		can1_port_tx_msg()
 * @brief 	put message to circular report buffer
 * 			it will be transmitted in background ASAP from flushing Thread
 * @return	HAL_BUSY - can not do it now, wait for release
 * 			HAL_ERROR- buffer overflow
 * 			HAL_OK   - scheduled for transmission
 * */
HAL_StatusTypeDef can1_port_tx_msg(uint8_t *str, int16_t len)
{
	int head, tail, size;
	HAL_StatusTypeDef	ret;

	/* add TX msg to circular buffer and increase circular buffer length */

	__HAL_LOCK(&txhandle);	//"return HAL_BUSY;" if locked
	head = can1_report_buf.head;
	tail = can1_report_buf.tail;

	size = CANBUFFLEN;

	if(CIRC_SPACE(head, tail, size) > (len))
	{
		while (len > 0)
		{
			can1_report_buf.buf[head]= *(str++);
			head= (head+1) & (size - 1);
			len--;
		}

		can1_report_buf.head = head;

		ret = HAL_OK;
	}
	else
	{
		/* if packet can not fit, setup TX Buffer overflow ERROR and exit */
		ret = HAL_ERROR;
	}
	__HAL_UNLOCK(&txhandle);
    return ret;
}

/* @fn 		can1_port_rx_msg()
 * @brief 	put message to circular local buffer
 * 			it will be transmitted in background ASAP from flushing Thread
 * @return	HAL_BUSY - can not do it now, wait for release
 * 			HAL_ERROR- buffer overflow
 * 			HAL_OK   - scheduled for transmission
 * */
HAL_StatusTypeDef can1_port_rx_msg(uint8_t *str, int16_t len)
{
	int head, tail, size;
	HAL_StatusTypeDef	ret;

	/* add TX msg to circular buffer and increase circular buffer length */

	__HAL_LOCK(&rxhandle);
	head = can1_local_buf.head;
	tail = can1_local_buf.tail;

	size = CANBUFFLEN;

	if(CIRC_SPACE(head, tail, size) > (len))
	{
		while (len > 0)
		{
			can1_local_buf.buf[head]= *(str++);
			head= (head+1) & (size - 1);
			len--;
		}

		can1_local_buf.head = head;

		ret = HAL_OK;
	}
	else
	{
		/* if packet can not fit, setup TX Buffer overflow ERROR and exit */
		ret = HAL_ERROR;
	}
	__HAL_UNLOCK(&rxhandle);
    return ret;
}

/**
  * @brief  DW_CAN1_DataTx
  *         received data to be send over CAN1 endpoint are managed in
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: CAND_OK if all operations are OK else CAND_FAIL
  */
CAND_StatusTypeDef DW_CAN1_DataTx (uint8_t* Buf, uint16_t Len)
{
	if (!Link_Ok)
	{	//Empty Circular Buffer
		can1_report_buf.head = 0;
		can1_report_buf.tail = 0;
	}

	return can1_port_tx_msg(Buf, Len);
}

/**
  * @brief  DW_CAN1_DataRx
  *         Data received from CAN1 device
  * 		This function formats the message received in order to make it
  * 		compatible with the header required by the serial protocol used.
  * @note   This is application-level function. it is necessary to protect
  * 		[Buf], [Len] from modification in the CAN driver
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval CAND_FAIL / CAND_OK
  * 		the function modifies the signal "local_have_data"
  *         which indicates future processing is necessary
  */
CAND_StatusTypeDef DW_CAN1_DataRx (uint8_t* Buf, uint16_t Len)
{
#if ((DATA_MODE==1) && (NORMAL_MODE==0))
  //If in Data Mode you plan to use some form of post-processing on the received
  //CAN data (can1_local_have_data = 1) then store the Rx data in a dedicated buffer
  //(can1_local_buff). 
  //This buffer will be used by the "process_can1message(can1_local_buff)" function
  //for subsequent processing
  if((can1_local_buff_offset+Len) <= CANBUFFLEN)
  {
	  memcpy(&can1_local_buff[can1_local_buff_offset], Buf, Len);
	  can1_local_buff_length = Len + can1_local_buff_offset;
	  can1_local_buff_offset += Len;
	  can1_local_have_data = 1;
  } else
  {
	  can1_local_buff_length = CANBUFFLEN;
	  can1_local_buff_offset = 0;
  }
  if (!Link_Ok)
  {	  //Empty Circular Buffer
	  can1_local_buf.head = 0;
	  can1_local_buf.tail = 0;
	  can1_local_buff_offset = 0;
	  can1_local_buff_length = 0;
  }

  //If in Data Mode use circular buffer to store CDC Rx Data.
  return can1_port_rx_msg(Buf, Len);
#else
  if( (can1_local_buff_offset+Len) <= sizeof(can1_local_buff))
  {
	  memcpy(&can1_local_buff[can1_local_buff_offset], Buf, Len);
  }
  else
  {
	  can1_local_buff_offset = 0;
	  can1_local_buff_length = 0;
  	  return CAND_FAIL;
  }

  can1_local_buff_offset = 0;	//In CANopen mode We receive 32bytes length packets over CAN,
  can1_local_buff_length = 0;	//sizeof(can1_local_buff) = 32, so we can zeroes the buffer data pointers
  can1_local_have_data = 1;

  return CAND_OK;
#endif
}

CAND_StatusTypeDef process_can1message(uint8_t* Buf)
{
	CAND_StatusTypeDef result = CAND_OK;

#if (DATA_MODE==1)		//If in Data Mode use circular buffer to store CAN Rx Data.
	result = 0;			//Do nothing. No FSM in Data-Mode!!!
#endif
#if (STM_PROGRAMMER_MODE==1)			//see STM doc AN3154
	if (can1app.canlen == 0)
	{
		/**
		 * The local node, connected with the STLINK-V2 programmer, sends the request to the remote node
		 * to change the CANBus bitrate and waits for the ACK message. When the ACK message is received
		 * the local node changes its bitrate. This is done in the TxData() function of tx_rx_data.c
		 * (see STM doc AN3154)
		 */
		if ((Buf[0] == 0x03) && (Buf[1] == 0x00))	//PDO with CobID 0x03 received
		{
			d_zero = Buf[24];			//in d_zero the D0 bit of the CAN message received: the local node
			if (!canlocal_br_set)		//will send it to the remote node in the function flush_can1_local_buff()
			{
				if ((Buf[24] == 0x01) || (Buf[24] == 0x02) || (Buf[24] == 0x03))
				{	//Bitrate setting received. Accept only 125, 250, 500 Kb/s bit rates (see STM doc AN3154)
					canlocal_br = canremote_br = Buf[24];	//in canlocal_br the received bit-rate
					canlocal_br_res = true;	//Reserve the local node's CANBus bitrate setting.
				}							//It will be set in the TxData() function of tx_rx_data.c
			}
			/**
			 * If the remote node has correctly received the request to change the bitrate from the local node,
			 * it sends the ACK message and immediately changes its bitrate (see STM doc AN3154)
			 */
			if (!canremote_br_set)
			{
				if (Buf[24] == 0x79)	//Accept only ACK
				{
					CAN_SetBR(&hcan1, canremote_br);
					canremote_br_set = true;
				}
			}
		} else
			d_zero = 0;
	}
#endif

	return result;
}

void send_can1message(uint8_t *string, int16_t len)
{
	memcpy(&can1_tx_buff[0], string, len);
#if ((DATA_MODE==0) && (NORMAL_MODE==0))
	can1_tx_buff[len] = '\r';
	can1_tx_buff[len+1] = '\n';
	can1_tx_buff_length = len + 2;
#else
	can1_tx_buff_length = len;
#endif
	DW_CAN1_DataTx(can1_tx_buff, can1_tx_buff_length);
	flush_can1_report_buff();
}

void can1_run(void)
{
	uint16_t prev_can1len = 0;

	if(can1app.canlen)
	{
	/* can1app.canlen is wrote in process_CAN_RX_irq function in "port.c"
	 * and contain the number of frames received.
	 */
		prev_can1len = can1app.canlen;
		if (DW_CAN1_DataRx(can1app.canbuf, can1app.canlen) == CAND_OK)
		{
		/*
		 * We must take into account that during the queuing of the message in the circular buffer,
		 * since the reception interrupts remain enabled, other messages can be received.
		 * So the length of the reception buffer must be reinitialized with the difference
		 * of the value it assumes at the output of the queuing function with the one it had at the input
		 * (passed as "Len" parameter)
		 */
			can1app.canlen = can1app.canlen - prev_can1len;
		}
#if ((DATA_MODE==0) && (NORMAL_MODE==1))
		else
			can1app.canlen = 0;
#endif
	}

	if(can1_local_have_data == 1)
	{
     	can1_status = process_can1message(can1_local_buff);
    	can1_local_have_data = 0;
	}
}
