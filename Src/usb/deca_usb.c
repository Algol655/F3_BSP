/*! ----------------------------------------------------------------------------
 * @file	deca_usb.c
 * @brief	Middleware for USB communications
 *
 * @author	Tommaso Sabatini, 2018
 */

/* Includes */
#include "usb/deca_usb.h"
#include "application/MEMS_app.h"
#include "application/SerialProtocol.h"
#include "usbd_def.h"
#define REPORT_BUFSIZE	0x2000
//VCP rx data buffers
uint16_t local_buff_length = 0;
uint8_t  local_buff[BUFFLEN];	/**< non circular local buffer, data received from VCP. */
static 	uint8_t lbuf[BUFFLEN];	/**< circular local buffer, data to be transmitted in flush_report_buff() Thread. */

static circBuf_t local_buf = {.buf = lbuf,
							  .head= 0,
							  .tail= 0,
							  .maxLen=MSG_PAYLOAD_LEN_MAX-3};
//VCP tx data buffers
int 	tx_buff_length = 0;
uint8_t tx_buff[BUFFLEN];
static int 	local_have_data = 0;
static uint8_t 	rbuf[REPORT_BUFSIZE];	/**< circular report buffer, data to be transmitted in flush_report_buff() Thread. Modified By Me!!! */

static circBuf_t report_buf = {.buf = rbuf,
							   .head= 0,
							   .tail= 0,
							   .maxLen=REPORT_BUFSIZE};

static uint8_t 	ubuf[CDC_DATA_FS_MAX_PACKET_SIZE];	/**< used to transmit new chunk of data in single USB flush */

//__STATIC_INLINE uint16_t DW_VCP_DataTx (uint8_t* Buf, uint32_t Len);

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
//extern HAL_StatusTypeDef port_tx_msg(uint8_t* str, uint16_t len);

/**
  * @brief  DW_VCP_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation (USBD_OK in all cases)
  */
uint16_t DW_VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{
   return USBD_OK;
}

/* @fn		flush_report_buff
 * @brief 	FLUSH should have higher priority than port_tx_msg()
 * 			it shall be called periodically from process, which can not be locked,
 * 			i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_report_buff(void)
{
	extern USBD_HandleTypeDef  hUsbDeviceFS;
	USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData);
	int i, head, tail, len, size = REPORT_BUFSIZE;
	uint32_t j = 0;

	__HAL_LOCK(&tx1handle);					//"return HAL_BUSY;" if locked
	head = report_buf.head;
	tail = report_buf.tail;
//	__HAL_UNLOCK(&tx1handle);
	len = CIRC_CNT(head, tail, size);

#if ((DATA_MODE==0) && (NORMAL_MODE==0))
	if( len > 0 )
#else
	while( len > 0 )
#endif
	{
		/*	check USB status - ready to TX */
		while ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) || (hcdc->TxState != 0))
		{									/**< Wait until USB is free.*/
			j++;
			if(j>0xffffe)
			{
				__HAL_UNLOCK(&tx1handle);
				return HAL_TIMEOUT;
			}
		}
		/* copy MAX allowed length from circular buffer to non-circular TX buffer */
		len = MIN(CDC_DATA_FS_MAX_PACKET_SIZE, len);
		for(i=0; i<len; i++)
		{
			ubuf[i] = report_buf.buf[tail];
			tail = (tail + 1) & (size - 1);
		}
		/* setup USB IT transfer */
		if(CDC_Transmit_FS(ubuf, (uint16_t)len) != USBD_OK)
		{
			/**< indicate USB transmit error */
		}
		len = CIRC_CNT(head, tail, size);
	}
//	__HAL_LOCK(&tx1handle);		//"return HAL_BUSY;" if locked
	report_buf.tail = tail;

	__HAL_UNLOCK(&tx1handle);
	return HAL_OK;
}

/* @fn		flush_local_buff
 * @brief 	FLUSH should have high priority
 * 			it shall be called periodically from process, which can not be locked,
 * 			i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_local_buff(void)		//Added & Modified By Me!!
{
#if ((DATA_MODE==1) && (NORMAL_MODE==0))
	int i, head, tail, len, size = BUFFLEN;

	__HAL_LOCK(&rx1handle);
	head = local_buf.head;
	tail = local_buf.tail;
	len = CIRC_CNT(head, tail, size);

	if( len > 0 )
	{
		/* copy MAX allowed length from circular buffer to circular TX buffer */
		len = MIN(local_buf.maxLen, len);

		for(i=0; i<len; i++)
		{
			msg_payload[i] = local_buf.buf[tail];
			tail = (tail + 1) & (size - 1);
		}

		local_buf.tail = tail;
	}
#else
	/* copy MAX allowed length from circular buffer to non-circular TX buffer */
	memcpy(&msg_payload[0], local_buff, local_buff_length);
	memset(local_buff, 0x0, local_buff_length);
#endif
	memcpy(&msg_payload[sizeof(msg_payload)-2], (uint8_t*) &(local_buff_length), sizeof(local_buff_length));//transmit msg length
	msg_payload[sizeof(msg_payload)-3] = d_zero;	//transmit the bit-rate setting or ack info
	local_buff_offset = 0;
	local_buff_length = 0;

#if ((DATA_MODE==1) && (NORMAL_MODE==0))
	__HAL_UNLOCK(&rx1handle);
#endif
	return HAL_OK;
}

/* @fn 		port_tx_msg()
 * @brief 	put message to circular report buffer
 * 			it will be transmitted in background ASAP from flushing Thread
 * @return	HAL_BUSY - can not do it now, wait for release
 * 			HAL_ERROR- buffer overflow
 * 			HAL_OK   - scheduled for transmission
 * */
HAL_StatusTypeDef port_tx_msg(uint8_t *str, int len)
{
	int head, tail, size;
	HAL_StatusTypeDef	ret;

	/* add TX msg to circular buffer and increase circular buffer length */

	__HAL_LOCK(&txhandle);	//return HAL_BUSY if locked
	head = report_buf.head;
	tail = report_buf.tail;
//	__HAL_UNLOCK(&txhandle);			//Modified By Me!!!
	size = REPORT_BUFSIZE;

	if(CIRC_SPACE(head, tail, size) > (len))
	{
		while (len > 0)
		{
			report_buf.buf[head]= *(str++);
			head= (head+1) & (size - 1);
			len--;
		}
//		__HAL_LOCK(&txhandle);	//return HAL_BUSY if locked //Modified By Me!!!
		report_buf.head = head;
//		__HAL_UNLOCK(&txhandle);		//Modified By Me!!!
		ret = HAL_OK;
	}
	else
	{
		/* if packet can not fit, setup TX Buffer overflow ERROR and exit */
		ret = HAL_ERROR;
	}
	__HAL_UNLOCK(&txhandle);			//Modified By Me!!!
    return ret;
}

/* @fn 		can1_port_rx_msg()
 * @brief 	put message to circular local buffer
 * 			it will be transmitted in background ASAP from flushing Thread
 * @return	HAL_BUSY - can not do it now, wait for release
 * 			HAL_ERROR- buffer overflow
 * 			HAL_OK   - scheduled for transmission
 * */
HAL_StatusTypeDef port_rx_msg(uint8_t *str, int16_t len)
{
	int head, tail, size;
	HAL_StatusTypeDef	ret;

	/* add TX msg to circular buffer and increase circular buffer length */

	__HAL_LOCK(&rxhandle);
	head = local_buf.head;
	tail = local_buf.tail;

	size = BUFFLEN;

	if(CIRC_SPACE(head, tail, size) > (len))
	{
		while (len > 0)
		{
			local_buf.buf[head]= *(str++);
			head= (head+1) & (size - 1);
			len--;
		}

		local_buf.head = head;

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
  * @brief  DW_VCP_DataTx
  *         CDC received data to be send over USB IN endpoint are managed in
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else VCP_FAIL
  */
USBD_StatusTypeDef DW_VCP_DataTx (uint8_t* Buf, uint32_t Len)
{
	if (!Link_Ok)
	{	//Empty Circular Buffer
		report_buf.head = 0;
		report_buf.tail = 0;
	}

	return port_tx_msg(Buf, Len);
}

/**
  * @brief  DW_VCP_DataRx
  * @brief	Data received from host device;
  * 		This function formats the message received in order to make it
  * 		compatible with the header required by the serial protocol used.
  * @note   This is application-level function. it is necessary to protect
  * 		[Buf], [Len] from modification in the usb driver
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval USBD_FAIL / USBD_OK
  * 		the function modifies the signal "local_have_data"
  *         which indicates future processing is necessary
  */
USBD_StatusTypeDef DW_VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
  #if ((DATA_MODE==1) && (NORMAL_MODE==0))
  //If in Data Mode you plan to use some form of post-processing on the received
  //VCP data (local_have_data = 1) then store the Rx data in a dedicated buffer
  //(local_buff).
  //This buffer will be used by the "process_usbmessage(local_buff)" function
  //for subsequent processing
  if((local_buff_offset+Len) <= BUFFLEN)
  {
	  memcpy(&local_buff[local_buff_offset], Buf, Len);
	  local_buff_length = Len + local_buff_offset;
	  local_buff_offset += Len;
	  local_have_data = 1;
  } else
  {
	  local_buff_length = BUFFLEN;
	  local_buff_offset = 0;
	  app.usbcts = false;
	  app.disabled = true;
  }
  if (!Link_Ok)
  {	  //Empty Circular Buffer
	  local_buf.head = 0;
	  local_buf.tail = 0;
	  local_buff_offset = 0;
	  local_buff_length = 0;
  }
  //If in Data Mode use circular buffer to store CDC Rx Data.
  return port_rx_msg(Buf, Len);
#else
  if((local_buff_offset+Len) <= BUFFLEN)
  {
	  memcpy(&local_buff[local_buff_offset], Buf, Len);
  }
  else
  {
	  local_buff_offset = 0;
	  local_buff_length = 0;
  	  return USBD_FAIL;
  }

  //Check if a message from GUI App is available
  if((local_buff[local_buff_offset] == (uint8_t)DEV_ADDR) && (local_buff[local_buff_offset+1] == DataStreamingDest))
  {								//Check if source & destination addresses are correct
	  local_buff_offset = 0;
	  local_buff_length = 0;
	  local_have_data = 1;
  } else	//Check if the command to enable the test environment is present (Cntrl-P)
  if (app.usblen == 1)
  {
//	  app.usblen = 0;
	  if (local_buff[local_buff_offset] == 0x10)
	  {
		  local_buff[0] = (uint8_t)DEV_ADDR;	//Put in Buff[0] DEV_ADDR = 50U
		  local_buff[2] = 0x10;					//Put ^P in the command field Buff[2]

		  local_buff_offset = 0;
		  local_buff_length = 0;
		  local_have_data = 1;
	  } else	//Check if the restart command is present (Cntrl-R)
	  if (local_buff[local_buff_offset] == 0x12)
	  {
		  local_buff[0] = (uint8_t)DEV_ADDR;	//Put in Buff[0] DEV_ADDR = 50U
		  local_buff[2] = 0x12;					//Put ^R in the command field Buff[2]

		  local_buff_offset = 0;
		  local_buff_length = 0;
		  local_have_data = 1;
	  }
  }
  else if (!Test_Mode) 			//If no useful message has been received, it moves the pointer to the first free location
  {								//of the receive buffer, that is to the first byte of the next message.
	  local_have_data = 0;
	  local_buff_length = Len + local_buff_offset;
	  local_buff_offset += Len;
  } else
  {
	  local_buff_offset = 0;
	  local_buff_length = 0;
	  local_have_data = 1;
  }
#endif

  return USBD_OK;
}

USBD_StatusTypeDef process_usbmessage(uint8_t* Buf)
{
	USBD_StatusTypeDef result = USBD_OK;

#if (DATA_MODE==1)		//If in Data Mode use circular buffer to store CDC Rx Data.
	result = 0;			//Do nothing. No FSM in Data-Mode!!!
#elif (GUI_SUPPORT==1)
	result = HandleUSB_MSG(&local_buff[0]);
#elif (NORMAL_MODE==1)
	result = HandleUSB_MSG(&local_buff[0]);
	memset(local_buff, 0x0, BUFFLEN);
	local_have_data = 0;
#endif
	return result;
}

void send_usbmessage(uint8_t *string, int len)
{
    if(usb_ready())
    {
    	if(local_have_data == 0)
    	{
    		memcpy(&tx_buff[0], string, len);
    #if ((DATA_MODE==0) && (GUI_SUPPORT==0) && (BLE_SUPPORT==0))
    		tx_buff[len] = '\r';
    		tx_buff[len+1] = '\n';
    		tx_buff_length = len + 2;
    #else
    		tx_buff_length = len;
    #endif
    		DW_VCP_DataTx(tx_buff, tx_buff_length);
	#if (GUI_SUPPORT==1)
			Message_Length = 0;
	#endif
    		flush_report_buff();
    	}
    }
}

/**
**===========================================================================
**  Abstract: program
**===========================================================================
*/
int usb_init(void)
{
	memset(local_buff, 0, sizeof(local_buff));
	app.usblen = 0;
    return 0;
}

void usb_run(void)
{
	uint16_t prev_usblen = 0;

	if(app.usblen)
	{
		/* app.usblen is wrote in "process_CDC_Receive_FS" function in "usbd_cdc_if.c"
		 * and contain the number of frames received.
		 */
		prev_usblen = app.usblen;
		if (DW_VCP_DataRx(app.usbbuf, app.usblen) == USBD_OK)
		{
		/*
		 * We must take into account that during the queuing of the message in the circular buffer,
		 * since the reception interrupts remain enabled, other messages can be received.
		 * So the length of the reception buffer must be reinitialized with the difference
		 * of the value it assumes at the output of the queuing function with the one it had at the input
		 * (passed as "Len" parameter)
		 */
			app.usblen = app.usblen - prev_usblen;
		}
#if ((DATA_MODE==0) && (NORMAL_MODE==1))
		else
			app.usblen = 0;
#endif
		app.usbrx_time = HAL_GetTick();	//Update the Rx timestamp
	}

	if(local_have_data == 1)
    {
       	usbd_status = process_usbmessage(local_buff);
		local_have_data = 0;
    }
}
