/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *          All the Devices Callbacks & ISR are here!
 *
 * @author  Tommaso Sabatini, 2018
 */

#include "platform/port.h"
#if (IO_EXP_PRESENT==1)
	#include "platform/mcp23017.h"
#endif
#if (PRESSURE_SENSOR_PRESENT==1)
#if (LPS25HB)
	#include "platform/LPS25HB_Driver.h"
	extern LPS25HB_MeasureTypeDef_st PRS_Values;
#elif (LPS22HB)
	#include "platform/LPS22HB_Driver.h"
	extern LPS22HB_MeasureTypeDef_st PRS_Values;
#endif	// LPS25HB
//	extern int64_t MEMS_LclData;
#endif	// PRESSURE_SENSOR_PRESENT
#if (HUMIDITY_SENSOR_PRESENT==1)
#if (HTS221)
	#include "platform/HTS221_Driver.h"
	extern HTS221_MeasureTypeDef_st HUM_Values;
#elif (SHT4x)
	#include "platform/SHT4x_Driver.h"
	extern SHT4x_MeasureTypeDef_st HUM_Values;
#endif	// HTS221
#endif	// HUMIDITY_SENSOR_PRESENT
#if (UVx_SENSOR_PRESENT==1)
#if (VEML6075)
	#include "platform/VEML6075_Driver.h"
	extern VEML6075_MeasureTypeDef_st UVx_Values;
#elif (LTR390UV)
	#include "platform/LTR390UV_Driver.h"
	extern LTR390UV_MeasureTypeDef_st UVx_Values;
#endif	// VEML6075
#endif	// UVx_SENSOR_PRESENT
#if (ALS_SENSOR_PRESENT==1)
	#include "platform/VEML7700_Driver.h"
	extern VEML7700_MeasureTypeDef_st ALS_Values;
#endif
#if (VOC_SENSOR_PRESENT==1)
#if (CCS811)
	#include "platform/CCS811_Driver.h"
	extern CCS811_MeasureTypeDef_st VOC_Values;
#elif (ENS160)
	#include "platform/ENS160_Driver.h"
	extern ENS160_MeasureTypeDef_st VOC_Values;
#endif	// CCS811
#endif	// VOC_SENSOR_PRESENT
#if (PARTICULATE_SENSOR_PRESENT==1)
	#include "platform/SPS30_Driver.h"
	extern SPS30_MeasureTypeDef_st PMS_Values;
#endif
#if (GAS_SENSOR_MODULE_PRESENT==1)
	#include "platform/ANLG_Driver.h"
	extern ANLG_MeasureTypeDef_st GAS_Values;
#endif
#if (IMU_PRESENT==1)
	#include "platform/LSM9DS1_Driver.h"
	extern LSM9DS1_MeasureTypeDef_st IMU_Values;
#endif

const uint8_t UART_delay = 20;
const uint8_t CDC_delay = 10;
const uint8_t debounce_delay = 20;

uint16_t LedPc6Timer_TimeOut = 300;
uint16_t ServiceTimer0_TimeOut = 1000;	//Used for CanOPEN timings, in milliseconds
uint16_t ServiceTimer1_TimeOut = 300;	//Used for the debounce timer
uint32_t io_exp_intcap, io_exp_gpio;
bool led_pc6_timer_expired = false;
bool timer1_expired = false;
bool timer3_expired = false;
bool rx_done = false;
bool StartDataStrmng = false;

/****************************************************************************//**
 * 							System Utilities Section
 *******************************************************************************/
/**
  * @brief  Use the FLASH last sector as EEPROM. Word Data are written
  * @brief  starting from the initial address of the last flash sector.
  * @brief  For STM32F105x, STM32F105x, MCU's last sector is 127, starting at
  * @brief  0x0803F800.
  * @param  data: The word to store. f_offset: offset from starting address.
  * @retval None
  */
void Write_Flash(uint32_t data, uint8_t f_offset)
{
#include "stm32_hal_legacy.h"
	FLASH_EraseInitTypeDef EraseInitStruct;
	static DateTime_t Stamp;
	extern FLASH_DATA_ORG FlashDataOrg;
	extern uint8_t DeviceName[4], HW_Version[4], SW_Version[4];
	extern uint32_t Vendor_ID, Prdct_Code, Rev_Number, Ser_Number;
	uint32_t SectorError;
//	const uint32_t FlashAddress = 0x0803F800;	//See processor reference manual

	//Get RTC Current Date & Time and copy them in the flash time data structure
	RTC_DateTimeStamp(&hrtc, &Stamp);
	memcpy(&FlashDataOrg.b_date, &Stamp.date[0], 3);
	memcpy(&FlashDataOrg.b_time, &Stamp.time[0], 3);

	//Get board data and copy them in the flash board data structure
	memcpy(&FlashDataOrg.b_mdata.DeviceName, &DeviceName[0], 4);
	memcpy(&FlashDataOrg.b_mdata.HW_Version, &HW_Version[0], 4);
	memcpy(&FlashDataOrg.b_mdata.SW_Version, &SW_Version[0], 4);
	FlashDataOrg.b_mdata.Vendor_ID = Vendor_ID;
	FlashDataOrg.b_mdata.Prdct_Code = Prdct_Code;
	FlashDataOrg.b_mdata.Rev_Number = Rev_Number;
	FlashDataOrg.b_mdata.Ser_Number = Ser_Number;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
//	FLASH_Erase_Sector(FLASH_SECTOR_127, VOLTAGE_RANGE_3);
	EraseInitStruct.TypeErase = TYPEERASE_PAGEERASE;
	EraseInitStruct.PageAddress = (uint32_t)(DATA_EEPROM_BASE);
	EraseInitStruct.NbPages = 1U;
	EraseInitStruct.Banks = FLASH_BANK_1;
	HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
//	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + f_offset, data);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_date_offset, FlashDataOrg.b_date);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_time_offset, FlashDataOrg.b_time);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_mdata.DeviceName_offset,
					  FlashDataOrg.b_mdata.DeviceName);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_mdata.HW_Version_offset,
					  FlashDataOrg.b_mdata.HW_Version);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_mdata.SW_Version_offset,
					  FlashDataOrg.b_mdata.SW_Version);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_mdata.Vendor_ID_offset,
					  FlashDataOrg.b_mdata.Vendor_ID);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_mdata.Prdct_Code_offset,
					  FlashDataOrg.b_mdata.Prdct_Code);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_mdata.Rev_Number_offset,
					  FlashDataOrg.b_mdata.Rev_Number);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_mdata.Ser_Number_offset,
					  FlashDataOrg.b_mdata.Ser_Number);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s0_offset, FlashDataOrg.b_status.s0);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s1_offset, FlashDataOrg.b_status.s1);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s2_offset, FlashDataOrg.b_status.s2);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s3_offset, FlashDataOrg.b_status.s3);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s4_offset, FlashDataOrg.b_status.s4);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s5_offset, FlashDataOrg.b_status.s5);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s6_offset, FlashDataOrg.b_status.s6);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s7_offset, FlashDataOrg.b_status.s7);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s8_offset, FlashDataOrg.b_status.s8);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s9_offset, FlashDataOrg.b_status.s9);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.sa_offset, FlashDataOrg.b_status.sa);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.sb_offset, FlashDataOrg.b_status.sb);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.sc_offset, FlashDataOrg.b_status.sc);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.sd_offset, FlashDataOrg.b_status.sd);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.se_offset, FlashDataOrg.b_status.se);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.sf_offset, FlashDataOrg.b_status.sf);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s10_offset, FlashDataOrg.b_status.s10);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s11_offset, FlashDataOrg.b_status.s11);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s12_offset, FlashDataOrg.b_status.s12);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s13_offset, FlashDataOrg.b_status.s13);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s14_offset, FlashDataOrg.b_status.s14);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s15_offset, FlashDataOrg.b_status.s15);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s16_offset, FlashDataOrg.b_status.s16);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, DATA_EEPROM_BASE + FlashDataOrg.b_status.s17_offset, FlashDataOrg.b_status.s17);
	HAL_FLASH_Lock();
}

/**
  * @brief  Use the FLASH last sector as EEPROM. Word Data are read
  * @brief  starting from the initial address of the last flash sector.
  * @brief  For STM32F40x, STM32F41x MCU's last sector is 11, starting at
  * @brief  0x0803F800.
  * @param  data: pointer to the word read. f_offset: offset from starting address.
  * @retval None
  */
void Read_Flash(uint32_t *data, uint8_t f_offset)
{
	static DateTime_t Stamp;
	extern FLASH_DATA_ORG FlashDataOrg;
	extern uint8_t DeviceName[4], HW_Version[4], SW_Version[4];
	extern uint32_t Vendor_ID, Prdct_Code, Rev_Number, Ser_Number;
//	const uint32_t FlashAddress = 0x0803F800;	//See processor reference manual

	FlashDataOrg.b_date = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_date_offset));
	FlashDataOrg.b_time = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_time_offset));
	FlashDataOrg.b_mdata.DeviceName = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_mdata.DeviceName_offset));
	FlashDataOrg.b_mdata.HW_Version = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_mdata.HW_Version_offset));
	FlashDataOrg.b_mdata.SW_Version = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_mdata.SW_Version_offset));
	FlashDataOrg.b_mdata.Vendor_ID = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_mdata.Vendor_ID_offset));
	FlashDataOrg.b_mdata.Rev_Number = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_mdata.Rev_Number_offset));
	FlashDataOrg.b_mdata.Ser_Number = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_mdata.Ser_Number_offset));
	FlashDataOrg.b_status.s0 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s0_offset));
	FlashDataOrg.b_status.s1 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s1_offset));
	FlashDataOrg.b_status.s2 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s2_offset));
	FlashDataOrg.b_status.s3 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s3_offset));
	FlashDataOrg.b_status.s4 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s4_offset));
	FlashDataOrg.b_status.s5 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s5_offset));
	FlashDataOrg.b_status.s6 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s6_offset));
	FlashDataOrg.b_status.s7 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s7_offset));
	FlashDataOrg.b_status.s8 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s8_offset));
	FlashDataOrg.b_status.s9 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s9_offset));
	FlashDataOrg.b_status.sa = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.sa_offset));
	FlashDataOrg.b_status.sb = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.sb_offset));
	FlashDataOrg.b_status.sc = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.sc_offset));
	FlashDataOrg.b_status.sd = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.sd_offset));
	FlashDataOrg.b_status.se = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.se_offset));
	FlashDataOrg.b_status.sf = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.sf_offset));
	FlashDataOrg.b_status.s10 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s10_offset));
	FlashDataOrg.b_status.s11 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s11_offset));
	FlashDataOrg.b_status.s12 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s12_offset));
	FlashDataOrg.b_status.s13 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s13_offset));
	FlashDataOrg.b_status.s14 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s14_offset));
	FlashDataOrg.b_status.s15 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s15_offset));
	FlashDataOrg.b_status.s16 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s16_offset));
	FlashDataOrg.b_status.s17 = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s17_offset));

	//Format the RTC Current Date & Time read from flash in the time data structure
	memcpy(&Stamp.date[0], &FlashDataOrg.b_date, 3);
	Stamp.date[0] = ByteToBcd(Stamp.date[0]);
	Stamp.date[1] = ByteToBcd(Stamp.date[1]);
	Stamp.date[2] = ByteToBcd(Stamp.date[2]);
	memcpy(&Stamp.time[0], &FlashDataOrg.b_time, 3);	//Not used; the time is stored in the RTC backup registers

//	RTC_DateRegulate(&hrtc, Stamp.date[2], Stamp.date[0], Stamp.date[1], 0x01);

	//Format the data read from flash in the board data structure (comment if declared as const)
//	memcpy(&DeviceName[0], &FlashDataOrg.b_mdata.DeviceName, 4);
//	memcpy(&HW_Version[0], &FlashDataOrg.b_mdata.HW_Version, 4);
//	memcpy(&SW_Version[0], &FlashDataOrg.b_mdata.SW_Version, 4);
//	Vendor_ID = FlashDataOrg.b_mdata.Vendor_ID;
//	Prdct_Code = FlashDataOrg.b_mdata.Prdct_Code;
//	Rev_Number = FlashDataOrg.b_mdata.Rev_Number;
//	Ser_Number = FlashDataOrg.b_mdata.Ser_Number;
}

/**
  * @brief  Enter in Stand-By mode and wake-up with PA0.0 pin
  * @param  none
  * @retval None
  */
void enter_stby_mode(void)
{
	/*## Disable all used wake-up sources #####################################*/
	/* Disable Wake-up timer */
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

	/*## Clear all related wake-up flags ######################################*/
    /* Clear PWR wake up Flag */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	/* Enable WKUP pin */
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

    /*## Enter Standby Mode ##################################################*/
	HAL_PWR_EnterSTANDBYMode();
	while( 1 ) {}
}

/**
  * @brief  Sets the vector table location and Offset.
  * @param  NVIC_VectTab: specifies if the vector table is in RAM or FLASH memory.
  *   This parameter can be one of the following values:
  *     @arg NVIC_VectTab_RAM: Vector Table in internal SRAM.
  *     @arg NVIC_VectTab_FLASH: Vector Table in internal FLASH.
  * @param  Offset: Vector Table base offset field. This value must be a multiple of 0x200.
  * @retval None
  */
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset)
{
	/* Check the parameters */
	assert_param(IS_NVIC_VECTTAB(NVIC_VectTab));
	assert_param(IS_NVIC_OFFSET(Offset));

	SCB->VTOR = NVIC_VectTab | (Offset & (uint32_t)0x1FFFFF80);
}

/* @fn    	JumpToBootloader
 * @brief 	Function to perform jump to system memory boot from user application.
 * 		  	Call function when you want to jump to system memory.
 * @Remark	In DFU mode, STM32CubeProgrammer "Run after programming" cause a BusFault
 * 			when the program restarts. I don't know why.
 * 			For the moment the workaround was to add NVIC_SystemReset in
 * 			HardFault_Handler and in BusFault_Handler Call-backs
 * */
void jump_to_bootloader(void)
{
	uint8_t i;
	void (*SysMemBootJump)(void);
#if (USE_IWDGT)
	extern IWDG_HandleTypeDef hiwdg;
#endif

	// 1)  reset USARTx via RCC_APB2RSTR
    __HAL_RCC_USART3_FORCE_RESET();
    HAL_Delay(5);
    __HAL_RCC_USART3_RELEASE_RESET();
    HAL_Delay(5);

//  NVIC_SystemReset();

	// 2) Set system memory address. For STM32F1xx, system memory is on 0x1FFF F000
	//								 For STM32F1xx XL density, system memory is on 0x1FFF E000
	//								 For STM32F10x, system memory is on 0x1FFF B000
	//								 For STM32F4xx, system memory is on 0x1FFF 0000
	// 								 (See AN2586, AN2606)
	volatile uint32_t addr = 0x1FFFB000;

	// 3) Disable all interrupts.
	__disable_irq();

	// 4) Disable RCC, set it to default (after reset) settings: Internal clock, no PLL, etc.
	HAL_RCC_DeInit();

	// 5) Disable systick timer and reset it to default values.
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
//	HAL_NVIC_DisableIRQ(SysTick_IRQn);					// Comment for STM32F1xxx family!!!!

	// 6) Clear Interrupt Enable Register & Interrupt Pending Register.
	for (i=0; i<5; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	// 7) Re-enable all interrupts if DFU mode selected
#if (BOOTLOADER_MODE == 1)
	__enable_irq();
#endif
	// ARM Cortex-M Programming Guide to Memory Barrier Instructions
	__DSB();

	// 8) Re-map system memory to address 0x0000 0000 in address space; For each family registers may be different
	NVIC_SetVectorTable(addr, 0x00);
//	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
	HAL_DeInit();
	// Remap is bot visible at once. Execute some unrelated command!
	__DSB();
	__ISB();

	// 9) Set jump memory location for system memory.
	//	  Use address with 4 bytes offset which specifies jump location where program starts
	SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));

	// 10) Set main stack pointer.
	__set_MSP(*(uint32_t *)addr);

	// 10a) Set the IWDT timeout to the maximum value to prevent the system from resetting
	// before starting the DFU bootloader. The bootloader runs with IWDT disabled
#if (USE_IWDGT)
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_IWDG_Refresh(&hiwdg);
#endif

	// 11)  Call our function to jump to set location
	SysMemBootJump();

	while( 1 ) {}
}

/* @fn    	RunActivePartition
 * @param   AppAddress: Specifies the address of the called application.
 * @brief 	Function to perform jump to other application from user application.
 * 		  	Call function when you want to switch between two applications both in FLASH
 * */
void RunActivePartition(uint32_t AppAddress)
{
	uint8_t i;
	void (*JumpToApplication)(void);

	__disable_irq();

	/* Reset RCC clock configuration */
	HAL_RCC_DeInit();

	/* Disable and reset systick timer */
	SysTick->CTRL= 0; SysTick->LOAD = 0; SysTick->VAL = 0;
	HAL_NVIC_DisableIRQ(SysTick_IRQn);

	/* Disable all interrupts */
	for (i=0; i<5; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	__enable_irq();
	__DSB();
	/* Set vector table offset register */
	NVIC_SetVectorTable(AppAddress, 0x00);
//	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
	HAL_DeInit();
	// Remap is not visible at once. Execute some unrelated command!
	__DSB();
	__ISB();

	/* Set jump memory location for system memory. */
	JumpToApplication = (void (*)(void)) (*((uint32_t *) (AppAddress + 4)));

	/* STM32 needs the stack pointer to start at the beginning of ** the application in flash. This must happen last */
	__set_MSP(*(uint32_t*) AppAddress);

	JumpToApplication();

	while( 1 ) {}
}

/****************************************************************************//**
 * 						General Purpose Utilities Section
 *******************************************************************************/
/* @fn    size
 * @brief returns the size of a character array using a pointer to
 * 		  the first element of the character array.
 * */
int size(uint8_t *ptr)
{
    //variable used to access the subsequent array elements.
    int offset = 0;
    //variable that counts the number of elements in your array
    int count = 0;
    //While loop that tests whether the end of the array has been reached
    while (*(ptr + offset) != '\0')
    {
        //increment the count variable
        ++count;
        //advance to the next element of the array
        ++offset;
    }
    //return the size of the array
    return count;
}

/* @fn    InRange
 * @brief returns the position of a value with respect to a range:
 * 		  1 -> min <= value < max; 2-> value > max; 3-> value < min
 * */
uint8_t InRange(uint16_t min, uint16_t max, uint16_t value)
{
	uint8_t pos = 0;
	if ((value >= min) && (value <= max))
	{
		pos = 1;
	} else if (value > max)
	{
		pos = 2;
	} else if (value < min)
	{
		pos = 3;
	}

	return pos;
}

/* @fn	  xtoi
 * @brief convert hex string to integer
 * */
int xtoi(char *hexstring)
{
	int	i = 0;

	if ((*hexstring == '0') && (*(hexstring+1) == 'x'))
		  hexstring += 2;
	while (*hexstring)
	{
		char c = toupper(*hexstring++);
		if ((c < '0') || (c > 'F') || ((c > '9') && (c < 'A')))
			break;
		c -= '0';
		if (c > 9)
			c -= 7;
		i = (i << 4) + c;
	}
	return i;
}

/* @fn		FlipFlop
 * @brief	Toggle LSB. Used for toggle a variable state following an event change
 * @param	ResetFF: reset the counter when true;
 * @param	mask return value toggle after mask cicles;
 * @param	period: counter resets after reached period value;
 * @return  N-th 1 if ON and 0 for OFF when mask = 0xN-th = 1;
 * @return  Counter (FlipFlop) value when mask = 0xFF;
 * */
uint8_t FlipFlop(bool ResetFF, uint8_t mask, uint8_t period)
{
	if (ResetFF)
		t_flip = 0;
	else
		if (t_flip < period)
		{
			t_flip++;
		} else
		{
			t_flip = 0;
		}

	return (t_flip & mask) ? 0x01 : 0;	//Return the n-th bit. N-th bit changes every n calls.
}

/* @fn		approxMovingAverage
 * @brief	Approximate a rolling average by applying a weighted average on input stream.
 * @brief	It is an approximation, so it's value will not match exactly with a true
 * 			rolling average. The greater the value of N, the better the approximation.
 * @brief	Note that this approximation is equivalent to an exponential moving average.
 * 	https://stackoverflow.com/questions/12636613/how-to-calculate-moving-average-without-keeping-the-count-and-data-total
 * 			(Modified by adding the correction factor 1+1/N)
 * 	Modified moving average (MMA), running moving average (RMA), or smoothed moving average (SMMA) as defined in:
 * 			"https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average"
 * @param	avg: previous average value;
 * @param	new_sample: new value of the input stream;
 * @param	N: the number of samples where you want to average over;
 * @return  the approximated rolling average over N samples;
 * */
float32_t approxMovingAverage(float32_t avg, float32_t new_sample, uint32_t N)
{
	float32_t CorrectionFactor;

/*	if ((service_timer3 > (N-1)) || (ColdRestart))
		CorrectionFactor = 1.0;
	else
		CorrectionFactor = 1.0+(1.0/N); */
	if ((service_timer3 > (N-1)) || (ColdRestart))
	{
		if ((new_sample - avg) > 0)
		{
			CorrectionFactor = 1.0+(1.0/N);
		} else
		{
			CorrectionFactor = 1.0-(1.0/N);
		}
	} else
	{
		CorrectionFactor = 1.0+(1.0/N);
	}

    avg -= avg / N;
    avg = (avg + new_sample/N) * CorrectionFactor;

    return avg;
}

/**
  * @brief  Converts a 2 digit decimal to BCD format.
  * @param  Value: Byte to be converted
  * @retval Converted byte
  */
uint8_t ByteToBcd(uint8_t Value)
{
  uint32_t bcdhigh = 0U;

  while (Value >= 10U)
  {
    bcdhigh++;
    Value -= 10U;
  }

  return ((uint8_t)(bcdhigh << 4U) | Value);
}

/**
 * Find maximum between two or more integer variables
 * @param args Total number of integers
 * @param ... List of integer variables to find maximum
 * @return Maximum among all integers passed
 */
int32_t max(int32_t args, ...)
{
	int32_t i, max, cur;
    va_list valist;
    va_start(valist, args);

    max = INT_MIN;

    for(i=0; i<args; i++)
    {
        cur = va_arg(valist, int32_t);	// Get next elements in the list
        if(max < cur)
            max = cur;
    }

    va_end(valist);		// Clean memory assigned by valist

    return max;
}

/**
 * Find minimum between two or more integer variables
 * @param args Total number of integers
 * @param ... List of integer variables to find minimum
 * @return Minimum among all integers passed
 */
int32_t min(int32_t args, ...)
{
	int32_t i, min, cur;
    va_list valist;
    va_start(valist, args);

    min = INT_MAX;

    for(i=0; i<args; i++)
    {
        cur = va_arg(valist, int32_t);	// Get next elements in the list
        if(min > cur)
            min = cur;
    }

    va_end(valist);		// Clean memory assigned by valist

    return min;
}

/****************************************************************************//**
 * 								Time section
 *******************************************************************************/
/* @fn	  portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 * 		  CLOCKS_PER_SEC frequency.
 * 		  The resolution of time32_incr is usually 1/1000 sec.
 * */
__INLINE uint32_t portGetTickCnt(void)
{
	return HAL_GetTick();
}

/* @fn	  usleep
 * @brief precise usleep() delay
 * */
#pragma GCC optimize ("O0")
int usleep(useconds_t usec)
{
	int i,j;
#pragma GCC ivdep
	for(i=0;i<usec;i++)
	{
#pragma GCC ivdep
		for(j=0;j<2;j++)
		{
			__NOP();
			__NOP();
		}
	}
	return 0;
}

#pragma GCC optimize ("Os")
/* @fn 	  Sleep
 * @brief Sleep delay in ms using SysTick timer
 * */
__INLINE void
Sleep(uint32_t x)
{
	HAL_Delay(x);
}

/****************************************************************************//**
 * 								END OF Time section
 *******************************************************************************/
/****************************************************************************//**
 * 							Olimex_STM32_H405 port section
 *******************************************************************************/
/* @fn		led_off
 * @brief	switch off the led from led_t enumeration
 * */
void led_off (led_t led)
{
	switch (led)
	{
	case LED_PC6:
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);	//Led "STAT" on PC12 on Olimex H405 Board!!
		break;
	case LED_PC7:
		write_port(0x3431, 0x30);	//Button Blue Led
		break;
/*	case LED_PC8:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		break;
	case LED_PC9:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		break;
	case LED_ALL:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		break; */
	default:
		// do nothing for undefined led number
		break;
	}
}

/* @fn		led_on
 * @brief	switch on the led from led_t enumeration
 * */
void led_on (led_t led)
{
	switch (led)
	{
	case LED_PC6:
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);	//Led "STAT" on PC12 on Olimex H405 Board!!
		break;
	case LED_PC7:
		write_port(0x3431, 0x31);	//Button Blue Led
		break;
/*	case LED_PC8:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		break;
	case LED_PC9:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		break;
	case LED_ALL:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		break; */
	default:
		// do nothing for undefined led number
		break;
	}
}

/* @fn		led_toggle
 * @brief	toggle the led from led_t enumeration
 * */
void led_toggle (led_t led)
{
	static bool toggle = false;

	switch (led)
	{
	case LED_PC6:
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);		//Led "STAT" on PC12 on Olimex H405 Board!!
		break;
	case LED_PC7:
		if (toggle)
		{
			write_port(0x3431, 0x31);	//Button Blue Led On
		} else
		{
			write_port(0x3431, 0x30);	//Button Blue Led Off
		}
		toggle = !toggle;
		break;
/*	case LED_PC8:
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
		break;
	case LED_PC9:
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
		break;
	case LED_ALL:
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
		break; */
	default:
		// do nothing for undefined led number
		break;
	}
}

/****************************************************************************//**
 * 								CallBacks section
 *******************************************************************************/
/* @fn		HAL_TIM_OC_DelayElapsedCallback
  * @brief  Timer 1 Interrupt Handler; Sensor Scheduler.
  * 		Output Compare callback in non blocking mode
  * 		Channel_1 -> 100Hz interrupt; set the update_100Hz flag
  * 		Channel_2 -> 50Hz interrupt;  set the update_50Hz flag
  * 		Channel_3 -> 16Hz interrupt;  set the update_16Hz flag
  * 		Channel_4 -> 25Hz interrupt;  set the update_25Hz flag
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint32_t uhCapture=0;

	HAL_TIM_Base_Stop_IT(&htim1);				//Stop Timer1
	/* TIM1_CH1 toggling with frequency = 100Hz */
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));
		update_100Hz = 1;
	}
	/* TIM1_CH2 toggling with frequency = 50Hz */
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));
		update_50Hz = 1;
	}
	/* TIM1_CH3 toggling with frequency = 16Hz */
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uhCapture + DEFAULT_uhCCR3_Val));
		update_16Hz = 1;
	}
	/* TIM1_CH4 toggling with frequency = 20 Hz */
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, (uhCapture + DEFAULT_uhCCR4_Val));
		update_25Hz = 1;
	}
	timer1_expired = true;
	HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
	process_timer1_irq();
}

/* @fn		HAL_TIM_PeriodElapsedCallback
 * @brief	Timer 6 Interrupt Handler; Called when Push Button is pressed for more then 5s.
 *          Display Menu Entries when Button is pressed for more then 5s
 * @brief	Timer 3 Interrupt Handler (process_timer3_irq():
 * 			Called when Timer3 counter expires (5 second period).
 * 			There isn't a Timer 6 Interrupt Handler; when Timer 6 expires (period 5s)
 * 			the timer5s_expired flag is set and the storage of VOC sensor (CCs811)
 * 			baseline data is performed in VOC_Sensor_Handler function (in MEMS_app.c)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/*	if (htim->Instance==htim1.Instance)
	{
		HAL_TIM_Base_Stop_IT(&htim1);			//Stop Timer1
		HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
		HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		timer1_expired = true;
		process_timer1_irq();
	} */
	if (htim->Instance==htim6.Instance)
	{
		HAL_TIM_Base_Stop_IT(&htim6);			//Stop Timer 6
		HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		timer5s_expired = true;
#if (VOC_SENSOR_PRESENT==1)
	#if (CCS811)
		Store_CCS811_Baseline = true;
	#endif
#endif
	}
	if (htim->Instance==htim3.Instance)
	{
		HAL_TIM_Base_Stop_IT(&htim3);			//Stop Timer3
		HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
		HAL_NVIC_DisableIRQ(TIM3_IRQn);
		timer3_expired = true;
		process_timer3_irq();
	}
	if (htim->Instance==htim7.Instance)
	{
		HAL_TIM_Base_Stop_IT(&htim7);			//Stop Timer7
		HAL_NVIC_ClearPendingIRQ(TIM7_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		process_timer7_irq();
	}
}

/* @fn		HAL_SYSTICK_Callback ()
 * @brief	Use SysTick timer to blink leds
 * */
void HAL_SYSTICK_Callback()
{
	static uint16_t led_pc6_timer = 0;
//	static uint16_t led_pc7_timer = 150;
//	static uint16_t led_pc8_timer = 0;
//	static uint16_t led_pc9_timer = 0;

	 /* We call this handler every 1ms */
	if (leds_test)
	{
		if (++led_pc6_timer >= LedPc6Timer_TimeOut)
		{
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			led_toggle(LED_PC6); 					//Led "STAT" on PC12 in Olimex H405 Board!!
			led_pc6_timer = 0;
		}
/*		if (++led_pc7_timer >= 450)
		{
			led_toggle(LED_PC7);
			led_pc7_timer = 150;
		}
		if (++led_pc8_timer == 1000)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			led_pc8_timer = 0;
		}
		if (++led_pc9_timer == 1000)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
			led_pc9_timer = 0;
		}*/
	} else
	{
		led_on(LED_PC6);							//Led "STAT" in current sink in Olimex H405 Board!!
/*		led_off(LED_PC7);
		led_offLED_PC8);
		led_off(LED_PC9); */
	}
	//Free running ServiceTimer0 management
	if (++service_timer0 >= ServiceTimer0_TimeOut)
	{
		service_timer0_expired = true;
		service_timer0 = 0;
	}
}

/**
  * @brief  Second event callback.
  * @param  hrtc: pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
	extern FLASH_DATA_ORG FlashDataOrg;
#if ((BLE_SUPPORT) && (BEACON_APP) && (USE_IWDGT))
	extern IWDG_HandleTypeDef hiwdg;
#endif
//	static DateTime_t Stamp;
	static uint16_t s = 0;
	static uint16_t m = 0;
	static uint16_t hd = 0;
	static uint16_t h = 0;
//	static uint8_t d = 0;
#if (USE_BKUP_SRAM)
	static uint32_t counter_time = 0;
	extern void Store_MeanValues_BackupRTC(void);
#endif
	const uint16_t Baseline_El_Store_Period = 24*7;
	const uint16_t GAS_SesorBoard_WarmUp = 45;	//Do not acquire the the analog sensors values
												//before the warm-up period (in minutes) has elapsed
	if (Test_Mode)								//GAS_SesorBoard_WarmUp must be greater timeout RUN_IN_TIME
	{											//where the CCS811 VOC sensor loads the BaseLine
	#if ((BLE_SUPPORT) && (BEACON_APP) && (USE_IWDGT))
		HAL_IWDG_Refresh(&hiwdg);
	#endif
#if (USE_BKUP_SRAM)
		counter_time = RTC_GetCounter();
		/**
		 * The current timestamp will be used in the main() function to recognize
		 * if the reset was generated by the WatchDog Timer or by a power-cycle
		 */
		BakUpRTC_Data[0] = BLE_MAGIC_NUMBER;
		BakUpRTC_Data[1] = (uint8_t)(counter_time & 0xFF);
		BakUpRTC_Data[2] = (uint8_t)((counter_time >> 8) & 0xFF);
		BakUpRTC_Data[3] = (uint8_t)((counter_time >> 16) & 0xFF);
		BakUpRTC_Data[4] = (uint8_t)((counter_time >> 24) & 0xFF);
#endif
	}
#if (USE_BKUP_SRAM)
	enable_backup_rtc();					//Store the timestamp every second
	writeBkpRTC((uint8_t *)BakUpRTC_Data, 6, 0);
	disable_backup_rtc();
#endif

	FlashDataOrg.b_status.s1++;				//Increment the total uptime timer
	update_1s = true;
	if (++s > 59)
	{
		update_1m = true;
		m++;
		if (m > GAS_SesorBoard_WarmUp)
			WarmUpPeriod_expired = true;
#if (CCS811)
		if (m == RUN_IN_TIME)
			load_baseline = true;
#endif
#if ((BLE_SUPPORT) && (BEACON_APP))
		BLE_DataReady = true;	//To allow BLE transmission to occur when valid sensor data
#endif				//is certainly available, start beaconing after 1 minute from the start of sensor reading.
		s = 0;
	}
	if (m > 59)
	{
		Write_Flash(0, 0);					//Store board data-base every hour
#if (USE_BKUP_SRAM)
		/*
		 * Every hour the average values of the sensors are stored in the processor's Static Ram Backup.
		 * In this way, after a "short" reset (not a power-cycle) the daily statistic will not be lost
		 * and will be immediately available for transmission.
		 */
		Store_MeanValues_BackupRTC();
#endif
		update_1h = true;
		m = 0;
		if (++hd > 24)						//Perform HTS221 memory boot every day
		{
#if (HTS221)
			HTS221_status = HTS221_MemoryBoot(HTS221_BADDR);
#endif
			update_1d = true;
			hd = 0;
		}
		if (++h > Baseline_El_Store_Period)	//Store CCS811 Baseline every week
		{
#if (VOC_SENSOR_PRESENT==1)
	#if (CCS811)
			Store_CCS811_Baseline = true;	//Update CCS811 Baseline in board database structure
	#endif
#endif
			h = 0;
		}
	}
}

/* @fn		HAL_GPIO_EXTI_Callback
 * @brief	IRQ HAL call-back for all EXTI configured lines
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_Delay(debounce_delay);					//Debounce delay
	switch (GPIO_Pin)
	{
	case GPIO_PIN_0:
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);		//It will be re-enabled in the process_pn_menu_irq() function
		process_pn_menu_irq();
		//break;
	case GPIO_PIN_2:
		HAL_NVIC_DisableIRQ(EXTI2_IRQn);	//It will be re-enabled in the process_IO_Expander_irq() function
		//Disables the interrupt of the button IO_EXP input to debounce button input
		//It will be re-enabled in the read_ports() function when the service_timer1 expires
#if (IO_EXP_PRESENT==1)
		static uint8_t send_data = 0x7F;

		mcp23017_write_registers(MCP23017_MASTER2_BADDR, MCP23017_GPINTENB, &send_data, 1);
#endif
		ServiceTimerStart(STim_1);
		process_IO_Expander_irq();
		break;
	default:
		break;
	}
}

#if (CAN_SUPPORT==1)
/*! -----------------------------------------------------------------------------
 * @Function: HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
 * @brief  Rx Fifo 0 message pending callback
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef	Rx_Header1, Rx_Header2;
	CAN_TxHeaderTypeDef	Tx_Header1, Tx_Header2;
	uint8_t				Rx_Data1[8], Rx_Data2[8];

	if(hcan->Instance==CAN1)
	{
		/* Get RX message */
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx_Header1, Rx_Data1) != HAL_OK)
		{
			/* Reception Error */
			Error_Handler();
		}
#if		(CAN_REPEATER_MODE==1)	//Accept all incoming headers if in repeater mode
		//The size of the RxHeader is limited to 24 bye (length of the TxHeader) as the last word of the RxHeader
		//is not significant for the retransmission of the CAN frame
		process_CAN_RX_irq(hcan, Rx_Header1, Rx_Data1, (uint32_t) sizeof(Tx_Header1), (uint32_t) sizeof(Rx_Data1));
#else
//		memcpy((void *)&RxHeader1, (void *)&Rx_Header1, sizeof(RxHeader1));		//Uncomment to disable header check (in CAN Normal Mode)
		/* Check Message */
		if ((Rx_Header1.StdId == RxHeader1.StdId) && (Rx_Header1.IDE == RxHeader1.IDE) && (Rx_Header1.DLC == RxHeader1.DLC))
		{
			process_CAN_RX_irq(hcan, Rx_Header1, Rx_Data1, (uint32_t) sizeof(Tx_Header1), (uint32_t) sizeof(Rx_Data1));
		}
#endif
	}
	else if(hcan->Instance==CAN2)
	{
		/* Get RX message */
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx_Header2, Rx_Data2) != HAL_OK)
		{
			/* Reception Error */
			Error_Handler();
		}
#if		(CAN_REPEATER_MODE==1)	//Accept all incoming headers if in repeater mode
		//The size of the RxHeader is limited to 24 bye (length of the TxHeader) as the last word of the RxHeader
		//is not significant for the retransmission of the CAN frame
		process_CAN_RX_irq(hcan, Rx_Header2, Rx_Data2, (uint32_t) sizeof(Tx_Header2), (uint32_t) sizeof(Rx_Data2));
#else
//		memcpy((void *)&RxHeader2, (void *)&Rx_Header2, sizeof(RxHeader2));		//Uncomment to disable header check (in CAN Normal Mode)
		/* Check Message */
		if ((Rx_Header2.StdId == RxHeader2.StdId) && (Rx_Header2.IDE == RxHeader2.IDE) && (Rx_Header2.DLC == RxHeader2.DLC))
		{
			process_CAN_RX_irq(hcan, Rx_Header2, Rx_Data2, (uint32_t) sizeof(Tx_Header2), (uint32_t) sizeof(Rx_Data2));
		}
#endif
	}
}

/**
  * @brief  Transmission Mailbox 0 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance==CAN1)
	{
		process_CAN_TX_irq(hcan);
	}
	else if(hcan->Instance==CAN2)
	{
		process_CAN_TX_irq(hcan);
	}
}

/**
  * @brief  Transmission Mailbox 1 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance==CAN1)
	{
		process_CAN_TX_irq(hcan);
	}
	else if(hcan->Instance==CAN2)
	{
		process_CAN_TX_irq(hcan);
	}
}

/**
  * @brief  Transmission Mailbox 2 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance==CAN1)
	{
		process_CAN_TX_irq(hcan);
	}
	else if(hcan->Instance==CAN2)
	{
		process_CAN_TX_irq(hcan);
	}
}
#endif	//CAN_SUPPORT==1
#if (USART_SUPPORT==1)
/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   Report the end of IT Rx transfer.
  * @note	BUFFERSIZE is the minimum amount of characters receivable in Non-Blocking-Mode
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	__HAL_UART_ENABLE_IT(UartHandle, UART_IT_RXNE);		//Re-enable UART data registry not empty interrupt
	__HAL_UART_CLEAR_OREFLAG(UartHandle);				//Clear the UART ORE pending flag (Overrun Error flag)
	__HAL_UART_CLEAR_NEFLAG(UartHandle);				//Clear the UART NFCF pending flag (Noise Detection Error flag)
	__HAL_UART_FLUSH_DRREGISTER(UartHandle);			//Flush the DR register to prevent overrun
	if(UartHandle->Instance==USART2)
	{
	    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
	}
	if(UartHandle->Instance==USART3)
	{
	    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
	}
	if(HAL_UART_Receive_DMA(UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)	//Re-Enable reception
	{
		rx_done = false;
		/* Start Error Routine*/
//		Error_Handler();
	}
	else
	{
		process_USART_RX_irq(UartHandle, (uint8_t *)aRxBuffer, usart3RxLength);
		rx_done = true;
	}
}

/**
  * @brief  Transmission complete callback.
  * @param  UartHandle pointer to a UART_HandleTypeDef structure that contains
  *         the configuration information for the specified USART.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance==USART2)
	{
	    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
	}
	if(UartHandle->Instance==USART3)
	{
	    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
	}
	process_USART_TX_irq(UartHandle);
}

/**
 * @brief UART error callbacks.
 * @param huart: pointer to a UART_HandleTypeDef structure that contains
 *        the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	HAL_UART_AbortReceive_IT(UartHandle);
	__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_PE);
	__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_RXNE);
	__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_FE);
	__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_NE);
	__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_ORE);
	__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_IDLE);
	__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_TC);
//	__HAL_UART_CLEAR_FLAG(UartHandle, UART_CLEAR_RTOF);
	__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_LBD);
	__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_CTS);
//	__HAL_UART_CLEAR_FLAG(UartHandle, UART_CLEAR_CMF);
	__HAL_UART_FLUSH_DRREGISTER(UartHandle);			//Flush the DR register to prevent overrun
	CLEAR_BIT(UartHandle->Instance->CR1, USART_CR1_RXNEIE);
	UartHandle->RxState= HAL_UART_STATE_READY;
	HAL_UART_Init(UartHandle);
	USART_Config(UartHandle);
}
#endif

/**
 * @brief UART IDLE callback.
 * @param UartHandle: pointer to a UART_HandleTypeDef structure that contains
 *        the configuration information for the specified UART module.
 * @param DMAtHandle: pointer to a DMA_HandleTypeDef structure that contains
 *        the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *UartHandle, DMA_HandleTypeDef *DMAHandle)
{
//	uint32_t j = 0;

	/* When the Host transmits an RS232 data packet it may happen that between one frame and
	 * another there may be a delay; when this delay is greater than the duration of one byte
	 * the idle callback is called before the end of the packet. This causes a shorter packet
	 * received than the transmitted one, and therefore a frame error.
	 * Sleep(x) introduces a delay in the execution of the idle callback, allowing the missing
	 * frames to be received. The value of x depends on the application, but cannot be greater
	 * than the duration of the packet. It may not even be necessary to introduce the delay
	 */
	Sleep(1);

	__HAL_UART_CLEAR_IDLEFLAG(UartHandle);			//Clear Idle Status Bit

	/* As an alternative to Sleep(x), when the length of the packet is known and fixed, it is
	 * possible to wait until the entire packet has been received
	 */
/*	while (RXBUFFERSIZE - (__HAL_DMA_GET_COUNTER(DMAHandle)) < PACKET_LENGTH)
	{
		j++;
		if(j>0xffffe)
			break;
	} */
//Start the timer to measure the latency time between receiving the packet and writing the GPIOs
/*	tick_start = HAL_GetTick();
#if (USE_TIMER6_AS_uS_DELAY==1)
	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_GPIO_WritePin(IO_CONN1_GPIO_Port, IO_CONN1_Pin, GPIO_PIN_SET);
#endif */

	usart3RxLength = RXBUFFERSIZE - (__HAL_DMA_GET_COUNTER(DMAHandle)); // Get length
	// DMA count reload, DMA completion interrupt will be generated,
    // Refer to RM0033-9.3.13, when DMA is off, DMA completion interrupt will be generated
    // Refer to RM0033-DMA_SxNDTR, when DMA is off, reload counter
	if (usart3RxLength)
	{
		__HAL_DMA_DISABLE(DMAHandle);					//Close DMA
		huart3.hdmarx->Instance->CNDTR = RXBUFFERSIZE;	//Reset DMA pointer
		__HAL_DMA_ENABLE(DMAHandle);					//Enable DMA
	}
	UartHandle->RxState= HAL_UART_STATE_READY;
	HAL_UART_RxCpltCallback(UartHandle);			//Needed only for M3 series
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   Report end of conversion.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
	conversion_ended = false;
	process_ADC_irq(AdcHandle);
}

/****************************************************************************//**
 * 								IRQ section
 *******************************************************************************/
/* @fn		Function: process_timer1_irq(void)
 * @brief	Timer1 Interrupt Handler; Executed when Timer1 expires
 * 			Get IMU axes when OC1 expires (at 100Hz rate)
 * 			Set Acc. and Gyro calibration request flags when OC2 expires (at 50Hz rate)
 * 			Set Activity Recognition request flag when OC4 expires (at 16Hz rate)
 * 			Set Magn. calibration request flag when OC4 expires (25Hz)
 */
void process_timer1_irq(void)
{
#if (IMU_PRESENT==1)
	if (update_100Hz)
	{
	//Get Local IMU Values
		LSM9DS1_status = LSM9DS1_Get_Measurement(&IMU_Values);
	//Get Local Time-Stamp
	    RTC_Handler(&hrtc, &dataseq[0]);		//Get RTC data at the MotionFX update rate
		lcl_imu_data_rdy = true;
		send_lcl_imu_data = true;				//Send IMU data at the MotionFX update rate
		update_100Hz = 0;
		__HAL_TIM_CLEAR_IT(&htim1 ,TIM_IT_CC1);	//Clear OC1 Update Interrupt.
	}
	if (update_50Hz)
	{
		__HAL_TIM_CLEAR_IT(&htim1 ,TIM_IT_CC2);	//Clear OC2 Update Interrupt.
		AC_TimeStamp++;
		AccCalRequest = 1;
		GyroCalRequest = 1;
		update_50Hz = 0;
	}
	if (update_16Hz)
	{
		__HAL_TIM_CLEAR_IT(&htim1 ,TIM_IT_CC3);	//Clear OC3 Update Interrupt.
		display_imu_data = true;
		ActRecRequest = 1;
		AR_TimeStamp += AR_ALGO_PERIOD;
		update_16Hz = 0;
	}
	if (update_25Hz)
	{
		__HAL_TIM_CLEAR_IT(&htim1 ,TIM_IT_CC4);	//Clear OC4 Update Interrupt.
		MagCalRequest = 1;
		update_25Hz = 0;
	}
#endif
//	__HAL_TIM_SetCounter(&htim1, 0);			//Clear Timer1
	HAL_TIM_Base_Start_IT(&htim1);				//ReStart Timer 1
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	timer1_expired = false;
}

void button_manage(void)
{
#if (IMU_PRESENT==1)
	extern void MotionFX_manager_start_9X(void);
	extern void MotionFX_manager_stop_9X(void);
#endif

#if (BLE_SUPPORT)
	//When in BLE mode, the system goes into the operating state autonomously
	StartDataStrmng = true;
#else	
	//Otherwise, it waits for the button to be pressed or for the command
	//from the host (when in GUI mode) to transition to the operational state
	StartDataStrmng = (bool)FlipFlop(false, 0xFF, NumberOfPages);
#endif
	//Reset to sub-page 1 on page change
	PM_toggle = 1;
#if (GUI_SUPPORT==0)
	#if (PRESSURE_SENSOR_PRESENT)
	send_lcl_prs_data = true;
	#if (GLCD_SUPPORT)
	display_prs_data = true;
	#endif	//GLCD_SUPPORT
	#endif	//PRESSURE_SENSOR_PRESENT
	#if (HUMIDITY_SENSOR_PRESENT)
	send_lcl_hum_data = true;
	#if (GLCD_SUPPORT)
	display_hum_data = true;
	#endif	//GLCD_SUPPORT
	#endif	//HUMIDITY_SENSOR_PRESENT
	#if (UVx_SENSOR_PRESENT)
	send_lcl_uvx_data = true;
	#if (GLCD_SUPPORT)
	display_uvx_data = true;
	#endif	//GLCD_SUPPORT
	#endif	//UVx_SENSOR_PRESENT
	#if (ALS_SENSOR_PRESENT)
	send_lcl_als_data = true;
	#if (GLCD_SUPPORT)
	display_als_data = true;
	#endif	//GLCD_SUPPORT
	#endif	//ALS_SENSOR_PRESENT
	#if (VOC_SENSOR_PRESENT)
	send_lcl_voc_data = true;
	#if (GLCD_SUPPORT)
	display_voc_data = true;
	#endif	//GLCD_SUPPORT
	#endif	//VOC_SENSOR_PRESENT
	#if (PARTICULATE_SENSOR_PRESENT)
	send_lcl_pms_data = true;
	#if (GLCD_SUPPORT)
	display_pms_data = true;
	#endif	//GLCD_SUPPORT
	#endif	//PARTICULATE_SENSOR_PRESENT
	#if (GAS_SENSOR_MODULE_PRESENT)
	send_lcl_gas_data = true;
	#if (GLCD_SUPPORT)
	display_gas_data = true;
	#endif	//GLCD_SUPPORT
	#endif	//GAS_SENSOR_MODULE_PRESENT
#endif	//GUI_SUPPORT==0
#if (IMU_PRESENT==1)
	MagCalRequest = 1;							//Request magnetometer recalibration  on push button
	#if (GUI_SUPPORT==0)
//	StartDataStrmng = !StartDataStrmng;			//Send data on push button
	if (StartDataStrmng)
	{
		TIM_OC_Timers_Start(&htim1);
		MotionFX_manager_start_9X();
	} else
	{
		TIM_OC_Timers_Stop(&htim1);
		MotionFX_manager_stop_9X();
	}
	#endif	//GUI_SUPPORT==0
#elif ((PRESSURE_SENSOR_PRESENT==1) || (HUMIDITY_SENSOR_PRESENT==1) || (UVx_SENSOR_PRESENT==1) || \
   (VOC_SENSOR_PRESENT==1) || (PARTICULATE_SENSOR_PRESENT==1) || (GAS_SENSOR_MODULE_PRESENT==1))
//	StartDataStrmng = !StartDataStrmng;			//Send data on push button
	#if (TLCD_SUPPORT==1)
	LCD_Clear();
	if(!StartDataStrmng)
	{
		send_tlcdmessage(WELCOME_STRING1, 26);
		send_tlcdmessage(MY_FW_VERSION,26);
	}
	#elif (GLCD_SUPPORT==1)
	if(!StartDataStrmng)
	{						//Reset the PreviousPage variable (used in ReDrawPage_S0 function)
		PreviousPage = 1;	//So the screen is displayed correctly
	    SendWelcomeMessage();
	}
	#endif	//GLCD_SUPPORT==1
#endif
#if (PARTICULATE_SENSOR_PRESENT==1)				//PowerOn SPS30 PMx sensor
	if(StartDataStrmng)
	{
		sps30_start_measurement();
	} else
	{
		sps30_stop_measurement();
	}
#endif
}

/* @fn		process_pn_menu_irq(void)
 * @brief	PA0 Interrupt Handler; Called when Push Button is pressed
 * 			(Falling Edge Interrupt, on STM32-H405 Olimex Board) or released (Rising Edge Interrupt)
 * 			On Falling Edge Interrupt starts timer6; On Rising Edge Interrupt reset Timer 6
 * */
void process_pn_menu_irq(void)
{
	if (!HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin))
	{
		__HAL_TIM_SET_COUNTER(&htim6, 0);				//Clear Timer 6
		__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);	//Clear Timer 6 Update Interrupt Flag
		HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
		HAL_TIM_Base_Start_IT(&htim6);					//Start Timer 6
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		//Send data on pushing button
		button_manage();
	} else
	{
		HAL_TIM_Base_Stop_IT(&htim6);				//Stop Timer 6
		HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
	}
	 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	 HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
	 HAL_NVIC_EnableIRQ(EXTI0_IRQn);		//It was be disabled in the HAL_GPIO_EXTI_Callback() function
}

/* @fn		Function: process_timer3_irq(void)
 * @brief	Timer3 Interrupt Handler;
 * 			Executed when Timer3 expires (APPLICATION_RUN_CYCLE, period 5s)
 * 			Pressure, Temperature, Humidity, VOC, Particulate Matter sensor data and
 * 			Time-Stamp are acquired when Timer3 expires
 */
void process_timer3_irq(void)
{
	static uint16_t DeltaT = 0;
	extern FLASH_DATA_ORG FlashDataOrg;
#if ((BLE_SUPPORT) && (BEACON_APP))
	#if (CCS811)
	extern void StoreMinMax(LPS25HB_MeasureTypeDef_st *PressTemp, HTS221_MeasureTypeDef_st *HumTemp, ANLG_MeasureTypeDef_st *Measurement_Value,
							CCS811_MeasureTypeDef_st *voc, SPS30_MeasureTypeDef_st *Particulate);
	#elif(ENS160)
	extern void StoreMinMax(LPS25HB_MeasureTypeDef_st *PressTemp, HTS221_MeasureTypeDef_st *HumTemp, ANLG_MeasureTypeDef_st *Measurement_Value,
	                        ENS160_MeasureTypeDef_st *voc, SPS30_MeasureTypeDef_st *Particulate);
	#endif
#endif
#if ((BLE_SUPPORT) && (SENSOR_APP))
	static bool button_manage_done = false;
#endif

	update_5s = true;
	__HAL_TIM_SET_COUNTER(&htim3, 0);	//Clear Timer3
	HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
	HAL_TIM_Base_Start_IT(&htim3);		//Start Timer3
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	timer3_expired = false;
	if (WarmUpPeriod_expired)			//service_timer3 takes into account the sensors operating
		service_timer3++;				//period, starting from the end of the warm_up time.
	if (++DeltaT >= (3600/APPLICATION_RUN_CYCLE))
	{
#if (PARTICULATE_SENSOR_PRESENT==1)		//Check every hour if the VOC sensor fan needs cleaning
		FlashDataOrg.b_status.s2++;		//Increment the SPS30 fan cleaning uptime timer
		if (++FlashDataOrg.b_status.s2 >= SENSIRION_AUTOCLEAN_INTERVAL)
		{
			FlashDataOrg.b_status.s2 = 0;
//			sps30_start_manual_fan_cleaning();	//Autoclean enabled in MX_SPS30_Init()
		}
#endif
		DeltaT = 0;
	}
//	Get Local Timestamp
#if (GUI_SUPPORT==1)
	RTC_Handler(&hrtc, &dataseq[0]);
#else
	RTC_Handler(&hrtc, &dataseq1[0]);
	#if ((BLE_SUPPORT) && (BEACON_APP))
	dataseq1[6] = 0x0a;
	memcpy(&BLE_TimeStamp, &dataseq1[3], 4);
	#endif
#endif
#if (PRESSURE_SENSOR_PRESENT==1)
	#if (LPS25HB)
		LPS25HB_status = LPS25HB_Get_Measurement(LPS25HB_BADDR, &PRS_Values);
	#elif (LPS22HB)
		LPS22HB_status = LPS22HB_Get_Measurement(LPS22HB_BADDR, &PRS_Values);
	#endif	// LPS25HB
//	MEMS_LclData = (uint32_t)PRS_Values.Pout; MEMS_LclData <<= 16;
//	MEMS_LclData |= (uint16_t)PRS_Values.Tout;
	lcl_prs_data_rdy = true;
	send_lcl_prs_data = true;
	display_prs_data = StartDataStrmng;
#endif	// PRESSURE_SENSOR_PRESENT
#if (HUMIDITY_SENSOR_PRESENT==1)
	#if (HTS221)
		HTS221_status = HTS221_Get_Measurement(HTS221_BADDR, &HUM_Values);
	#elif (SHT4x)
		SHT4x_status = SHT4x_Get_Measurement(SHT4x_BADDR, &HUM_Values);
	#endif	// HTS221
	lcl_hum_data_rdy = true;
	send_lcl_hum_data = true;
	display_hum_data = StartDataStrmng;
#endif	// HUMIDITY_SENSOR_PRESENT
#if (UVx_SENSOR_PRESENT==1)
	#if (VEML6075)
		VEML6075_status = VEML6075_Get_Measurement(&hi2c1, &UVx_Values);
	#elif (LTR390UV)
		LTR390UV_status = LTR390UV_Get_Measurement(LTR390UV_BADDR, &UVx_Values);
	#endif	//VEML6075
	lcl_uvx_data_rdy = true;
	send_lcl_uvx_data = true;
	display_uvx_data = StartDataStrmng;
#endif	//UVx_SENSOR_PRESENT
#if (ALS_SENSOR_PRESENT==1)
	VEML7700_status = VEML7700_Get_Measurement(&hi2c1, &ALS_Values);
	lcl_als_data_rdy = true;
	send_lcl_als_data = true;
	display_als_data = StartDataStrmng;
#endif
#if (VOC_SENSOR_PRESENT==1)
	//If available Set Set Environment Parameters for CCS811 environmental compensation
	#if (HUMIDITY_SENSOR_PRESENT==1)
		extern float hum_value, temp_value;
	#if (CCS811)
		CCS811_SetEnvironmentalData(hum_value, temp_value);
//		CCS811_SetEnvironmentalData(65.0, 25.0);
	#elif (ENS160)
		ENS160_SetEnvironmentalData(hum_value, temp_value);
//		ENS160_SetEnvironmentalData(65.0, 25.0);
	#endif	//CCS811
	#endif	//HUMIDITY_SENSOR_PRESENT
		if (WarmUpPeriod_expired)
		{
	#if (CCS811)
			CCS811_status = CCS811_Get_Measurement(&VOC_Values);
	#elif (ENS160)
			ENS160_status = ENS160_Get_Measurement(&VOC_Values);
			ENS160_status = ENS160_Get_Raw_Data(&VOC_Values);
	#endif
		}
	lcl_voc_data_rdy = true;
	send_lcl_voc_data = true;
	display_voc_data = StartDataStrmng;
	#if (CCS811)
	if ((WarmUpPeriod_expired) && (CCS811_Save_Baseline_Reserved))
	{
		CCS811_Save_Baseline(true);
		CCS811_Save_Baseline_Reserved = false;		//From TestEnv function
	}
	#endif
#endif
#if (PARTICULATE_SENSOR_PRESENT==1)
	SPS30_status = SPS30_Get_Measurement(&PMS_Values);
	lcl_pms_data_rdy = true;
	send_lcl_pms_data = true;
	display_pms_data = StartDataStrmng;
#endif
#if (GAS_SENSOR_MODULE_PRESENT==1)
	if (WarmUpPeriod_expired)
	{
		/*
		 * After the WarmUp period of the analog sensors the "read_analogs()" function is called
		 * inside the "ANLG_Get_Measurement(&GAS_Values)" function
		 */
		ANLG_status = ANLG_Get_Measurement(&GAS_Values);
	} else
	{
		/*
		 * During the wait, the analog inputs are still read to allow the PB IIR filters
		 * inserted on the analog inputs to stabilize the reading of the sensors analog values.
		 */
		if(!Test_Mode)
			read_analogs();
	}
	lcl_gas_data_rdy = true;
	send_lcl_gas_data = true;
	display_gas_data = StartDataStrmng;
#endif
#if ((BLE_SUPPORT) && (BEACON_APP))
	if ((MidNight) && !(MinMaxStored))
	{
		StoreMinMax(&PRS_Values, &HUM_Values, &GAS_Values, &VOC_Values, &PMS_Values);
		MinMaxStored = true;
		if (Restart_Reverved)
		{
			NVIC_SystemReset();
		}
	} else
	if (!(MidNight) && (MinMaxStored))
	{
		MinMaxStored = false;
	}
#endif
/*
 * In the "Sensor" application the button_manage() function, which activates the BLE
 * and sensor functions, is done after 5s the programming of the IMU, otherwise the
 * MotionFX algorithm does not work. I didn't understand why, but that's it!
 */
#if ((BLE_SUPPORT) && (SENSOR_APP))
	if(!button_manage_done)
	{
		button_manage();
		button_manage_done = true;
	}
#endif
}

/* @fn		Function: process_timer7_irq(void)
 * @brief	Timer7 Interrupt Handler; Executed when Timer7 expires (every 100ms).
 */
void process_timer7_irq(void)
{
	static uint16_t ServiceTimer = 0;
	static bool LedPC7_Blinking = false;

	HAL_TIM_Base_Start_IT(&htim7);			//Restart Timer

	if (RiskReport)
	{
		LedPC7_Blinking = true;
		ServiceTimer++;
		if (ServiceTimer >= 3)
		{
			ServiceTimer = 0;
			led_toggle(LED_PC7);
		}
	} else
	if (LedPC7_Blinking)
	{
		led_on(LED_PC7);
		LedPC7_Blinking = false;
	}
	//ServiceTimer1 management: push button debounce timer
	if (ServiceTimer1.Start)
	{
		ServiceTimer1.Counter++;
		if (ServiceTimer1.Counter >= ServiceTimer1.TimeOut)
		{
			ServiceTimer1.Expired = true;
			ServiceTimer1.Start = false;
			ServiceTimer1.Counter = 0;
		}
	}
	//ServiceTimer2 management: HTS221 heather timeout
	if (ServiceTimer2.Start)
	{
		ServiceTimer2.Counter++;
		if (ServiceTimer2.Counter >= ServiceTimer2.TimeOut)
		{
			ServiceTimer2.Expired = true;
			ServiceTimer2.Start = false;
			ServiceTimer2.Counter = 0;
#if (HTS221)
			HTS221_status = HTS221_Set_HeaterState(HTS221_BADDR, HTS221_DISABLE);
#endif
		}
	}
	//ServiceTimer3 management: HTS221 waiting time between two heater activations
	if (ServiceTimer3.Start)
	{
		ServiceTimer3.Counter++;
		if (ServiceTimer3.Counter >= ServiceTimer3.TimeOut)
		{
			ServiceTimer3.Expired = true;
			ServiceTimer3.Start = false;
			ServiceTimer3.Counter = 0;
		}
	}
	//ServiceTimer4 management: HTS221 waiting time to measures restart after the heater activation
	if (ServiceTimer4.Start)
	{
		ServiceTimer4.Counter++;
		if (ServiceTimer4.Counter >= ServiceTimer4.TimeOut)
		{
			ServiceTimer4.Expired = true;
			ServiceTimer4.Start = false;
			ServiceTimer4.Counter = 0;
		}
	}

	HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

/* @fn		Function: process_IO_Expander_irq(void)
 * @brief	PC2 Interrupt Handler; Called by one of the MC23017 IO Expanders when an input line changes.
 * 			Active on Falling Edge
 */
void process_IO_Expander_irq(void)
{
#if (IO_EXP_PRESENT==1)
	static uint16_t i = 0;

	mcp23017_read_registers(MCP23017_MASTER1_BADDR, 0, 22);	//Read all MC23017_1 Registers
    while (!I2C_done)
	{
    	Sleep(1);
    	if (++i > 100)
    	{
    		I2C_done = true;
    		i= 0;
    	}
	}
// 	HAL_Delay(i2c_delay);
 	mcp23017_read_registers(MCP23017_MASTER2_BADDR, 0, 22);	//Read all MC23017_2 Registers
	while (!I2C_done)
	{
    	Sleep(1);
    	if (++i > 100)
    	{
    		I2C_done = true;
    		i= 0;
    	}
	}
// 	HAL_Delay(i2c_delay);
//  Combine four 8-bit unsigned ints into one 32-bit unsigned int
// 	io_exp_intcap = ((intcap2_b << 24) | (intcap2_a << 16) | (intcap1_b << 8) | (intcap1_a));
 	io_exp_intcap = ((intcap2_b << 8) | (intcap1_b));
// 	io_exp_gpio = ((gpio2_b << 24) | (gpio2_a << 16) | (gpio1_b << 8) | (gpio1_a));
 	io_exp_gpio = ((gpio2_b << 8) | (gpio1_b));
	input_changed = true;
	refresh = true;
 // Format Display String
	if (Test_Mode)
	{
		for (uint8_t i=0; i <= num_digital_in; i++)
		{
			if (!BIT_CHECK(io_exp_gpio,i))
			{
				L20_menu_items_row7[i*3+3]=0x30;
			}
			else
			{
				L20_menu_items_row7[i*3+3]=0x31;
			}
		}
		CDC_Transmit_FS((uint8_t*)L20_menu_items_row7,strlen((const char*)L20_menu_items_row7));
		HAL_Delay(CDC_delay);
		CDC_Transmit_FS((uint8_t*)L20_menu_items_row4,strlen((const char*)L20_menu_items_row4));
	}

	HAL_NVIC_EnableIRQ(EXTI2_IRQn);		//It was be disabled in the HAL_GPIO_EXTI_Callback() function
#endif
}
#if (CAN_SUPPORT==1)
/**
 * @Function: process_CAN_RX_irq(CAN_HandleTypeDef* hcan, uint8_t* Buf_Head, uint8_t* Buf_Data, uint32_t Len_Head, uint32_t Len_Data)
 * @brief  Rx Fifo 0 message pending ISR
 *         Data received over CAN endpoint are buffered through this function
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: CAND_OK if all operations are OK else CAND_FAIL
 */
uint8_t process_CAN_RX_irq(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef Buf_Head, uint8_t* Buf_Data, uint32_t Len_Head, uint32_t Len_Data)
{
	/* [CAN_HOST] =LongData==> [CAN_DEVICE CANRX_IRQ (me)]===> UserRxBufferFS ===> process_CAN_RX_irq(Buf,Len) ===> app.canbuf
	 *                                                      ^                                        |                  |
	 *                                                      |                                        |                  |
	 *                                                      +-------<-------<--------<--------<------+                  +->signalCanRx
	 * */
	int tmp1 = sizeof(can1app.canbuf) - can1app.canlen;
	int tmp2 = sizeof(can2app.canbuf) - can2app.canlen;

	if(hcan->Instance==CAN1)
	{
		if(tmp1 > 0)
		{
			tmp1 = MIN(Len_Head , tmp1);
			memcpy(&can1app.canbuf[can1app.canlen], (void *)&Buf_Head, tmp1);	//we need intermediate buffer can1app.canbuf to receive long CAN packet
			can1app.canlen +=tmp1;
		}
		tmp1 = sizeof(can1app.canbuf) - can1app.canlen;
		if(tmp1 > 0)
		{
			tmp1 = MIN(Len_Data , tmp1);
			memcpy(&can1app.canbuf[can1app.canlen], Buf_Data, tmp1);
			can1app.canlen +=tmp1;
		}
	}
	else if(hcan->Instance==CAN2)
	{
		if(tmp2 > 0)
		{
			tmp2 = MIN(Len_Head , tmp2);
			memcpy(&can1app.canbuf[can1app.canlen], (void *)&Buf_Head, tmp2);	//we need intermediate buffer can2app.canbuf to receive long CAN packet
			can2app.canlen +=tmp2;												//inside ISR.
		}
		tmp2 = sizeof(can2app.canbuf) - can2app.canlen;
		if(tmp2 > 0)
		{
			tmp2 = MIN(Len_Data , tmp2);
			memcpy(&can2app.canbuf[can2app.canlen], Buf_Data, tmp2);
			can2app.canlen +=tmp2;
		}
	}
	return (CAND_OK);
}

/**
 * @Function: process_CAN_TX_irq(CAN_HandleTypeDef* hcan)
 * @brief  Tx Maibox x ISR
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval Result of the operation: CAND_OK if all operations are OK else CAND_FAIL
 */
uint8_t process_CAN_TX_irq(CAN_HandleTypeDef* hcan)
{
	static uint16_t count = 0;

	can_tx_done = true;
	count++;
	return (CAND_OK);
}
#endif	//CAN_SUPPORT==1

/**
 * @Function: process_ADC_irq(ADC_HandleTypeDef* hcan)
 * @brief  ADCx Conversion end ISR
 * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval Result of the operation: ADCD_OK if all operations are OK else ADCD_FAIL
 */
uint8_t process_ADC_irq(ADC_HandleTypeDef* hadc)
{
#if (GAS_SENSOR_MODULE_PRESENT==1)
	memcpy((void*)&adc_values, (void*)&ADCxConvertedValue, sizeof(ADCxConvertedValue));	// Extract ADC values from buffer
	conversion_ended = true;
#endif

	return (ADCD_OK);
}

#if (USART_SUPPORT==1)
/**
 * @Function: process_USART_RX_irq(UART_HandleTypeDef* huart, uint8_t* Buf, uint32_t Len)
 * @brief  Rx Fifo 0 message pending ISR Data received over USARTx endpoint are buffered through this function
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains the configuration information for the specified USART.
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USARD_OK if all operations are OK else USARTD_FAIL
 */
uint8_t process_USART_RX_irq(UART_HandleTypeDef* huart, uint8_t* Buf, uint32_t Len)
{
	/* [USART_HOST] =LongData==> [USART_DEVICE USARTRX_IRQ (me)]===> aRxBuffer ===> process_USARTx_RX_irq(Buf,Len) ===> app.usartbuf
	 *                                                            ^                                      |                  |
	 *                                                            |                                      |                  |
	 *                                                            +------------<------------<------------+                  +->signalUsbUartRx
	 * */
	int tmp1 = sizeof(usart2app.usartbuf) - usart2app.usartlen;
	int tmp2 = sizeof(usart3app.usartbuf) - usart3app.usartlen;

	if(huart->Instance==USART2)
	{
		if(tmp1 > 0)
		{
			tmp1 = MIN(Len , tmp1);											//BUFFERSIZE is the minimum amount of characters receivable in Non-Blocking-Mode
			memcpy(&usart2app.usartbuf[usart2app.usartlen], (void *)&Buf[0], tmp1);	//we need intermediate buffer usart2app.usartbuf to receive long USART packet
			usart2app.usartlen +=tmp1;												//inside ISR.
			memset(aRxBuffer, 0x00, RXBUFFERSIZE);
		}
	}
	if(huart->Instance==USART3)
	{
		if(tmp2 > 0)
		{
			tmp2 = MIN(Len , tmp2);											//BUFFERSIZE is the minimum amount of characters receivable in Non-Blocking-Mode
			memcpy(&usart3app.usartbuf[usart3app.usartlen], (void *)&Buf[0], tmp2);	//we need intermediate buffer usart3app.usartbuf to receive long USART packet
			usart3app.usartlen +=tmp2;												//inside ISR.
			memset(aRxBuffer, 0x00, RXBUFFERSIZE);
		}
	}

	return (USARTD_OK);
}

/**
 * @Function: process_USART_TX_irq((UART_HandleTypeDef *UartHandle)
 * @brief  Tx Mailbox x ISR
 * @param  UartHandle: pointer to a UART_HandleTypeDef structure that contains
 *         the configuration information for the specified USART.
 * @retval Result of the operation: USARTD_OK if all operations are OK else USARTD_FAIL
 */
uint8_t process_USART_TX_irq(UART_HandleTypeDef *UartHandle)
{
	return (USARTD_OK);
}
#endif

/****************************************************************************//**
 * 								END OF IRQ section
 *******************************************************************************/
/* @fn		usb_ready
 * @brief 	return 1 if USB driver ready to output
 * */
int usb_ready(void)
{
	extern USBD_HandleTypeDef  hUsbDeviceFS;

	if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
	{
		return 0;
	}

	return 1;
}

/* @fn		usart2_ready
 * @brief 	return 1 if USART driver ready to output
 * */
int usart_ready(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance==USART2)
	{
	/*	if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
		{
			return 0;
		}
	*/
	}
	else if(UartHandle->Instance==USART3)
	{
	/*	if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
		{
			return 0;
		}
	*/
	}

	return 1;
}

/****************************************************************************//**
 * 			           Circular Buffer management section
 *******************************************************************************/
/* @fn		Push RX msg into the circular buffer and increase circular buffer length
 * 			i.e. from independent high priority thread
 * */
/*uint8_t circBufPush(circBuf_t* c, uint8_t* data, uint16_t Len)
{
    // next is where head will point to after this write.
	uint32_t next = c->head + Len;
	__HAL_LOCK(&txhandle);	//"return HAL_BUSY;" if locked

    if (next >= c->maxLen)
        next = 0;

    if (next == c->tail) 						// check if circular buffer is full
    {
		__HAL_UNLOCK(&txhandle);				// if packet can not fit, setup RX Buffer overflow ERROR and exit
        return HAL_ERROR;       				// and return with an error.
    }

    memcpy(&c->buf[c->head], &data[0], Len);	// Load data and then move.
//  c->buf[c->head] = data; 					// Load data and then move
    c->head = next;								// head to next data offset.

	__HAL_UNLOCK(&txhandle);
    return HAL_OK;  							// return success to indicate successful push.
} */

/* @fn		Pop RX msg from the circular buffer and decrease circular buffer length
 * 			i.e. from independent high priority thread
 * */
/*uint8_t circBufPop(circBuf_t* c, uint8_t* data, uint16_t Len)
{
	__HAL_LOCK(&txhandle);						//"return HAL_BUSY;" if locked

    // if the head isn't ahead of the tail, we don't have any characters
    if (c->head == c->tail) 					// check if circular buffer is empty
    {
		__HAL_UNLOCK(&txhandle);				// setup RX Buffer empty ERROR and exit
		return HAL_ERROR;          				// and return with an error
    }

    // next is where tail will point to after this read.
    int next = c->tail + Len;
    if(next >= c->maxLen)
        next = 0;
    memcpy(&data[0], &c->buf[c->tail], Len);	// Load data and then move.
//  *data = c->buf[c->tail]; 					// Read data and then move
    c->tail = next;         					// tail to next data offset.

    __HAL_UNLOCK(&txhandle);
    return HAL_OK;  							// return success to indicate successful push.
} */
