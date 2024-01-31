/*
 * TestEnv.c
 *
 *  Created on: 17 nov 2020
 *  Author: Tommaso Sabatini
 */

#pragma GCC optimize ("Os")

#include "platform/port.h"
#include "application/TestEnv.h"
#include "application/MEMS_app.h"

const uint8_t mystring0[] ="\r\n\nALGOL TSXX - Personal Air Monitoring Station";
const uint8_t mystring1[] ="Premere il pulsante per piu' di 5s";
const uint8_t mystring2[] =" per attivare i menu' di test...\r\n";
uint8_t mystring3[8];
const uint8_t mystring4[] = "\r\n\n  ---- Bye!! ----\r\n\n";
const uint8_t mystring5[] = "\r\n\n  Are you sure you want to update the FW? [N/y]";
const uint8_t mystring6[] = "\r\n\n  Close the terminal, connect the programming cable and start the programming tool on the PC...";
#if (PRESSURE_SENSOR_PRESENT==1)
	#if (LPS25HB)
		const uint8_t mystring9a[] = "\r\n\n  The current absolute pressure read by the LPS25HB sensor is:";
	#elif (LPS22HB)
		const uint8_t mystring9a[] = "\r\n\n  The current absolute pressure read by the LPS22HB sensor is:";
	#endif
	const uint8_t mystring9b[] = "  Enter the reference absolute pressure in mbar in XXXX.X format: ";
	const uint8_t mystring9c[] = "\r\n\n  The current altitude is:";
	const uint8_t mystring9d[] = "  Enter the new altitude in meters (meters above the sea level): ";
#endif
#if (HUMIDITY_SENSOR_PRESENT==1)
	const uint8_t mystring10a[] = "\r\n\n  The current humidity read by the HTS221 sensor is:";
	const uint8_t mystring10b[] = "  Enter the reference hum. in % RH: ";
#endif
#if ((PRESSURE_SENSOR_PRESENT==1) || (HUMIDITY_SENSOR_PRESENT==1))
	#if (HUMIDITY_SENSOR_PRESENT==1)
		#if (HTS221)
			const uint8_t mystring8a[] = "\r\n\n  The current temperature read by the HTS221 sensor is:";
		#elif (SHT4x)
			const uint8_t mystring8a[] = "\r\n\n  The current temperature read by the SHT4x sensor is:";
		#endif
	#else
		#if (LPS25HB)
			const uint8_t mystring8a[] = "\r\n\n  The current temperature read by the LPS25HB sensor is:";
		#elif (LPS22HB)
			const uint8_t mystring8a[] = "\r\n\n  The current temperature read by the LPS22HB sensor is:";
		#endif
	#endif
	const uint8_t mystring8b[] = "  Enter the reference temp. in °C in XX.X or -XX.X format: ";
#endif
const uint8_t mystring8c[] = "\r\n\n  ** Restart the board to make the changes effective! **";
#if ((VOC_SENSOR_PRESENT) && (CCS811))
	const uint8_t mystring11a[] = "\r\n\n  The current CCS811 Ro is:";
	const uint8_t mystring11b[] = "  The stored CCS811 Ro is:";
	const uint8_t mystring11c[] = "";
	const uint8_t mystring11d[] = "  Enter \"1\" to reserve the baseline save: ";
	const uint8_t mystring11e[] = "\r\n\n  CCS811 current Ro will be saved after the 30 min. burn-in period";
#endif
#if (GAS_SENSOR_MODULE_PRESENT==1)
	const uint8_t mystring12a[] = "  The current CO read by the MiCS-6814 sensor is:";
	//const uint8_t mystring12b[] = "  Enter the reference CO in mg/m3: ";
	const uint8_t mystring12b[] = "  Enter the reference CO in mV/10: ";
	const uint8_t mystring12c[] = "\r\n\n  The current MiCS-6814 Ro_CO is:";
	const uint8_t mystring12d[] = "  Enter the reference Ro_CO in ohm: ";
	const uint8_t mystring12e[] = "\r\n\n  The read value of Rs is:";
	const uint8_t mystring12f[] = "  If you are in clean air use this value as reference value.";
	const uint8_t mystring12g[] = "\r\n\n  The read value of Rf_CO is:";
	const uint8_t mystring12h[] = "  Enter the new Rf in ohm: ";
	const uint8_t mystring14a[] = "  The current NO2 read by the MiCS-6814 sensor is:";
	//const uint8_t mystring14b[] = "  Enter the reference NO2 in ug/m3: ";
	const uint8_t mystring14b[] = "  Enter the reference NO2 in mV/10: ";
	const uint8_t mystring14c[] = "\r\n\n  The current MiCS-6814 Ro_NO2 is:";
	const uint8_t mystring14d[] = "  Enter the reference Ro_NO2 in ohm: ";
	const uint8_t mystring14e[] = "\r\n\n  The read value of Rf_NO2 is:";
	const uint8_t mystring15a[] = "  The current NH3 read by the MiCS-6814 sensor is:";
	//const uint8_t mystring15b[] = "  Enter the reference NH3 in ug/m3: ";
	const uint8_t mystring15b[] = "  Enter the reference NH3 in mV/10: ";
	const uint8_t mystring15c[] = "\r\n\n  The current MiCS-6814 Ro_NH3 is:";
	const uint8_t mystring15d[] = "  Enter the reference Ro_NH3 in ohm: ";
	const uint8_t mystring15e[] = "\r\n\n  The read value of Rf_NH3 is:";
	//const uint8_t mystring13b[] = "  Enter the reference CH2O in ug/m3: ";
	const uint8_t mystring13b[] = "  Enter the reference CH2O in mV: ";
	#if !(CH2O_FROM_EC)
		const uint8_t mystring13c[] = "\r\n\n  The current SMD1001 Vo_CH2O is:";
		const uint8_t mystring13d[] = "  Enter the reference Vo_CH2O in mVolts: ";
		const uint8_t mystring13a[] = "  The current CH2O read by the SMD1001-CH2O sensor is:";
		const uint8_t mystring13f[] = "\r\n\n  The read value of Vs is:";
	#else
		const uint8_t mystring13a[] = "  The current CH2O read by the ZE8-CH2O sensor is:";
	#endif
	#if (OUTDOOR_MODE)
		const uint8_t mystring16a[] = "  The current O3 read by the ZE25-O3 sensor is:";
		//const uint8_t mystring16b[] = "  Enter the reference O3 in ug/m3: ";
		const uint8_t mystring16b[] = "  Enter the reference O3 in mV: ";
		const uint8_t mystring17a[] = "  The current SO2 read by the ME4-SO2 sensor is:";
		//const uint8_t mystring17b[] = "  Enter the reference SO2 in ug/m3: ";
		const uint8_t mystring17b[] = "  Enter the reference SO2 in mV: ";
		const uint8_t mystring18a[] = "  The current C6H6 read by the ME4-C6H6 sensor is:";
		//const uint8_t mystring18b[] = "  Enter the reference C6H6 in ug/m3: ";
		const uint8_t mystring18b[] = "  Enter the reference C6H6 in mV: ";
	#endif	//OUTDOOR_MODE
	const uint8_t mystring18c[] = "\r\n\n  The current reference is:";
#endif	//GAS_SENSOR_MODULE_PRESENT
const uint8_t top_menu_items_row1[]="\r\n\n+-------------------+\r\n";
const uint8_t top_menu_items_row2[]="|  SERVICE CONSOLE  |\r\n";
const uint8_t top_menu_items_row3[]="+-------------------+\r\n";
const uint8_t L10_menu_items_row1[]="\r\n\n+---------------------+\r\n";
const uint8_t L10_menu_items_row3[]="+---------------------+\r\n";
const uint8_t L10_menu_items_row5[]="|   ESC.: MAIN MENU   |\r\n";
#if	(OUTPUT_TEST==1)
	const uint8_t top_menu_items_row4[]="| OUTPUTS TEST: [1] |\r\n";
	const uint8_t L10_menu_items_row2[]="|     OUTPUTS TEST    |\r\n";
	const uint8_t L10_menu_items_row4[]="|   BANK....: [1..6]  |\r\n";

	const uint8_t L11_menu_items_row2[]="|    BANK 'X' TEST    |\r\n";
	const uint8_t L11_menu_items_row4[]="|   OUT....: [1..6]   |\r\n";
	const uint8_t L11_menu_items_row5[]="| ESC.: OUT TEST MENU |\r\n";

	const uint8_t L12_menu_items_row2[]="| B.'X' SWITCH n TEST |\r\n";
	const uint8_t L12_menu_items_row4[]="|   OUT n ON....: 1   |\r\n";
	const uint8_t L12_menu_items_row5[]="|   OUT n OFF...: 0   |\r\n";
	const uint8_t L12_menu_items_row6[]="| ESC.: BANK 'A' TEST |\r\n";
#endif
#if	(INPUT_TEST==1)
	const uint8_t top_menu_items_row5[]="| INPUTS TEST.: [2] |\r\n";
	const uint8_t L20_menu_items_row1[]="\r\n\n+-------------------------------------------------------+\r\n";
	const uint8_t L20_menu_items_row2[]="|                  DIGITAL INPUTS TEST                  |\r\n";
	const uint8_t L20_menu_items_row3[]="|                    ESC.: MAIN MENU                    |\r\n";
	const uint8_t L20_menu_items_row4[]="+-------------------------------------------------------+\r\n";
	const uint8_t L20_menu_items_row5[]="| 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 |\r\n";
	const uint8_t L20_menu_items_row6[]="|                                                       |\r\n";
	uint8_t L20_menu_items_row7[]="|                                                       |\r\n";
#endif
#if	(CAN1_TEST==1)
	const uint8_t top_menu_items_row6[]="| CAN_1 TEST..: [3] |\r\n";
	const uint8_t L30_menu_items_row2[]="|     CAN '1' TEST    |\r\n";
#endif
#if	(CAN2_TEST==1)
	const uint8_t top_menu_items_row7[]="| CAN_2 TEST..: [4] |\r\n";
	const uint8_t L40_menu_items_row2[]="|     CAN '2' TEST    |\r\n";
#endif
#if	(ANALOG_TEST==1)
	const uint8_t top_menu_items_row8[]="| ANALOG TEST.: [5] |\r\n";
	const uint8_t L50_menu_items_row5a[]="| 01    02    03    04    05    06    07    08  "  ;
	const uint8_t L50_menu_items_row5b[]="  09    10    11    12    13    14    15    16    |\r\n";
	const uint8_t L50_menu_items_row6a[]="|                                                ";
	const uint8_t L50_menu_items_row6b[]="                                                 |\r\n";
	const uint8_t L50_menu_items_row2a[]="|                                       ANALOG IN";
	const uint8_t L50_menu_items_row2b[]="PUTS TEST                                        |\r\n";
	uint8_t L50_menu_items_row7a[]="| 0.000 0.000 0.000 0.000 0.000 0.000 0.000 0.000";
	uint8_t L50_menu_items_row7b[]=" 0.000 0.000 0.000 0.000 0.000 0.000 0.000 0.000 |\r\n";
#endif
const uint8_t top_menu_items_row9[]="  SELECT......: ";
const uint8_t top_menu_items_row10[]="| FW UPDATE...: [6] |\r\n";
#if (BLE_SUPPORT==1)
	const uint8_t top_menu_items_row12[]="| BLE MAC Addr: [7] |\r\n";
	const uint8_t mystring21a[] = "\r\n\n  The current BLE MAC address is:";
	const uint8_t mystring21b[] = "  Enter the new MAC address last 3 bytes (NIC), in hex format and without the colon: ";
#endif
#if (UTILITIES==1)
	const uint8_t top_menu_items_row13[]="| UTILITIES...: [8] |\r\n";
	const uint8_t L80_menu_items_row2[]= "|   BOARD UTILITIES   |\r\n";
	#if (SET_DATE_TIME==1)
		const uint8_t L80_menu_items_row4[]= "| SET DATE/TIME...: 1 |\r\n";
		const uint8_t mystring7[] = "\r\n\n  Open the 'RTC_SetUp.ttl' macro...\r\n\n";
	#endif
	#if ((PRESSURE_SENSOR_PRESENT==1) || (HUMIDITY_SENSOR_PRESENT==1))
		const uint8_t L80_menu_items_row5[]= "| T SENSOR CALIB..: 2 |\r\n";
	#endif
	#if (PRESSURE_SENSOR_PRESENT==1)
		const uint8_t L80_menu_items_row6[]= "| P SENSOR CALIB..: 3 |\r\n";
	#endif
	#if (HUMIDITY_SENSOR_PRESENT==1)
		const uint8_t L80_menu_items_row7[]= "| RH SENSOR CALIB.: 4 |\r\n";
	#endif
	#if ((VOC_SENSOR_PRESENT) && (CCS811))
		const uint8_t L80_menu_items_row8[]= "| VOC SENSOR CALIB: 5 |\r\n";
	#endif
	#if (GAS_SENSOR_MODULE_PRESENT==1)
		const uint8_t L80_menu_items_row9[]= "| CO SENSOR CALIB.: 6 |\r\n";
		const uint8_t L80_menu_items_row10[]="| CH2O SENSOR CAL.: 7 |\r\n";
		const uint8_t L80_menu_items_row11[]="| NO2 SENSOR CALIB: 8 |\r\n";
		const uint8_t L80_menu_items_row12[]="| NH3 SENSOR CALIB: 9 |\r\n";
		#if (OUTDOOR_MODE)
			const uint8_t L80_menu_items_row13[]="| O3 SENSOR CALIB.: A |\r\n";
			const uint8_t L80_menu_items_row14[]="| SO2 SENSOR CALIB: B |\r\n";
			const uint8_t L80_menu_items_row15[]="| C6H6 SENSOR CAL.: C |\r\n";
		#endif	//OUTDOOR_MODE
	#endif	//GAS_SENSOR_MODULE_PRESENT
#endif	//UTILITIES==1
const uint8_t L50_menu_items_row1a[]="\r\n\n+------------------------------------------------";
const uint8_t L50_menu_items_row1b[]="-------------------------------------------------+\r\n";
const uint8_t L50_menu_items_row3a[]="|                                         ESC.";
const uint8_t L50_menu_items_row3b[]=": MAIN MENU                                         |\r\n";
uint8_t L50_menu_items_row4a[]="+------------------------------------------------";
uint8_t L50_menu_items_row4b[]="-------------------------------------------------+\r\n";
#if (RTC_CALIB==1)
	const uint8_t L80_menu_items_row20[]="| RTC CALIB.......: D |\r\n";
#endif
#if (USE_BKUP_SRAM==1)
	const uint8_t L80_menu_items_row23[]="| CLEAR BKUP_SRAM.: G |\r\n";
	const uint8_t mystring22a[] = "\r\n\n  Warning!!!\r\n  The internal BackUp RAM will be deleted at the next restart!";
	const uint8_t mystring22b[] = "\r\n  Are you sure? [N/y]";
	const uint8_t mystring22c[] = "\r\n  *** The internal BackUp RAM will be deleted at the next restart ***";
	const uint8_t mystring22d[] = "\r\n  *** The internal BackUp RAM will NOT be deleted at the next restart ***";
#endif
#if (UWB_MODE==1)
	const uint8_t L80_menu_items_row21[]="| SET OPER. MODE..: E |\r\n";
	const uint8_t L80_menu_items_row22[]="| SET NODE ROLE...: F |\r\n";
	const uint8_t mystring19a[] = "\r\n\n  The current operational mode is: ";
	#if (RTLS_FW==1)
		const uint8_t mystring19b[] = "  Enter the new op mode (R=RNG; L=RTLS; D=DATA): ";
		const uint8_t mystring20b[] = "  Enter the new node role in xyzw format, where:\r\n"\
									  "  x = t (TAG); x = a (ANCHOR).\r\n"\
									  "  y = 0..7 for TAG; y = 0..3 for ANCHOR.\r\n"\
									  "  z = L (Long Range, 110Kb/s); z = s (Short Range, 6.8Mb/s).\r\n"\
									  "  w = 2 (3.993GHz Band); w = 5 (6.489GHz Band): ";
	#elif (RANGING_FW==1)
		const uint8_t mystring19b[] = "  Enter the new op mode (R=RNG; D=DATA): ";
		const uint8_t mystring20b[] = "  Enter the new node role in xyz format, where:\r\n"\
									  "  x = t (TAG); x = a (ANCHOR).\r\n" \
		  	  	  	  	  	  	  	  "  y = L (Long Range, 110Kb/s); y = s (Short Range, 6.8Mb/s).\r\n"\
									  "  z = 2 (3.993GHz Band); z = 5 (6.489GHz Band): ";
	#endif
	const uint8_t mystring20a[] = "\r\n\n  The current node role is: ";
#elif (PWR_NODE_MODE==1)
	const uint8_t L80_menu_items_row21[]="| SET OPER. MODE..: E |\r\n";
	const uint8_t mystring19a[] = "\r\n\n  The current operational mode is: ";
	const uint8_t mystring19b[] = "  Enter the new op mode (E=ENAV; M=MLRS): ";
#endif
const uint8_t mystring_w1[] = "\r\n\n  ** Writing in flash...";

uint8_t sel, sel1;
uint16_t output = 0;
bool FirstTime = true;
uint8_t j = 0;
bool point = false; bool numb = false;

void CDC_Tx_FS(const uint8_t *Buffer, uint16_t Length)
{
	CDC_Transmit_FS((uint8_t*)Buffer, Length);
   	HAL_Delay(CDC_delay);
}

void DisplayAnalogValues()
{
#if	(ANALOG_TEST==1)
	CDC_Tx_FS(L50_menu_items_row7a,strlen((const char*)L50_menu_items_row7a));
	CDC_Tx_FS(L50_menu_items_row7b,strlen((const char*)L50_menu_items_row7b));
	CDC_Tx_FS(L50_menu_items_row4a,strlen((const char*)L50_menu_items_row4a));
	CDC_Tx_FS(L50_menu_items_row4b,strlen((const char*)L50_menu_items_row4b));
#endif
}

#pragma GCC optimize ("Os")
void GetNumericString(bool nopoint, bool nominus)
{
	char Char = 0;

	memset(mystring3, 0, sizeof(mystring3));
	do
	{
		Char = (char)app.usbbuf[app.usblen-1];
		switch (Char)
		{
			case 0x41 ... 0x5A:
			case 0x61 ... 0x7A:
			{
				mystring3[j] = app.usbbuf[app.usblen-1];
				app.usbbuf[app.usblen-1] = 0;
				CDC_Tx_FS(&mystring3[j], 1);
				j++;
				numb = false;
			}
			break;
			case 0x30 ... 0x39:
			{
				mystring3[j] = app.usbbuf[app.usblen-1];
				app.usbbuf[app.usblen-1] = 0;
				CDC_Tx_FS(&mystring3[j], 1);
				j++;
				numb = true;
			}
			break;
			case 0x2E:	// '.'
			{
				if ((j > 0) && (!point) && (numb) && (nopoint))
				{
					mystring3[j] = app.usbbuf[app.usblen-1];
					app.usbbuf[app.usblen-1] = 0;
					CDC_Tx_FS(&mystring3[j], 1);
					j++;
					point = true;
				}
			}
			break;
			case 0x2D:	// '-'
			{
				if ((j == 0) && (nominus))
				{
					mystring3[j] = app.usbbuf[app.usblen-1];
					app.usbbuf[app.usblen-1] = 0;
					CDC_Tx_FS(&mystring3[j], 1);
					j++;
				}
			}
			break;
			case 0x08:	// 'BackSpace'
			{
				CDC_Tx_FS(&app.usbbuf[app.usblen-1], 1);
				app.usbbuf[app.usblen-1] = ' ';
				CDC_Tx_FS(&app.usbbuf[app.usblen-1], 1);
				app.usbbuf[app.usblen-1] = 0x08;
				CDC_Tx_FS(&app.usbbuf[app.usblen-1], 1);
				app.usbbuf[app.usblen-1] = 0;
				j--;
				if (mystring3[j] == '.')
					point = false;
				mystring3[j] = 0;
			}
			break;
			case 0x0A:	// 'NewLine (/n)'
			case 0x0D:	// 'CarriageReturn (/r)'
			case 0x1B:	// 'Escape'
			{
				app.usbbuf[app.usblen-1] = 0x04;
			}
			break;
			default:
				// do nothing for undefined choices
				break;
		}
	} while ((app.usbbuf[app.usblen-1] != 0x04) && (app.usbbuf[app.usblen-1] != 0x1B));

	j = 0; point = false; numb = false;
}

#pragma GCC optimize ("Os")
/* @fn 		PrintNumHeader()
 * @brief 	Displays numerical messages for sensor calibration
 * @param   msg1: Pointer to the first message to be displayed
 * @param   msg2: Pointer to the second message to be displayed
 * @param	fval: Read value of the sensor to be converted into a string
 * @param	Float: If true, the sensor value is a float type
 * @param	meas_unit: Unit of measure to display
 * @return	null
 * */
void PrintNumHeader(const uint8_t *msg1, const uint8_t *msg2, float32_t fval, bool Float, char meas_unit[])
{
	uint8_t ascii_val[32];

	CDC_Tx_FS((uint8_t*)msg1,strlen((const char*)msg1));
	if (Float)
		sprintf((char*)&ascii_val[0], " %.1f%s\r\n", fval, meas_unit);
	else
		sprintf((char*)&ascii_val[0], " %ld%s\r\n", (int32_t)fval, meas_unit);
	CDC_Tx_FS((uint8_t*)ascii_val,strlen((const char*)ascii_val));
	CDC_Tx_FS((uint8_t*)msg2,strlen((const char*)msg2));
}

#pragma GCC optimize ("Os")
/* @fn 		PrintTextHeader()
 * @brief 	Displays text messages for settings
 * @param   msg1: Pointer to the first message to be displayed
 * @param   msg2: Pointer to the second message to be displayed
 * @param	ascii_val: Pointer to the set string value
 * @param	meas_unit: Unit of measure to display
 * @return	null
 * */
void PrintTextHeader(const uint8_t *msg1, const uint8_t *msg2, uint8_t *ascii_val, char meas_unit[])
{
	CDC_Tx_FS((uint8_t*)msg1,strlen((const char*)msg1));
	if (ascii_val != NULL)
	{
		strcat((char*)ascii_val, "\r\n");
		CDC_Tx_FS((uint8_t*)ascii_val,strlen((const char*)ascii_val));
	}
	CDC_Tx_FS((uint8_t*)msg2,strlen((const char*)msg2));
}

#pragma GCC optimize ("Os")
/* @fn 		top_menu()
 * @brief 	Display and select the top menu items (TOP MENU)
 * @return	null
 * */
void top_menu()
{
	extern uint8_t Version[];

	if (app.usblen)
		app.usbbuf[app.usblen-1] = 0;
	leds_test = true;
	FirstTime = false;
	Test_Mode = true;
	led_pc6_timer = 0;

	CDC_Tx_FS((uint8_t*)mystring0,strlen((const char*)mystring0));	//Transmit welcome string
	CDC_Tx_FS((uint8_t*)Version,strlen((const char*)Version));		//Transmit version string
	CDC_Tx_FS((uint8_t*)top_menu_items_row1,strlen((const char*)top_menu_items_row1));
	CDC_Tx_FS((uint8_t*)top_menu_items_row2,strlen((const char*)top_menu_items_row2));
	CDC_Tx_FS((uint8_t*)top_menu_items_row3,strlen((const char*)top_menu_items_row3));
#if	(OUTPUT_TEST==1)
	CDC_Tx_FS((uint8_t*)top_menu_items_row4,strlen((const char*)top_menu_items_row4));
#endif
#if	(INPUT_TEST==1)
	CDC_Tx_FS((uint8_t*)top_menu_items_row5,strlen((const char*)top_menu_items_row5));
#endif
#if	(CAN1_TEST==1)
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
	CDC_Tx_FS((uint8_t*)top_menu_items_row6,strlen((const char*)top_menu_items_row6));
#endif
#if	(CAN2_TEST==1)
	HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
	HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
	CDC_Tx_FS((uint8_t*)top_menu_items_row7,strlen((const char*)top_menu_items_row7));
#endif
#if	(ANALOG_TEST==1)
	CDC_Tx_FS((uint8_t*)top_menu_items_row8,strlen((const char*)top_menu_items_row8));
#endif
#if	(FW_UPDATE==1)
	CDC_Tx_FS((uint8_t*)top_menu_items_row10,strlen((const char*)top_menu_items_row10));
#endif
#if	(BLE_SUPPORT==1)
	CDC_Tx_FS((uint8_t*)top_menu_items_row12,strlen((const char*)top_menu_items_row12));
#endif
#if	(UTILITIES==1)
	CDC_Tx_FS((uint8_t*)top_menu_items_row13,strlen((const char*)top_menu_items_row13));
#endif
	CDC_Tx_FS((uint8_t*)top_menu_items_row3,strlen((const char*)top_menu_items_row3));
	CDC_Tx_FS((uint8_t*)top_menu_items_row9,strlen((const char*)top_menu_items_row9));
	do
	{
		sel = app.usbbuf[app.usblen-1];
		mystring3[0]=sel;
		switch (sel)
		{
#if	(OUTPUT_TEST==1)
			case 0x31:
				L10_menu();
				break;
#endif
#if	(INPUT_TEST==1)
			case 0x32:
				L20_menu();
				break;
#endif
#if	(CAN1_TEST==1)
			case 0x33:
				L30_menu();
				break;
#endif
#if	(CAN2_TEST==1)
			case 0x34:
				L40_menu();
				break;
#endif
#if	(ANALOG_TEST==1)
			case 0x35:
				L50_menu();
				break;
#endif
#if	(FW_UPDATE==1)
			case 0x36:
				L60_menu();
				break;
#endif
#if	(BLE_SUPPORT==1)
			case 0x37:
				L70_menu();
				break;
#endif
#if	(UTILITIES==1)
			case 0x38:
				L80_menu();
				break;
#endif
			case 0x1B:
				memcpy(&mystring3[0], (const uint8_t *) "ESC", 3);
				CDC_Tx_FS((uint8_t*)mystring4,strlen((const char*)mystring4));
				FirstTime = true;
				Test_Mode = false;
//				leds_test = false;
				led_off(LED_ALL);
#if	(CAN1_TEST==1)
			    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
			    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
#endif
#if	(CAN2_TEST==1)
			    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
			    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
				break;
#endif
			default:
				// do nothing for undefined choices
				break;
		}
	}
	while (sel != 0x1B);

}	// end top_menu

#if	(OUTPUT_TEST==1)
#pragma GCC optimize ("Os")
/* @fn 		L10_menu()
 * @brief 	Display and select the level 1,0 menu items (OUTPUTS TEST, BANK SELECT)
 * @return	null
 * */
void L10_menu()
{
	app.usbbuf[app.usblen-1] = 0;
	output = 0;

	CDC_Tx_FS(mystring3,strlen((const char*)mystring3));
	memcpy(&mystring3[0], (const uint8_t *) "    ", 4);
	CDC_Tx_FS((uint8_t*)L10_menu_items_row1,strlen((const char*)L10_menu_items_row1));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row2,strlen((const char*)L10_menu_items_row2));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row4,strlen((const char*)L10_menu_items_row4));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row5,strlen((const char*)L10_menu_items_row5));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	CDC_Tx_FS((uint8_t*)top_menu_items_row9,strlen((const char*)top_menu_items_row9));
	do
	{
		sel = app.usbbuf[app.usblen-1];
		output = sel << 8;					//in output upper byte the bank number
		switch (sel)
		{
			case 0x31 ... 0x36:
				sprintf((char*)&mystring3[0], "%c", sel);
				L11_menu();
				break;
			case 0x1B:
				memcpy(&mystring3[0], (const uint8_t *) "ESC", 3);
				top_menu();
				break;
			default:
				// do nothing for undefined choices
				break;
		}
	}
	while (sel != 0x1B);

}	// end L10_menu

#pragma GCC optimize ("Os")
/* @fn 		L11_menu()
 * @brief 	Display and select the level 1,1 menu items (OUTPUT TEST, BANK 'x' OUTPUTS TEST)
 * @return	null
 * */
void L11_menu()
{
	app.usbbuf[app.usblen-1] = 0;
	Test_Mode = true;

	CDC_Tx_FS(mystring3,strlen((const char*)mystring3));
	memcpy(&mystring3[0], (const uint8_t *) "    ", 4);
	CDC_Tx_FS((uint8_t*)L10_menu_items_row1,strlen((const char*)L10_menu_items_row1));
	CDC_Tx_FS((uint8_t*)L11_menu_items_row2,strlen((const char*)L11_menu_items_row2));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	CDC_Tx_FS((uint8_t*)L11_menu_items_row4,strlen((const char*)L11_menu_items_row4));
	CDC_Tx_FS((uint8_t*)L11_menu_items_row5,strlen((const char*)L11_menu_items_row5));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	CDC_Tx_FS((uint8_t*)top_menu_items_row9,strlen((const char*)top_menu_items_row9));
	do
	{
		sel = app.usbbuf[app.usblen-1];
		output = output & 0xFF00;		//clear lower byte
		output |= sel;					//in output lower byte the port number
		switch (sel)
		{
			case 0x31 ... 0x36:
				sprintf((char*)&mystring3[0], "%c", sel);
				L12_menu();
				break;
			case 0x1B:
				memcpy(&mystring3[0], (const uint8_t *) "ESC", 3);
				L10_menu();
				break;
			default:
				// do nothing for undefined choices
				break;
		}
	}
	while (sel != 0x1B);

}	// end L11_menu

#pragma GCC optimize ("Os")
/* @fn 		L12_menu()
 * @brief 	Display and select the level 1,2 menu items (BANK 'x' TEST OUTPUT "n")
 * @return	null
 * */
void L12_menu()
{
	app.usbbuf[app.usblen-1] = 0;
	Test_Mode = true;

	CDC_Tx_FS(mystring3,strlen((const char*)mystring3));
	memcpy(&mystring3[0], (const uint8_t *) "    ", 4);
	CDC_Tx_FS((uint8_t*)L10_menu_items_row1,strlen((const char*)L10_menu_items_row1));
	CDC_Tx_FS((uint8_t*)L12_menu_items_row2,strlen((const char*)L12_menu_items_row2));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	CDC_Tx_FS((uint8_t*)L12_menu_items_row4,strlen((const char*)L12_menu_items_row4));
	CDC_Tx_FS((uint8_t*)L12_menu_items_row5,strlen((const char*)L12_menu_items_row5));
	CDC_Tx_FS((uint8_t*)L12_menu_items_row6,strlen((const char*)L12_menu_items_row6));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	CDC_Tx_FS((uint8_t*)top_menu_items_row9,strlen((const char*)top_menu_items_row9));
	do
	{
		sel = app.usbbuf[app.usblen-1];
		switch (sel)
		{
			case 0x30 ... 0x33:
				sprintf((char*)&mystring3[0], "%c", sel);
				write_port(output, sel);
				break;
			case 0x1B:
				memcpy(&mystring3[0], (const uint8_t *) "ESC", 3);
				L11_menu();
				break;
			default:
				// do nothing for undefined choices
				break;
		}
	}
	while (sel != 0x1B);

}	// end L12_menu
#endif	/* OUTPUT_TEST */

#if	(INPUT_TEST==1)
#pragma GCC optimize ("Os")
/* @fn 		L20_menu()
 * @brief 	Display and select the level 2,0 menu items (DIGITAL INPUTS TEST)
 * @return	null
 * */
void L20_menu()
{
	app.usbbuf[app.usblen-1] = 0;
	Test_Mode = true;
	output = 0;

	CDC_Tx_FS(mystring3,strlen((const char*)mystring3));
	memcpy(&mystring3[0], (const uint8_t *) "    ", 4);
	CDC_Tx_FS((uint8_t*)L20_menu_items_row1,strlen((const char*)L20_menu_items_row1));
	CDC_Tx_FS((uint8_t*)L20_menu_items_row2,strlen((const char*)L20_menu_items_row2));
	CDC_Tx_FS((uint8_t*)L20_menu_items_row3,strlen((const char*)L20_menu_items_row3));
	CDC_Tx_FS((uint8_t*)L20_menu_items_row4,strlen((const char*)L20_menu_items_row4));
	CDC_Tx_FS((uint8_t*)L20_menu_items_row5,strlen((const char*)L20_menu_items_row5));
	CDC_Tx_FS((uint8_t*)L20_menu_items_row4,strlen((const char*)L20_menu_items_row4));
	do
	{
		sel = app.usbbuf[app.usblen-1];
		output = sel << 8;					//in output upper byte the bank number
		switch (sel)
		{
			case 0x1B:
				memcpy(&mystring3[0], (const uint8_t *) "ESC", 3);
				top_menu();
				break;
			default:
				// do nothing for undefined choices
				break;
		}
	}
	while (sel != 0x1B);

}	// end L20_menu
#endif	/* INPUT_TEST */

#if	(CAN1_TEST==1)
#pragma GCC optimize ("Os")
/* @fn 		L30_menu()
 * @brief 	Display and select the level 3,0 menu items (CAN 1 TEST)
 * @return	null
 * */
void L30_menu()
{
	static uint32_t	TxMailbox1;
	static uint8_t 	can1_ubuf[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
	static CAN_TxHeaderTypeDef Test_Message =
	{
		/*##-1- Configure the CAN1 Test_Message #####################################*/
		.StdId = 0x603,
		.ExtId = 0x603,
		.RTR = CAN_RTR_DATA,
		.IDE = CAN_ID_STD,
		.DLC = 8,
		.TransmitGlobalTime = DISABLE
	};

 	app.usbbuf[app.usblen-1] = 0;
 	Test_Mode = true;
 	output = 0;
	leds_test = false;
	led_off(LED_PC6);
	led_off(LED_PC7);

    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

	CDC_Tx_FS(mystring3,strlen((const char*)mystring3));
	memcpy(&mystring3[0], (const uint8_t *) "    ", 4);
	CDC_Tx_FS((uint8_t*)L10_menu_items_row1,strlen((const char*)L10_menu_items_row1));
	CDC_Tx_FS((uint8_t*)L30_menu_items_row2,strlen((const char*)L30_menu_items_row2));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row5,strlen((const char*)L10_menu_items_row5));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	do
	{
		sel = app.usbbuf[app.usblen-1];
		output = sel << 8;
		if (update_1s)
		{
			update_1s = false;
			/* Start the Transmission process */
			memcpy((void *)&TxHeader1, (void *)&Test_Message, sizeof(Test_Message));
			if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, can1_ubuf, &TxMailbox1) != HAL_OK)
			{
				/**< indicate CAN1 transmit error */
			}
			led_toggle (LED_PC6);
			/* Check the Receive process */
			if (memcmp(&can1_ubuf[0], &can1app.canbuf[24], 8) == 0)
			{
				led_toggle (LED_PC7);
				memset(&can1app.canbuf[24],0x00, 8);
				can1app.canlen = 0;
			}
		}
		switch (sel)
		{
			case 0x1B:
				memcpy(&mystring3[0], (const uint8_t *) "ESC", 3);
				top_menu();
				break;
			default:
				// do nothing for undefined choices
				break;
		}
	}
	while (sel != 0x1B);

}	// end L30_menu
#endif	/* CAN1_TEST */

#if	(CAN2_TEST==1)
#pragma GCC optimize ("Os")
/* @fn 		L40_menu()
 * @brief 	Display and select the level 4,0 menu items (CAN 2 TEST)
 * @return	null
 * */
void L40_menu()
{
	static uint32_t	TxMailbox2;
	static uint8_t 	can2_ubuf[8] = {0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11};
	static CAN_TxHeaderTypeDef Test_Message =
	{
		/*##-1- Configure the CAN1 Test_Message #####################################*/
		.StdId = 0x655,
		.ExtId = 0x655,
		.RTR = CAN_RTR_DATA,
		.IDE = CAN_ID_STD,
		.DLC = 8,
		.TransmitGlobalTime = DISABLE
	};

 	app.usbbuf[app.usblen-1] = 0;
 	Test_Mode = true;
	output = 0;
	leds_test = false;
	led_off(LED_PC6);
	led_off(LED_PC7);

	HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
	HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);

	CDC_Tx_FS(mystring3,strlen((const char*)mystring3));
	memcpy(&mystring3[0], (const uint8_t *) "    ", 4);
	CDC_Tx_FS((uint8_t*)L10_menu_items_row1,strlen((const char*)L10_menu_items_row1));
	CDC_Tx_FS((uint8_t*)L40_menu_items_row2,strlen((const char*)L40_menu_items_row2));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row5,strlen((const char*)L10_menu_items_row5));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	do
	{
		sel = app.usbbuf[app.usblen-1];
		output = sel << 8;
		if (update_1s)
		{
			update_1s = false;
			/* Start the Transmission process */
			memcpy((void *)&TxHeader1, (void *)&Test_Message, sizeof(Test_Message));
			if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, can2_ubuf, &TxMailbox2) != HAL_OK)
			{
				/**< indicate CAN1 transmit error */
			}
			led_toggle (LED_PC6);
			/* Check the Receive process */
			if (memcmp(&can2_ubuf[0], &can2app.canbuf[24], 8) == 0)
			{
				led_toggle (LED_PC7);
				memset(&can2app.canbuf[24],0x00, 8);
				can2app.canlen = 0;
			}
		}
		switch (sel)
		{
			case 0x1B:
				memcpy(&mystring3[0], (const uint8_t *) "ESC", 3);
				top_menu();
				break;
			default:
				// do nothing for undefined choices
				break;
		}
	}
	while (sel != 0x1B);

}	// end L40_menu
#endif	/* CAN2_Test */

#if	(ANALOG_TEST==1)
#pragma GCC optimize ("Os")
/* @fn 		L50_menu()
 * @brief 	Display and select the level 5,0 menu items (ANALOG INPUTS TEST)
 * @return	null
 * */
void L50_menu()
{
 	conversion_ended = false;
 	Test_Mode = true;
 	app.usbbuf[app.usblen-1] = 0;
	output = 0;

	CDC_Tx_FS(mystring3,strlen((const char*)mystring3));
	memcpy(&mystring3[0], (const uint8_t *) "    ", 4);
	CDC_Tx_FS((uint8_t*)L50_menu_items_row1a,strlen((const char*)L50_menu_items_row1a));
	CDC_Tx_FS((uint8_t*)L50_menu_items_row1b,strlen((const char*)L50_menu_items_row1b));
	CDC_Tx_FS((uint8_t*)L50_menu_items_row2a,strlen((const char*)L50_menu_items_row2a));
	CDC_Tx_FS((uint8_t*)L50_menu_items_row2b,strlen((const char*)L50_menu_items_row2b));
	CDC_Tx_FS((uint8_t*)L50_menu_items_row3a,strlen((const char*)L50_menu_items_row3a));
	CDC_Tx_FS((uint8_t*)L50_menu_items_row3b,strlen((const char*)L50_menu_items_row3b));
	CDC_Tx_FS((uint8_t*)L50_menu_items_row4a,strlen((const char*)L50_menu_items_row4a));
	CDC_Tx_FS((uint8_t*)L50_menu_items_row4b,strlen((const char*)L50_menu_items_row4b));
	CDC_Tx_FS((uint8_t*)L50_menu_items_row5a,strlen((const char*)L50_menu_items_row5a));
	CDC_Tx_FS((uint8_t*)L50_menu_items_row5b,strlen((const char*)L50_menu_items_row5b));
//	CDC_Tx_FS(L50_menu_items_row6a,strlen((const char*)L50_menu_items_row6a));
//	CDC_Tx_FS(L50_menu_items_row6b,strlen((const char*)L50_menu_items_row6b));
//	CDC_Tx_FS(L50_menu_items_row7a,strlen((const char*)L50_menu_items_row7a));
//	CDC_Tx_FS(L50_menu_items_row7b,strlen((const char*)L50_menu_items_row7b));
	CDC_Tx_FS(L50_menu_items_row4a,strlen((const char*)L50_menu_items_row4a));
	CDC_Tx_FS(L50_menu_items_row4b,strlen((const char*)L50_menu_items_row4b));
	do
	{
		sel = app.usbbuf[app.usblen-1];
		output = sel << 8;
		if (update_1s)
		{
			update_1s = false;
			read_analogs();
		}
		switch (sel)
		{
			case 0x1B:
				memcpy(&mystring3[0], (const uint8_t *) "ESC", 3);
				top_menu();
				break;
			default:
				// do nothing for undefined choices
				break;
		}
	}
	while (sel != 0x1B);

}	// end L50_menu
#endif	/* ANALOG_TEST */

#if	(FW_UPDATE==1)
#pragma GCC optimize ("Os")
/* @fn 		L60_menu()
 * @brief 	Display and select the level 6,0 menu items (FW UPDATE)
 * @return	null
 * */
void L60_menu()
{
//	extern I2C_HandleTypeDef hi2c1;
//	extern I2C_HandleTypeDef hi2c2;

//	conversion_ended = false;
 	Test_Mode = true;
 	app.usbbuf[app.usblen-1] = 0;
	output = 0;

	CDC_Tx_FS((uint8_t*)mystring5,strlen((const char*)mystring5));
	GetNumericString(false, false);
	if (strlen((const char*)(mystring3)))
	{
		if (!((strcmp((const char*)mystring3, "y")) && (strcmp((const char*)mystring3, "Y"))))
		{
			CDC_Tx_FS((uint8_t*)mystring6,strlen((const char*)mystring6));
		//	HAL_ADC_MspDeInit(&hadc1);
		//	HAL_I2C_MspDeInit(&hi2c1);
		//	HAL_I2C_MspDeInit(&hi2c2);
		//	HAL_UART_MspDeInit(&huart3);
			HAL_NVIC_DisableIRQ(EXTI0_IRQn);
			HAL_NVIC_DisableIRQ(EXTI2_IRQn);
			HAL_NVIC_DisableIRQ(EXTI4_IRQn);
		//	HAL_GPIO_WritePin(PC_11_DISC_GPIO_Port, PC_11_DISC_Pin, GPIO_PIN_SET);	//Disconnect USB (Only for Olimex STM32-H103 Board)

			jump_to_bootloader();
		}
	}
	top_menu();
}	// end L60_menu
#endif	/* FW_UPDATE */

#if	(BLE_SUPPORT==1)
#pragma GCC optimize ("Os")
/* @fn 		L70_menu()
 * @brief 	Display and select the level 7,0 menu items (Set BLE MAC Address)
 * @return	null
 * */
void L70_menu_Strings()
{
	uint8_t ascii_val1[32];
	uint8_t bdaddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	extern FLASH_DATA_ORG FlashDataOrg;

	CDC_Tx_FS((uint8_t*)mystring21a,strlen((const char*)mystring21a));
	(((bdaddr)[0] =  (uint8_t) (FlashDataOrg.b_status.s12)     ) , \
	 ((bdaddr)[1] =  (uint8_t) (FlashDataOrg.b_status.s12>>8)  ) , \
	 ((bdaddr)[2] =  (uint8_t) (FlashDataOrg.b_status.s12>>16) ));
	(((bdaddr)[3] =  (uint8_t) (FlashDataOrg.b_status.s13)     ) , \
	 ((bdaddr)[4] =  (uint8_t) (FlashDataOrg.b_status.s13>>8)  ) , \
	 ((bdaddr)[5] =  (uint8_t) (FlashDataOrg.b_status.s13>>16) ));

	sprintf((char*)&ascii_val1[0], " %02X:%02X:%02X:%02X:%02X:%02X\r\n", bdaddr[3],bdaddr[4],bdaddr[5],
																		 bdaddr[0],bdaddr[1],bdaddr[2]);
	CDC_Tx_FS((uint8_t*)ascii_val1,strlen((const char*)ascii_val1));
	CDC_Tx_FS((uint8_t*)mystring21b,strlen((const char*)mystring21b));

}

#pragma GCC optimize ("Os")
void L70_menu()
{
	extern FLASH_DATA_ORG FlashDataOrg;
	char tmp0, tmp1;
	bool updated = false;
 	Test_Mode = true;

 	app.usbbuf[app.usblen-1] = 0;
	L70_menu_Strings();
	GetNumericString(false, false);
	if (strlen((const char*)(mystring3)))
	{
		//Swap byte 4 <-> byte 0, byte 5 <-> byte 1
		tmp0 = mystring3[0];
		tmp1 = mystring3[1];
		mystring3[0] = mystring3[4];
		mystring3[1] = mystring3[5];
		mystring3[4] = tmp0;
		mystring3[5] = tmp1;
		//Set the new NIC section of the BLE MAC Address
		FlashDataOrg.b_status.s12 = (uint32_t)strtol((const char*)mystring3, NULL, 16);
//		FlashDataOrg.b_status.s12 = (uint32_t)(xtoi((const char*)mystring3));
		updated = true;
	}
	//Force exit on carriage return
	if (app.usbbuf[app.usblen-1] == 0x04)
	{
		if (updated)
		{
			updated = false;
#if (WRITE_FLASH==1)
			//Update the new calibration values when exit
			CDC_Tx_FS((uint8_t*)mystring_w1,strlen((const char*)mystring_w1));
			Write_Flash(0, 0);
			CDC_Tx_FS((uint8_t*)mystring8c,strlen((const char*)mystring8c));
#endif	//WRITE_FLASH
		}
		top_menu();
	}
}
#endif	//BLE_SUPPORT==1

#if	(UTILITIES==1)
#pragma GCC optimize ("Os")
/* @fn 		L80_menu()
 * @brief 	Display and select the level 8,0 menu items (UTILITIES)
 * @return	null
 * */
void L80_menu_Strings()
{
	CDC_Tx_FS((uint8_t*)L10_menu_items_row1,strlen((const char*)L10_menu_items_row1));
	CDC_Tx_FS((uint8_t*)L80_menu_items_row2,strlen((const char*)L80_menu_items_row2));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
#if (SET_DATE_TIME==1)
	CDC_Tx_FS((uint8_t*)L80_menu_items_row4,strlen((const char*)L80_menu_items_row4));
#endif
#if ((PRESSURE_SENSOR_PRESENT==1) || (HUMIDITY_SENSOR_PRESENT==1))
	CDC_Tx_FS((uint8_t*)L80_menu_items_row5,strlen((const char*)L80_menu_items_row5));
#endif
#if (PRESSURE_SENSOR_PRESENT==1)
	CDC_Tx_FS((uint8_t*)L80_menu_items_row6,strlen((const char*)L80_menu_items_row6));
#endif
#if (HUMIDITY_SENSOR_PRESENT==1)
	CDC_Tx_FS((uint8_t*)L80_menu_items_row7,strlen((const char*)L80_menu_items_row7));
#endif
#if ((VOC_SENSOR_PRESENT) && (CCS811))
	CDC_Tx_FS((uint8_t*)L80_menu_items_row8,strlen((const char*)L80_menu_items_row8));
#endif
#if (GAS_SENSOR_MODULE_PRESENT==1)
	CDC_Tx_FS((uint8_t*)L80_menu_items_row9,strlen((const char*)L80_menu_items_row9));
	CDC_Tx_FS((uint8_t*)L80_menu_items_row10,strlen((const char*)L80_menu_items_row10));
	CDC_Tx_FS((uint8_t*)L80_menu_items_row11,strlen((const char*)L80_menu_items_row11));
	CDC_Tx_FS((uint8_t*)L80_menu_items_row12,strlen((const char*)L80_menu_items_row12));
#if (OUTDOOR_MODE)
	CDC_Tx_FS((uint8_t*)L80_menu_items_row13,strlen((const char*)L80_menu_items_row13));
	CDC_Tx_FS((uint8_t*)L80_menu_items_row14,strlen((const char*)L80_menu_items_row14));
	CDC_Tx_FS((uint8_t*)L80_menu_items_row15,strlen((const char*)L80_menu_items_row15));
#endif	//OUTDOOR_MODE
#endif	//GAS_SENSOR_MODULE_PRESENT
#if (RTC_CALIB==1)
	CDC_Tx_FS((uint8_t*)L80_menu_items_row20,strlen((const char*)L80_menu_items_row20));
#endif
#if (USE_BKUP_SRAM==1)
	CDC_Tx_FS((uint8_t*)L80_menu_items_row23,strlen((const char*)L80_menu_items_row23));
#endif
#if (UWB_MODE==1)
	CDC_Tx_FS((uint8_t*)L80_menu_items_row21,strlen((const char*)L80_menu_items_row21));
	CDC_Tx_FS((uint8_t*)L80_menu_items_row22,strlen((const char*)L80_menu_items_row22));
#elif (PWR_NODE_MODE==1)
	CDC_Tx_FS((uint8_t*)L80_menu_items_row21,strlen((const char*)L80_menu_items_row21));
#endif
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row5,strlen((const char*)L10_menu_items_row5));
	CDC_Tx_FS((uint8_t*)L10_menu_items_row3,strlen((const char*)L10_menu_items_row3));
	CDC_Tx_FS((uint8_t*)top_menu_items_row9,strlen((const char*)top_menu_items_row9));
}	// end L80_menu_Strings

#pragma GCC optimize ("Os")
/* @fn 		L80_menu()
 * @brief 	Display and select the level 8,0 menu items (UTILITIES)
 * @return	null
 * */
void L80_menu()
{
	static volatile uint8_t len;

#if (PRESSURE_SENSOR_PRESENT==1)
	float32_t Pref = 0.0;
	extern int32_t P_Correction;
	#if (LPS25HB)
		extern LPS25HB_MeasureTypeDef_st PRS_Values;
	#elif (LPS22HB)
		extern LPS22HB_MeasureTypeDef_st PRS_Values;
	#endif
	extern uint16_t MSL;
#endif
#if (HUMIDITY_SENSOR_PRESENT==1)
	uint16_t RHref = 0;
	extern int32_t RH_Correction;
	#if (HTS221)
		extern HTS221_MeasureTypeDef_st HUM_Values;
	#elif (SHT4x)
		extern SHT4x_MeasureTypeDef_st HUM_Values;
	#endif
#endif
#if ((PRESSURE_SENSOR_PRESENT==1) || (HUMIDITY_SENSOR_PRESENT==1))
	float32_t Tref = 0.0;
	extern int16_t T_Correction;
#endif
#if (VOC_SENSOR_PRESENT==1) && (CCS811)
	extern uint32_t CCS811_VOC_Ro;
	extern uint32_t CCS811_VOC_Ro_Stored;
#endif
#if (GAS_SENSOR_MODULE_PRESENT==1)
	extern uint16_t CH2O, CO, NO2, NH3;
	uint16_t COref = 0; uint16_t CH2Oref = 0; int16_t NO2ref = 0; int16_t NH3ref = 0;
	extern int8_t CO_Corr; extern int8_t CH2O_Corr;
	extern int8_t NO2_Corr; extern int8_t NH3_Corr;
	extern uint32_t MiCS_6814_CO_Ro; extern uint32_t MiCS_6814_NO2_Ro; extern uint32_t MiCS_6814_NH3_Ro;
	extern uint32_t MiCS_6814_CO_Rf; extern uint32_t MiCS_6814_NO2_Rf; extern uint32_t MiCS_6814_NH3_Rf;
	extern float32_t MiCS_6814_CO_Rs; extern float32_t MiCS_6814_NO2_Rs; extern float32_t MiCS_6814_NH3_Rs;
#if !(CH2O_FROM_EC)
	extern uint32_t SMD1001_CH2O_Vo;
//	extern uint32_t SMD1001_CH2O_Rf;
	extern float32_t SMD1001_CH2O_Vs;
#endif
#if (OUTDOOR_MODE)
	extern uint16_t O3, SO2, C6H6;
	int16_t O3ref = 0; int16_t SO2ref = 0; int16_t C6H6ref = 0;
	extern int8_t O3_Corr; extern int8_t SO2_Corr; extern int8_t C6H6_Corr; int8_t Spare_Corr = 0;
#endif	//OUTDOOR_MODE
//	extern ANLG_MeasureTypeDef_st GAS_Values;
#endif	//GAS_SENSOR_MODULE_PRESENT
#if (SET_DATE_TIME==1)
	extern bool SendCntrlMsg;
#endif
#if (WRITE_FLASH==1)
	extern FLASH_DATA_ORG FlashDataOrg;
#endif
#if (UWB_MODE==1)
	char node_opmode[8] = "RTLS";
	char node_role[8] = "A0L2";
	char TempBuf[8] = {};
	uint8_t Res1 = 0; uint8_t Res2 = 0;
	extern uint8_t s1switch; extern uint8_t OpMode;
#elif (PWR_NODE_MODE==1)
	char node_opmode[8] = "MLRS";
	char TempBuf[8] = {};
	extern uint8_t OpMode;
	uint8_t Res1 = 0; uint8_t Res2 = 0; uint8_t s1switch = 0;
#endif
	bool updated = false;
 	Test_Mode = true;
 	app.usbbuf[app.usblen-1] = 0;
	output = 0;
	L80_menu_Strings();

	do
	{
		len = (uint8_t)app.usblen;	//We need to save the receive buffer pointer
		if (len == 0)		//because it is used in the main application
			len = 1;
		sel = app.usbbuf[len-1];
		switch (sel)
		{
#if (SET_DATE_TIME==1)
			case 0x31:
				app.usbbuf[len-1] = 0;
				CDC_Tx_FS((uint8_t*)mystring7,strlen((const char*)mystring7));
				do
				{
					sel1 = app.usbbuf[app.usblen];	//The "usb_run()" function must use the
					if (SendCntrlMsg)				//original receive buffer pointer
					{
	#if (GUI_SUPPORT==1)
						dataseq[Message_Length] = '\r';
						dataseq[Message_Length+1] = '\n';
						Message_Length = Message_Length + 2;
	#endif
						send_usbmessage(&dataseq[0], Message_Length);
						SendCntrlMsg = false;
					}
					usb_run();
				} while ((sel1 != 0x04) && (sel1 != 0x1B));
				break;
#endif
#if ((PRESSURE_SENSOR_PRESENT==1) || (HUMIDITY_SENSOR_PRESENT==1))
			case 0x32:
				app.usbbuf[len-1] = 0;
				PrintNumHeader((uint8_t*)mystring8a, (uint8_t*)mystring8b, temp_value, true, "°C");
				GetNumericString(true, true);
				//Calculate the new T_Correction
				if (strlen((const char*)(mystring3)))
				{
					Tref = atof((const char*)mystring3);
	#if (HUMIDITY_SENSOR_PRESENT)
					FlashDataOrg.b_status.s3 = (uint32_t)lrintf((Tref * 10.0) - (HUM_Values.Tout));
	#else
					FlashDataOrg.b_status.s3 = (uint32_t)lrintf((Tref * 10.0) - (PRS_Values.Tout));
	#endif
					T_Correction = (int16_t)FlashDataOrg.b_status.s3;
					updated = true;
				}
				break;
#endif
#if (PRESSURE_SENSOR_PRESENT==1)
			case 0x33:
				app.usbbuf[len-1] = 0;
				PrintNumHeader((uint8_t*)mystring9a, (uint8_t*)mystring9b, press_value, true, "mbar");
				GetNumericString(true, false);
				//Calculate the new P_Correction
				if (strlen((const char*)(mystring3)))
				{
					Pref = atof((const char*)mystring3);
					FlashDataOrg.b_status.s4 = (uint32_t)lrintf((Pref * 100.0) - (PRS_Values.Pout));
					P_Correction = (int32_t)FlashDataOrg.b_status.s4;
					updated = true;
				}

				app.usbbuf[app.usblen-1] = 0;
				PrintNumHeader((uint8_t*)mystring9c, (uint8_t*)mystring9d, MSL, false, "m");
				GetNumericString(true, true);
				//Calculate the new P_Correction
				if (strlen((const char*)(mystring3)))
				{
					MSL = (uint16_t)atoi((const char*)mystring3);
					//Pack-> Bit 0..15: MSL, Altitude in meters of the city where the device is located
					FlashDataOrg.b_status.se &= 0xFFFF0000;
					FlashDataOrg.b_status.se |= (uint32_t)(MSL & 0x0000FFFF);
					updated = true;
				}
				break;
#endif
#if (HUMIDITY_SENSOR_PRESENT==1)
			case 0x34:
				app.usbbuf[len-1] = 0;
				PrintNumHeader((uint8_t*)mystring10a, (uint8_t*)mystring10b, hum_value, false, "%");
				GetNumericString(false, false);
				//Calculate the new RH_Correction
				if (strlen((const char*)(mystring3)))
				{
					RHref = atoi((const char*)mystring3);
					FlashDataOrg.b_status.s5 = (uint32_t)lrintf((RHref * 10.0) - (HUM_Values.Hout));
					RH_Correction = (int32_t)FlashDataOrg.b_status.s5;
					updated = true;
				}
				break;
#endif
#if ((VOC_SENSOR_PRESENT) && (CCS811))
			case 0x35:
				app.usbbuf[len-1] = 0;
				mystring3[0] = 0;
	#if (CCS811)
				CCS811_Save_Baseline(false);
	#endif
				PrintNumHeader((uint8_t*)mystring11a, (uint8_t*)mystring11c, CCS811_VOC_Ro, false, " ");
				PrintNumHeader((uint8_t*)mystring11b, (uint8_t*)mystring11d, CCS811_VOC_Ro_Stored, false, " ");
				GetNumericString(false, false);
				//Calculate the new CCS811_VOC_Ro
				if (mystring3[0] == 0x31)
				{
//					CCS811_Save_Baseline(true);
					PrintNumHeader((uint8_t*)mystring11e, (uint8_t*)mystring11c, CCS811_VOC_Ro, false, " ");
					CCS811_Save_Baseline_Reserved = true;
//					FlashDataOrg.b_status.s0 = (uint32_t)(atoi((const char*)mystring3));
//					CCS811_Restore_Baseline(false);
//					updated = true;
				}
				break;
#endif
#if (GAS_SENSOR_MODULE_PRESENT==1)
			case 0x36:
				app.usbbuf[len-1] = 0;
				PrintNumHeader((uint8_t*)mystring12g, (uint8_t*)mystring12h, MiCS_6814_CO_Rf, false, " ohm");
				GetNumericString(false, false);
				//Enter the new MiCS_6814_CO_Rf
				if (strlen((const char*)(mystring3)))
				{
					FlashDataOrg.b_status.sf = MiCS_6814_CO_Rf = (uint32_t)(atoi((const char*)mystring3));
					updated = true;
				} else
				{
					PrintNumHeader((uint8_t*)mystring12e, (uint8_t*)mystring12f, (uint32_t)lrintf(MiCS_6814_CO_Rs), false, " ohm");

					app.usbbuf[app.usblen-1] = 0;
					PrintNumHeader((uint8_t*)mystring12c, (uint8_t*)mystring12d, MiCS_6814_CO_Ro, false, " ohm");
					GetNumericString(false, false);
					//Enter the new MiCS_6814_CO_Ro
					if (strlen((const char*)(mystring3)))
					{
						FlashDataOrg.b_status.sb = MiCS_6814_CO_Ro = (uint32_t)(atoi((const char*)mystring3));
						updated = true;
					}

					app.usbbuf[app.usblen-1] = 0;
					PrintNumHeader((uint8_t*)mystring18c, NULL, CO_Corr, false, "mV/10: 1=10mV");
					PrintNumHeader((uint8_t*)mystring12a, (uint8_t*)mystring12b, CO, false, "mg/m3");
					GetNumericString(false, true);
					COref = atoi((const char*)mystring3);
					//Calculate the new CO_Corr
					if (strlen((const char*)(mystring3)))
					{
						//While waiting to implement the mg/m3->Volts conversion, the reference value is entered in mVolts/10
						//and coincides with the correction value
						CO_Corr = (int8_t)lrintf(COref);
	//					CO_Corr = (int8_t)lrintf(COref - GAS_Values.CO);
						updated = true;
					}
				}
				break;
			case 0x37:
				app.usbbuf[len-1] = 0;
	#if !(CH2O_FROM_EC)
				PrintNumHeader((uint8_t*)mystring13f, (uint8_t*)mystring12f, (uint32_t)lrintf(SMD1001_CH2O_Vs), false, " Volts");
				PrintNumHeader((uint8_t*)mystring13c, (uint8_t*)mystring13d, SMD1001_CH2O_Vo, false, " mVolts");
				GetNumericString(false, false);
				//Enter the new MiCS_6814_CO_Ro
				if (strlen((const char*)(mystring3)))
				{
					FlashDataOrg.b_status.s14 = SMD1001_CH2O_Vo = (uint32_t)(atoi((const char*)mystring3));
					updated = true;
				}
				app.usbbuf[app.usblen-1] = 0;
	#endif
				PrintNumHeader((uint8_t*)mystring18c, NULL, CH2O_Corr, false, "mV");
				PrintNumHeader((uint8_t*)mystring13a, (uint8_t*)mystring13b, CH2O, false, "ug/m3");
				GetNumericString(false, true);
				CH2Oref = atoi((const char*)mystring3);
				//Calculate the new CH2O_Corr
				if (strlen((const char*)(mystring3)))
				{
					//While waiting to implement the ug/m3->Volts conversion, the reference value is entered in mVolts/10
					//and coincides with the correction value
					CH2O_Corr = (int8_t)lrintf(CH2Oref);
//					CH2O_Corr = (int8_t)lrintf((CH2Oref - GAS_Values.CH2O);
					updated = true;
				}
				break;
			case 0x38:
				app.usbbuf[len-1] = 0;
				PrintNumHeader((uint8_t*)mystring14e, (uint8_t*)mystring12h, MiCS_6814_NO2_Rf, false, " ohm");
				GetNumericString(false, false);
				//Enter the new MiCS_6814_CO_Rf
				if (strlen((const char*)(mystring3)))
				{
					FlashDataOrg.b_status.s11 = MiCS_6814_NO2_Rf = (uint32_t)(atoi((const char*)mystring3));
					updated = true;
				} else
				{
					PrintNumHeader((uint8_t*)mystring12e, (uint8_t*)mystring12f, (uint32_t)lrintf(MiCS_6814_NO2_Rs), false, " ohm");

					app.usbbuf[app.usblen-1] = 0;
					PrintNumHeader((uint8_t*)mystring14c, (uint8_t*)mystring14d, MiCS_6814_NO2_Ro, false, " ohm");
					GetNumericString(false, false);
					//Enter the new MiCS_6814_NO2_Ro
					if (strlen((const char*)(mystring3)))
					{
						FlashDataOrg.b_status.sd = MiCS_6814_NO2_Ro = (uint32_t)(atoi((const char*)mystring3));
						updated = true;
					}

					app.usbbuf[app.usblen-1] = 0;
					PrintNumHeader((uint8_t*)mystring18c, NULL, NO2_Corr, false, "mV/10: 1=10mV");
					PrintNumHeader((uint8_t*)mystring14a, (uint8_t*)mystring14b, NO2, false, "ug/m3");
					GetNumericString(false, true);
					NO2ref = atoi((const char*)mystring3);
					//Calculate the new NO2_Corr
					if (strlen((const char*)(mystring3)))
					{
						//While waiting to implement the ug/m3->Volts conversion, the reference value is entered in mVolts/10
						//and coincides with the correction value
						NO2_Corr = (int8_t)NO2ref;
	//					NO2_Corr = (int8_t)lrintf((NO2ref - GAS_Values.NO2);
						updated = true;
					}
				}
				break;
			case 0x39:
				app.usbbuf[len-1] = 0;
				PrintNumHeader((uint8_t*)mystring15e, (uint8_t*)mystring12h, MiCS_6814_NH3_Rf, false, " ohm");
				GetNumericString(false, false);
				//Enter the new MiCS_6814_CO_Rf
				if (strlen((const char*)(mystring3)))
				{
					FlashDataOrg.b_status.s10 = MiCS_6814_NH3_Rf = (uint32_t)(atoi((const char*)mystring3));
					updated = true;
				} else
				{
					PrintNumHeader((uint8_t*)mystring12e, (uint8_t*)mystring12f, (uint32_t)lrintf(MiCS_6814_NH3_Rs), false, " ohm");

					app.usbbuf[app.usblen-1] = 0;
					PrintNumHeader((uint8_t*)mystring15c, (uint8_t*)mystring15d, MiCS_6814_NH3_Ro, false, " ohm");
					GetNumericString(false, false);
					//Enter the new MiCS_6814_NH3_Ro
					if (strlen((const char*)(mystring3)))
					{
						FlashDataOrg.b_status.sc = MiCS_6814_NH3_Ro = (uint32_t)(atoi((const char*)mystring3));
						updated = true;
					}

					app.usbbuf[app.usblen-1] = 0;
					PrintNumHeader((uint8_t*)mystring18c, NULL, NH3_Corr, false, "mV/10: 1=10mV");
					PrintNumHeader((uint8_t*)mystring15a, (uint8_t*)mystring15b, NH3, false, "ug/m3");
					GetNumericString(false, true);
					NH3ref = atoi((const char*)mystring3);
					//Calculate the new NH3_Corr
					if (strlen((const char*)(mystring3)))
					{
						//While waiting to implement the ug/m3->Volts conversion, the reference value is entered in mVolts/10
						//and coincides with the correction value
						NH3_Corr = (int8_t)NH3ref;
	//					NH3_Corr = (int8_t)lrintf((NH3ref - GAS_Values.NH3);
						updated = true;
					}
				}
				break;
	#if (OUTDOOR_MODE)
			case 0x41:	//"A"
			case 0x61:	//"a"
				app.usbbuf[len-1] = 0;
				PrintNumHeader((uint8_t*)mystring18c, NULL, O3_Corr, false, "mV");
				PrintNumHeader((uint8_t*)mystring16a, (uint8_t*)mystring16b, O3, false, "ug/m3");
				GetNumericString(false, true);
				O3ref = atoi((const char*)mystring3);
				//Calculate the new O3_Corr
				if (strlen((const char*)(mystring3)))
				{
					//While waiting to implement the ug/m3->Volts conversion, the reference value is entered in mVolts/10
					//and coincides with the correction value
					O3_Corr = (int8_t)lrintf(O3ref);
//					O3_Corr = (int8_t)lrintf(O3ref - GAS_Values.O3);
					updated = true;
				}
				break;
			case 0x42:	//"B"
			case 0x62:	//"b"
				app.usbbuf[len-1] = 0;
				PrintNumHeader((uint8_t*)mystring18c, NULL, SO2_Corr, false, "mV");
				PrintNumHeader((uint8_t*)mystring17a, (uint8_t*)mystring17b, SO2, false, "ug/m3");
				GetNumericString(false, true);
				SO2ref = atoi((const char*)mystring3);
				//Calculate the new SO2_Corr
				if (strlen((const char*)(mystring3)))
				{
					//While waiting to implement the ug/m3->Volts conversion, the reference value is entered in mVolts/10
					//and coincides with the correction value
					SO2_Corr = (int8_t)lrintf(SO2ref);
//					SO2_Corr = (int8_t)lrintf(SO2ref - GAS_Values.SO2);
					updated = true;
				}
				break;
			case 0x43:	//"C"
			case 0x63:	//"c"
				app.usbbuf[len-1] = 0;
				PrintNumHeader((uint8_t*)mystring18c, NULL, C6H6_Corr, false, "mV");
				PrintNumHeader((uint8_t*)mystring18a, (uint8_t*)mystring18b, C6H6, false, "ug/m3");
				GetNumericString(false, true);
				C6H6ref = atoi((const char*)mystring3);
				//Calculate the new C6H6_Corr
				if (strlen((const char*)(mystring3)))
				{
					//While waiting to implement the ug/m3->Volts conversion, the reference value is entered in mVolts/10
					//and coincides with the correction value
					C6H6_Corr = (int8_t)lrintf(C6H6ref);
//					C6H6_Corr = (int8_t)lrintf(C6H6ref - GAS_Values.C6H6);
					updated = true;
				}
				break;
	#endif	//OUTDOOR_MODE
#endif	//GAS_SENSOR_MODULE_PRESENT
#if (USE_BKUP_SRAM==1)
			case 0x47:	//"G"
			case 0x67:	//"g"
				app.usbbuf[len-1] = 0;
				PrintTextHeader((uint8_t*)mystring22a, (uint8_t*)mystring22b, NULL, "");
				GetNumericString(false, false);
				if (strlen((const char*)(mystring3)))
				{
					if (!((strcmp((const char*)mystring3, "y")) && (strcmp((const char*)mystring3, "Y"))))
					{
						memset(&BakUpRTC_Data[6], 0x00, 68);
						enable_backup_rtc();
						writeBkpRTC((uint8_t *)BakUpRTC_Data, sizeof(BakUpRTC_Data), 0);
						disable_backup_rtc();
						PrintTextHeader((uint8_t*)mystring22c, NULL, NULL, "");
					} else
					{
						PrintTextHeader((uint8_t*)mystring22d, NULL, NULL, "");
					}
				} else
				{
					PrintTextHeader((uint8_t*)mystring22d, NULL, NULL, "");
				}
				break;
#endif
#if (UWB_MODE==1)
			case 0x45:	//"E"
			case 0x65:	//"e"
				app.usbbuf[len-1] = 0;
				OpMode = (uint8_t)((FlashDataOrg.b_status.s0 >> 8) & 0x000000FF);
				if BIT_CHECK(OpMode,0)
	#if (RTLS_FW==1)
					strcpy(node_opmode, "RTLS");
	#elif (RANGING_FW==1)
				strcpy(node_opmode, "RNG");
	#endif
				else if BIT_CHECK(OpMode,1)
					strcpy(node_opmode, "DATA");
				PrintTextHeader((uint8_t*)mystring19a, (uint8_t*)mystring19b, (uint8_t*)node_opmode, "");
				GetNumericString(false, false);
				strncpy(TempBuf, (char*)&mystring3[0], 1);
	#if (RTLS_FW==1)
				if ((*TempBuf == 'l') || (*TempBuf == 'L'))
	#elif (RANGING_FW==1)
				if ((*TempBuf == 'r') || (*TempBuf == 'R'))
	#endif
				{
					BIT_SET(OpMode,0);
					BIT_CLEAR(OpMode,1);
				}
				else if ((*TempBuf == 'd') || (*TempBuf == 'D'))
				{
					BIT_SET(OpMode,1);
					BIT_CLEAR(OpMode,0);
				}
				OpMode &= 0x03;
				if (strlen((const char*)(mystring3)) == 1)
				{
					updated = true;
				}
				break;
			case 0x46:	//"F"
			case 0x66:	//"f"
				app.usbbuf[len-1] = 0;
				memcpy(node_role, &FlashDataOrg.b_status.s2, 4);
				PrintTextHeader((uint8_t*)mystring20a, (uint8_t*)mystring20b, (uint8_t*)node_role, "");
				GetNumericString(false, false);
	#if (RTLS_FW==1)
				BIT_SET(s1switch,0);			//Bit 0 is always "1"
				//Set Node Role (TAG/ANCHOR)
				strncpy(TempBuf, (char*)&mystring3[0], 1);
				if ((*TempBuf == 'a') || (*TempBuf == 'A'))
					BIT_SET(s1switch,3);
				else if ((*TempBuf == 't') || (*TempBuf == 'T'))
					BIT_CLEAR(s1switch,3);
				//Set Node Number (TAG: 0..7; ANCHOR: 0..3)
				strncpy(TempBuf, (char*)&mystring3[1], 1);
				uint8_t NodeNumber = atoi((const char*)TempBuf);
				if (BIT_CHECK(NodeNumber, 0))
					BIT_SET(s1switch,6);
				else
					BIT_CLEAR(s1switch,6);

				if (BIT_CHECK(NodeNumber, 1))
					BIT_SET(s1switch,5);
				else
					BIT_CLEAR(s1switch,5);

				if (BIT_CHECK(NodeNumber, 2))
					BIT_SET(s1switch,4);
				else
					BIT_CLEAR(s1switch,4);
				//Set Data Rate (L = 110Kb/s; S = 6.8Mb/s)
				strncpy(TempBuf, (char*)&mystring3[2], 1);
				if ((*TempBuf == 's') || (*TempBuf == 'S'))
					BIT_SET(s1switch,1);
				else if ((*TempBuf == 'l') || (*TempBuf == 'L'))
					BIT_CLEAR(s1switch,1);
				//Set Channel (2 = 3.993GHz; 5 = 6.489GHz)
				strncpy(TempBuf, (char*)&mystring3[3], 1);
				if (*TempBuf == '5')
					BIT_SET(s1switch,2);
				else if (*TempBuf == '2')
					BIT_CLEAR(s1switch,2);

/*				SWITCH((char*)(mystring3))
					CASE ("t0l2")
					CASE ("T0L2")
						strcpy(node_role, (const char*)(mystring3));
						FlashDataOrg.b_status.s0 = TAG0_L2;
						break;
					CASE ("t1l2")
					CASE ("T1L2")
						strcpy(node_role, (const char*)(mystring3));
						FlashDataOrg.b_status.s0 = TAG1_L2;
						break;
					DEFAULT
						break;
				END */
				if (strlen((const char*)(mystring3)) == 4)
				{
					memset(node_role, 0, strlen(node_role));
					strncpy(node_role, (char*)&mystring3[0], 4);
					upstr(node_role);
					memcpy(&FlashDataOrg.b_status.s2, node_role, 4);
					updated = true;
				}
	#elif (RANGING_FW==1)
				s1switch |= 0x07;				//Bit 0..2 are always "1"
				BIT_SET(s1switch,5);			//Bit 5 is always "1"
				//Set Node Role (TAG/ANCHOR)
				strncpy(TempBuf, (char*)&mystring3[0], 1);
				if ((*TempBuf == 'a') || (*TempBuf == 'A'))
					BIT_SET(s1switch,3);
				else if ((*TempBuf == 't') || (*TempBuf == 'T'))
					BIT_CLEAR(s1switch,3);
				//Set Data Rate (L = 110Kb/s; S = 6.8Mb/s)
				strncpy(TempBuf, (char*)&mystring3[1], 1);
				if ((*TempBuf == 's') || (*TempBuf == 'S'))
					BIT_SET(s1switch,4);
				else if ((*TempBuf == 'l') || (*TempBuf == 'L'))
					BIT_CLEAR(s1switch,4);
				//Set Channel (2 = 3.993GHz; 5 = 6.489GHz)
				strncpy(TempBuf, (char*)&mystring3[2], 1);
				if (*TempBuf == '5')
					BIT_SET(s1switch,6);
				else if (*TempBuf == '2')
					BIT_CLEAR(s1switch,6);

				if (strlen((const char*)(mystring3)) == 3)
				{
					memset(node_role, 0, strlen(node_role));
					strncpy(node_role, (char*)&mystring3[0], 3);
					upstr(node_role);
					memcpy(&FlashDataOrg.b_status.s2, node_role, 4);
					updated = true;
				}
	#endif	//RTLS_FW==0 RANGING_FW==1
				break;
#elif (PWR_NODE_MODE==1)	//UWB_MODE==1
			case 0x45:	//"E"
			case 0x65:	//"e"
				app.usbbuf[len-1] = 0;
				OpMode = (uint8_t)((FlashDataOrg.b_status.s0 >> 8) & 0x000000FF);
				if BIT_CHECK(OpMode,0)
					strcpy(node_opmode, "MLRS");
				else if BIT_CHECK(OpMode,1)
					strcpy(node_opmode, "ENAV");
				PrintTextHeader((uint8_t*)mystring19a, (uint8_t*)mystring19b, (uint8_t*)node_opmode, "");
				GetNumericString(false, false);
				strncpy(TempBuf, (char*)&mystring3[0], 1);
				if ((*TempBuf == 'm') || (*TempBuf == 'M'))
				{
					BIT_SET(OpMode,0);
					BIT_CLEAR(OpMode,1);
				}
				else if ((*TempBuf == 'e') || (*TempBuf == 'E'))
				{
					BIT_SET(OpMode,1);
					BIT_CLEAR(OpMode,0);
				}
				OpMode &= 0x03;
				if (strlen((const char*)(mystring3)) == 1)
				{
					updated = true;
				}
				break;
#endif
			case 0x04:
				memcpy(&mystring3[0], (const uint8_t *) "EOT", 3);
				L80_menu_Strings();
				app.usbbuf[len-1] = 0;
				break;
			case 0x1B:
				if (updated)
				{
					updated = false;
#if (GAS_SENSOR_MODULE_PRESENT==1)
	#if (OUTDOOR_MODE)
					//Pack-> Bit 0..7: CH2O_Corr; Bit 7..15: O3_Corr; Bit 16..23: NO2_Corr; Bit 24..32: NH3_Corr
					FlashDataOrg.b_status.s9 = ((uint32_t)(NH3_Corr & 0xFF) << 24) | ((uint32_t)(NO2_Corr & 0xFF) << 16) |
											   ((uint32_t)(O3_Corr & 0xFF) << 8) | (uint32_t)(CH2O_Corr & 0xFF);
					//Pack-> Bit 0..7: CO_Corr; Bit 7..15: SO2_Corr; Bit 16..23: C6H6_Corr; Bit 24..32: Spare_Corr
					FlashDataOrg.b_status.sa = ((uint32_t)(Spare_Corr & 0xFF) << 24) | ((uint32_t)(C6H6_Corr & 0xFF) << 16) |
											   ((uint32_t)(SO2_Corr & 0xFF) << 8) | (uint32_t)(CO_Corr & 0xFF);
	#else
					//Pack-> Bit 0..7: CH2O_Corr; Bit 16..23: NO2_Corr; Bit 24..32: NH3_Corr
					FlashDataOrg.b_status.s9 &= 0x0000FF00;
					FlashDataOrg.b_status.s9 = ((uint32_t)(NH3_Corr & 0xFF) << 24) | ((uint32_t)(NO2_Corr & 0xFF) << 16) |
											   (uint32_t)(CH2O_Corr & 0xFF);
					//Pack-> Bit 0..7: CO_Corr
					FlashDataOrg.b_status.sa &= 0xFFFFFF00;
					FlashDataOrg.b_status.sa |= (uint32_t)(CO_Corr & 0x000000FF);
	#endif	//OUTDOOR_MODE
#endif	//GAS_SENSOR_MODULE_PRESENT
#if ((UWB_MODE==1) || (PWR_NODE_MODE==1))
					//Pack-> Bit 0..7: Node Role; Bit 7..15: Op Mode; Bit 16..23: Res1; Bit 24..32: Res2
					FlashDataOrg.b_status.s0 = ((uint32_t)(Res2 & 0xFF) << 24) | ((uint32_t)(Res1 & 0xFF) << 16) |
											   ((uint32_t)(OpMode & 0xFF) << 8) | (uint32_t)(s1switch & 0xFF);
#endif	//UWB_MODE==1
#if (WRITE_FLASH==1)
					//Update the new calibration values when exit
					CDC_Tx_FS((uint8_t*)mystring_w1,strlen((const char*)mystring_w1));
					Write_Flash(0, 0);
					memset(mystring3, 0, sizeof(mystring3));
					CDC_Tx_FS((uint8_t*)mystring8c,strlen((const char*)mystring8c));
#endif	//WRITE_FLASH
				}
				memcpy(&mystring3[0], (const uint8_t *) "ESC", 3);
				top_menu();
				break;
			default:
				memset(mystring3, 0, sizeof(mystring3));
				break;
		}
	}
	while (sel != 0x1B);

}	// end L80_menu
#endif	/* UTILITIES */
