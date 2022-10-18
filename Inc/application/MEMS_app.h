/*! ----------------------------------------------------------------------------
 * @file	MEMS_app.h
 * @brief	My MEMS Application Programs Collection Headers
 *          T.B.D.
 *
 * @author  Tommaso Sabatini, 2019
 */

#ifndef MEMS_APP_H_
#define MEMS_APP_H_

#include "platform/port.h"
#include "application/SerialProtocol.h"
#include "main.h"
#include <stdint.h>

#define STREAMING_MSG_LENGTH		115
#define DataStreamingDest			0x01

#define FW_ID "4"
#define FW_VERSION "8.0.0"
#if (USE_STM32F4XX_NUCLEO==1)
	#define LIB_VERSION "8.2.0"
#elif (defined (USE_STM32L0XX_NUCLEO))
	#define LIB_VERSION "8.1.0"
#elif (USE_SENSUS191==1)
	#define LIB_VERSION "8.2.0"
#else
#error Not supported platform
#endif

#if (USE_SENSUS191==1)
	#define EXPANSION_BOARD "IKS01A2"
#else
#error Not supported shield
#endif

/* MotionFX Exported types ---------------------------------------------------*/
/**
 * @brief  Serial message array indexes enumeration based on DataLogCustomLite application
 */
typedef enum
{
  DEST        = 0,
  SOURCE      = DEST      + 1,
  COMMAND     = SOURCE    + 1,
  RTC_H       = COMMAND   + 1,
  RTC_M       = RTC_H     + 1,
  RTC_S_INT   = RTC_M     + 1,
  RTC_S_DEC   = RTC_S_INT + 1,
  STREAM_DATA = RTC_S_DEC + 1,
} MSG_INDEXES;

typedef struct
{
  uint8_t info_type;
  uint8_t info_index;
  uint16_t variable_count;
  uint8_t variable_type;
  uint16_t stream_position;
  const char *config_string;
  uint8_t already_processed;
  void *p_node;
} sDISPLAY_INFO,*psDISPLAY_INFO;

typedef struct
{
  uint8_t var_count;
  uint8_t var_type;
  uint8_t conn_size;
  uint8_t conn[1];
} sCONFIG_RECORD;

/* MotionAC Exported types ---------------------------------------------------*/
typedef enum
{
  DYNAMIC_CALIBRATION,
  SIX_POINT_CALIBRATION
} MAC_calibration_mode_t;

typedef enum
{
  MAC_DISABLE_LIB,
  MAC_ENABLE_LIB
} MAC_enable_lib_t;

/* Exported macro ------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define VAR_TYPE_FLOAT    1
#define VAR_TYPE_INT32    2
#define VAR_TYPE_BIT      3
#define VAR_TYPE_INPUT    0x80

#define INFO_TYPE_ABITS   1
#define INFO_TYPE_AINT32  2
#define INFO_TYPE_AFLOAT  3
#define INFO_TYPE_GRAPH   4
#define INFO_TYPE_LOGIC   5
#define INFO_TYPE_SCATTER 6
#define INFO_TYPE_FUSION  7
#define INFO_TYPE_INT32   8
#define INFO_TYPE_FLOAT   9
#define INFO_TYPE_BAR     10
#define INFO_TYPE_HISTO   11
#define INFO_TYPE_PLOT3D  12
#define INFO_TYPE_FFT     13

/* Exported variables --------------------------------------------------------*/
typedef enum SensorCommands
{
	SC_ACCELEROMETER		= 0x01,
	SC_GYROSCOPE 			= 0x02,
	SC_MAGNETOMETER			= 0x03,
	SC_TEMPERATURE			= 0x04,
	SC_HUMIDITY				= 0x05,
	SC_PRESSURE 			= 0x06,
	SC_UV					= 0x07,
	SC_SEND_DEFAULTS 		= 0xFF,
	SC_GET_SENSOR_NAME		= 0x01,
	SC_READ_REGISTER		= 0x02,
	SC_WRITE_REGISTER 		= 0x03,
	SC_GET_FULL_SCALE_LIST	= 0x04,
	SC_SET_FULL_SCALE 		= 0x05,
	SC_GET_ODR_LIST 		= 0x06,
	SC_SET_ODR 				= 0x07,
	SC_GET_CUSTOM_NAMES 	= 0x08,
	SC_GET_CUSTOM_CONFIG 	= 0x09,
	SC_GET_CUSTOM_NAMES2	= 0x0A,
	SC_GET_CUSTOM_NAMES3	= 0x0B,
	SC_SET_CUSTOM_VALUES	= 0x0C,
	SC_GET_CONFIG_STRING	= 0x19,
	SC_GET_CUSTOM_VALUES	= 0x1A
} SENSOR_COMMANDS;

typedef enum MEMSapplicationModes
{
	CMD_Ping				= 0x01,
	CMD_Read_PresString		= 0x02,
	CMD_NACK				= 0x03,
	CMD_CheckModeSupport	= 0x04,
	CMD_UploadXX			= 0x05,
	CMD_ChangeSF			= 0x07,
	CMD_Start_Data_Strmng	= 0x08,
	CMD_Stop_Data_Strmng	= 0x09,
	CMD_Set_DateTime		= 0x0C,
	CMD_Enter_DFU_Mode		= 0x0E,
	CMD_Reset				= 0x0F,
	CMD_Rst					= 0x12,	//'^R'
	CMD_Sensor				= 0x50,
	CMD_Algo_Mode			= 0x51,
	CMD_PRESSURE_Init		= 0x60,
	CMD_HUM_TEMP_Init		= 0x62,
	CMD_ACC_GYRO_Init		= 0x76,
	CMD_MAGNETO_Init		= 0x7A,
	CMD_Reply_Add			= 0x80,
	CMD_Get_Measurement		= 0x86,	//Get Multi-in-One ZPHS01B Sensor Module measurement
	CMD_OpenTestEnv_Mode	= 0x10	//'^P'
} MEMS_APP_MODE;

typedef enum RmtCntrlCmd
{
	CMD_Check				= 0x23, //'#'
	CMD_Read_Version		= 0x56,	//'V'
	CMD_Set_Date_Time		= 0x54,	//'T'
	CMD_Get_Date_Time		= 0x47,	//'G'
	CMD_OpenTestEnv			= 0x10,	//'^P'
	CMD_Restart				= 0x12	//'^R'
} RMT_CNTRL_CMD;

MEMS_APP_MODE mems_applicazion_mode;
RMT_CNTRL_CMD rmt_cntrl_cmd;
bool SendCntrlMsg;

#if (PRESSURE_SENSOR_PRESENT==1)			//Defined in main.h
	#include "platform/LPS25HB_Driver.h"
	//Conversion formula from absolute pressure to sea-level pressure
	#define P0(p,h,t)	((p) * (pow((1.0F - (0.0065F * (h))/((t) + 273.15F + 0.0065F * (h))), -5.257)))
	LPS25HB_MeasureTypeDef_st PRS_Values;
	int16_t T_Out;
	int32_t P_Out;							//Absolute Pressure
	int32_t P0_Out;							//Sea-level pressure
	double_t P_OutDbl;						//Absolute Pressure
	double_t P0_OutDbl;						//Sea-level pressure
	double_t Altitude;
	double_t Pressure;
	double_t Delta_h;						//Tag - Anchor Delta-Altitude
	float press_value, temp_value;
	#if (GUI_SUPPORT==1)			//Defined in main.h
		float Temperature_C_1_data[1], Temperature_C_2_data[1], Pressure_hPa_1_data[1], Altitude_C_1_data[1];
		float Mux_Float_1_out[8];	//Used in UnicleoGUI. It output also eTVOC, eCO2 values
	#endif
#endif
	double_t Temperature;
#if (HUMIDITY_SENSOR_PRESENT==1)			//Defined in main.h
	#include "platform/HTS221_Driver.h"
	// Celsius(°C) to Fahrenheit(°F) Conversion
	#define C2F(x)	((1.8F * (x)) + 32.0F)
	// Fahrenheit(°F) to Celsius(°C) Conversion
	#define F2C(x)	(0.5555F * ((x) - 32.0F))
	// Heat Index Formula Coefficients
	#define C1	-42.379F
	#define C2	2.04901523F
	#define C3	10.14333127F
	#define C4	-0.22475541F
	#define C5	-6.83783e-3
	#define C6	-5.481717e-2
	#define C7	1.22874e-3
	#define C8	8.5282e-4
	#define C9	-1.99e-6
	// Heat Index Formula
	#define HI(x,y)		(C1 + C2*(x) + C3*(y) + C4*(x)*(y) + C5*((x)*(x)) + C6*((y)*(y)) + C7*(((x)*(x))*(y)) + \
						 C8*(((y)*(y))*(x)) + C9*(((x)*(x))*((y)*(y))))
	// Sinner Index Formula
	#define SSI(x,y)	(1.98F * ((x) - (0.5555F - 0.0055F * (y)) * ((x) - 58.0F)) - 56.83F)
	HTS221_MeasureTypeDef_st HUM_Values;
	uint16_t Hum_Out;
	uint16_t T2_Out, T3_Out;	//Used by BLE in app_bluenrg_2.c User_Process() function
	uint8_t	Humidity;
	float TemperatureD, hum_value, temp_value, SI, HI;
	#if (GUI_SUPPORT==1)			//Defined in main.h
		float Humidity_percent_1_data[1];	//Used in UnicleoGUI
	#endif
#endif
#if (UVx_SENSOR_PRESENT==1)					//Defined in main.h
	#include "platform/VEML6075_Driver.h"
	VEML6075_MeasureTypeDef_st UVx_Values;
	float UVa, UVb, UV_Index;
	#if (GUI_SUPPORT==1)					//Defined in main.h
		float UVA_1_data[1], UVB_1_data[1];	//Used in UnicleoGUI
	#endif
#endif
#if (VOC_SENSOR_PRESENT==1)					//Defined in main.h
	#if (CCS811)
		#include "platform/CCS811_Driver.h"
		CCS811_MeasureTypeDef_st VOC_Values;
	#elif (ENS160)
		#include "platform/ENS160_Driver.h"
		ENS160_MeasureTypeDef_st VOC_Values;
	#endif
	uint16_t eq_TVOC, eq_CO2;
	uint16_t eq_TVOC_1h_Mean, eq_CO2_8h_Mean;
	uint32_t CO_Out;	//Used by BLE in app_bluenrg_2.c User_Process() function
	#if (GUI_SUPPORT==1)					//Defined in main.h
		int32_t eTVOC_1_data[1], eCO2_1_data[1];
	#endif
#endif
#if (PARTICULATE_SENSOR_PRESENT==1)			//Defined in main.h
	#define PM1p0_CORRECTION(x)		0.7301*(x) + 0.9695
	#define PM2p5_CORRECTION(x)		0.7166*(x) + 2.5472
	#define PM4p0_CORRECTION(x)		0.7166*(x) + 6.4032
	#define PM10p0_CORRECTION(x)	0.6448*(x) + 12.8063
	#include "platform/SPS30_Driver.h"
	SPS30_MeasureTypeDef_st PMS_Values;
	uint16_t MC_1p0, MC_2p5, MC_4p0, MC_10p0, NC_0p5, NC_1p0, NC_2p5, NC_4p0, NC_10p0;
	uint16_t MC_1p0_24h_Mean, MC_2p5_24h_Mean, MC_4p0_24h_Mean, MC_10p0_24h_Mean;
	float TypicalParticleSize;
	#if (GUI_SUPPORT==1)			//Defined in main.h
		int32_t PM1p0_1_data[1], PM1p0_2_data[1], PM2p5_1_data[1], PM2p5_2_data[1];	//Used in UnicleoGUI
		int32_t PM4p0_1_data[1], PM4p0_2_data[1], PM10_1_data[1], PM10_2_data[1];	//Used in UnicleoGUI
	#endif
#endif
#if (GAS_SENSOR_MODULE_PRESENT==1)			//Defined in main.h
	#include "platform/ANLG_Driver.h"
	ANLG_MeasureTypeDef_st GAS_Values;
	uint16_t CH2O, CH2O_8h_Mean, CO, CO_8h_Mean;
	#if (FULL_MODE==1)
		uint16_t NO2, NH3, O3, SO2, C6H6;
		uint16_t NO2_1h_Mean, NH3_8h_Mean, O3_8h_Mean, SO2_1h_Mean, C6H6_24h_Mean;
	#endif
	#if (GUI_SUPPORT==1)			//Defined in main.h
		int32_t CO_data[1]; int32_t CH2O_data[1];	//For UnicleoGUI
		#if (FULL_MODE==1)
			int32_t NO2_data[1]; int32_t NH3_data[1]; int32_t O3_data[1];	//For UnicleoGUI
			int32_t SO2_data[1]; int32_t C6H6_data[1];						//For UnicleoGUI
		#endif
	#endif
	#if (VOC_SENSOR_PRESENT==0)
		uint32_t CO_Out;	//Used by BLE in app_bluenrg_2.c User_Process() function
	#endif
#endif
#if (IMU_PRESENT==1)						//Defined in main.h
	#include "platform/LSM9DS1_Driver.h"
	#include "application/MotionGC_Manager.h"
	#include "application/motion_gc.h"
	#include "application/MotionFX_Manager.h"
	#include "application/motion_fx.h"
	#include "application/MotionAR_Manager.h"
	#include "application/motion_ar.h"
	#include "application/MotionAC_Manager.h"
	#include "application/motion_ac.h"
	MFX_input_t data_in;
	LSM9DS1_MeasureTypeDef_st IMU_Values;
	SensorAxes_t MagOffset;
	uint16_t avg_cnt;
	uint8_t MagCalRequest, MagCalStatus, AccCalRequest, AccCalStatus, ActRecRequest, GyroCalRequest, GyroCalStatus;
	float ans_float;
	float Heading_9X_1_data[1], Quaternions_9X_1_data[4];
	float Rotation_Vector_9X_1_data[3], Linear_Acceleration_9X_1_data[3], Linear_Speed_9X_1_data[5];
	float Linear_Acceleration_9X_0_data[3], Linear_Speed_9X_0_data[5];	//Previous acceleration and velocity samples
	int32_t Calibrated_Acceleration_g_1_quality[1], Calibrated_Magnetic_Field_uT_1_quality[1], Calibrated_Gyroscope_g_1_status[1], Activity_Index_1_data[1];
	int32_t Input_Value_Int_1_out[1], Mux_Int_1_out[4], Mux_Int_2_out[8];
	extern const double_t g_to_ms2;			//g to m/s^2 conversion factor
	extern const double_t mg_to_g;			//mg to g conversion factor = 0.001
	extern const double_t mg_to_ms2;		//mg to m/s^2 conversion factor = 0.00980665
	extern const double_t mdps_to_dps;		//mdps to dps conversion factor = 0.001
	extern const double_t mGauss_to_uT;		//mGauss to uTesla conversion factor = 0.1
	extern const double_t mGauss_to_uT50;	//mGauss to uTesla/50 conversion factor = 0.002
	extern const double_t uT50_to_mGauss;	//uTesla/50 to mGauss conversion factor = 500.0
	#if ((PRESSURE_SENSOR_PRESENT==0) && (IMU_PRESENT==1))			//Defined in main.h
		double_t Pressure;
		double_t Altitude;
		int16_t T_OutRaw, T_Out;
	#endif
	MFX_MagCal_output_t mag_cal_test, mag_data_out;
	MFX_MagCal_input_t mag_data_in;
	MAC_calibration_mode_t CalibrationMode;
	char FX_lib_version[35], GC_lib_version[35], AR_lib_version[35], AC_lib_version[35];
	int FX_lib_version_len, GC_lib_version_len, AR_lib_version_len, AC_lib_version_len;
#else
	float Constant_Float_1_out[1], Divide_1_out[1];
	int32_t Mux_Int_1_out[4], Mux_Int_2_out[4], Mux_Int_3_out[4];
#endif

/* Exported functions ------------------------------------------------------- */
/**
 * @brief  Display function block update
 */
void Display_Update(void *source, sDISPLAY_INFO *header);
void AB_Init(void);
void AB_Handler(void);
#if (IMU_PRESENT==1)						//Defined in main.h
	void AC_Handler(int* Len, LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff);
#endif
int SC_Get_Custom_Config(uint8_t* Buff);

/**
  * @brief  Handle Sensors command
  * @param  Msg the pointer to the message to be handled
  * @param  custom_values the pointer to the custom values
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int Handle_Sensor_command(uint8_t* Buff);

/**
 * @brief  Inputs function block update
 */
void Input_Value_Init(void *source, sDISPLAY_INFO *header);

#if (PRESSURE_SENSOR_PRESENT==1)			//Defined in main.h
/**
 * @brief  Handles the LPS25HB PRESS, TEMP sensor data getting/sending.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Fill the PRESS, TEMP parts of the Buff stream
 * @param  PressTemp: Pointer to PressTemp data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
void Pressure_Sensor_Handler(LPS25HB_MeasureTypeDef_st *PressTemp, uint8_t* Buff);
#endif

#if (HUMIDITY_SENSOR_PRESENT==1)			//Defined in main.h
/**
 * @brief  Handles the HTS221 HUM, TEMP sensor data getting/sending.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Fill the HUM parts of the Buff stream
 * @param  HumTemp: Pointer to HumTemp data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
void Humidity_Sensor_Handler(HTS221_MeasureTypeDef_st *HumTemp, uint8_t* Buff);
#endif

#if (UVx_SENSOR_PRESENT==1)					//Defined in main.h
/**
 * @brief  Handles the VEML6075 UVA UVB, UV Index sensor data getting/sending.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Fill the UV lights parts of the Buff stream
 * @param  UVx: Pointer to UV data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
void UVx_Sensor_Handler(VEML6075_MeasureTypeDef_st *UVx, uint8_t* Buff);
#endif

#if (VOC_SENSOR_PRESENT==1)					//Defined in main.h
/**
 * @brief  Handles the CCS811/ENS160 sensor data getting/sending.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Calculate the hourly/daily average of the concentration values
 * @brief  Fill the eTVOC, eCO2 parts of the Buff stream
 * @param  voc: Pointer to eVOC_eCO2 data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
	#if (CCS811)
	void VOC_Sensor_Handler(CCS811_MeasureTypeDef_st *voc, uint8_t* Buff);
	#elif (ENS160)
	void VOC_Sensor_Handler(ENS160_MeasureTypeDef_st *voc, uint8_t* Buff);
	#endif
#endif

#if (PARTICULATE_SENSOR_PRESENT==1)			//Defined in main.h
/**
 * @brief  Handles the SPS30 Particulate Matter sensor data getting/sending.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Calculate the hourly/daily average of the concentration values
 * @brief  Fill the Particulate parts of the Buff stream
 * @param  Particulate: Pointer to PMx data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
void Particulate_Sensor_Handler(SPS30_MeasureTypeDef_st *Particulate, uint8_t* Buff);
#endif

#if (GAS_SENSOR_MODULE_PRESENT==1)			//Defined in main.h
/**
 * @brief  Handles the analog gas sensor board data getting.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Calculate the hourly/daily average of the concentration values
 * @brief  Fill the gases parts of the Buff stream
 * @param  anlg: Pointer to analog inputs data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
void Gas_Sensor_Handler(ANLG_MeasureTypeDef_st *anlg, uint8_t* Buff);
#endif

void Refresh_AQI(void);

#if (IMU_PRESENT==1)						//Defined in main.h
/**
* @brief  Handles the LSM9DS1 ACC, GYR, MAG axes data getting/sending
* @brief  Build the Buff array from the uint32_t (LSB first)
* @brief  Fill the ACC, GYR, MAG parts of the Buff stream
* @param  IMU_Axes: Axes pointer to axes data structure
* @param  Buff: Stream pointer
* @retval None
*/
void IMU_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff);

/**
 * @brief  Magnetometer calibration data handler (By MotionFX)
 * @param  Buff: Pointer of the Magnetometer calibration part of the stream
 * @retval None
 */
void MC_Data_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff);

/**
 * @brief  Gyroscope calibration data handler
 * @param  Buff: Pointer of the Gyroscope calibration part of the stream
 * @retval None
 */
void GC_Data_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff);

/**
 * @brief  Accelerometer calibration data handler
 * @param  Buff: Pointer of the Accelerometer calibration part of the stream
 * @retval None
 */
void AC_Data_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff);

/**
 * @brief  Activity Recognition data handler
 * @param  Msg the Activity Recognition data part of the stream
 * @retval None
 */
void AR_Data_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff);

/**
 * @brief  Sensor Fusion data handler
 * @param  IMU_Axes: Axes pointer to axes data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
void FX_Data_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff);

/**
 * @brief  Velocity estimation from linear acceleration
 * @param  LinAcc_0: pointer to previous acceleration sample (Linear_Acceleration_9X_0_data)
 * @param  LinAcc_1: pointer to current acceleration sample (Linear_Acceleration_9X_1_data)
 * @param  LinVel_0: pointer to previous velocity sample (Linear_Speed_9X_0_data)
 * @param  LinVel_1: pointer to current velocity sample (Linear_Speed_9X_1_data)
 * @param  CalGyroStatus: pointer to Calibrated_Gyroscope_g_1_status
 * 		   Used to recognize the IMU stationary status: velocity is set to 0
 * 		   when this parameter is 1.
 * @retval None
 */
void VE_Data_Handler(float *LinAcc_0, float *LinAcc_1, float *LinVel_0, float *LinVel_1, int32_t* CalGyroStatus);

#endif

#endif /* MEMS_APP_H_ */
