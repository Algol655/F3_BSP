/*
 ******************************************************************************
 * @file    LSM9DS1_Driver.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          LSM9DS1_Driver.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 * Last committed from 
 * GITHUB/STMicroelectronics/STMems_Standard_C_drivers/lsm9ds1: 2020-07-03
 ******************************************************************************
 */
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LSM9DS1_DRIVER_H
#define LSM9DS1_DRIVER_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "arm_math.h"
// User include starts here
#if defined(STM32F405xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F105xC)
	#include "stm32f1xx_hal.h"
#endif
#include "i2c.h"
// User include ends here

/** @addtogroup LSM9DS1
  * @{
  *
  */

/** @defgroup LSM9DS1_sensors_common_types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

/**
  * @defgroup axisXbitXX_t
  * @brief    These unions are useful to represent different sensors data type.
  *           These unions are not need by the driver.
  *
  *           REMOVING the unions you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  * @{
  */
typedef union
{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} Type3Axis16bit_t;

typedef union
{
  int16_t i16bit;
  uint8_t u8bit[2];
} Type1Axis16bit_t;

typedef union
{
  int32_t i32bit[3];
  uint8_t u8bit[12];
} Type3Axis32bit_t;

typedef union
{
  int32_t i32bit;
  uint8_t u8bit[4];
} Type1Axis32bit_t;

/**
  * @}
  *
  */
typedef struct
{
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
} bitwise_t;

typedef enum
{
  LSM9DS1_OK	=   0x00,
  LSM9DS1_ERROR	=   0x01
} LSM9DS1_Error_et;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

#endif /* MEMS_SHARED_TYPES */

/**
  * @}
  *
  */

  /** @addtogroup  LSM9DS1_Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*lsm9ds1_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*lsm9ds1_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct 
{
  /** Component mandatory fields **/
  lsm9ds1_write_ptr  write_reg;
  lsm9ds1_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} LSM9DS1_CTX_t;

/**
  * @}
  *
  */

/** @defgroup LSM9DS1_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> 0xD5 if SA0=1 -> 0xD7 **/
//#define LSM9DS1_IMU_I2C_ADD_L      0xD5U
#define LSM9DS1_IMU_I2C_ADD_L      0xD6U
#define LSM9DS1_IMU_I2C_ADD_H      0xD6U

/** I2C Device Address 8 bit format  if SA1=0 -> 0x39 if SA1=1 -> 0x3D **/
#define LSM9DS1_MAG_I2C_ADD_L      0x3CU
#define LSM9DS1_MAG_I2C_ADD_H      0x3CU

/** Device Identification (Who am I) **/
#define LSM9DS1_IMU_ID             0x68U

/** Device Identification (Who am I) **/
#define LSM9DS1_MAG_ID             0x3DU

/**
  * @}
  *
  */

#define LSM9DS1_ACT_THS            0x04U
typedef struct 
{
  uint8_t act_ths                  : 7;
  uint8_t sleep_on_inact_en        : 1;
} LSM9DS1_ACT_THS_t;

#define LSM9DS1_ACT_DUR            0x05U
#define LSM9DS1_INT_GEN_CFG_XL     0x06U
typedef struct 
{
  uint8_t xlie_xl                  : 1;
  uint8_t xhie_xl                  : 1;
  uint8_t ylie_xl                  : 1;
  uint8_t yhie_xl                  : 1;
  uint8_t zlie_xl                  : 1;
  uint8_t zhie_xl                  : 1;
  uint8_t _6d                      : 1;
  uint8_t aoi_xl                   : 1;
} LSM9DS1_INT_GEN_CFG_XL_t;

#define LSM9DS1_INT_GEN_THS_X_XL   0x07U
#define LSM9DS1_INT_GEN_THS_Y_XL   0x08U
#define LSM9DS1_INT_GEN_THS_Z_XL   0x09U
#define LSM9DS1_INT_GEN_DUR_XL     0x0AU
typedef struct 
{
  uint8_t dur_xl                   : 7;
  uint8_t wait_xl                  : 1;
} LSM9DS1_INT_GEN_DUR_XL_t;

#define LSM9DS1_REFERENCE_G        0x0BU
#define LSM9DS1_INT1_CTRL          0x0CU
typedef struct 
{
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fth                 : 1;
  uint8_t int1_ovr                 : 1;
  uint8_t int1_fss5                : 1;
  uint8_t int1_ig_xl               : 1;
  uint8_t int1_ig_g                : 1;
} LSM9DS1_INT1_CTRL_t;

#define LSM9DS1_INT2_CTRL          0x0DU
typedef struct 
{
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fth                 : 1;
  uint8_t int2_ovr                 : 1;
  uint8_t int2_fss5                : 1;
  uint8_t not_used_01              : 1;
  uint8_t int2_inact               : 1;
} LSM9DS1_INT2_CTRL_t;

#define LSM9DS1_WHO_AM_I           0x0FU
#define LSM9DS1_CTRL_REG1_G        0x10U
typedef struct 
{
  uint8_t bw_g                     : 2;
  uint8_t not_used_01              : 1;
  uint8_t fs_g                     : 2;
  uint8_t odr_g                    : 3;
} LSM9DS1_CTRL_REG1_G_t;

#define LSM9DS1_CTRL_REG2_G        0x11U
typedef struct 
{
  uint8_t out_sel                  : 2;
  uint8_t int_sel                  : 2;
  uint8_t not_used_01              : 4;
} LSM9DS1_CTRL_REG2_G_t;

#define LSM9DS1_CTRL_REG3_G        0x12U
typedef struct 
{
  uint8_t hpcf_g                   : 4;
  uint8_t not_used_01              : 2;
  uint8_t hp_en                    : 1;
  uint8_t lp_mode                  : 1;
} LSM9DS1_CTRL_REG3_G_t;

#define LSM9DS1_ORIENT_CFG_G       0x13U
typedef struct 
{
  uint8_t orient                   : 3;
  uint8_t signz_g                  : 1;
  uint8_t signy_g                  : 1;
  uint8_t signx_g                  : 1;
  uint8_t not_used_01              : 2;
} LSM9DS1_ORIENT_CFG_G_t;

#define LSM9DS1_INT_GEN_SRC_G      0x14U
typedef struct 
{
  uint8_t xl_g                     : 1;
  uint8_t xh_g                     : 1;
  uint8_t yl_g                     : 1;
  uint8_t yh_g                     : 1;
  uint8_t zl_g                     : 1;
  uint8_t zh_g                     : 1;
  uint8_t ia_g                     : 1;
  uint8_t not_used_01              : 1;
} LSM9DS1_INT_GEN_SRC_G_t;

#define LSM9DS1_OUT_TEMP_L         0x15U
#define LSM9DS1_OUT_TEMP_H         0x16U
#define LSM9DS1_STATUS_REG         0x17U
typedef struct 
{
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t tda                      : 1;
  uint8_t boot_status              : 1;
  uint8_t inact                    : 1;
  uint8_t ig_g                     : 1;
  uint8_t ig_xl                    : 1;
  uint8_t not_used_01              : 1;
} LSM9DS1_STATUS_REG_t;

#define LSM9DS1_OUT_X_L_G          0x18U
#define LSM9DS1_OUT_X_H_G          0x19U
#define LSM9DS1_OUT_Y_L_G          0x1AU
#define LSM9DS1_OUT_Y_H_G          0x1BU
#define LSM9DS1_OUT_Z_L_G          0x1CU
#define LSM9DS1_OUT_Z_H_G          0x1DU
#define LSM9DS1_CTRL_REG4          0x1EU
typedef struct 
{
  uint8_t _4d_xl1                   : 1;
  uint8_t lir_xl1                  : 1;
  uint8_t not_used_01              : 1;
  uint8_t xen_g                    : 1;
  uint8_t yen_g                    : 1;
  uint8_t zen_g                    : 1;
  uint8_t not_used_02              : 2;
} LSM9DS1_CTRL_REG4_t;

#define LSM9DS1_CTRL_REG5_XL       0x1FU
typedef struct 
{
  uint8_t not_used_01              : 3;
  uint8_t xen_xl                   : 1;
  uint8_t yen_xl                   : 1;
  uint8_t zen_xl                   : 1;
  uint8_t dec                      : 2;
} LSM9DS1_CTRL_REG5_XL_t;

#define LSM9DS1_CTRL_REG6_XL       0x20U
typedef struct 
{
  uint8_t bw_xl                    : 2;
  uint8_t bw_scal_odr              : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 3;
} LSM9DS1_CTRL_REG6_XL_t;

#define LSM9DS1_CTRL_REG7_XL       0x21U
typedef struct 
{
  uint8_t hpis1                    : 1;
  uint8_t not_used_01              : 1;
  uint8_t fds                      : 1;
  uint8_t not_used_02              : 2;
  uint8_t dcf                      : 2;
  uint8_t hr                       : 1;
} LSM9DS1_CTRL_REG7_XL_t;

#define LSM9DS1_CTRL_REG8          0x22U
typedef struct 
{
  uint8_t sw_reset                 : 1;
  uint8_t ble                      : 1;
  uint8_t if_add_inc               : 1;
  uint8_t sim                      : 1;
  uint8_t pp_od                    : 1;
  uint8_t h_lactive                : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
} LSM9DS1_CTRL_REG8_t;

#define LSM9DS1_CTRL_REG9          0x23U
typedef struct 
{
  uint8_t stop_on_fth              : 1;
  uint8_t fifo_en                  : 1;
  uint8_t i2c_disable              : 1;
  uint8_t drdy_mask_bit            : 1;
  uint8_t fifo_temp_en             : 1;
  uint8_t not_used_01              : 1;
  uint8_t sleep_g                  : 1;
  uint8_t not_used_02              : 1;
} LSM9DS1_CTRL_REG9_t;

#define LSM9DS1_CTRL_REG10         0x24U
typedef struct 
{
  uint8_t st_xl                    : 1;
  uint8_t not_used_01              : 1;
  uint8_t st_g                     : 1;
  uint8_t not_used_02              : 5;
} LSM9DS1_CTRL_REG10_t;

#define LSM9DS1_INT_GEN_SRC_XL     0x26U
typedef struct 
{
  uint8_t xl_xl                    : 1;
  uint8_t xh_xl                    : 1;
  uint8_t yl_xl                    : 1;
  uint8_t yh_xl                    : 1;
  uint8_t zl_xl                    : 1;
  uint8_t zh_xl                    : 1;
  uint8_t ia_xl                    : 1;
  uint8_t not_used_01              : 1;
} LSM9DS1_INT_GEN_SRC_XL_t;

#define LSM9DS1_OUT_X_L_XL         0x28U
#define LSM9DS1_OUT_X_H_XL         0x29U
#define LSM9DS1_OUT_Y_L_XL         0x2AU
#define LSM9DS1_OUT_Y_H_XL         0x2BU
#define LSM9DS1_OUT_Z_L_XL         0x2CU
#define LSM9DS1_OUT_Z_H_XL         0x2DU
#define LSM9DS1_FIFO_CTRL          0x2EU
typedef struct 
{
  uint8_t fth                      : 5;
  uint8_t fmode                    : 3;
} LSM9DS1_FIFO_CTRL_t;

#define LSM9DS1_FIFO_SRC           0x2FU
typedef struct 
{
  uint8_t fss                      : 6;
  uint8_t ovrn                     : 1;
  uint8_t fth                      : 1;
} LSM9DS1_FIFO_SRC_t;

#define LSM9DS1_INT_GEN_CFG_G      0x30U
typedef struct 
{
  uint8_t xlie_g                   : 1;
  uint8_t xhie_g                   : 1;
  uint8_t ylie_g                   : 1;
  uint8_t yhie_g                   : 1;
  uint8_t zlie_g                   : 1;
  uint8_t zhie_g                   : 1;
  uint8_t lir_g                    : 1;
  uint8_t aoi_g                    : 1;
} LSM9DS1_INT_GEN_CFG_G_t;

#define LSM9DS1_INT_GEN_THS_XH_G   0x31U
typedef struct 
{
  uint8_t ths_g_x                  : 7;
  uint8_t dcrm_g                   : 1;
} LSM9DS1_INT_GEN_THS_XH_G_t;

#define LSM9DS1_INT_GEN_THS_XL_G   0x32U
typedef struct 
{
  uint8_t ths_g_x                  : 8;
} LSM9DS1_INT_GEN_THS_XL_G_t;

#define LSM9DS1_INT_GEN_THS_YH_G   0x33U
typedef struct 
{
  uint8_t ths_g_y                  : 7;
  uint8_t not_used_01              : 1;
} LSM9DS1_INT_GEN_THS_YH_G_t;

#define LSM9DS1_INT_GEN_THS_YL_G   0x34U
typedef struct 
{
  uint8_t ths_g_y                  : 8;
} LSM9DS1_INT_GEN_THS_YL_G_t;

#define LSM9DS1_INT_GEN_THS_ZH_G   0x35U
typedef struct 
{
  uint8_t ths_g_z                  : 7;
  uint8_t not_used_01              : 1;
} LSM9DS1_INT_GEN_THS_ZH_G_t;

#define LSM9DS1_INT_GEN_THS_ZL_G   0x36U
typedef struct 
{
  uint8_t ths_g_z                  : 8;
} LSM9DS1_INT_GEN_THS_ZL_G_t;

#define LSM9DS1_INT_GEN_DUR_G      0x37U
typedef struct {
  uint8_t dur_g                    : 7;
  uint8_t wait_g                   : 1;
} LSM9DS1_INT_GEN_DUR_G_t;

#define LSM9DS1_OFFSET_X_REG_L_M   0x05U
#define LSM9DS1_OFFSET_X_REG_H_M   0x06U
#define LSM9DS1_OFFSET_Y_REG_L_M   0x07U
#define LSM9DS1_OFFSET_Y_REG_H_M   0x08U
#define LSM9DS1_OFFSET_Z_REG_L_M   0x09U
#define LSM9DS1_OFFSET_Z_REG_H_M   0x0AU

#define LSM9DS1_WHO_AM_I_M         0x0FU
#define LSM9DS1_CTRL_REG1_M        0x20U
typedef struct 
{
  uint8_t st                       : 1;
  uint8_t fast_odr                 : 1;
  uint8_t _do                      : 3;
  uint8_t om                       : 2;
  uint8_t temp_comp                : 1;
} LSM9DS1_CTRL_REG1_M_t;

#define LSM9DS1_CTRL_REG2_M        0x21U
typedef struct 
{
  uint8_t not_used_01              : 2;
  uint8_t soft_rst                 : 1;
  uint8_t reboot                   : 1;
  uint8_t not_used_02              : 1;
  uint8_t fs                       : 2;
  uint8_t not_used_03              : 1;
} LSM9DS1_CTRL_REG2_M_t;

#define LSM9DS1_CTRL_REG3_M        0x22U
typedef struct 
{
  uint8_t md                       : 2;
  uint8_t sim                      : 1;
  uint8_t not_used_01              : 2;
  uint8_t lp                       : 1;
  uint8_t not_used_02              : 1;
  uint8_t i2c_disable              : 1;
} LSM9DS1_CTRL_REG3_M_t;

#define LSM9DS1_CTRL_REG4_M        0x23U
typedef struct 
{
  uint8_t not_used_01              : 1;
  uint8_t ble                      : 1;
  uint8_t omz                      : 2;
  uint8_t not_used_02              : 4;
} LSM9DS1_CTRL_REG4_M_t;

#define LSM9DS1_CTRL_REG5_M        0x24U
typedef struct 
{
  uint8_t not_used_01              : 6;
  uint8_t bdu                      : 1;
  uint8_t fast_read                : 1;
} LSM9DS1_CTRL_REG5_M_t;

#define LSM9DS1_STATUS_REG_M       0x27U
typedef struct 
{
  uint8_t xda                      : 1;
  uint8_t yda                      : 1;
  uint8_t zda                      : 1;
  uint8_t zyxda                    : 1;
  uint8_t _xor                     : 1;
  uint8_t yor                      : 1;
  uint8_t zor                      : 1;
  uint8_t zyxor                    : 1;
} LSM9DS1_STATUS_REG_M_t;

#define LSM9DS1_OUT_X_L_M          0x28U
#define LSM9DS1_OUT_X_H_M          0x29U
#define LSM9DS1_OUT_Y_L_M          0x2AU
#define LSM9DS1_OUT_Y_H_M          0x2BU
#define LSM9DS1_OUT_Z_L_M          0x2CU
#define LSM9DS1_OUT_Z_H_M          0x2DU
#define LSM9DS1_INT_CFG_M          0x30U
typedef struct 
{
  uint8_t ien                      : 1;
  uint8_t iel                      : 1;
  uint8_t iea                      : 1;
  uint8_t not_used_01              : 2;
  uint8_t zien                     : 1;
  uint8_t yien                     : 1;
  uint8_t xien                     : 1;
} LSM9DS1_INT_CFG_M_t;

#define LSM9DS1_INT_SRC_M          0x31U
typedef struct 
{
  uint8_t _int                     : 1;
  uint8_t mroi                     : 1;
  uint8_t nth_z                    : 1;
  uint8_t nth_y                    : 1;
  uint8_t nth_x                    : 1;
  uint8_t pth_z                    : 1;
  uint8_t pth_y                    : 1;
  uint8_t pth_x                    : 1;
} LSM9DS1_INT_SRC_M_t;

#define LSM9DS1_INT_THS_L_M        0x32U
typedef struct 
{
  uint8_t ths                      : 8;
} LSM9DS1_INT_THS_L_M_t;

#define LSM9DS1_INT_THS_H_M        0x33U
typedef struct 
{
  uint8_t ths                      : 7;
  uint8_t not_used_01              : 1;
} LSM9DS1_INT_THS_H_M_t;

/**
  * @defgroup LSM9DS1_Driverister_Union
  * @brief    This union group all the registers that has a bit-field
  *           description.
  *           This union is useful but not need by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  * @{
  */
/*typedef union
{
  LSM9DS1_ACT_THS_t                act_ths;
  LSM9DS1_INT_GEN_CFG_XL_t         int_gen_cfg_xl;
  LSM9DS1_INT_GEN_DUR_XL_t         int_gen_dur_xl;
  LSM9DS1_INT1_CTRL_t              int1_ctrl;
  LSM9DS1_INT2_CTRL_t              int2_ctrl;
  LSM9DS1_CTRL_REG1_G_t            ctrl_reg1_g;
  LSM9DS1_CTRL_REG2_G_t            ctrl_reg2_g;
  LSM9DS1_CTRL_REG3_G_t            ctrl_reg3_g;
  LSM9DS1_ORIENT_CFG_G_t           orient_cfg_g;
  LSM9DS1_INT_GEN_SRC_G_t          int_gen_src_g;
  LSM9DS1_STATUS_REG_t             status_reg;
  LSM9DS1_CTRL_REG4_t              ctrl_reg4;
  LSM9DS1_CTRL_REG5_XL_t           ctrl_reg5_xl;
  LSM9DS1_CTRL_REG6_XL_t           ctrl_reg6_xl;
  LSM9DS1_CTRL_REG7_XL_t           ctrl_reg7_xl;
  LSM9DS1_CTRL_REG8_t              ctrl_reg8;
  LSM9DS1_CTRL_REG9_t              ctrl_reg9;
  LSM9DS1_CTRL_REG10_t             ctrl_reg10;
  LSM9DS1_INT_GEN_SRC_XL_t         int_gen_src_xl;
  LSM9DS1_FIFO_CTRL_t              fifo_ctrl;
  LSM9DS1_FIFO_SRC_t               fifo_src;
  LSM9DS1_INT_GEN_CFG_G_t          int_gen_cfg_g;
  LSM9DS1_INT_GEN_THS_XH_G_t       int_gen_ths_xh_g;
  LSM9DS1_INT_GEN_THS_XL_G_t       int_gen_ths_xl_g;
  LSM9DS1_INT_GEN_THS_YH_G_t       int_gen_ths_yh_g;
  LSM9DS1_INT_GEN_THS_YL_G_t       int_gen_ths_yl_g;
  LSM9DS1_INT_GEN_THS_ZH_G_t       int_gen_ths_zh_g;
  LSM9DS1_INT_GEN_THS_ZL_G_t       int_gen_ths_zl_g;
  LSM9DS1_INT_GEN_DUR_G_t          int_gen_dur_g;
  LSM9DS1_CTRL_REG1_M_t            ctrl_reg1_m;
  LSM9DS1_CTRL_REG2_M_t            ctrl_reg2_m;
  LSM9DS1_CTRL_REG3_M_t            ctrl_reg3_m;
  LSM9DS1_CTRL_REG4_M_t            ctrl_reg4_m;
  LSM9DS1_CTRL_REG5_M_t            ctrl_reg5_m;
  LSM9DS1_STATUS_REG_M_t           status_reg_m;
  LSM9DS1_INT_CFG_M_t              int_cfg_m;
  LSM9DS1_INT_SRC_M_t              int_src_m;
  LSM9DS1_INT_THS_L_M_t            int_ths_l_m;
  LSM9DS1_INT_THS_H_M_t            int_ths_h_m;
  bitwise_t                        bitwise;
  uint8_t                          byte;
} LSM9DS1_Driver_t; */

/**
  * @}
  *
  */

//LSM9DS1_Error_et LSM9DS1_ReadReg(uint8_t B_Addr, uint8_t reg, uint8_t* data, uint16_t len);
//LSM9DS1_Error_et LSM9DS1_WriteReg(uint8_t B_Addr, uint8_t reg, uint8_t* data, uint16_t len);

extern double_t LSM9DS1_From_Fs2g_To_mg(int16_t lsb);
extern double_t LSM9DS1_From_Fs4g_To_mg(int16_t lsb);
extern double_t LSM9DS1_From_Fs8g_To_mg(int16_t lsb);
extern double_t LSM9DS1_From_Fs16g_To_mg(int16_t lsb);

extern double_t LSM9DS1_From_Fs245dps_To_mdps(int16_t lsb);
extern double_t LSM9DS1_From_Fs500dps_To_mdps(int16_t lsb);
extern double_t LSM9DS1_From_Fs2000dps_To_mdps(int16_t lsb);

extern double_t LSM9DS1_From_Fs4gauss_To_mG(int16_t lsb);
extern double_t LSM9DS1_From_Fs8gauss_To_mG(int16_t lsb);
extern double_t LSM9DS1_From_Fs12gauss_To_mG(int16_t lsb);
extern double_t LSM9DS1_From_Fs16gauss_To_mG(int16_t lsb);

extern double_t LSM9DS1_From_LSB_To_Celsius(int16_t lsb);

typedef enum 
{
  LSM9DS1_245dps = 0,
  LSM9DS1_500dps = 1,
  LSM9DS1_2000dps = 3,
} LSM9DS1_GY_FS_t;
LSM9DS1_Error_et LSM9DS1_Gy_Full_Scale_Set(uint8_t B_Addr, LSM9DS1_GY_FS_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Full_Scale_Get(uint8_t B_Addr, LSM9DS1_GY_FS_t *val);

typedef enum 
{
  LSM9DS1_IMU_OFF              = 0x00,
  LSM9DS1_GY_OFF_XL_10Hz       = 0x10,
  LSM9DS1_GY_OFF_XL_50Hz       = 0x20,
  LSM9DS1_GY_OFF_XL_119Hz      = 0x30,
  LSM9DS1_GY_OFF_XL_238Hz      = 0x40,
  LSM9DS1_GY_OFF_XL_476Hz      = 0x50,
  LSM9DS1_GY_OFF_XL_952Hz      = 0x60,
  LSM9DS1_XL_OFF_GY_14Hz9      = 0x01,
  LSM9DS1_XL_OFF_GY_59Hz5      = 0x02,
  LSM9DS1_XL_OFF_GY_119Hz      = 0x03,
  LSM9DS1_XL_OFF_GY_238Hz      = 0x04,
  LSM9DS1_XL_OFF_GY_476Hz      = 0x05,
  LSM9DS1_XL_OFF_GY_952Hz      = 0x06,
  LSM9DS1_IMU_14Hz9            = 0x11,
  LSM9DS1_IMU_59Hz5            = 0x22,
  LSM9DS1_IMU_119Hz            = 0x33,
  LSM9DS1_IMU_238Hz            = 0x44,
  LSM9DS1_IMU_476Hz            = 0x55,
  LSM9DS1_IMU_952Hz            = 0x66,
  LSM9DS1_XL_OFF_GY_14Hz9_LP   = 0x81,
  LSM9DS1_XL_OFF_GY_59Hz5_LP   = 0x82,
  LSM9DS1_XL_OFF_GY_119Hz_LP   = 0x83,
  LSM9DS1_IMU_14Hz9_LP         = 0x91,
  LSM9DS1_IMU_59Hz5_LP         = 0xA2,
  LSM9DS1_IMU_119Hz_LP         = 0xB3,
} lsm9ds1_imu_odr_t;
LSM9DS1_Error_et LSM9DS1_IMU_Data_Rate_Set(uint8_t B_Addr, lsm9ds1_imu_odr_t val);
LSM9DS1_Error_et LSM9DS1_IMU_Data_Rate_Get(uint8_t B_Addr, lsm9ds1_imu_odr_t *val);

typedef struct 
{
  uint8_t orient              : 3;
  uint8_t signz_g             : 1; /*(0: positive; 1: negative)*/
  uint8_t signy_g             : 1; /*(0: positive; 1: negative)*/
  uint8_t signx_g             : 1; /*(0: positive; 1: negative)*/
} LSM9DS1_GY_ORIENT_t;
LSM9DS1_Error_et LSM9DS1_Gy_Orient_Set(uint8_t B_Addr, LSM9DS1_GY_ORIENT_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Orient_Get(uint8_t B_Addr, LSM9DS1_GY_ORIENT_t *val);
LSM9DS1_Error_et LSM9DS1_Xl_Flag_Data_Ready_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Gy_Flag_Data_Ready_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Temp_Flag_Data_Ready_Get(uint8_t B_Addr, uint8_t *val);

typedef struct 
{
  uint8_t xen_g             : 1;
  uint8_t yen_g             : 1;
  uint8_t zen_g             : 1;
} LSM9DS1_GY_AXIS_t;
LSM9DS1_Error_et LSM9DS1_Gy_Axis_Set(uint8_t B_Addr, LSM9DS1_GY_AXIS_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Axis_Get(uint8_t B_Addr, LSM9DS1_GY_AXIS_t *val);

typedef struct 
{
  uint8_t xen_xl             : 1;
  uint8_t yen_xl             : 1;
  uint8_t zen_xl             : 1;
} LSM9DS1_XL_AXIS_t;
LSM9DS1_Error_et LSM9DS1_Xl_Axis_Set(uint8_t B_Addr, LSM9DS1_XL_AXIS_t val);
LSM9DS1_Error_et LSM9DS1_Xl_Axis_Get(uint8_t B_Addr, LSM9DS1_XL_AXIS_t *val);

typedef enum 
{
  LSM9DS1_NO_DECIMATION     = 0,
  LSM9DS1_EVERY_2_SAMPLES   = 1,
  LSM9DS1_EVERY_4_SAMPLES   = 2,
  LSM9DS1_EVERY_8_SAMPLES   = 3,
} LSM9DS1_DEC_t;
LSM9DS1_Error_et LSM9DS1_Xl_Decimation_Set(uint8_t B_Addr, LSM9DS1_DEC_t val);
LSM9DS1_Error_et LSM9DS1_Xl_Decimation_Get(uint8_t B_Addr, LSM9DS1_DEC_t *val);

typedef enum 
{
  LSM9DS1_2g     = 0,
  LSM9DS1_16g    = 1,
  LSM9DS1_4g     = 2,
  LSM9DS1_8g     = 3,
} LSM9DS1_XL_FS_t;
LSM9DS1_Error_et LSM9DS1_Xl_Full_Scale_Set(uint8_t B_Addr, LSM9DS1_XL_FS_t val);
LSM9DS1_Error_et LSM9DS1_Xl_Full_Scale_Get(uint8_t B_Addr, LSM9DS1_XL_FS_t *val);
LSM9DS1_Error_et LSM9DS1_Block_Data_Update_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Block_Data_Update_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Mag_Offset_Set(uint8_t B_Addr, uint8_t *buff);
LSM9DS1_Error_et LSM9DS1_Mag_Offset_Get(uint8_t B_Addr, uint8_t *buff);

typedef enum 
{
  LSM9DS1_MAG_POWER_DOWN    = 0xC0,
  LSM9DS1_MAG_LP_0Hz625     = 0x00,
  LSM9DS1_MAG_LP_1Hz25      = 0x01,
  LSM9DS1_MAG_LP_2Hz5       = 0x02,
  LSM9DS1_MAG_LP_5Hz        = 0x03,
  LSM9DS1_MAG_LP_10Hz       = 0x04,
  LSM9DS1_MAG_LP_20Hz       = 0x05,
  LSM9DS1_MAG_LP_40Hz       = 0x06,
  LSM9DS1_MAG_LP_80Hz       = 0x07,
  LSM9DS1_MAG_MP_0Hz625     = 0x10,
  LSM9DS1_MAG_MP_1Hz25      = 0x11,
  LSM9DS1_MAG_MP_2Hz5       = 0x12,
  LSM9DS1_MAG_MP_5Hz        = 0x13,
  LSM9DS1_MAG_MP_10Hz       = 0x14,
  LSM9DS1_MAG_MP_20Hz       = 0x15,
  LSM9DS1_MAG_MP_40Hz       = 0x16,
  LSM9DS1_MAG_MP_80Hz       = 0x17,
  LSM9DS1_MAG_HP_0Hz625     = 0x20,
  LSM9DS1_MAG_HP_1Hz25      = 0x21,
  LSM9DS1_MAG_HP_2Hz5       = 0x22,
  LSM9DS1_MAG_HP_5Hz        = 0x23,
  LSM9DS1_MAG_HP_10Hz       = 0x24,
  LSM9DS1_MAG_HP_20Hz       = 0x25,
  LSM9DS1_MAG_HP_40Hz       = 0x26,
  LSM9DS1_MAG_HP_80Hz       = 0x27,
  LSM9DS1_MAG_UHP_0Hz625    = 0x30,
  LSM9DS1_MAG_UHP_1Hz25     = 0x31,
  LSM9DS1_MAG_UHP_2Hz5      = 0x32,
  LSM9DS1_MAG_UHP_5Hz       = 0x33,
  LSM9DS1_MAG_UHP_10Hz      = 0x34,
  LSM9DS1_MAG_UHP_20Hz      = 0x35,
  LSM9DS1_MAG_UHP_40Hz      = 0x36,
  LSM9DS1_MAG_UHP_80Hz      = 0x37,
  LSM9DS1_MAG_UHP_155Hz     = 0x38,
  LSM9DS1_MAG_HP_300Hz      = 0x28,
  LSM9DS1_MAG_MP_560Hz      = 0x18,
  LSM9DS1_MAG_LP_1000Hz     = 0x08,
  LSM9DS1_MAG_ONE_SHOT      = 0x70,
} LSM9DS1_MAG_DATA_RATE_t;
LSM9DS1_Error_et LSM9DS1_Mag_Data_Rate_Set(uint8_t B_Addr, LSM9DS1_MAG_DATA_RATE_t val);
LSM9DS1_Error_et LSM9DS1_Mag_Data_Rate_Get(uint8_t B_Addr, LSM9DS1_MAG_DATA_RATE_t *val);

typedef enum 
{
  LSM9DS1_4Ga    = 0,
  LSM9DS1_8Ga    = 1,
  LSM9DS1_12Ga   = 2,
  LSM9DS1_16Ga   = 3,
} LSM9DS1_MAG_FS_t;
LSM9DS1_Error_et LSM9DS1_Mag_Full_Scale_Set(uint8_t B_Addr, LSM9DS1_MAG_FS_t val);
LSM9DS1_Error_et LSM9DS1_Mag_Full_Scale_Get(uint8_t B_Addr, LSM9DS1_MAG_FS_t *val);
LSM9DS1_Error_et LSM9DS1_Mag_Flag_Data_Ready_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Temperature_Raw_Get(uint8_t B_Addr, uint8_t *buff);
LSM9DS1_Error_et LSM9DS1_Angular_Rate_Raw_Get(uint8_t B_Addr, uint8_t *buff);
LSM9DS1_Error_et LSM9DS1_Acceleration_Raw_Get(uint8_t B_Addr, uint8_t *buff);
LSM9DS1_Error_et LSM9DS1_Magnetic_Raw_Get(uint8_t B_Addr, uint8_t *buff);
LSM9DS1_Error_et LSM9DS1_Magnetic_Overflow_Get(uint8_t B_Addr, uint8_t *val);

typedef struct 
{
  uint8_t imu;
  uint8_t mag;
} LSM9DS1_ID_t;
LSM9DS1_Error_et LSM9DS1_Dev_Id_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_ID_t *buff);

typedef struct 
{
  LSM9DS1_STATUS_REG_M_t status_mag;
  LSM9DS1_STATUS_REG_t   status_imu;
} LSM9DS1_STATUS_t;
LSM9DS1_Error_et LSM9DS1_Dev_Status_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_STATUS_t *val);
LSM9DS1_Error_et LSM9DS1_Dev_Reset_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Dev_Reset_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t *val);

typedef enum 
{
  LSM9DS1_LSB_LOW_ADDRESS = 0,
  LSM9DS1_MSB_LOW_ADDRESS = 1,
} LSM9DS1_BLE_t;
LSM9DS1_Error_et LSM9DS1_Dev_Data_Format_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_BLE_t val);
LSM9DS1_Error_et LSM9DS1_Dev_Data_Format_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_BLE_t *val);
LSM9DS1_Error_et LSM9DS1_Dev_Boot_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Dev_Boot_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Reference_Set(uint8_t B_Addr, uint8_t *buff);
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Reference_Get(uint8_t B_Addr, uint8_t *buff);

typedef enum 
{
  LSM9DS1_LP_STRONG        = 0,
  LSM9DS1_LP_MEDIUM        = 1,
  LSM9DS1_LP_LIGHT         = 2,
  LSM9DS1_LP_ULTRA_LIGHT   = 3,
} LSM9DS1_GY_LP_BW_t;
LSM9DS1_Error_et LSM9DS1_Gy_Filter_LP_Bandwidth_Set(uint8_t B_Addr, LSM9DS1_GY_LP_BW_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Filter_LP_Bandwidth_Get(uint8_t B_Addr, LSM9DS1_GY_LP_BW_t *val);

typedef enum 
{
  LSM9DS1_LPF1_OUT              = 0x00,
  LSM9DS1_LPF1_HPF_OUT          = 0x01,
  LSM9DS1_LPF1_LPF2_OUT         = 0x02,
  LSM9DS1_LPF1_HPF_LPF2_OUT     = 0x12,
} LSM9DS1_GY_OUT_PATH_t;
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Out_Path_Set(uint8_t B_Addr, LSM9DS1_GY_OUT_PATH_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Out_Path_Get(uint8_t B_Addr, LSM9DS1_GY_OUT_PATH_t *val);

typedef enum 
{
  LSM9DS1_LPF1_INT              = 0x00,
  LSM9DS1_LPF1_HPF_INT          = 0x01,
  LSM9DS1_LPF1_LPF2_INT         = 0x02,
  LSM9DS1_LPF1_HPF_LPF2_INT     = 0x12,
} LSM9DS1_GY_INT_PATH_t;
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Int_Path_Set(uint8_t B_Addr, LSM9DS1_GY_INT_PATH_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Int_Path_Get(uint8_t B_Addr, LSM9DS1_GY_INT_PATH_t *val);

typedef enum 
{
  LSM9DS1_HP_EXTREME       = 0,
  LSM9DS1_HP_ULTRA_STRONG  = 1,
  LSM9DS1_HP_STRONG        = 2,
  LSM9DS1_HP_ULTRA_HIGH    = 3,
  LSM9DS1_HP_HIGH          = 4,
  LSM9DS1_HP_MEDIUM        = 5,
  LSM9DS1_HP_LOW           = 6,
  LSM9DS1_HP_ULTRA_LOW     = 7,
  LSM9DS1_HP_LIGHT         = 8,
  LSM9DS1_HP_ULTRA_LIGHT   = 9,
} LSM9DS1_GY_HP_BW_t;
LSM9DS1_Error_et LSM9DS1_Gy_Filter_HP_Bandwidth_Set(uint8_t B_Addr, LSM9DS1_GY_HP_BW_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Filter_HP_Bandwidth_Get(uint8_t B_Addr, LSM9DS1_GY_HP_BW_t *val);

typedef enum 
{
  LSM9DS1_AUTO      = 0x00,
  LSM9DS1_408Hz     = 0x10,
  LSM9DS1_211Hz     = 0x11,
  LSM9DS1_105Hz     = 0x12,
  LSM9DS1_50Hz      = 0x13,
} LSM9DS1_XL_AA_BW_t;
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Aalias_Bandwidth_Set(uint8_t B_Addr, LSM9DS1_XL_AA_BW_t val);
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Aalias_Bandwidth_Get(uint8_t B_Addr, LSM9DS1_XL_AA_BW_t *val);

typedef enum 
{
  LSM9DS1_HP_DIS  = 0,
  LSM9DS1_HP_EN   = 1,
} LSM9DS1_XL_HP_PATH_t;
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Int_Path_Set(uint8_t B_Addr, LSM9DS1_XL_HP_PATH_t val);
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Int_Path_Get(uint8_t B_Addr, LSM9DS1_XL_HP_PATH_t *val);

typedef enum 
{
  LSM9DS1_LP_OUT     = 0,
  LSM9DS1_HP_OUT     = 1,
} LSM9DS1_XL_OUT_PATH_t;
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Out_Path_Set(uint8_t B_Addr, LSM9DS1_XL_OUT_PATH_t val);
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Out_Path_Get(uint8_t B_Addr, LSM9DS1_XL_OUT_PATH_t *val);

typedef enum 
{
  LSM9DS1_LP_DISABLE     	= 0x00,
  LSM9DS1_LP_ODR_DIV_50     = 0x10,
  LSM9DS1_LP_ODR_DIV_100    = 0x11,
  LSM9DS1_LP_ODR_DIV_9      = 0x12,
  LSM9DS1_LP_ODR_DIV_400    = 0x13,
} LSM9DS1_XL_LP_BW_t;
LSM9DS1_Error_et LSM9DS1_Xl_Filter_LP_Bandwidth_Set(uint8_t B_Addr, LSM9DS1_XL_LP_BW_t val);
LSM9DS1_Error_et LSM9DS1_Xl_Filter_LP_Bandwidth_Get(uint8_t B_Addr, LSM9DS1_XL_LP_BW_t *val);

typedef enum 
{
  LSM9DS1_HP_ODR_DIV_50   = 0,
  LSM9DS1_HP_ODR_DIV_100  = 1,
  LSM9DS1_HP_ODR_DIV_9    = 2,
  LSM9DS1_HP_ODR_DIV_400  = 3,
} LSM9DS1_XL_HP_BW_t;
LSM9DS1_Error_et LSM9DS1_Xl_Filter_HP_Bandwidth_Set(uint8_t B_Addr, LSM9DS1_XL_HP_BW_t val);
LSM9DS1_Error_et LSM9DS1_Xl_Filter_HP_Bandwidth_Get(uint8_t B_Addr, LSM9DS1_XL_HP_BW_t *val);
LSM9DS1_Error_et LSM9DS1_Filter_Settling_Mask_Set(uint8_t B_Addr, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Filter_Settling_Mask_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Auto_Increment_Set(uint8_t B_Addr, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Auto_Increment_Get(uint8_t B_Addr, uint8_t *val);

typedef enum 
{
  LSM9DS1_SPI_4_WIRE = 0,
  LSM9DS1_SPI_3_WIRE = 1,
} LSM9DS1_SIM_t;
LSM9DS1_Error_et LSM9DS1_SPI_Mode_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_SIM_t val);
LSM9DS1_Error_et LSM9DS1_SPI_Mode_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_SIM_t *val);

typedef enum 
{
  LSM9DS1_I2C_ENABLE = 0,
  LSM9DS1_I2C_DISABLE = 1,
} LSM9DS1_I2C_DIS_t;
LSM9DS1_Error_et LSM9DS1_I2C_Interface_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_I2C_DIS_t val);
LSM9DS1_Error_et LSM9DS1_I2C_Interface_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_I2C_DIS_t *val);

typedef enum 
{
  LSM9DS1_LOGIC_OR    = 0,
  LSM9DS1_LOGIC_AND   = 1,
} LSM9DS1_PIN_LOGIC_t;
LSM9DS1_Error_et LSM9DS1_Pin_Logic_Set(uint8_t B_Addr, LSM9DS1_PIN_LOGIC_t val);
LSM9DS1_Error_et LSM9DS1_Pin_Logic_Get(uint8_t B_Addr, LSM9DS1_PIN_LOGIC_t *val);

typedef struct 
{
  uint8_t int1_drdy_xl      : 1;
  uint8_t int1_drdy_g       : 1;
  uint8_t int1_boot         : 1;
  uint8_t int1_fth          : 1;
  uint8_t int1_ovr          : 1;
  uint8_t int1_fss5         : 1;
  uint8_t int1_ig_xl        : 1;
  uint8_t int1_ig_g         : 1;
} LSM9DS1_PIN_INT1_ROUTE_t;
LSM9DS1_Error_et LSM9DS1_Pin_Int1_Route_Set(uint8_t B_Addr, LSM9DS1_PIN_INT1_ROUTE_t val);
LSM9DS1_Error_et LSM9DS1_Pin_Int1_Route_Get(uint8_t B_Addr, LSM9DS1_PIN_INT1_ROUTE_t *val);

typedef struct 
{
  uint8_t int2_drdy_xl       : 1;
  uint8_t int2_drdy_g        : 1;
  uint8_t int2_drdy_temp     : 1;
  uint8_t int2_fth           : 1;
  uint8_t int2_ovr           : 1;
  uint8_t int2_fss5          : 1;
  uint8_t int2_inact         : 1;
} LSM9DS1_PIN_INT2_ROUTE_t;
LSM9DS1_Error_et LSM9DS1_Pin_Int2_Route_Set(uint8_t B_Addr, LSM9DS1_PIN_INT2_ROUTE_t val);
LSM9DS1_Error_et LSM9DS1_Pin_Int2_Route_Get(uint8_t B_Addr, LSM9DS1_PIN_INT2_ROUTE_t *val);

typedef enum 
{
  LSM9DS1_INT_PULSED  = 0,
  LSM9DS1_INT_LATCHED = 1,
} LSM9DS1_LIR_t;
LSM9DS1_Error_et LSM9DS1_Pin_Notification_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_LIR_t val);
LSM9DS1_Error_et LSM9DS1_Pin_Notification_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_LIR_t *val);

typedef enum 
{
  LSM9DS1_PUSH_PULL   = 0,
  LSM9DS1_OPEN_DRAIN  = 1,
} LSM9DS1_PP_OD_t;
LSM9DS1_Error_et LSM9DS1_Pin_Mode_Set(uint8_t B_Addr, LSM9DS1_PP_OD_t val);
LSM9DS1_Error_et LSM9DS1_Pin_Mode_Get(uint8_t B_Addr, LSM9DS1_PP_OD_t *val);

typedef struct 
{
  uint8_t ien          : 1;
} LSM9DS1_PIN_M_ROUTE_t;
LSM9DS1_Error_et LSM9DS1_Pin_INT_M_Route_Set(uint8_t B_Addr, LSM9DS1_PIN_M_ROUTE_t val);
LSM9DS1_Error_et LSM9DS1_Pin_INT_M_Route_Get(uint8_t B_Addr, LSM9DS1_PIN_M_ROUTE_t *val);

typedef enum 
{
  LSM9DS1_ACTIVE_LOW    = 0,
  LSM9DS1_ACTIVE_HIGH   = 1,
} LSM9DS1_POLARITY_t;
LSM9DS1_Error_et LSM9DS1_Pin_Polarity_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_POLARITY_t val);
LSM9DS1_Error_et LSM9DS1_Pin_Polarity_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_POLARITY_t *val);

typedef struct 
{
  uint8_t xlie_xl             : 1;
  uint8_t xhie_xl             : 1;
  uint8_t ylie_xl             : 1;
  uint8_t yhie_xl             : 1;
  uint8_t zlie_xl             : 1;
  uint8_t zhie_xl             : 1;
} LSM9DS1_XL_TRSHLD_EN_t;
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Axis_Set(uint8_t B_Addr, LSM9DS1_XL_TRSHLD_EN_t val);
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Axis_Get(uint8_t B_Addr, LSM9DS1_XL_TRSHLD_EN_t *val);
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Set(uint8_t B_Addr, uint8_t *buff);
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Get(uint8_t B_Addr, uint8_t *buff);
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Min_Sample_Set(uint8_t B_Addr, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Min_Sample_Get(uint8_t B_Addr, uint8_t *val);

typedef struct 
{
  uint8_t xl_g             : 1;
  uint8_t xh_g             : 1;
  uint8_t yl_g             : 1;
  uint8_t yh_g             : 1;
  uint8_t zl_g             : 1;
  uint8_t zh_g             : 1;
  uint8_t ia_g             : 1;
} LSM9DS1_GY_TRSHLD_SRC_t;
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Src_Get(uint8_t B_Addr, LSM9DS1_GY_TRSHLD_SRC_t *val);

typedef struct 
{
  uint8_t xl_xl             : 1;
  uint8_t xh_xl             : 1;
  uint8_t yl_xl             : 1;
  uint8_t yh_xl             : 1;
  uint8_t zl_xl             : 1;
  uint8_t zh_xl             : 1;
  uint8_t ia_xl             : 1;
} LSM9DS1_XL_TRSHLD_SRC_t;
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Src_Get(uint8_t B_Addr, LSM9DS1_XL_TRSHLD_SRC_t *val);

typedef struct 
{
  uint8_t xlie_g             : 1;
  uint8_t xhie_g             : 1;
  uint8_t ylie_g             : 1;
  uint8_t yhie_g             : 1;
  uint8_t zlie_g             : 1;
  uint8_t zhie_g             : 1;
} LSM9DS1_GY_TRSHLD_EN_t;
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Axis_Set(uint8_t B_Addr, LSM9DS1_GY_TRSHLD_EN_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Axis_Get(uint8_t B_Addr, LSM9DS1_GY_TRSHLD_EN_t *val);

typedef enum 
{
  LSM9DS1_RESET_MODE = 0,
  LSM9DS1_DECREMENT_MODE = 1,
} LSM9DS1_DCRM_G_t;
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Mode_Set(uint8_t B_Addr, LSM9DS1_DCRM_G_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Mode_Get(uint8_t B_Addr, LSM9DS1_DCRM_G_t *val);
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_X_Set(uint8_t B_Addr, uint16_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_X_Get(uint8_t B_Addr, uint16_t *val);
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Y_Set(uint8_t B_Addr, uint16_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Y_Get(uint8_t B_Addr, uint16_t *val);
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Z_Set(uint8_t B_Addr, uint16_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Z_Get(uint8_t B_Addr, uint16_t *val);
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Min_Sample_Set(uint8_t B_Addr, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Min_Sample_Get(uint8_t B_Addr, uint8_t *val);

typedef struct 
{
  uint8_t zien             : 1;
  uint8_t yien             : 1;
  uint8_t xien             : 1;
} LSM9DS1_MAG_TRSHLD_AXIS_t;
LSM9DS1_Error_et LSM9DS1_Mag_Trshld_Axis_Set(uint8_t B_Addr, LSM9DS1_MAG_TRSHLD_AXIS_t val);
LSM9DS1_Error_et LSM9DS1_Mag_Trshld_Axis_Get(uint8_t B_Addr, LSM9DS1_MAG_TRSHLD_AXIS_t *val);

typedef struct 
{
  uint8_t _int              : 1;
  uint8_t nth_z             : 1;
  uint8_t nth_y             : 1;
  uint8_t nth_x             : 1;
  uint8_t pth_z             : 1;
  uint8_t pth_y             : 1;
  uint8_t pth_x             : 1;
} LSM9DS1_MAG_TRSHLD_SRC_t;
LSM9DS1_Error_et LSM9DS1_Mag_Trshld_Src_Get(uint8_t B_Addr, LSM9DS1_MAG_TRSHLD_SRC_t *val);
int32_t lsm9ds1_mag_trshld_set(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Mag_Trshld_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Act_Trshld_Set(uint8_t B_Addr, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Act_Trshld_Get(uint8_t B_Addr, uint8_t *val);

typedef enum 
{
  LSM9DS1_GYRO_POWER_DOWN = 0,
  LSM9DS1_GYRO_SLEEP = 1,
} LSM9DS1_ACT_MODE_t;
LSM9DS1_Error_et LSM9DS1_Act_Mode_Set(uint8_t B_Addr, LSM9DS1_ACT_MODE_t val);
LSM9DS1_Error_et LSM9DS1_Act_Mode_Get(uint8_t B_Addr, LSM9DS1_ACT_MODE_t *val);
LSM9DS1_Error_et LSM9DS1_Act_Duration_Set(uint8_t B_Addr, uint8_t *buff);
LSM9DS1_Error_et LSM9DS1_Act_Duration_Get(uint8_t B_Addr, uint8_t *buff);

typedef enum 
{
  LSM9DS1_ACTIVITY    = 0,
  LSM9DS1_INACTIVITY  = 1,
} LSM9DS1_INACT_t;
LSM9DS1_Error_et LSM9DS1_Act_Src_Get(uint8_t B_Addr, LSM9DS1_INACT_t *val);

typedef enum 
{
  LSM9DS1_POS_MOVE_RECO_DISABLE   = 0x00,
  LSM9DS1_6D_MOVE_RECO            = 0x01,
  LSM9DS1_4D_MOVE_RECO            = 0x05,
  LSM9DS1_6D_POS_RECO             = 0x03,
  LSM9DS1_4D_POS_RECO             = 0x07,
} LSM9DS1_6D_MODE_t;
LSM9DS1_Error_et LSM9DS1_6d_Mode_Set(uint8_t B_Addr, LSM9DS1_6D_MODE_t val);
LSM9DS1_Error_et LSM9DS1_6d_Mode_Get(uint8_t B_Addr, LSM9DS1_6D_MODE_t *val);
LSM9DS1_Error_et LSM9DS1_6d_Trshld_Set(uint8_t B_Addr, uint8_t *buff);
LSM9DS1_Error_et LSM9DS1_6d_Trshld_Get(uint8_t B_Addr, uint8_t *buff);

typedef struct 
{
  uint8_t xl_xl             : 1;
  uint8_t xh_xl             : 1;
  uint8_t yl_xl             : 1;
  uint8_t yh_xl             : 1;
  uint8_t zl_xl             : 1;
  uint8_t zh_xl             : 1;
  uint8_t ia_xl             : 1;
} LSM9DS1_6D_SRC_t;
LSM9DS1_Error_et LSM9DS1_6d_Src_Get(uint8_t B_Addr, LSM9DS1_6D_SRC_t *val);
LSM9DS1_Error_et LSM9DS1_Fifo_Stop_On_Wtm_Set(uint8_t B_Addr, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Fifo_Stop_On_Wtm_Get(uint8_t B_Addr, uint8_t *val);

typedef enum 
{
  LSM9DS1_FIFO_OFF                = 0x00,
  LSM9DS1_BYPASS_MODE             = 0x10,
  LSM9DS1_FIFO_MODE               = 0x11,
  LSM9DS1_STREAM_TO_FIFO_MODE     = 0x13,
  LSM9DS1_BYPASS_TO_STREAM_MODE   = 0x14,
  LSM9DS1_STREAM_MODE             = 0x16,
} LSM9DS1_FIFO_MD_t;
LSM9DS1_Error_et LSM9DS1_Fifo_Mode_Set(uint8_t B_Addr, LSM9DS1_FIFO_MD_t val);
LSM9DS1_Error_et LSM9DS1_Fifo_Mode_Get(uint8_t B_Addr, LSM9DS1_FIFO_MD_t *val);
LSM9DS1_Error_et LSM9DS1_Fifo_Temp_Batch_Set(uint8_t B_Addr, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Fifo_Temp_Batch_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Fifo_Watermark_Set(uint8_t B_Addr, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Fifo_Watermark_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Fifo_Full_Flag_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Fifo_Data_Level_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Fifo_Ovr_Flag_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Fifo_Wtm_Flag_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Xl_Self_Test_Set(uint8_t B_Addr, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Xl_Self_Test_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Gy_Self_Test_Set(uint8_t B_Addr, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Gy_Self_Test_Get(uint8_t B_Addr, uint8_t *val);
LSM9DS1_Error_et LSM9DS1_Mag_Self_Test_Set(uint8_t B_Addr, uint8_t val);
LSM9DS1_Error_et LSM9DS1_Mag_Self_Test_Get(uint8_t B_Addr, uint8_t *val);

typedef struct			//Added By Me!!!
{
  int16_t AXIS_X;
  int16_t AXIS_Y;
  int16_t AXIS_Z;
} SensorAxesRaw_t;

typedef struct			//Added By Me!!!
{
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;
} SensorAxes_t;

typedef struct			//Added By Me!!!
{
  int16_t 			TRaw_Out;
  int16_t 			T_Out;
  SensorAxesRaw_t 	GyRaw_Out;
  SensorAxes_t		Gy_Out;
  SensorAxesRaw_t 	AccRaw_Out;
  SensorAxes_t		Acc_Out;
  SensorAxesRaw_t 	MagRaw_Out;
  SensorAxes_t		Mag_Out;
} LSM9DS1_MeasureTypeDef_st;

LSM9DS1_Error_et LSM9DS1_Temperature_Get(int16_t *temperature_raw, int16_t *temperature);							//Added By Me!!!
LSM9DS1_Error_et LSM9DS1_Angular_Rate_Get(SensorAxesRaw_t *angular_velocity_raw, SensorAxes_t *angular_velocity);	//Added By Me!!!
LSM9DS1_Error_et LSM9DS1_Acceleration_Get(SensorAxesRaw_t *acceleration_raw, SensorAxes_t *acceleration);			//Added By Me!!!
LSM9DS1_Error_et LSM9DS1_Magnetic_Get(SensorAxesRaw_t *magnetic_field_raw, SensorAxes_t *magnetic_field);			//Added By Me!!!
LSM9DS1_Error_et LSM9DS1_Get_Measurement(LSM9DS1_MeasureTypeDef_st *Measurement_Value);								//Added By Me!!!

#define LSM9DS1_TEMP_BIAS			(27)			//Added By Me!!!
/**
  *@}
  *
  */

/**
* @brief Registers Init Values.						//Added By Me!!!
* \code
* Values write in the registers in the lps25hb_setup() funcion
* \endcode
*/
// LSM9DS1 Register Addresses Values
// Accelerometer and gyroscope registers						    bit 7 -----> bit 0    | bit functions |
#define LSM9DS1_ACT_THS_VAL				((uint8_t)0x80)		// 0x04 SLEEP_ON/INACT_EN - ACT_THS - ACT_THS - ACT_THS - ACT_THS - ACT_THS - ACT_THS - ACT_THS
#define LSM9DS1_ACT_DUR_VAL 			((uint8_t)0x00)		// 0x05 ACT_DUR - ACT_DUR - ACT_DUR - ACT_DUR - ACT_DUR - ACT_DUR - ACT_DUR - ACT_DUR
#define LSM9DS1_INT_GEN_CFG_XL_VAL 		((uint8_t)0x00)		// 0x06 AOI_XL - 6D - ZHIE_XL - ZLIE_XL - YHIE_XL - YLIE_XL - XHIE_XL - XLIE_XL
#define LSM9DS1_INT_GEN_THS_X_XL_VAL	((uint8_t)0x00)		// 0x07 THS_XL_X7 - THS_XL_X6 - THS_XL_X5 - THS_XL_X4 - THS_XL_X3 - THS_XL_X2 - THS_XL_X1 - THS_XL_X0
#define LSM9DS1_INT_GEN_THS_Y_XL_VAL 	((uint8_t)0x00)		// 0x08 THS_XL_Y7 - THS_XL_Y6 - THS_XL_Y5 - THS_XL_Y4 - THS_XL_Y3 - THS_XL_Y2 - THS_XL_Y1 - THS_XL_Y0
#define LSM9DS1_INT_GEN_THS_Z_XL_VAL 	((uint8_t)0x00) 	// 0x09 THS_XL_Z7 - THS_XL_Z6 - THS_XL_Z5 - THS_XL_Z4 - THS_XL_Z3 - THS_XL_Z2 - THS_XL_Z1 - THS_XL_Z0
#define LSM9DS1_INT_GEN_DUR_XL_VAL 		((uint8_t)0x00) 	// 0x0a WAIT_XL - DUR_XL6 - DUR_XL5 - DUR_XL4 - DUR_XL3 - DUR_XL2 - DUR_XL1 - DUR_XL0
#define LSM9DS1_REFERENCE_G_VAL 		((uint8_t)0x00) 	// 0x0b REF7_G - REF6_G - REF5_G - REF4_G - REF3_G - REF2_G - REF1_G - REF0_G
#define LSM9DS1_INT1_CTRL_VAL 			((uint8_t)0x00) 	// 0x0c INT1_IG_G - INT1_IG_XL - INT1_FSS5 - INT1_OVR - INT1_FTH - INT1_Boot - INT1_DRDY_G - INT1_DRDY_XL
//#define LSM9DS1_INT1_CTRL_VAL 		((uint8_t)0xAA) 	// 0x0c My Test Register
#define LSM9DS1_INT2_CTRL_VAL 			((uint8_t)0x00)		// 0x0d	INT2_IN_ACT - 0 - INT2_FSS5 - INT2_OVR - INT2_FTH - INT2_DRDY_TEMP INT2_DRDY_G - INT2_DRDY_XL
#define LSM9DS1_WHO_AM_I_VAL 			((uint8_t)0x68)		// 0x0f	0 - 1 - 1 - 0 - 1 - 0 - 0 - 0

#define LSM9DS1_CTRL_REG1_G_VAL 		((uint8_t)0x6B) 	// 0x10	ODR_G2 - ODR_G1 - ODR_G0 - FS_G1 - FS_G0 - 0 - BW_G1 - BW_G0
#define LSM9DS1_CTRL_REG2_G_VAL 		((uint8_t)0x0F)		// 0x11	0 - 0 - 0 - 0 - INT_SEL1 - INT_SEL0 - OUT_SEL1 - OUT_SEL0
#define LSM9DS1_CTRL_REG3_G_VAL			((uint8_t)0x83)		// 0x12	LP_mode - HP_EN - 0 - 0 - HPCF3_G - HPCF2_G - HPCF1_G - HPCF0_G
#define LSM9DS1_ORIENT_CFG_G_VAL 		((uint8_t)0x00)		// 0x13	0 - 0 - SignX_G - SignY_G - SignZ_G - Orient_2 - Orient_1 - Orient_0
#define LSM9DS1_INT_GEN_SRC_G_VAL		((uint8_t)0x00)		// 0x14	0 - IA_G - ZH_G - ZL_G - YH_G - YL_G - XH_G - XL_G
#define LSM9DS1_OUT_TEMP_L_VAL 			((uint8_t)0x00)		// 0x15	Temp07 - Temp06 - Temp05 - Temp04 - Temp03 - Temp02 - Temp01 - Temp00
#define LSM9DS1_OUT_TEMP_H_VAL 			((uint8_t)0x00)		// 0x16	Temp11 - Temp11 - Temp11 - Temp11 - Temp11 - Temp10 - Temp09 - Temp08
#define LSM9DS1_STATUS_REG_VAL			((uint8_t)0x00)		// 0x17 0 - IG_XL - IG_G - INACT - BOOT_STATUS - TDA - GDA - XLDA
#define LSM9DS1_OUT_X_L_G_VAL 			((uint8_t)0x00)		// 0x18	OUT_X_G07 - OUT_X_G06 - OUT_X_G05 - OUT_X_G04 - OUT_X_G03 - OUT_X_G02 - OUT_X_G01 - OUT_X_G00
#define LSM9DS1_OUT_X_H_G_VAL 			((uint8_t)0x00)		// 0x19	OUT_X_G11 - OUT_X_G11 - OUT_X_G11 - OUT_X_G11 - OUT_X_G11 - OUT_X_G10 - OUT_X_G09 - OUT_X_G08
#define LSM9DS1_OUT_Y_L_G_VAL   		((uint8_t)0x00)		// 0x1a OUT_Y_G07 - OUT_Y_G06 - OUT_Y_G05 - OUT_Y_G04 - OUT_Y_G03 - OUT_Y_G02 - OUT_Y_G01 - OUT_Y_G00
#define LSM9DS1_OUT_Y_H_G_VAL 			((uint8_t)0x00)		// 0x1b	OUT_Y_G11 - OUT_Y_G11 - OUT_Y_G11 - OUT_Y_G11 - OUT_Y_G11 - OUT_Y_G10 - OUT_Y_G09 - OUT_Y_G08
#define LSM9DS1_OUT_Z_L_G_VAL			((uint8_t)0x00)		// 0x1c	OUT_Z_G07 - OUT_Z_G06 - OUT_Z_G05 - OUT_Z_G04 - OUT_Z_G03 - OUT_Z_G02 - OUT_Z_G01 - OUT_Z_G00
#define LSM9DS1_OUT_Z_H_G_VAL 			((uint8_t)0x00)		// 0x1d	OUT_Z_G11 - OUT_Z_G11 - OUT_Z_G11 - OUT_Z_G11 - OUT_Z_G11 - OUT_Z_G10 - OUT_Z_G09 - OUT_Z_G08
#define LSM9DS1_CTRL_REG4_VAL 			((uint8_t)0x38)		// 0x1e	0 - 0 - Zen_G - Yen_G - Xen_G - 0 - LIR_XL1 - 4D_XL1
#define LSM9DS1_CTRL_REG5_XL_VAL    	((uint8_t)0x38)		// 0x1f	DEC_1 - DEC_0 - Zen_XL - Yen_XL - Xen_XL - 0 - 0 - 0

#define LSM9DS1_CTRL_REG6_XL_VAL 		((uint8_t)0x71)		// 0x20	ODR_XL2 - ODR_XL1 - ODR_XL0 - FS1_XL - FS0_XL - BW_SCAL_ODR - BW_XL1 - BW_XL0
#define LSM9DS1_CTRL_REG7_XL_VAL		((uint8_t)0xC0)		// 0x21	HR - DCF1 - DCF0 - 0 - 0 - FDS - 0 - HPIS1
#define LSM9DS1_CTRL_REG8_VAL 			((uint8_t)0x04)		// 0x22	BOOT - BDU - H_LACTIVE - PP_OD - SIM - IF_ADD_INC - BLE - SW_RESET
#define LSM9DS1_CTRL_REG9_VAL 			((uint8_t)0x00)		// 0x23	0 - SLEEP_G - 0 - FIFO_TEMP_EN - DRDY_mask_bit - I2C_DISABLE - FIFO_EN - STOP_ON_FTH
#define LSM9DS1_CTRL_REG10_VAL    		((uint8_t)0x00)		// 0x24	0 - 0 - 0 - 0 - 0 - ST_G - 0 - ST_XL
#define LSM9DS1_INT_GEN_SRC_XL_VAL 		((uint8_t)0x00)		// 0x26	0 - IA_XL - ZH_XL - ZL_XL - YH_XL - YL_XL - XH_XL - XL_XL
#define LSM9DS1_STATUS_REG2_VAL			((uint8_t)0x00)		// 0x27	0 - IG_XL - IG_G - INACT - BOOT_STATUS - TDA - GDA - XLDA
#define LSM9DS1_OUT_X_L_XL_VAL 			((uint8_t)0x00)		// 0x28	OUT_X_XL07 - OUT_X_XL06 - OUT_X_XL05 - OUT_X_XL04 - OUT_X_XL03 - OUT_X_XL02 - OUT_X_XL01 - OUT_X_XL00
#define LSM9DS1_OUT_X_H_XL_VAL 			((uint8_t)0x00)		// 0x29	OUT_X_XL11 - OUT_X_XL11 - OUT_X_XL11 - OUT_X_XL11 - OUT_X_XL11 - OUT_X_XL10 - OUT_X_XL09 - OUT_X_XL08
#define LSM9DS1_OUT_Y_L_XL_VAL    		((uint8_t)0x00)		// 0x2a	OUT_Y_XL07 - OUT_Y_XL06 - OUT_Y_XL05 - OUT_Y_XL04 - OUT_Y_XL03 - OUT_Y_XL02 - OUT_Y_XL01 - OUT_Y_XL00
#define LSM9DS1_OUT_Y_H_XL_VAL 			((uint8_t)0x00)		// 0x2b	OUT_Y_XL11 - OUT_Y_XL11 - OUT_Y_XL11 - OUT_Y_XL11 - OUT_Y_XL11 - OUT_Y_XL10 - OUT_Y_XL09 - OUT_Y_XL08
#define LSM9DS1_OUT_Z_L_XL_VAL			((uint8_t)0x00)		// 0x2c	OUT_Z_XL07 - OUT_Z_XL06 - OUT_Z_XL05 - OUT_Z_XL04 - OUT_Z_XL03 - OUT_Z_XL02 - OUT_Z_XL01 - OUT_Z_XL00
#define LSM9DS1_OUT_Z_H_XL_VAL 			((uint8_t)0x00)		// 0x2d	OUT_Z_XL11 - OUT_Z_XL11 - OUT_Z_XL11 - OUT_Z_XL11 - OUT_Z_XL11 - OUT_Z_XL10 - OUT_Z_XL09 - OUT_Z_XL08
#define LSM9DS1_FIFO_CTRL_VAL 			((uint8_t)0xC0)		// 0x2e	FMODE2 - FMODE1 - FMODE0 - FTH4 - FTH3 - FTH2 - FTH1 - FTH0
#define LSM9DS1_FIFO_SRC_VAL    		((uint8_t)0x00)		// 0x2f	FTH - OVRN - FSS5 - FSS4 - FSS3 - FSS2 - FSS1 - FSS0

#define LSM9DS1_INT_GEN_CFG_G_VAL 		((uint8_t)0x00)		// 0x30	AOI_G - LIR_G - ZHIE_G - ZLIE_G - YHIE_G - YLIE_G - XHIE_G - XLIE_G
#define LSM9DS1_INT_GEN_THS_XH_G_VAL	((uint8_t)0x00)		// 0x31	DCRM_G - THS_G_X14 - THS_G_X13 - THS_G_X12 - THS_G_X11 - THS_G_X10 - THS_G_X09 - THS_G_X08
#define LSM9DS1_INT_GEN_THS_XL_G_VAL 	((uint8_t)0x00)		// 0x32	THS_G_X07 - THS_G_X06 - THS_G_X05 - THS_G_X04 - THS_G_X03 - THS_G_X02 - THS_G_X01 - THS_G_X00
#define LSM9DS1_INT_GEN_THS_YH_G_VAL 	((uint8_t)0x00)		// 0x33	0 - THS_G_Y14 - THS_G_Y13 - THS_G_Y12 - THS_G_Y11 - THS_G_Y10 - THS_G_Y09 - THS_G_Y08        
#define LSM9DS1_INT_GEN_THS_YL_G_VAL    ((uint8_t)0x00)		// 0x34	THS_G_Y07 - THS_G_Y06 - THS_G_Y05 - THS_G_Y04 - THS_G_Y03 - THS_G_Y02 - THS_G_Y01 - THS_G_Y00
#define LSM9DS1_INT_GEN_THS_ZH_G_VAL 	((uint8_t)0x00)		// 0x35	0 - THS_G_Z14 - THS_G_Z13 - THS_G_Z12 - THS_G_Z11 - THS_G_Z10 - THS_G_Z09 - THS_G_Z08
#define LSM9DS1_INT_GEN_THS_ZL_G_VAL	((uint8_t)0x00)		// 0x36	THS_G_Z07 - THS_G_Z06 - THS_G_Z05 - THS_G_Z04 - THS_G_Z03 - THS_G_Z02 - THS_G_Z01 - THS_G_Z00
//#define LSM9DS1_INT_GEN_THS_ZL_G_VAL	((uint8_t)0xAA)		// 0x36	My Test Register
#define LSM9DS1_INT_GEN_DUR_G_VAL 		((uint8_t)0x00)		// 0x37	WAIT_G - DUR_G6 - DUR_G5 - DUR_G4 - DUR_G3 - DUR_G2 - DUR_G1 - DUR_G0

// Magnetometer registers
#define LSM9DS1_OFFSET_X_REG_L_M_VAL 	((uint8_t)0x00)		// 0x05	OFXM07 - OFXM06 - OFXM05 - OFXM04 - OFXM03 - OFXM02 - OFXM01 - OFXM00
#define LSM9DS1_OFFSET_X_REG_H_M_VAL    ((uint8_t)0x00)		// 0x06	OFXM15 - OFXM14 - OFXM13 - OFXM12 - OFXM11 - OFXM10 - OFXM09 - OFXM08
#define LSM9DS1_OFFSET_Y_REG_L_M_VAL 	((uint8_t)0x00)		// 0x07	OFYM07 - OFYM06 - OFYM05 - OFYM04 - OFYM03 - OFYM02 - OFYM01 - OFYM00
#define LSM9DS1_OFFSET_Y_REG_H_M_VAL	((uint8_t)0x00)		// 0x08	OFYM15 - OFYM14 - OFYM13 - OFYM12 - OFYM11 - OFYM10 - OFYM09 - OFYM08
#define LSM9DS1_OFFSET_Z_REG_L_M_VAL 	((uint8_t)0x00)		// 0x09	OFZM07 - OFZM06 - OFZM05 - OFZM04 - OFZM03 - OFZM02 - OFZM01 - OFZM00
//#define LSM9DS1_OFFSET_Z_REG_L_M_VAL 	((uint8_t)0x55)		// 0x09	My Test Register
#define LSM9DS1_OFFSET_Z_REG_H_M_VAL 	((uint8_t)0x00)		// 0x0a	OFZM15 - OFZM14 - OFZM13 - OFZM12 - OFZM11 - OFZM10 - OFZM09 - OFZM08
#define LSM9DS1_WHO_AM_I_M_VAL    		((uint8_t)0x3D)		// 0x0f	0 - 0 - 1 - 1 - 1 - 1 - 0 - 1

#define LSM9DS1_CTRL_REG1_M_VAL 		((uint8_t)0xFE)		// 0x20	TEMP_COMP - OM1 - OM0 - DO2 - DO1 - DO0 - FAST_ODR - ST
#define LSM9DS1_CTRL_REG2_M_VAL			((uint8_t)0x20)		// 0x21	0 - FS1 - FS0 - 0 - REBOOT - SOFT_RST - 0 - 0
#define LSM9DS1_CTRL_REG3_M_VAL 		((uint8_t)0x00)		// 0x22	I2C_DISABLE - 0 - LP - 0 - 0 - SIM - MD1 - MD0
#define LSM9DS1_CTRL_REG4_M_VAL 		((uint8_t)0x0C)		// 0x23	0 - 0 - 0 - 0 - 0 - OMZ1 - OMZ0 - BLE - 0
#define LSM9DS1_CTRL_REG5_M_VAL    		((uint8_t)0x40)		// 0x24	FAST_READ - BDU - 0 - 0 - 0 - 0 - 0 - 0
#define LSM9DS1_STATUS_REG_M_VAL 		((uint8_t)0x00)		// 0x27	ZYXOR - ZOR - YOR - XOR - ZYXDA - ZDA - YDA - XDA
#define LSM9DS1_OUT_X_L_M_VAL			((uint8_t)0x00)		// 0x28	OUT_X_M07 - OUT_X_M06 - OUT_X_M05 - OUT_X_M04 - OUT_X_M03 - OUT_X_M02 - OUT_X_M01 - OUT_X_M00
#define LSM9DS1_OUT_X_H_M_VAL 			((uint8_t)0x00)		// 0x29	OUT_X_M11 - OUT_X_M11 - OUT_X_M11 - OUT_X_M11 - OUT_X_M11 - OUT_X_M10 - OUT_X_M09 - OUT_X_M08
#define LSM9DS1_OUT_Y_L_M_VAL 			((uint8_t)0x00)		// 0x2a	OUT_Y_M07 - OUT_Y_M06 - OUT_Y_M05 - OUT_Y_M04 - OUT_Y_M03 - OUT_Y_M02 - OUT_Y_M01 - OUT_Y_M00
#define LSM9DS1_OUT_Y_H_M_VAL    		((uint8_t)0x00)		// 0x2b	OUT_Y_M11 - OUT_Y_M11 - OUT_Y_M11 - OUT_Y_M11 - OUT_Y_M11 - OUT_Y_M10 - OUT_Y_M09 - OUT_Y_M08
#define LSM9DS1_OUT_Z_L_M_VAL 			((uint8_t)0x00)		// 0x2c	OUT_Z_M07 - OUT_Z_M06 - OUT_Z_M05 - OUT_Z_M04 - OUT_Z_M03 - OUT_Z_M02 - OUT_Z_M01 - OUT_Z_M00
#define LSM9DS1_OUT_Z_H_M_VAL			((uint8_t)0x00)		// 0x2d	OUT_Z_M11 - OUT_Z_M11 - OUT_Z_M11 - OUT_Z_M11 - OUT_Z_M11 - OUT_Z_M10 - OUT_Z_M09 - OUT_Z_M08

#define LSM9DS1_INT_CFG_M_VAL 			((uint8_t)0x00)		// 0x30	XIEN - YIEN - ZIEN - 0 - 0 - IEA - IEL - IEN
#define LSM9DS1_INT_SRC_M_VAL 			((uint8_t)0x00)		// 0x31	PTH_X - PTH_Y - PTH_Z - NTH_X - NTH_Y - NTH_Z - MROI - INT
#define LSM9DS1_INT_THS_L_M_VAL    		((uint8_t)0x00)		// 0x32	THS07 - THS06 - THS05 - THS04 - THS03 - THS02 - THS01 - THS00
#define LSM9DS1_INT_THS_H_M_VAL 		((uint8_t)0x00)		// 0x33	0 - THS14 - THS13 - THS12 - THS11 - THS10 - THS09 - THS08

/**
* @}
*/

/** @defgroup LSM9DS1_My_Function_Prototypes
* @{
*/
LSM9DS1_Error_et  LSM9DS1_ReadReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data);
LSM9DS1_Error_et  LSM9DS1_WriteReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data);
LSM9DS1_Error_et  LSM9DS1_WriteReg_DMA(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data);
LSM9DS1_Error_et  LSM9DS1_Init(uint8_t B_Addr, uint8_t RegAddr, uint8_t *RegValues, uint8_t Size);
//LSM9DS1_Error_et  LSM9DS1_Init_New(uint8_t B_Addr);
LSM9DS1_Error_et  MX_LSM9DS1_Init(void);
/**
* @}
*/

#ifdef __cplusplus
}
#endif

#endif /* LSM9DS1_DriverS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
