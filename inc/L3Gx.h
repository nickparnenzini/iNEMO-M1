/********************************************************************************
 * @file    L3GX.h
 * @author  ART Team IMS-Systems Lab
 * @version V2.3.0
 * @date    12 April 2012
 * @brief   Header for L3Gx.c file
 * @details
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __L3GX_H
#define __L3GX_H

#include <stdint.h>
#include "HAL_L3Gx.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @addtogroup Sensor_Libraries           Sensor Libraries
 * @{
 */

/**
 * @defgroup L3Gx
 * @brief  This module contains all the functions to configure the L3Gx (L3G4200D or L3GD20)
 * gyroscopic sensor.
 * @details
 * Since this code is platform independent an implementation of the SPI or I2C driver must
 * be provided by the user according to the used platform.
 * Every function makes use of the <i>L3gxBufferRead</i> and/or <i>L3gxBufferWrite</i>
 * as low level functions to write bytes through the used digital interface.
 * In order to link and use this code the user should define and export these functions in a header
 * file called "HAL_L3Gx.h" (included by this module).
 * @{
 */




/**
 * @defgroup L3GX_Exported_Types          L3Gx Exported Types
 * @{
 */

/**
 * @brief  Gyroscope Functional state. Used to enable or disable a specific option.
 */
typedef enum
{
  L3G_DISABLE = 0,
  L3G_ENABLE = !L3G_DISABLE
}L3GFunctionalState;


/**
 * @brief  Gyroscope Flag status. Used to set/reset the sensor flags.
 */
typedef enum
{
  L3G_RESET = 0,
  L3G_SET = !L3G_RESET
}L3GFlagStatus;


/**
 * @brief  Gyroscope Output Data Rate and LPF bandwidth
 */
typedef enum
{

  L3G_ODR_100_HZ_CUTOFF_12_5       = 0x00,         /*!< Output Data Rate = 100 Hz -  LPF Cut-Off = 12.5 Hz */
  L3G_ODR_100_HZ_CUTOFF_25         = 0x10,         /*!< Output Data Rate = 100 Hz -  LPF Cut-Off = 25 Hz */
  L3G_ODR_200_HZ_CUTOFF_12_5       = 0x40,         /*!< Output Data Rate = 200 Hz -  LPF Cut-Off = 12.5 Hz */
  L3G_ODR_200_HZ_CUTOFF_25         = 0x50,         /*!< Output Data Rate = 200 Hz -  LPF Cut-Off = 25 Hz */
  L3G_ODR_200_HZ_CUTOFF_50         = 0x60,         /*!< Output Data Rate = 200 Hz -  LPF Cut-Off = 50 Hz */
  L3G_ODR_200_HZ_CUTOFF_70         = 0x70,         /*!< Output Data Rate = 200 Hz -  LPF Cut-Off = 70 Hz */
  L3G_ODR_400_HZ_CUTOFF_20         = 0x80,         /*!< Output Data Rate = 400 Hz -  LPF Cut-Off = 20 Hz */
  L3G_ODR_400_HZ_CUTOFF_25         = 0x90,         /*!< Output Data Rate = 400 Hz -  LPF Cut-Off = 25 Hz */
  L3G_ODR_400_HZ_CUTOFF_50         = 0xA0,         /*!< Output Data Rate = 400 Hz -  LPF Cut-Off = 50 Hz */
  L3G_ODR_400_HZ_CUTOFF_110        = 0xB0,         /*!< Output Data Rate = 400 Hz -  LPF Cut-Off = 110 Hz */
  L3G_ODR_800_HZ_CUTOFF_30         = 0xC0,         /*!< Output Data Rate = 800 Hz -  LPF Cut-Off = 30 Hz */
  L3G_ODR_800_HZ_CUTOFF_35         = 0xD0,         /*!< Output Data Rate = 800 Hz -  LPF Cut-Off = 35 Hz */
  L3G_ODR_800_HZ_CUTOFF_50         = 0xE0,         /*!< Output Data Rate = 800 Hz -  LPF Cut-Off = 50 Hz */
  L3G_ODR_800_HZ_CUTOFF_110        = 0xF0,         /*!< Output Data Rate = 800 Hz -  LPF Cut-Off = 110 Hz */

  L3G_ODR_95_HZ_CUTOFF_12_5        = 0x00,         /*!< Output Data Rate = 95 Hz -  LPF Cut-Off = 12.5 Hz */
  L3G_ODR_95_HZ_CUTOFF_25          = 0x10,         /*!< Output Data Rate = 95 Hz -  LPF Cut-Off = 25 Hz */
  L3G_ODR_190_HZ_CUTOFF_12_5       = 0x40,         /*!< Output Data Rate = 190 Hz -  LPF Cut-Off = 12.5 Hz */
  L3G_ODR_190_HZ_CUTOFF_25         = 0x50,         /*!< Output Data Rate = 190 Hz -  LPF Cut-Off = 25 Hz */
  L3G_ODR_190_HZ_CUTOFF_50         = 0x60,         /*!< Output Data Rate = 190 Hz -  LPF Cut-Off = 50 Hz */
  L3G_ODR_190_HZ_CUTOFF_70         = 0x70,         /*!< Output Data Rate = 190 Hz -  LPF Cut-Off = 70 Hz */
  L3G_ODR_380_HZ_CUTOFF_20         = 0x80,         /*!< Output Data Rate = 380 Hz -  LPF Cut-Off = 20 Hz */
  L3G_ODR_380_HZ_CUTOFF_25         = 0x90,         /*!< Output Data Rate = 380 Hz -  LPF Cut-Off = 25 Hz */
  L3G_ODR_380_HZ_CUTOFF_50         = 0xA0,         /*!< Output Data Rate = 380 Hz -  LPF Cut-Off = 50 Hz */
  L3G_ODR_380_HZ_CUTOFF_100        = 0xB0,         /*!< Output Data Rate = 380 Hz -  LPF Cut-Off = 100 Hz */
  L3G_ODR_760_HZ_CUTOFF_30         = 0xC0,         /*!< Output Data Rate = 760 Hz -  LPF Cut-Off = 30 Hz */
  L3G_ODR_760_HZ_CUTOFF_35         = 0xD0,         /*!< Output Data Rate = 760 Hz -  LPF Cut-Off = 35 Hz */
  L3G_ODR_760_HZ_CUTOFF_50         = 0xE0,         /*!< Output Data Rate = 760 Hz -  LPF Cut-Off = 50 Hz */
  L3G_ODR_760_HZ_CUTOFF_100        = 0xF0,         /*!< Output Data Rate = 760 Hz -  LPF Cut-Off = 100 Hz */


}GyroOutputDataRate;



/**
 * @brief  Gyroscope Power Mode
 */
typedef enum
{
  L3G_NORMAL_SLEEP_MODE    = 0x08,        /*!< Normal mode or Sleep mode enabled. To go in sleep mode all axes shall be disabled. */
  L3G_POWER_DOWN_MODE      = 0x00         /*!< Power Down mode enabled */
}GyroPowerMode;


/**
 * @brief  Gyroscope Axes
 */
typedef enum
{
  L3G_X_AXIS_DIS   = 0x00,          /*!< X Axis disabled */
  L3G_X_AXIS_EN    = 0x01,          /*!< X Axis enabled */
  L3G_Y_AXIS_DIS   = 0x00,          /*!< Y Axis disabled */
  L3G_Y_AXIS_EN    = 0x02,          /*!< Y Axis enabled */
  L3G_Z_AXIS_DIS   = 0x00,          /*!< Z Axis disabled */
  L3G_Z_AXIS_EN    = 0x04,          /*!< Z Axis enabled */
  L3G_ALL_AXES_DIS = 0x00,          /*!< All axes disabled */
  L3G_ALL_AXES_EN  = 0x07           /*!< All axes enabled */
}GyroAxesEnabling;


/**
 * @brief  Gyroscope Full scale selection
 */
typedef enum
{
  L3G_FS_250_DPS    = 0x00,      /*!< ±250 dps */
  L3G_FS_500_DPS    = 0x10,      /*!< ±500 dps */
  L3G_FS_2000_DPS   = 0x20       /*!< ±2000 dps */
}GyroFullScale;


/**
 * @brief Gyroscope Block Data Update selection
 */
typedef enum
{
  L3G_CONTINUOS_UPDATE   = 0x00,     /*!< Continuos Update */
  L3G_BLOCK_UPDATE      = 0x80      /*!< Single Update: output registers not updated until MSB and LSB reading */
}GyroBlockDataUpdate;


/**
 * @brief  Gyroscope Endianness selection
 */
typedef enum
{
  L3G_LITTLE_ENDIAN   = 0x00,     /*!< Little Endian: data LSB @ lower address */
  L3G_BIG_ENDIAN      = 0x40      /*!< Big Endian: data MSB @ lower address */
}GyroEndianness;


/**
 * @brief  Gyroscope High Pass Mode Filter Selection
 */
typedef enum
{
  L3G_HPFM_NORMAL     = 0x00,       /*!< Normal Mode */
  L3G_HPFM_REFERENCE  = 0x10,       /*!< Reference Signal for filtering */
  L3G_HPFM_AOI        = 0x30        /*!< Autoreset On Interrupt event */
}GyroHPFMode;


/**
 * @brief Gyroscope Irq on line 2 list
 */
typedef enum
{
  L3G_I2_EMPTY = 0x01,            /*!< FIFO empty flag */
  L3G_I2_OVERRUN = 0x02,          /*!< Data over-run flag */
  L3G_I2_WTM = 0x04,              /*!< Watermark flag */
  L3G_I2_DRDY = 0x08              /*!< Data ready flag */
}L3GIrq2List;


/**
 * @brief Gyroscope Init structure definition
 */
typedef struct
{
  GyroPowerMode xPowerMode;  	          /*!< Low power mode selection (see table 19 datasheet) */
  GyroOutputDataRate xOutputDataRate;     /*!< Output Data Rate */
  GyroAxesEnabling xEnabledAxes;          /*!< Axes Enable */
  GyroFullScale xFullScale;               /*!< Full Scale */
  GyroBlockDataUpdate xDataUpdate;        /*!< Data Update mode : Continuos update or data don`t change until MSB and LSB nex reading */
  GyroEndianness xEndianness;             /*!< Endianess */
}L3GInit;


/**
 * @brief Gyroscope Filter Init structure definition
 */
typedef struct
{
  L3GFunctionalState xHPF; 	    /*!< HPF enabling/disabling */
  uint8_t cHPFCutOff;               /*!< This value shall be between 0 and 10. The HPF cut-off frequency will be ODR[Hz]/(12.5(1+cHPFCutOff))*/
  GyroHPFMode xHPF_Mode;  	    /*!< HPF MODE: Normal mode or Reference signal for filtering*/
  uint8_t cHPFReference;            /*!< Reference value for filtering*/
}L3GFilterInit;


/**
 * @brief Gyroscope Data status. It notifies if data on axis are available or overrided.
 */
typedef struct
{
  L3GFlagStatus X_Da:1;
  L3GFlagStatus Y_Da:1;
  L3GFlagStatus Z_Da:1;
  L3GFlagStatus ZYX_Da:1;
  L3GFlagStatus X_Or:1;
  L3GFlagStatus Y_Or:1;
  L3GFlagStatus Z_Or:1;
  L3GFlagStatus ZYX_Or:1;
}L3GDataStatus;


/**
 * @brief Gyroscope Events on axis bitfield structure
 * @note Workaround for X-Y axis inverded.
 */
typedef struct
{
  L3GFlagStatus Y_LOW:1;                  /*!< X axis low event */
  L3GFlagStatus Y_HIGH:1;                 /*!< X axis high event */
  L3GFlagStatus X_LOW:1;                  /*!< Y axis low event */
  L3GFlagStatus X_HIGH:1;                 /*!< Y axis high event */
  L3GFlagStatus Z_LOW:1;                  /*!< Z axis low event */
  L3GFlagStatus Z_HIGH:1;                 /*!< Z axis high event */
  uint8_t :2;                       /*!< 2 bits padding */
}L3GAxisEvents;


/**
 * @brief  Gyroscope FIFO Working Mode
 */
typedef enum
{
  L3G_BYPASS_MODE = 0x00,                /*!< Bypass mode: don't use the FIFO */
  L3G_FIFO_MODE = 0x20,                  /*!< FIFO mode */
  L3G_STREAM_MODE = 0x40,                /*!< Stream mode */
  L3G_STREAM_TO_FIFO = 0x60,             /*!< Stream to FIFO mode */
  L3G_BYPASS_TO_STREAM= 0x80             /*!< Bypass to Stream mode */
}L3GFifoMode;



/**
 * @brief Gyroscope FIFO Init structure definition
 */
typedef struct
{
  L3GFifoMode xFifoMode;               /*!< FIFO operating mode */
  uint8_t cWtm;                        /*!< WaterMark level for FIFO in range [0, 31] */
}L3GFifoInit;


/**
 * @brief Gyroscope FIFO Status bitfield structure
 */
typedef struct
{
  uint8_t FIFO_FSS:5;                 /*!< FIFO unread samples */
  L3GFlagStatus FIFO_EMPTY_FLAG:1;          /*!< FIFO Empty flag */
  L3GFlagStatus FIFO_OVRN_FLAG:1;           /*!< FIFO Overrun flag */
  L3GFlagStatus FIFO_WTM_FLAG:1;            /*!< FIFO Watermark flag */

}L3GFifoStatus;


/**
 * @brief Gyroscope External events on axis combination
 */
typedef enum
{
  L3G_OR_COMBINATION = 0x00,            /*!< OR combination of enabled IRQs */
  L3G_AND_COMBINATION = 0x80,           /*!< AND combination of enabled IRQs */

}L3GIrqOnaxisCombination;

/**
 * @}
 */


/**
  * @defgroup L3GX_Exported_Constants       L3Gx Exported Constants
  *@{
  */

/**
 * @defgroup L3GX_Sensitivity_Defines       L3Gx Sensitivity Defines
 * @{
 */

#define L3G_Sensitivity_250dps     (float) 114.285f  	/*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float) 57.1429f  	/*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float) 14.285f	/*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */

/**
 * @}
 */

/**
 * @}
 */


/** @defgroup L3GX_Register_Mapping           L3Gx Register Mapping
 * @{
 */

/**
 *  \brief Gyroscope WHO_AM_I Register: Device identification register.
 *  \code
 *   Read
 *   Default value: 0xD3
 *
 *   \endcode
 */
#define L3G_WHO_AM_I        0x0F


/**
 *  \brief Gyroscope Control Register 1
 *  \code
 *   Read Write
 *   Default value: 0x07
 *
 *   7:6 DR1-DR0: Output Data Rate selection
 *   5:4 BW1-BW0: Bandwidth selection
 *
 *				DR1-DR0  |  BW1-BW0  | Gyro ODR [Hz] |  LPF cut-off freq[Hz]
 * 			  ---------------------------------------------------------------
 *				  00 	 |    00     |     100       |        12.5
 *				  00 	 |    01     |     100       |        25
 *				  00 	 |    10     |     100       |        25
 *				  00 	 |    11     |     100       |        25
 * 			  ---------------------------------------------------------------
 *				  01 	 |    00     |     200       |        12.5
 *				  01 	 |    01     |     200       |        25
 *				  01 	 |    10     |     200       |        50
 *				  01 	 |    11     |     200       |        70
 * 			  ---------------------------------------------------------------
 *				  10 	 |    00     |     400       |        20
 *				  10 	 |    01     |     400       |        25
 *				  10 	 |    10     |     400       |        50
 *				  10 	 |    11     |     400       |       110
 * 			  ---------------------------------------------------------------
 *				  11 	 |    00     |     800       |        30
 *				  11 	 |    01     |     800       |        35
 *				  11 	 |    10     |     800       |        50
 *				  11 	 |    11     |     800       |       110
 * 			  ---------------------------------------------------------------
 *
 *   3 PD:  Power down mode enable. Default value: 0 (0: power down mode, sleep, 1: normal mode or sleep mode)
 *   2 Zen: Z axis enable. (0 - Z axis disabled; 1- Z axis enabled)
 *   1 Yen: Y axis enable. (0 - Y axis disabled; 1- Y axis enabled)
 *   0 Xen: X axis enable. (0 - X axis disabled; 1- X axis enabled)
 *   \endcode
 */
#define L3G_CTRL_REG1		0x20


/**
 *  \brief Gyroscope Control Register 2
 *  \code
 *   Read Write
 *   Default value: 0x00
 *
 *   7:6 0-0: 			Value loaded at boot. This value must not be changed
 *   5:4 HPM1-HPM0:		High Pass filter Mode Selection
 *
 *				HPM1 | HPM0 |              High Pass filter Mode
 * 			  -------------------------------------------------------------------
 *				 0   |	0   |  Normal mode (reset reading HP_RESET_FILTER)
 *				 0   |  1   |         Reference signal for filtering
 *                               1   |  0   |                   Normal mode
 *				 1   |  1   |               Autoreset on interrupt event
 * 			  -------------------------------------------------------------------
 *
 *
 *
 *   3-0 HPCF3-HPCF0:	High Pass filter Cut Off frequency selection
 *
 *				HPCF3-HPCF0  | ODR= 100Hz | ODR= 200Hz | ODR= 400Hz | ODR= 800Hz
 * 			  -----------------------------------------------------------------------
 *				  0000 	     |    8	  |    15      |    30	    |    56
 *				  0001       |    4       |     8      |    15      |    30
 * 				  0010 	     |    2       |     4      |     8      |    15
 * 				  0011 	     |	  1	  | 	2      |     4	    |	  8
 * 				  0100 	     |	  0.5 	  |     1      |     2 	    | 	  4
 *				  0101	     |	  0.2 	  |     0.5    |     1	    |	  2
 * 				  0110	     | 	  0.1	  | 	0.2    |     0.5    |	  1
 *				  0111	     | 	  0.05 	  |	0.1    |     0.2    | 	  0.5
 *				  1000 	     |	  0.02	  |     0.05   |     0.1    |	  0.2
 *				  1001 	     |    0.01    |	0.02   |     0.05   | 	  0.1
 * 			  -----------------------------------------------------------------------
 *   \endcode
 */
#define L3G_CTRL_REG2       0x21


/**
 *  \brief Gyroscope Control Register 3
 *  \code
 *   Read Write
 *   Default value: 0x00
 *
 *   7 I1_Int1: 	Interrupt enable on INT1 pin. Default value 0. (0: Disable; 1: Enable)
 *   6 I1_Boot:		Boot status available on INT1. Default value 0. (0: Disable; 1: Enable)
 *   5 H_Lactive: 	Interrupt active configuration on INT1. Default value 0. (0: High; 1:Low)
 *   4 PP_OD: 		Push- Pull / Open drain. Default value: 0. (0: Push- Pull; 1: Open drain)
 *   3 I2_DRDY: 	Date Ready on DRDY/INT2. Default value 0. (0: Disable; 1: Enable)
 *   2 I2_WTM: 		FIFO Watermark interrupt on DRDY/INT2. Default value: 0. (0: Disable; 1: Enable)
 *   1 I2_ORun: 	FIFO Overrun interrupt on DRDY/INT2 Default value: 0. (0: Disable; 1: Enable)
 *   0 I2_Empty: 	FIFO Empty interrupt on DRDY/INT2. Default value: 0. (0: Disable; 1: Enable)
 *
 *   \endcode
 */
#define L3G_CTRL_REG3       0x22


/**
 *  \brief Gyroscope Control Register 4
 *  \code
 *   Read Write
 *   Default value: 0x00
 *
 *   7   BDU: 		Block Data Update. Default value: 0 (0: continous update; 1: output registers not updated until MSB and LSB reading)
 *   6   BLE: 		Big/Little Endian Data Selection. Default value 0. (0: Data LSB @ lower address; 1: Data MSB @ lower address)
 *   5-4 FS1-FS0: 	Full Scale selection. Default value: 00 (00: 250 dps; 01: 500 dps; 10: 2000 dps; 11: 2000 dps)
 *   3   -
 *   2-1 ST1-ST0: 	Self Test Enable. Default value: 00 (00: Self Test Disabled)
 *
 *				ST1 | ST0 | Self test mode
 * 			  ------------------------------------
 *				 0  |  0  |  Normal mode
 *				 0  |  1  |  Self test 0
 *				 1  |  0  |     --
 *				 1  |  1  |  Self test 1
 * 			  ------------------------------------
 *
 *   0   SIM: 		SPI Serial Interface Mode selection. Default value: 0 (0: 4-wire interface; 1: 3-wire interface).
 *
 *   \endcode
 */
#define L3G_CTRL_REG4       0x23


/**
 *  \brief Gyroscope Control Register 5
 *  \code
 *   Read Write
 *   Default value: 0x00
 *
 *   7   BOOT: 			Reboot memory content. Default value: 0 (0: normal mode; 1: reboot memory content)
 *   6   FIFO_EN:		FIFO enable. Default value: 0 (0: FIFO disable; 1: FIFO Enable)
 *   5   -
 *   4   HPen: 			High Pass filter Enable. Default value: 0 (0: HPF disabled; 1: HPF enabled. See Figure 20)
 *   3-2 INT1_Sel1-INT1_Sel0:	INT1 selection configuration. Default value: 0 (See Figure 20)
 *
 *				Hpen    |  INT_SEL1  |  INT_SEL2  |								Description
 * 			  ----------------------------------------------------------------------------------------------------------------
 *				  x  	|     0      |     0	  |  Non-high-pass-filtered data are used for interrupt generation
 *				  x  	|     0      |     1      |  High-pass-filtered data are used for interrupt generation
 *				  0  	|     1      |     x	  |  Low-pass-filtered data are used for interrupt generation
 *				  1  	|     1      |	   x  	  |  High-pass and low-pass-filtered data are used for interrupt generation
 * 			  ----------------------------------------------------------------------------------------------------------------
 *
 *   1-0 Out_Sel1-Out_Sel0:	Out selection configuration. Default value: 0
 *
 *				Hpen 	|  OUT_SEL1  |  OUT_SEL0  |  			  		       Description
 * 			  -------------------------------------------------------------------------------------------------------------
 *				  x  	|     0      |     0	  |  Data in DataReg and FIFO are non-highpass-filtered
 *				  x  	|     0      |     1      |  Data in DataReg and FIFO are high-passfiltered
 *				  0  	|     1      |     x	  |  Data in DataReg and FIFO are low-passfiltered by LPF2
 *				  1  	|     1      |	   x  	  |  Data in DataReg and FIFO are high-pass and low-pass-filtered by LPF2
 * 			  -------------------------------------------------------------------------------------------------------------
 *
 *   \endcode
 */
#define L3G_CTRL_REG5       0x24


/**
 *  \brief Gyroscope REFERENCE/DATACAPTURE Register
 *  \code
 *   Read Write
 *   Default value: 0x00
 *
 *   7-0 Ref7-Ref0: Reference value for Interrupt generation. Default value: 0
 *
 *   \endcode
 */
#define L3G_REFERENCE       0x25


/**
 *  \brief Gyroscope OUT_TEMP Register
 *  \code
 *   Read
 *   Default value: (The value is expressed as 16bit two’s complement)
 *
 *   7-0 Temp7-Temp0: Temperature data.
 *
 *   \endcode
 */
#define L3G_OUT_TEMP        0x26


/**
 *  \brief Gyroscope STATUS Register
 *  \code
 *   Read
 *   Default value: (The value is expressed as 16bit two’s complement)
 *
 *   7 ZYXOR:	X, Y, Z -axis data overrun. Default value: 0 (0: no overrun has occurred; 1: new data has overwritten the previous one before it was read)
 *   6 ZOR:		Z axis data overrun. Default value: 0 (0: no overrun has occurred; 1: a new data for the Z-axis has overwritten the previous one)
 *   5 YOR:		Y axis data overrun. Default value: 0 (0: no overrun has occurred; 1: a new data for the Y-axis has overwritten the previous one)
 *   4 XOR:		X axis data overrun. Default value: 0 (0: no overrun has occurred; 1: a new data for the X-axis has overwritten the previous one)
 *   3 ZYXDA:	X, Y, Z -axis new data available. Default value: 0 (0: a new set of data is not yet available; 1: a new set of data is available)
 *   2 ZDA:		Z axis new data available. Default value: 0 (0: a new data for the Z-axis is not yet available; 1: a new data for the Z-axis is available)
 *   1 YDA:		Y axis new data available. Default value: 0 (0: a new data for the Y-axis is not yet available;1: a new data for the Y-axis is available)
 *   0 XDA:		X axis new data available. Default value: 0 (0: a new data for the X-axis is not yet available; 1: a new data for the X-axis is available)
 *
 *   \endcode
*/
#define L3G_STATUS_REG      0x27


/**
 *  \brief Gyroscope X-axis angular rate data. LSB Register.
 *  \code
 *  	Read
 *  	Default value: ( The value is expressed as 16bit two’s complement)
 *  	7:0 XOUT7-XOUT0: angular rate Data LSB (if in Little Endian Mode --> BLE bit in CTRL_REG4 is 0)
 *                       angular rate Data MSB (if in Big Endian Mode --> BLE bit in CTRL_REG4 is 1)
 * \endcode
 */
#define L3G_OUT_X_L			0x28


/**
 *  \brief  Gyroscope X-axis angular rate data. MSB Register.
 *  \code
 *  	Read
 *  	Default value: ( The value is expressed as 16bit two’s complement)
 *  	7:0 XOUT15-XOUT8: angular rate Data MSB (if in Little Endian Mode --> BLE bit in CTRL_REG1 is 0)
 *                        angular rate Data LSB (if in Big Endian Mode --> BLE bit in CTRL_REG1 is 1)
 * \endcode
 */
#define L3G_OUT_X_H     	0x29


/**
 *  \brief Gyroscope Y-axis angular rate data. LSB Register.
 *  \code
 *  	Read
 *  	Default value: ( The value is expressed as 16bit two’s complement)
 *  	7:0 YOUT7-YOUT0: angular rate Data LSB (if in Little Endian Mode --> BLE bit in CTRL_REG4 is 0)
 *                       angular rate Data MSB (if in Big Endian Mode --> BLE bit in CTRL_REG4 is 1)
 * \endcode
 */
#define L3G_OUT_Y_L			0x2A


/**
 *  \brief  Gyroscope Y-axis angular rate data. MSB Register.
 *  \code
 *  	Read
 *  	Default value: ( The value is expressed as 16bit two’s complement)
 *  	7:0 YOUT15-YOUT8: angular rate Data MSB (if in Little Endian Mode --> BLE bit in CTRL_REG1 is 0)
 *                        angular rate Data LSB (if in Big Endian Mode --> BLE bit in CTRL_REG1 is 1)
 * \endcode
 */
#define L3G_OUT_Y_H     	0x2B


/**
 *  \brief Gyroscope Z-axis angular rate data. LSB Register.
 *  \code
 *  	Read
 *  	Default value: ( The value is expressed as 16bit two’s complement)
 *  	7:0 ZOUT7-ZOUT0: angular rate Data LSB (if in Little Endian Mode --> BLE bit in CTRL_REG4 is 0)
 *                       angular rate Data MSB (if in Big Endian Mode --> BLE bit in CTRL_REG4 is 1)
 * \endcode
 */
#define L3G_OUT_Z_L			0x2C


/**
 *  \brief  Gyroscope Z-axis angular rate data. MSB Register.
 *  \code
 *  	Read
 *  	Default value: ( The value is expressed as 16bit two’s complement)
 *  	7:0 ZOUT15-ZOUT8: angular rate Data MSB (if in Little Endian Mode --> BLE bit in CTRL_REG1 is 0)
 *                        angular rate Data LSB (if in Big Endian Mode --> BLE bit in CTRL_REG1 is 1)
 * \endcode
 */
#define L3G_OUT_Z_H     	0x2D


/**
 *  \brief  Gyroscope FIFO Control Register.
 *  \code
 *  	Read Write
 *  	Default value: ( The value is expressed as 16bit two’s complement)
 *  	7:5 FM2-FM0: 	FIFO mode selection. Default value: 00
 *
 *				FM2  |  FM1  |  FM0  |   	FIFO mode
 * 			  ----------------------------------------------------
 *				 0   |	 0   |	 0   |     Bypass mode
 *				 0   |   0   |	 1   |      FIFO mode
 *				 0   |	 1   | 	 0   |     Stream mode
 *				 0   |	 1   |	 1   |  Stream-to-FIFO mode
 *				 1   |   0   |	 0   | Bypass-to-Stream mode
 * 			  ----------------------------------------------------
 *
 *	4-0 WTM4-WTM0:	FIFO threshold. Watermark level setting
 *
 * \endcode
 */
#define L3G_FIFO_CTRL_REG     	0x2E


/**
 *  \brief  Gyroscope FIFO Source Register.
 *  \code
 *  	Read
 *  	Default value: output
 *  	7   WTM: 	 	Watermark status. (0: FIFO filling is lower than WTM level; 1: FIFO filling is equal or higher than WTM level)
 *      6   OVRN:  		Overrun bit status. (0: FIFO is not completely filled; 1:FIFO is completely filled)
 *	5   EMPTY: 		FIFO empty bit. (0: FIFO not empty; 1: FIFO empty)
 *	4-1 FSS4-FSS1:	        FIFO stored data level
 *
 * \endcode
 */
#define L3G_FIFO_SRC_REG     	0x2F


/**
 *  \brief Gyroscope Configuration Register for Interrupt 1 source.
 *  \code
 *  	Read
 *  	Default value: 0x00
 *  	7 AND/OR:	AND/OR combination of Interrupt events.Default value: 0 (0: OR combination of interrupt events 1: AND combination of interrupt events
 *  	6 LIR:  	Latch Interrupt Request. Default value: 0 (0: interrupt request not latched; 1: interrupt request latched) Cleared by reading INT1_SRC reg.
 *  	5 ZHIE:  	Enable interrupt generation on Z high event. (0: disable interrupt request; 1: enable interrupt request on measured angular rate value higher than preset threshold)
 *  	4 ZLIE:  	Enable interrupt generation on Z low event. (0: disable interrupt request;  1: enable interrupt request on measured angular rate value lower than preset threshold)
 *  	3 YHIE:  	Enable interrupt generation on Y high event. (0: disable interrupt request; 1: enable interrupt request on measured angular rate value higher than preset threshold)
 *  	2 YLIE:  	Enable interrupt generation on Y low event. (0: disable interrupt request;  1: enable interrupt request on measured angular rate value lower than preset threshold)
 *  	1 XHIE:  	Enable interrupt generation on X high event. (0: disable interrupt request; 1: enable interrupt request on measured angular rate value higher than preset threshold)
 *  	0 XLIE:  	Enable interrupt generation on X low event. (0: disable interrupt request; 1: enable interrupt request on measured angular rate value lower than preset threshold)
 *
 *  \endcode
 */
#define L3G_INT1_CFG_REG		0x30


/**
 *  \brief Gyroscope Interrupt 1 source register.
 *  \code
 *  	Read
 *  	Default value: output
 *  	7 0
 *  	6 IA : Interrupt active. Default value: 0 (0: no interrupt has been generated; 1: one or more interrupts have been generated)
 *  	5 ZH:  Z high. Default value: 0 (0: no interrupt, 1: Z High event has occurred)
 *  	4 ZL:  Z low. Default value: 0 (0: no interrupt; 1: Z Low event has occurred)
 *  	3 YH:  Y high. Default value: 0 (0: no interrupt, 1: Y High event has occurred)
 *  	2 YL:  Y low. Default value: 0 (0: no interrupt; 1: Y Low event has occurred)
 *  	1 XH:  X high. Default value: 0 (0: no interrupt, 1: X High event has occurred)
 *  	0 XL:  X low. Default value: 0 (0: no interrupt; 1: X Low event has occurred)
 *
 * \endcode
 */
#define L3G_INT1_SCR_REG		0x31


/**
 *  \brief Gyroscope Interrupt 1 Threshold on X-axis. MSB Register.
 *  \code
 *	Read Write
 *  	Default value: 0x00
 *  	7 -
 *  	6 THSX14-THSX08: Interrupt 1 threshold.
 *
 * \endcode
 */
#define L3G_INT1_THS_XH_REG		0x32


/**
 *  \brief Gyroscope Interrupt 1 Threshold on X-axis. LSB Register.
 *  \code
 *	Read Write
 *  	Default value: 0x00
 *  	7-0 THSX7-THSX0 Interrupt 1 threshold.
 *
 * \endcode
 */
#define L3G_INT1_TSH_XL_REG  	0x33


/**
 *  \brief Gyroscope Interrupt 1 Threshold on Y-axis. MSB Register.
 *  \code
 *	Read Write
 *  	Default value: 0x00
 *  	7 -
 *  	6 THSY14-THSY08: Interrupt 1 threshold.
 *
 * \endcode
 */
#define L3G_INT1_TSH_YH_REG  	0x34


/**
 *  \brief Gyroscope Interrupt 1 Threshold on Y-axis. LSB Register.
 *  \code
 *	Read Write
 *  	Default value: 0x00
 *  	7-0 THSY7-THSY0 Interrupt 1 threshold.
 *
 * \endcode
 */
#define L3G_INT1_TSH_YL_REG  	0x35


/**
 *  \brief Gyroscope Interrupt 1 Threshold on Z-axis. MSB Register.
 *  \code
 *	Read Write
 *  	Default value: 0x00
 *  	7 -
 *  	6 THSZ14-THSZ08: Interrupt 1 threshold.
 *
 * \endcode
 */
#define L3G_INT1_TSH_ZH_REG  	0x36


/**
 *  \brief Gyroscope Interrupt 1 Threshold on Z-axis. LSB Register.
 *  \code
 *	Read Write
 *  	Default value: 0x00
 *  	7-0 THSZ7-THSZ0 Interrupt 1 threshold.
 *
 * \endcode
 */
#define L3G_INT1_TSH_ZL_REG  	0x37


/**
 *  \brief Gyroscope Interrupt 1 Duration Register.
 *  \code
 *	Read Write
 *  	Default value: 0x00
 *	7 	WAIT:	WAIT enable. Default value: 0 (0: disable; 1: enable). Wait =’0’: the interrupt falls immediately if signal crosses the selected threshold; Wait =’1’: if signal crosses the selected threshold, the interrupt falls only after the duration has counted number of samples at the selected data rate, written into the duration counter register.
 *  	6-0 D6-D0: 	Duration value (these bits set the minimum duration of the Interrupt event to be recognized. Duration steps and maximum values depend on the ODR chosen.Default value: 000 0000).
 *
 * \endcode
 */
#define L3G_INT1_DURATION_REG	0x38


/**
 * @}
 */


/** @defgroup L3GX_Exported_Functions           L3Gx Exported Functions
 * @{
 */

void L3gxConfig(L3GInit *pxL3GInitStruct);
void L3gxGetInfo(L3GInit *pxL3GInitStruct);
void L3gxFilterConfig(L3GFilterInit *pxL3GFilterInitStruct);
void L3gxFilterGetInfo(L3GFilterInit* pxL3GFilterInitStruct);
void L3gxLowpower(GyroPowerMode xPowerMode);
void L3gxSetDataRate(GyroOutputDataRate xDataRate);
GyroOutputDataRate L3gxGetEnumDataRate(void);
void L3gxSetFullScale(GyroFullScale xFullScale);
GyroFullScale L3gxGetEnumFullScale(void);
void L3gxGetSensitivity(float* pfSensitivityXYZ);
void L3gxReboot(void);
void L3gxReadRawData(int16_t* pnRawData);
void L3gxReadAngRate(float* pfData);
void L3gxReadFifo(float* pfData, uint8_t cDataToRead);
L3GDataStatus L3gxGetDataStatus(void);
void L3gxIrq1Config(L3GFunctionalState xNewState);
void L3gxIrq2Config(L3GIrq2List xIrq2Config, L3GFunctionalState xNewState);
void L3gxFifoInit(L3GFifoInit* pxFifoInit);
L3GFifoStatus L3gxFifoGetStatus(void);
void L3gxFifo(L3GFunctionalState xNewState);
void L3gxSetAxisIrqs(L3GIrqOnaxisCombination xIrqCombinations, L3GAxisEvents* pxAxisEvents, L3GFunctionalState xLatched);
L3GFunctionalState L3gxGetAxisIrqs(L3GAxisEvents* pxAxisEvents);
void L3gxSetThreshold(float *pfData);
float L3gxReadTemp(void);


/**
 * @}
 */

/**
 * @defgroup L3GX_Exported_Macros       L3Gx Exported Macros
 * @{
 */

/** @defgroup L3GX_I2C_Communication     L3Gx I2C Communication
 * @{
 */

#define L3gxByteRead(pVal,cAddress)      L3gxBufferRead(pVal,cAddress,1)
#define L3gxByteWrite(pVal,cAddress)     L3gxBufferWrite(pVal,cAddress,1)


/**
 * @}
 */

/** @defgroup L3GX_L3G4200D     L3Gx L3G4200D
 * @{
 */


#define L3g4200dConfig 			L3gxConfig
#define L3g4200dGetInfo 		L3gxGetInfo
#define L3g4200dFilterConfig 		L3gxFilterConfig
#define L3g4200dFilterGetInfo 		L3gxFilterGetInfo
#define L3g4200dLowpower 		L3gxLowpower
#define L3g4200dSetDataRate 		L3gxSetDataRate
#define L3g4200dGetEnumDataRate 	L3gxGetEnumDataRate
#define L3g4200dSetFullScale 		L3gxSetFullScale
#define L3g4200dGetEnumFullScale 	L3gxGetEnumFullScale
#define L3g4200dGetSensitivity          L3gxGetSensitivity
#define L3g4200dReboot 			L3gxReboot
#define L3g4200dReadRawData 		L3gxReadRawData
#define L3g4200dReadAngRate 		L3gxReadAngRate
#define L3g4200dReadFifo 		L3gxReadFifo
#define L3g4200dGetDataStatus 		L3gxGetDataStatus
#define L3g4200dIrq1Config 		L3gxIrq1Config
#define L3g4200dIrq2Config 		L3gxIrq2Config
#define L3g4200dFifoInit 		L3gxFifoInit
#define L3g4200dFifoGetStatus 		L3gxFifoGetStatus
#define L3g4200dFifo 			L3gxFifo
#define L3g4200dSetAxisIrqs 		L3gxSetAxisIrqs
#define L3g4200dGetAxisIrqs 		L3gxGetAxisIrqs
#define L3g4200dSetThreshold 		L3gxSetThreshold
#define L3g4200dReadTemp 		L3gxReadTemp


/**
 *@}
 */

/** @defgroup L3GX_L3GD20     	L3Gx L3GD20
 * @{
 */


#define L3gd20Config 			L3gxConfig
#define L3gd20GetInfo 			L3gxGetInfo
#define L3gd20FilterConfig 		L3gxFilterConfig
#define L3gd20FilterGetInfo 	        L3gxFilterGetInfo
#define L3gd20Lowpower 			L3gxLowpower
#define L3gd20SetDataRate 		L3gxSetDataRate
#define L3gd20GetEnumDataRate 	        L3gxGetEnumDataRate
#define L3gd20SetFullScale 		L3gxSetFullScale
#define L3gd20GetEnumFullScale 	        L3gxGetEnumFullScale
#define L3gd20GetSensitivity            L3gxGetSensitivity
#define L3gd20Reboot 			L3gxReboot
#define L3gd20ReadRawData 		L3gxReadRawData
#define L3gd20ReadAngRate 		L3gxReadAngRate
#define L3gd20ReadFifo 			L3gxReadFifo
#define L3gd20GetDataStatus 	        L3gxGetDataStatus
#define L3gd20Irq1Config 		L3gxIrq1Config
#define L3gd20Irq2Config 		L3gxIrq2Config
#define L3gd20FifoInit 			L3gxFifoInit
#define L3gd20FifoGetStatus 	        L3gxFifoGetStatus
#define L3gd20Fifo 			L3gxFifo
#define L3gd20SetAxisIrqs 		L3gxSetAxisIrqs
#define L3gd20GetAxisIrqs 		L3gxGetAxisIrqs
#define L3gd20SetThreshold 		L3gxSetThreshold
#define L3gd20ReadTemp 			L3gxReadTemp


/**
 *@}
 */

/**
 *@}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
