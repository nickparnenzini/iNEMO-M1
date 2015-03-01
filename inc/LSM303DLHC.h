/**
 * @file    LSM303DLHC.h
 * @author  ART Team IMS-Systems Lab
 * @version V2.3.0
 * @date    12 April 2012
 * @brief   Header for LSM303DLHC.c file
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
#ifndef __LSM303DLHC_H
#define __LSM303DLHC_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "HAL_LSM303DLHC.h"

#ifdef __cplusplus
 extern "C" {
#endif

   
/**
 * @addtogroup Sensor_Libraries           Sensor Libraries
 * @{
 */
   
/**
 * @addtogroup LSM303DLHC
 * @brief  This module contains all the functions to configure the LSM303DLHC accelerometer+magnetometer.
 * @details
 * Since this code is platform independent an implementation of the I2C driver must
 * be provided by the user according to the used platform.
 * Every function makes use of the <i>Lsm303dlhcAccI2CBufferRead</i> and/or <i>Lsm303dlhcAccI2CBufferWrite</i>
 * for the accelerometer and <i>Lsm303dlhcMagI2CBufferRead</i> and/or <i>Lsm303dlhcMagI2CBufferWrite</i> for
 * magnetometer as low level functions to write bytes through the used digital interface.
 * In order to link and use this code the user should define and export these functions in a header
 * file called "HAL_LSM303DLHC.h" (included by this module).
 * @{
 */

/**
 * @addtogroup Accelerometer
 * @{
 */

/**
 * @defgroup Accelerometer_Exported_Types             Accelerometer Exported Types
 * @{
 */

/**
 * @brief  Accelerometer/Magnetometer Functional state. Used to enable or disable a specific option.
 */
typedef enum
{
  LSM_DISABLE = 0,
  LSM_ENABLE = !LSM_DISABLE
}LSMFunctionalState;


/**
 * @brief  Accelerometer/Magnetometer Flag status. Used to set/reset the sensor flags.
 */   
typedef enum
{
  LSM_RESET = 0,
  LSM_SET = !LSM_RESET
}LSMFlagStatus;


/**
 * @brief  Accelerometer Output Data Rate
 */
typedef enum
{
  LSM_ODR_1_HZ       = 0x10,         /*!< Output Data Rate = 1 Hz */
  LSM_ODR_10_HZ      = 0x20,         /*!< Output Data Rate = 10 Hz */
  LSM_ODR_25_HZ      = 0x30,         /*!< Output Data Rate = 25 Hz */
  LSM_ODR_50_HZ      = 0x40,         /*!< Output Data Rate = 50 Hz */
  LSM_ODR_100_HZ     = 0x50,         /*!< Output Data Rate = 100 Hz */
  LSM_ODR_200_HZ     = 0x60,         /*!< Output Data Rate = 200 Hz */
  LSM_ODR_400_HZ     = 0x70,         /*!< Output Data Rate = 400 Hz */
  LSM_ODR_1620_HZ    = 0x80,         /*!< Output Data Rate = 1620 Hz only in Low Power Mode */
  LSM_ODR_1344_HZ    = 0x90          /*!< Output Data Rate = 1344 Hz in Normal mode and 5376 Hz in Low Power Mode */
}AccOutputDataRate;


/**
 * @brief  Accelerometer Power Mode
 */
typedef enum
{
  LSM_NORMAL_MODE    = 0x00,        /*!< Normal mode enabled */
  LSM_LOW_POWER_MODE = 0x08         /*!< Low Power mode enabled */
}AccPowerMode;


/**
 * @brief  Accelerometer Axes
 */
typedef enum
{
  LSM_X_AXIS_DIS   = 0x00,          /*!< X Axis disabled */
  LSM_X_AXIS_EN    = 0x01,          /*!< X Axis enabled */
  LSM_Y_AXIS_DIS   = 0x00,          /*!< Y Axis disabled */
  LSM_Y_AXIS_EN    = 0x02,          /*!< Y Axis enabled */
  LSM_Z_AXIS_DIS   = 0x00,          /*!< Z Axis disabled */
  LSM_Z_AXIS_EN    = 0x04,          /*!< Z Axis enabled */
  LSM_ALL_AXES_DIS = 0x00,          /*!< All axes disabled */
  LSM_ALL_AXES_EN  = 0x07           /*!< All axes enabled */
}AccAxesEnabling;


/**
 * @brief  Accelerometer Full scale selection
 */
typedef enum
{
  LSM_FS_2G   = 0x00,       /*!< ±2 g */
  LSM_FS_4G   = 0x10,       /*!< ±4 g */
  LSM_FS_8G   = 0x20,       /*!< ±8 g */
  LSM_FS_16G  = 0x30        /*!< ±16 g */
}AccFullScale;


/**
 * @brief  Accelerometer Block Data Update selection
 */
typedef enum
{
  LSM_CONTINUOS_UPDATE   = 0x00,     /*!< Continuos Update */
  LSM_BLOCK_UPDATE      = 0x80      /*!< Single Update: output registers not updated until MSB and LSB reading */
}AccBlockDataUpdate;


/**
 * @brief  Accelerometer Endianness selection
 */
typedef enum
{
  LSM_LITTLE_ENDIAN   = 0x00,     /*!< Little Endian: data LSB @ lower address */
  LSM_BIG_ENDIAN      = 0x40      /*!< Big Endian: data MSB @ lower address */
}AccEndianness;


/**
 * @brief  Accelerometer High Pass Mode Filter Selection
 */
typedef enum
{
  LSM_HPFM_NORMAL     = 0x00,       /*!< Normal Mode */
  LSM_HPFM_REFERENCE  = 0x40,       /*!< Reference Signal for filtering */
  LSM_HPFM_AOI        = 0xC0        /*!< Autoreset On Interrupt event */
}AccHPFMode;


/**
 * @brief  Accelerometer High Pass Filter Cut-Off
 */
typedef enum
{
  LSM_HPCF_8  = 0x00,       /*!< ft= ODR[hz]/(6*8). For more details see Accelerometer Control Register 2 description */
  LSM_HPCF_16 = 0x10,       /*!< ft= ODR[hz]/(6*16). For more details see Accelerometer Control Register 2 description */
  LSM_HPCF_32 = 0x20,       /*!< ft= ODR[hz]/(6*32). For more details see Accelerometer Control Register 2 description */
  LSM_HPCF_64 = 0x30        /*!< ft= ODR[hz]/(6*64). For more details see Accelerometer Control Register 2 description */
}AccHPFCutOff;



/**
 * @brief Accelerometer Irq on line 1 list
 */
typedef enum
{
  LSM_I1_OVERRUN = 0x02,
  LSM_I1_WTM = 0x04,
  LSM_I1_DRDY2 = 0x08,
  LSM_I1_DRDY1 = 0x10,
  LSM_I1_AOI2 = 0x20,
  LSM_I1_AOI1 = 0x40,
  LSM_I1_CLICK = 0x80
}LSMAIrq1List;


/**
 * @brief Accelerometer Irq on line 2 list
 */
typedef enum
{
  LSM_I2_CLICK = 0x80,
  LSM_I2_INT1 = 0x40,
  LSM_I2_INT2 = 0x20,
  LSM_I2_BOOT = 0x10,
  LSM_I2_P2ACT = 0x08,
  LSM_I2_H_LACTIVE = 0x02,
}LSMAIrq2List;


/**
 * @brief Accelerometer data status. It notifies if data on axis are available or overrided
 */
typedef struct
{
  LSMFlagStatus X_Da:1;
  LSMFlagStatus Y_Da:1;
  LSMFlagStatus Z_Da:1;
  LSMFlagStatus ZYX_Da:1;
  LSMFlagStatus X_Or:1;
  LSMFlagStatus Y_Or:1;
  LSMFlagStatus Z_Or:1;
  LSMFlagStatus ZYX_Or:1;
}LSMADataStatus;


/**
 * @brief Accelerometer Init structure definition
 */
typedef struct
{
  AccPowerMode xPowerMode;                /*!< Power mode selection */
  AccOutputDataRate xOutputDataRate;      /*!< Output Data Rate */
  AccAxesEnabling xEnabledAxes;           /*!< Axes to be enabled */
  AccFullScale xFullScale;                /*!< Full Scale */
  AccBlockDataUpdate xDataUpdate;         /*!< Data Update mode : Continuos update or data don`t change until MSB and LSB nex reading */
  AccEndianness xEndianness;              /*!< Endianness */
  LSMFunctionalState xHighResolution;     /*!< High Resolution enabling/disabling */
}LSMAccInit;


/**
 * @brief Accelerometer Filter Init structure definition
 */
typedef struct
{
  LSMFunctionalState xHPF;        /*!< HPF enabling/disabling */
  AccHPFMode xHPF_Mode;           /*!< HPF MODE: Normal mode, Reference signal or Auntoreset on interrupt event for filtering */
  uint8_t cHPFReference;          /*!< Reference value for filtering. Used only in case the mode is "Reference Signal" */
  AccHPFCutOff xHPFCutOff;        /*!< HPF_frequency ft=ODR/6*HPc  HPc=8,16,32,64 */
  LSMFunctionalState xHPFClick;   /*!< HPF_enabling/disabling on CLICK function */
  LSMFunctionalState xHPFAOI2;    /*!< HPF_enabling/disabling for AOI function on interrupt 2 */
  LSMFunctionalState xHPFAOI1;    /*!< HPF_enabling/disabling for AOI function on interrupt 1 */
}LSMAccFilterInit;


/**
 * @brief  Accelerometer FIFO Working Mode
 */
typedef enum
{
  LSM_BYPASS_MODE = 0x00,                /*!< Bypass mode: don't use the FIFO */
  LSM_FIFO_MODE = 0x40,                  /*!< FIFO mode */
  LSM_STREAM_MODE = 0x80,                /*!< Stream mode */
  LSM_TRIGGER_MODE = 0xC0                /*!< Trigger mode */
}LSMAFifoMode;


/**
 * @brief  Accelerometer Sensor IRQ line
 */
typedef enum
{
  LSM_INT1_LINE=0x01,                     /*!< Sensor IRQ line 1 */
  LSM_INT2_LINE                           /*!< Sensor IRQ line 2 */
}LSMAIrqLine;


/**
 * @brief Accelerometer FIFO Init structure definition
 */
typedef struct
{
  LSMAFifoMode xFifoMode;               /*!< FIFO operating mode */
  LSMAIrqLine  xTriggerSel;             /*!< External interrupt line linked to the FIFO trigger event */
  uint8_t cWtm;                         /*!< WaterMark level for FIFO in range [0, 31] */
}LSMAccFifoInit;


/**
 * @brief Accelerometer FIFO Status bitfield structure
 */
typedef struct
{
  uint8_t FIFO_FSS:5;                 /*!< FIFO unread samples */
  LSMFlagStatus FIFO_EMPTY_FLAG:1;          /*!< FIFO Empty flag */
  LSMFlagStatus FIFO_OVRN_FLAG:1;           /*!< FIFO Overrun flag */
  LSMFlagStatus FIFO_WTM_FLAG:1;            /*!< FIFO Watermark flag */

}LSMAccFifoStatus;

/**
 * @brief Accelerometer Events on axis bitfield structure
 */
typedef struct
{
  LSMFlagStatus X_LOW:1;                  /*!< X axis low event */
  LSMFlagStatus X_HIGH:1;                 /*!< X axis high event */
  LSMFlagStatus Y_LOW:1;                  /*!< Y axis low event */
  LSMFlagStatus Y_HIGH:1;                 /*!< Y axis high event */
  LSMFlagStatus Z_LOW:1;                  /*!< Z axis low event */
  LSMFlagStatus Z_HIGH:1;                 /*!< Z axis high event */
  uint8_t :2;                             /*!< 2 bits padding */
}LSMAccAxisEvents;


/**
 * @brief Accelerometer External events on axis combination
 */
typedef enum
{
  LSM_OR_COMBINATION = 0x00,            /*!< OR combination of enabled IRQs */
  LSM_AND_COMBINATION = 0x80,           /*!< AND combination of enabled IRQs */
  LSM_SIXD_MOV_RECOGNITION = 0x40,        /*!< 6D movement recognition */
  LSM_SIXD_POS_RECOGNITION = 0xC0        /*!< 6D position recognition */

}LSMAccIrqOnaxisCombination;


/**
 * @brief Accelerometer Click initialization structure
 */
typedef struct
{
  uint16_t nClickThreshold;                  /*!< Click acceleration threshold expressed in mg */
  LSMFunctionalState xNegativeDetection;    /*!< Click negative detection */
  uint8_t cClickTimeLimit;                   /*!< Click time limit expressed in ms */
  uint8_t cDClickTimeLatency;               /*!< Click time latency expressed in ms (only for double click) */
  uint8_t cDClickTimeWindow;                /*!< Click time window expressed in ms (only for double click) */
}LSMAccClickInit;


/**
 * @}
 */

/**
 * @defgroup Accelerometer_Exported_Constants         Accelerometer Exported Constants
 * @{
 */


/**
 * @defgroup Accelerometer_Sensitivity_Defines        Accelerometer Sensitivity Defines
 * @{
 */

#define LSM_Acc_Sensitivity_2g     1.0    /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     0.5    /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     0.25   /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    0.0834 /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

/**
 * @}
 */ /* end of group Accelerometer_Sensitivity_Defines */

/**
 * @}
 */

/**
 * @defgroup Accelerometer_Exported_Macros       Accelerometer Exported Macros
 * @{
 */

/** @defgroup Accelerometer_I2C_Communication     Accelerometer I2C Communication
 * @{
 */

#define Lsm303dlhcAccI2CBufferRead(pVal,cAddress,nBytes)      Lsm303dlhcI2CBufferRead(LSM_A_I2C_ADDRESS,pVal,cAddress,nBytes)
#define Lsm303dlhcAccI2CBufferWrite(pVal,cAddress,nBytes)     Lsm303dlhcI2CBufferWrite(LSM_A_I2C_ADDRESS,pVal,cAddress,nBytes)

#define Lsm303dlhcAccI2CByteRead(pVal,cAddress)               Lsm303dlhcAccI2CBufferRead(pVal,cAddress,1)
#define Lsm303dlhcAccI2CByteWrite(pVal,cAddress)              Lsm303dlhcAccI2CBufferWrite(pVal,cAddress,1)

/**
 * @}
 */

/**
 *@}
 */

/** @defgroup Accelerometer_Register_Mapping          Accelerometer Register Mapping
 * @{
 */

/**
 * @brief Accelerometer I2C Slave Address
 */
#define LSM_A_I2C_ADDRESS         0x33

/**
 * @brief Accelerometer Control Register 1
 * \code
 * Read/write
 * Default value: 0x07
 * 7:4 ODR3-ODR0: Data Rate selection
 *     ODR3 | ODR2 | ODR1 | ODR0 | Power Mode Sel & Out Data Rate[Hz]
 *     --------------------------------------------------------------
 *       0  |  0   |  0   |  0   |  Power Down
 *       0  |  0   |  0   |  1   |  Normal / Low Power (1 Hz)
 *       0  |  0   |  1   |  0   |  Normal / Low Power (10 Hz)
 *       0  |  0   |  1   |  1   |  Normal / Low Power (25 Hz)
 *       0  |  1   |  0   |  0   |  Normal / Low Power (50 Hz)
 *       0  |  1   |  0   |  1   |  Normal / Low Power (100 Hz)
 *       0  |  1   |  1   |  0   |  Normal / Low Power (200 Hz)
 *       0  |  1   |  1   |  1   |  Normal / Low Power (400 Hz)
 *       1  |  0   |  0   |  0   |  Low Power (1620 Hz)
 *       1  |  0   |  0   |  1   |  Normal (1344 Hz) / Low Power (5376 Hz)
 * 3 LPen: Low Power Mode Enable. 0 - Normal Mode   1 - Low Power Mode
 * 2 Zen: Z axis enable. 0 - Z axis disabled  1- Z axis enabled
 * 1 Yen: Y axis enable. 0 - Y axis disabled  1- Y axis enabled
 * 0 Xen: X axis enable. 0 - X axis disabled  1- X axis enabled
 * \endcode
 */
#define LSM_A_CTRL1_REG_ADDR     0x20


/**
 * @brief Accelerometer Control Register 2
 * \code
 * Read/write
 * Default value: 0x00
 * 7:6 HPM1-HPM0: High pass filter mode selection:
 *     HPM1 | HPM0 |   High pass filter mode
 *     -----------------------------------------
 *     0  |  0   |   Normal mode (reset reading HP_RESET_FILTER)
 *     0  |  1   |   Reference signal for filtering
 *     1  |  0   |   Normal mode (reset reading HP_RESET_FILTER)
 *     1  |  1   |   Autoreset on interrupt event
 * 5:4 HPCF1-HPCF0: High pass filter cut-off frequency (ft) configuration
 *     ft= ODR[hz]/6*HPc
 *     HPCF1 | HPCF0 | HPc | ft[Hz]     | ft[Hz]      | ft[Hz]      | ft[Hz]      | ft[Hz]      | ft[Hz]       | ft[Hz]       | ft[Hz]
 *           |       |     | ODR 1 Hz   | ODR 10 Hz   | ODR 25 Hz   | ODR 50 Hz   | ODR 100 Hz  | ODR 200 HZ   | ODR 400 Hz   | ODR 1344 HZ
 *     ------------------------------------------------------------------------------------------------------------------------------------
 *       0   |   0   | 8   |  0.02      |  0.2        |  0.52       |  1.04       |  2.08       |  4.16        |  8.33        |  28
 *       0   |   1   | 16  |  0.01      |  0.1        |  0.26       |  0.52       |  1.04       |  2.08        |  4.16        |  14
 *       1   |   0   | 32  |  0.005     |  0.05       |  0.13       |  0.26       |  0.52       |  1.04        |  2.08        |  7
 *       1   |   1   | 64  |  0.0026    |  0.026      |  0.065      |  0.13       |  0.26       |  0.52        |  1.04        |  3.5
 * 3 FDS: Filtered data selection. 0 - internal filter bypassed; 1 - data from internal filter sent to output register and FIFO
 * 2 HPCLICK: High pass filter enabled for CLICK function. 0 - filter bypassed; 1 - filter enabled
 * 1 HPIS2: High pass filter enabled for interrupt 2 source. 0 - filter bypassed; 1 - filter enabled
 * 0 HPIS1: High pass filter enabled for interrupt 1 source. 0 - filter bypassed; 1 - filter enabled
 * \endcode
 */
#define LSM_A_CTRL2_REG_ADDR     0x21


/**
 * @brief Accelerometer Control Register 3 Interrupt Control Register
 * \code
 * Read/write
 * Default value: 0x00
 * 7 I1_CLICK: Click interrupt on INT1. 0 - disable; 1 - enable
 * 6 I1_AOI1: AOI1 interrupt on INT1. 0 - disable; 1 - enable
 * 5 I1_AOI2: AOI2 interrupt on INT1. 0 - disable; 1 - enable
 * 4 I1_DRDY1: DRDY1 interrupt on INT1. 0 - disable; 1 - enable
 * 3 I1_DRDY2: DRDY2 interrupt on INT1. 0 - disable; 1 - enable
 * 2 I1_WTM: FIFO watermark interrupt on INT1. 0 - disable; 1 - enable
 * 1 I1_OVERRUN: FIFO overrun interrup on INT1. 0 - disable; 1 - enable
 * 0 -: Not used. It shall be zero
 * \endcode
 */
#define LSM_A_CTRL3_REG_ADDR     0x22


/**
 * @brief Accelerometer Control Register 4
 * \code
 * Read/write
 * Default value: 0x00
 * 7 BDU: Block data update. 0 -continuos update; 1- output registers not updated between MSB and LSB reading
 * 6 BLE: Big/little endian data selection. 0 - data LSB @ lower address; 1 - data MSB @ lower address
 * 5:4 FS1 - FS0: Full-scale selection
 *     FS1 | FS0 |   Full Scale
 *     -----------------------------------------
 *      0  |  0   |   ±2 g
 *      0  |  1   |   ±4 g
 *      1  |  0   |   ±8 g
 *      1  |  1   |   ±16 g
 * 3 HR: High resolution output mode. 0 - disable;  1- enable
 * 2:1 -: Not used. They shall be zero
 * 0 SIM: SPI serial interface mode selection. 0 - 4 wire interface; 1 - 3 wire interface
 * \endcode
 */
#define LSM_A_CTRL4_REG_ADDR     0x23


/**
 * @brief Accelometer Control Register 5
 * \code
 * Read/write
 * Default value: 0x00
 * 7 BOOT: Reboot memory content. 0 - Normal mode; 1 - Reboot memory content
 * 6 FIFO_EN: FIFO enable. 0 - FIFO disabled; 1 - FIFO enabled
 * 5:4 -: Not used. They shall be zero
 * 3 LIR_INT1: Latch interrupt request on INT1_SRC register. 0 - Interrupt request not latched; 1 - Interrupt request latched
 * 2 D4D_INT1: 4D detection enabling (enabled on INT1 when 6D bit on INT1_CFG is set to 1). 0 - 4D detection disabled; 1 - 4D detection enabled
 * 1 LIR_INT2: Latch interrupt request on INT2_SRC register. 0 - Interrupt request not latched; 1 - Interrupt request latched
 * 0 D4D_INT1: 4D detection enabling (enabled on INT2 when 6D bit on INT2_CFG is set to 1). 0 - 4D detection disabled; 1 - 4D detection enabled
 * \endcode
 */
#define LSM_A_CTRL5_REG_ADDR     0x24


/**
 * @brief Accelometer Control Register 6
 * \code
 * Read/write
 * Default value: 0x00
 * 7 I2_CLICKen: CLICK interrupt on PAD2. 0 - disable; 1 - enable
 * 6 I2_INT1: Interrupt 1 on PAD2. 0 - disable; 1 - enable
 * 5 I2_INT2: Interrupt 2 on PAD2. 0 - disable; 1 - enable
 * 4 BOOT_I1: Reboot memory content on PAD2. 0 - disable; 1 - enable
 * 3 P2_ACT: Active function status on PAD2. 0 - disable; 1 - enable
 * 2 -: Not used. It shall be zero
 * 1 H_LACTIVE: Interrupt active high, low. 0 - active high; 1 - active low
 * 0 -: Not used. It shall be zero.
 * \endcode
 */
#define LSM_A_CTRL6_REG_ADDR     0x25


/**
 * @brief Accelrometer HP Reference register
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 Ref7 - Ref0:  Reference value for high-pass filter.
 * This register sets the acceleration value taken as a reference for the high-pass filter output
 * When filter is turned on (at least one of FDS, HPen2, or HPen1 bit is equal to ‘1’) and HPM
 * bits are set to “01”, filter out is generated taking this value as a reference.
 * \endcode
 */
#define LSM_A_REFERENCE_REG_ADDR     0x26


/**
 * @brief Accelerometer Status Register
 * \code
 * Read
 * Default value: 0x00
 * 7 ZYXOR: X, Y and Z axis data overrun. 0 - No overrun has occurred; 1 - A new set of data has overwritten the previous ones
 * 6 ZOR: Z axis data overrun. 0 - No overrun has occurred; 1 - A new data for the Z-axis has overwritten the previous one
 * 5 YOR: Y axis data overrun. 0 - No overrun has occurred; 1 - A new data for the Y-axis has overwritten the previous one
 * 4 XOR: X axis data overrun. 0 - No overrun has occurred; 1 - A new data for the X-axis has overwritten the previous one
 * 3 ZYXDA: X, Y and Z axis new data available. 0 - A new set of data is not yet available; 1 - A new set of data available
 * 2 ZDA: Z axis new data available. 0 - A new data for Z-axis is not yet available; 1 - A new data for Z-axis is available
 * 1 YDA: Y axis new data available. 0 - A new data for Y-axis is not yet available; 1 - A new data for Y-axis is available
 * 0 XDA: X axis new data available. 0 - A new data for X-axis is not yet available; 1 - A new data for X-axis is available
 * \endcode
 */
#define LSM_A_STATUS_REG_ADDR     0x27


/**
 * @brief Accelerometer X axis Output Data Low Register
 * \code
 * Read
 * Default value: ( The value is expressed as 16bit two’s complement). Because the resolution is 12 bit,
 *                the entire acceleration data (expressed as signed 16-bit) shall be divided by 16.
 * 7:0 XOUT7-XOUT0: ACC Data LSB (if in Little Endian Mode --> BLE bit in CTRL_REG4 is 0)
 *                  ACC Data MSB (if in Big Endian Mode --> BLE bit in CTRL_REG4 is 1)
 * \endcode
 */
#define LSM_A_OUT_X_L_REG_ADDR     0x28


/**
 * @brief  Acceleration X-axis Output Data High Register
 * \code
 * Read
 * Default value: ( The value is expressed as 16bit two’s complement). Because the resolution is 12 bit,
 *                the entire acceleration data (expressed as signed 16-bit) shall be divided by 16.
 * 7:0 XOUT15-XOUT8: ACC Data MSB (if in Little Endian Mode --> BLE bit in CTRL_REG1 is 0)
 *                   ACC Data LSB (if in Big Endian Mode --> BLE bit in CTRL_REG1 is 1)
 * \endcode
 */
#define LSM_A_OUT_X_H_REG_ADDR     0x29


/**
 * @brief Acceleration Y-axis Output Data Low Register
 * \code
 * Read
 * Default value: ( The value is expressed as 16bit two’s complement). Because the resolution is 12 bit,
 *                the entire acceleration data (expressed as signed 16-bit) shall be divided by 16.
 * 7:0 YOUT7-YOUT0: ACC Data LSB (if in Little Endian Mode --> BLE bit in CTRL_REG4 is 0)
 *                  ACC Data MSB (if in Big Endian Mode --> BLE bit in CTRL_REG4 is 1)
 * \endcode
 */
#define LSM_A_OUT_Y_L_REG_ADDR     0x2A


/**
 * @brief  Acceleration Y-axis Output Data High Register
 * \code
 * Read
 * Default value: ( The value is expressed as 16bit two’s complement). Because the resolution is 12 bit,
 *               the entire acceleration data (expressed as signed 16-bit) shall be divided by 16.
 * 7:0 YOUT15-YOUT8: ACC Data MSB (if in Little Endian Mode --> BLE bit in CTRL_REG1 is 0)
 *                   ACC Data LSB (if in Big Endian Mode --> BLE bit in CTRL_REG1 is 1)
 * \endcode
 */
#define LSM_A_OUT_Y_H_REG_ADDR     0x2B


/**
 * @brief  Acceleration Z-axis Output Data Low Register
 * \code
 * Read
 * Default value: ( The value is expressed as 16bit two’s complement). Because the resolution is 12 bit,
 *               the entire acceleration data (expressed as signed 16-bit) shall be divided by 16.
 * 7:0 ZOUT7-ZOUT0: ACC Data LSB (if in Little Endian Mode --> BLE bit in CTRL_REG4 is 0)
 *                  ACC Data MSB (if in Big Endian Mode --> BLE bit in CTRL_REG4 is 1)
 * \endcode
 */
#define LSM_A_OUT_Z_L_REG_ADDR     0x2C


/**
 * @brief  Acceleration Z-axis Output Data High Register
 * \code
 * Read
 * Default value: ( The value is expressed as 16bit two’s complement). Because the resolution is 12 bit,
 *               the entire acceleration data (expressed as signed 16-bit) shall be divided by 16.
 * 7:0 ZOUT15-ZOUT8: ACC Data MSB (if in Little Endian Mode --> BLE bit in CTRL_REG1 is 0)
 *                  ACC Data LSB (if in Big Endian Mode --> BLE bit in CTRL_REG1 is 1)
 * \endcode
 */
#define LSM_A_OUT_Z_H_REG_ADDR     0x2D


/**
 * @brief FIFO Control Register
 * \code
 * Read/write
 * Default value: 0x00
 * 7:6 FM1 - FM0: FIFO Mode selection
 *     FM1 | FM0  |  FIFO Mode Configuration
 *     -------------------------------------
 *      0  |  0   |   Bypass Mode
 *      0  |  1   |   FIFO Mode
 *      1  |  0   |   Stream Mode
 *      1  |  1   |   Trigger Mode
 * 5 TR: Trigger selection. 0 - trigger event linked to trigger signal on INT1; 1 - trigger event linked to trigger signal on INT2
 * 4:0 FTH4 - FTH0: ?????
 * \endcode
 */
#define LSM_A_FIFO_CTRL_REG_ADDR     0x2E


/**
 * @brief FIFO Status Register
 * \code
 * Read
 * Default value: 0x00
 * 7 WTM:
 * 6 OVRN_FIFO:
 * 5 EMPTY:
 * 4:0 FSS4 - FSSO:
 * \endcode
 */
#define LSM_A_FIFO_STATUS_REG_ADDR     0x2F


/**
 * @brief Accelerometer Configuration Register for Interrupt 1 source.
 * \code
 * Read/write
 * Default value: 0x00
 * 7 AOI: AND/OR combination of Interrupt events. See table below
 * 6 6D:  6 direction detection function enable. See table below
 * 5 ZHIE/ZUPE:  Enable interrupt generation on Z high event or on direction recognition. 0: disable interrupt request; 1: enable interrupt request on measured acceleration for value higher than preset threshold
 * 4 ZLIE/ZDOWNNE:  Enable interrupt generation on Z low event or on direction recognition. 0: disable interrupt request;  1: enable interrupt request on measured acceleration for value lower than preset threshold
 * 3 YHIE/YUPE:  Enable interrupt generation on Y high event or on direction recognition. 0: disable interrupt request; 1: enable interrupt request on measured acceleration for value higher than preset threshold
 * 2 YLIE/YDOWNE:  Enable interrupt generation on Y low event or on direction recognition. 0: disable interrupt request;  1: enable interrupt request on measured acceleration for value lower than preset threshold
 * 1 XHIE/XUPE:  Enable interrupt generation on X high event or on direction recognition. 0: disable interrupt request; 1: enable interrupt request on measured acceleration for value higher than preset threshold
 * 0 XLIE/XDOWNE:  Enable interrupt generation on X low event or on direction recognition. 0: disable interrupt request; 1: enable interrupt request on measured acceleration for value lower than preset threshold
 *            AOI     |   6D         | Interrupt mode
 *       --------------------------------------------------------
 *             0      |       0      | OR combination of interrupt events
 *             0      |       1      | 6 direction movement recognition
 *             1      |       0      | AND combination of interrupt events
 *             1      |       1      | 6 direction position recognition
 * \endcode
 */
#define LSM_A_INT1_CFG_REG_ADDR 0x30


/**
 * @brief Accelerometer Interrupt 1 source register.
 * Reading at this address clears INT1_SRC IA bit (and the interrupt signal on INT 1 pin) and
 * allows the refreshment of data in the INT1_SRC register if the latched option was chosen.
 * \code
 * Read
 * Default value: 0x00
 * 7 0: It shall be zero for correct working of the device
 * 6 IA : Interrupt active. 0: no interrupt has been generated; 1: one or more interrupts have been generated
 * 5 ZH:  Z high. 0: no interrupt, 1: Z High event has occurred
 * 4 ZL:  Z low. 0: no interrupt; 1: Z Low event has occurred
 * 3 YH:  Y high. 0: no interrupt, 1: Y High event has occurred
 * 2 YL:  Y low. 0: no interrupt; 1: Y Low event has occurred
 * 1 XH:  X high. 0: no interrupt, 1: X High event has occurred
 * 0 XL:  X low. 0: no interrupt; 1: X Low event has occurred
 * \endcode
 */
#define LSM_A_INT1_SRC_REG_ADDR 0x31


/**
 * @brief Accelerometer Interrupt 1 Threshold Register
 * \code
 * Read/write
 * Default value: 0x00
 * 7 0: It shall be zero for correct working of the device
 * 6 THS6-THS0: Interrupt 1 threshold.
 * \endcode
 */
#define LSM_A_INT1_THS_REG_ADDR 0x32


/**
 * @brief Acceleroemter INT1_DURATION Register
 * \code
 * Read/write
 * Default value: 0x00
 * 7 0: It shall be zero for correct working of the device
 * 6 D6-D0: Duration value. (Duration  steps and maximum values depend on the ODR chosen)
 * \endcode
 */
#define LSM_A_INT1_DURATION_REG_ADDR 0x33


/**
 * @brief INT2_CFG Register Configuration register for Interrupt 2 source.
 * \code
 * Read/write
 * Default value: 0x00
 * 7 AOI: AND/OR combination of Interrupt events. See table below
 * 6 6D:  6 direction detection function enable. See table below
 * 5 ZHIE:  Enable interrupt generation on Z high event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
 * 4 ZLIE:  Enable interrupt generation on Z low event. 0: disable interrupt request;  1: enable interrupt request on measured accel. value lower than preset threshold
 * 3 YHIE:  Enable interrupt generation on Y high event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
 * 2 YLIE:  Enable interrupt generation on Y low event. 0: disable interrupt request;  1: enable interrupt request on measured accel. value lower than preset threshold
 * 1 XHIE:  Enable interrupt generation on X high event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
 * 0 XLIE:  Enable interrupt generation on X low event. 0: disable interrupt request; 1: enable interrupt request on measured accel. value lower than preset threshold
 *            AOI     |   6D         | Interrupt mode
 *       --------------------------------------------------------
 *             0      |       0      | OR combination of interrupt events
 *             0      |       1      | 6 direction movement recognition
 *             1      |       0      | AND combination of interrupt events
 *             1      |       1      |  6 direction position recognition
 * \endcode
 */
#define LSM_A_INT2_CFG_REG_ADDR 0x34


/**
 * @brief INT2_SCR Register Interrupt 2 source register.
 * Reading at this address clears INT2_SRC IA bit (and the interrupt signal on INT 2 pin) and
 * allows the refreshment of data in the INT2_SRC register if the latched option was chosen.
 * \code
 * Read
 * Default value: 0x00
 * 7 0: It shall be zero for correct working of the device
 * 6 IA : Interrupt active. 0: no interrupt has been generated; 1: one or more interrupts have been generated
 * 5 ZH:  Z high. 0: no interrupt, 1: Z High event has occurred
 * 4 ZL:  Z low. 0: no interrupt; 1: Z Low event has occurred
 * 3 YH:  Y high. 0: no interrupt, 1: Y High event has occurred
 * 2 YL:  Y low. 0: no interrupt; 1: Y Low event has occurred
 * 1 YH:  X high. 0: no interrupt, 1: X High event has occurred
 * 0 YL:  X low. 0: no interrupt; 1: X Low event has occurred
 * \endcode
 */
#define LSM_A_INT2_SRC_REG_ADDR 0x35


/**
 * @brief Accelerometer Interrupt 2 Threshold Register
 * \code
 * Read/write
 * Default value: 0x00
 * 7 0: It shall be zero for correct working of the device
 * 6 THS6-THS0: Interrupt 2 threshold.
 * \endcode
 */
#define LSM_A_INT2_THS_REG_ADDR 0x36


/**
 * @brief Acceromter INT2_DURATION Register
 * \code
 * Read/write
 * Default value: 0x00
 * 7 0: It shall be zero for correct working of the device
 * 6 D6-D0: Duration value. (Duration  steps and maximum values depend on the ODR chosen)
 * \endcode
 */
#define LSM_A_INT2_DURATION_REG_ADDR 0x37


/**
 * @brief CLICK Configuration register.
 * \code
 * Read/write
 * Default value: 0x00
 * 7:6 -: It shall be zero for correct working of the device
 * 5 ZD: Enable interrupt double click on Z axis. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
 * 4 ZS: Enable interrupt single click on Z axis. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
 * 3 YD: Enable interrupt double click on Y axis. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
 * 2 YS: Enable interrupt single click on Y axis. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
 * 1 XD: Enable interrupt double click on X axis. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
 * 0 XS: Enable interrupt single click on X axis. 0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold
 * \endcode
*/
#define LSM_A_CLICK_CONF_REG_ADDR 0x38


/**
 * @brief CLICK source register.
 * \code
 * Read
 * Default value: 0x00
 * 7 0: It shall be zero for correct working of the device
 * 6 IA: Interrupt active. 0: no interrupt has been generated; 1: one or more interrupts have been generated
 * 5 DCLICK: Double CLICK-CLICK enable. 0: Double CLICK-CLICK detection disable; 1: Double CLICK-CLICK detection enable
 * 4 SCLICK: Single CLICK-CLICK enable. 0: Single CLICK-CLICK detection disable; 1: Single CLICK-CLICK detection enable
 * 3 Sign: CLICK-CLICK Sign. 0: positive detection; 1: negative detection
 * 2 Z: Z CLICK-CLICK detection. 0: no interrupt; 1: Z high event has occurred
 * 1 Y: Y CLICK-CLICK detection. 0: no interrupt; 1: Y high event has occurred
 * 0 X: X CLICK-CLICK detection. 0: no interrupt; 1: X high event has occurred
 * \endcode
*/
#define LSM_A_CLICK_SRC_REG_ADDR 0x39


/**
 * @brief CLICK Threshold Register. 1LSB = full-scale/128. THS6 through THS0 define the threshold which is used by the system to
 *        start the click detection procedure. The threshold value is expressed over 7 bits as an unsigned number.
 * \code
 * Read/write
 * Default value: 0x00
 * 7 0: It shall be zero for correct working of the device
 * 6 THS6-THS0: CLICK-CLICK threshold.
 * \endcode
 */
#define LSM_A_CLICK_THS_REG_ADDR 0x3A


/**
 * @brief CLICK Time Limit Register. 1LSB = 1/ODR. TLI7 through TLI0 define the maximum time interval that can elapse
 *        between the start of the click detection procedure (the accelration on the selected channel exceeds the programmed
 *        threshold) and when the acceleration goes back below the threshold
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 TLI7-TLI0: CLICK-CLICK time limit.
 * \endcode
 */
#define LSM_A_CLICK_TIME_LIMIT_REG_ADDR 0x3B


/**
 * @brief CLICK Time Latency Register. 1LSB = 1/ODR. TLA7 through TLA0 define the time interval that starts after the first
 *        click detection where the click detection procedure is disabled, in cases where the device is configured for double click
 *        detection.
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 TLA7-TLA0: CLICK-CLICK time latency.
 * \endcode
 */
#define LSM_A_CLICK_TIME_LATENCY_REG_ADDR 0x3C


/**
 * @brief CLICK Time Window Register. 1LSB = 1/ODR. TW7 through TW0 define the maximum interval of time
 *        that can elapse after the end of latency interval in which the click detection procedure can start,
 *        in cases where the device is configured for double click detection.
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 TW7-TW0: CLICK-CLICK time window.
 * \endcode
 */
#define LSM_A_CLICK_TIME_WINDOW_REG_ADDR 0x3D


/**
 * @}
 */



/** @defgroup Accelerometer_Exported_Functions          Accelerometer Exported Functions
 * @{
 */
void Lsm303dlhcAccConfig(LSMAccInit* pxLSMAccInitStruct);
void Lsm303dlhcAccGetInfo(LSMAccInit* pxLSMAccInitStruct);
void Lsm303dlhcAccFilterConfig(LSMAccFilterInit* pxLSMAccFilterInitStruct);
void Lsm303dlhcAccFilterGetInfo(LSMAccFilterInit* pxLSMAccFilterInitStruct);
void Lsm303dlhcAccLowPowerMode(LSMFunctionalState xFunctionalState);
void Lsm303dlhcAccSetFullScale(AccFullScale xFullScale);
AccFullScale Lsm303dlhcAccGetEnumFullScale(void);
uint8_t Lsm303dlhcAccGetFullScale(void);
void Lsm303dlhcAccGetSensitivity(float *pfSensitivityXYZ);
void Lsm303dlhcAccSetDataRate(AccOutputDataRate xDataRate);
AccOutputDataRate Lsm303dlhcAccGetEnumDataRate(void);
uint16_t Lsm303dlhcAccGetDataRate(void);
void Lsm303dlhcAccRebootCmd(void);
void Lsm303dlhcAccReadOutReg(uint8_t* pcReg);
void Lsm303dlhcAccReadRawData(int16_t* pnRawData);
void Lsm303dlhcAccReadAcc(float* pfData);
void Lsm303dlhcAccReadAccFifo(int16_t* pnData, uint8_t cDataToRead);
LSMADataStatus Lsm303dlhcAccGetDataStatus(void);
void Lsm303dlhcAccIrq1Config(LSMAIrq1List xLSMAIrq1Config, LSMFunctionalState xNewState);
void Lsm303dlhcAccIrq2Config(LSMAIrq2List xLSMAIrq2Config, LSMFunctionalState xNewState);
void Lsm303dlhcAccFifo(LSMFunctionalState xNewState);
void Lsm303dlhcAccFifoInit(LSMAccFifoInit* pxLSMAccFifoInit);
LSMAccFifoStatus Lsm303dlhcAccFifoGetStatus(void);
void Lsm303dlhcAccSetAxisIrqs(LSMAccIrqOnaxisCombination xIrqCombinations, LSMAccAxisEvents* pxAxisEvents, LSMAIrqLine xIRQLine, LSMFunctionalState xLatched);
LSMFunctionalState Lsm303dlhcAccGetAxisIrqs(LSMAccAxisEvents* pxAxisEvents, LSMAIrqLine xIRQLine);
void Lsm303dlhcAccSetThreshold(int16_t nData, LSMAIrqLine xIRQLine);
void Lsm303dlhcAccSetIrqDuration(uint8_t cDuration, LSMAIrqLine xIRQLine);
void Lsm303dlhcAccClickInit(LSMAccClickInit* pxClickInit);

/**
 * @}
 */ 

/**
 * @}
 */


/**
 * @addtogroup Magnetometer
 * @{
 */

/**
 * @defgroup Magnetometer_Exported_Types            Magnetometer Exported Types
 * @{
 */

/**
 * @brief  Magnetometer Output Data Rate
 */
typedef enum
{
  LSM_ODR_0_75_HZ   = 0x00,                       /*!< Output Data Rate = 0.75 Hz */
  LSM_ODR_1_5_HZ    = 0x04,                       /*!< Output Data Rate = 1.5 Hz */
  LSM_ODR_3_0_HZ    = 0x08,                       /*!< Output Data Rate = 3 Hz */
  LSM_ODR_7_5_HZ    = 0x0C,                       /*!< Output Data Rate = 7.5 Hz */
  LSM_ODR_15_HZ     = 0x10,                       /*!< Output Data Rate = 15 Hz */
  LSM_ODR_30_HZ     = 0x14,                       /*!< Output Data Rate = 30 Hz */
  LSM_ODR_75_HZ     = 0x18,                       /*!< Output Data Rate = 75 Hz */
  LSM_ODR_220_HZ    = 0x1C                        /*!< Output Data Rate = 220 Hz */
}MagOutputDataRate;


/**
 * @brief  Magnetometer Full Scale
 */
typedef enum
{
  LSM_FS_1_3_GA  = 0x20,                       /*!< Full scale = ±1.3 Gauss */
  LSM_FS_1_9_GA  = 0x40,                       /*!< Full scale = ±1.9 Gauss */
  LSM_FS_2_5_GA  = 0x60,                       /*!< Full scale = ±2.5 Gauss */
  LSM_FS_4_0_GA  = 0x80,                       /*!< Full scale = ±4.0 Gauss */
  LSM_FS_4_7_GA  = 0xA0,                       /*!< Full scale = ±4.7 Gauss */
  LSM_FS_5_6_GA  = 0xC0,                       /*!< Full scale = ±5.6 Gauss */
  LSM_FS_8_1_GA  = 0xE0                        /*!< Full scale = ±8.1 Gauss */
}MagFullScale;


/**
 * @brief  Magnetometer Working Mode
 */
typedef enum
{
  LSM_CONTINUOS_CONVERSION  = 0x00,       /*!< Continuous-Conversion Mode */
  LSM_SINGLE_CONVERSION     = 0x01,       /*!< Single-Conversion Mode */
  LSM_SLEEP                 = 0x02        /*!< Sleep Mode */
}MagWorkingMode;



/**
 * @brief Magnetometer Init structure definition
 */
typedef struct
{
  MagOutputDataRate xOutputDataRate;      /*!< Magnetometer Output data Rate */
  MagFullScale xFullScale;                /*!< Full Scale Configuration */
  MagWorkingMode xWorkingMode;            /*!< Mode Configuration: Continuos, Single or Sleep Mode */
  LSMFunctionalState xTemperatureSensor;  /*!< Temperature sensor enabling/disabling */
}LSMMagInit;


/**
 * @brief Magnetometer data status. It notifies if data on axis are available or overrun.
 */
typedef struct
{
  uint8_t xDataReady:1;                 /*!< DataReady bit flag */
  uint8_t xDataLock:1;                  /*!< DataLock bit flag */
  uint8_t :6;                           /*!< RFU */
}LSMMDataStatus;

/**
 * @}
 */

/**
 * @defgroup Magnetometer_Exported_Constants      Magnetometer Exported Constants
 * @{
 */

/**
 * @defgroup Magnetometer_Sensitivity             Magnetometer Sensitivity  
 * @{
 */

#define LSM_Magn_Sensitivity_XY_1_3Ga     1100  /*!< magnetometer X Y axes sensitivity for 1.3 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_1_9Ga     855   /*!< magnetometer X Y axes sensitivity for 1.9 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_2_5Ga     670   /*!< magnetometer X Y axes sensitivity for 2.5 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_4Ga       450   /*!< magnetometer X Y axes sensitivity for 4 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_4_7Ga     400   /*!< magnetometer X Y axes sensitivity for 4.7 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_5_6Ga     330   /*!< magnetometer X Y axes sensitivity for 5.6 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_XY_8_1Ga     230   /*!< magnetometer X Y axes sensitivity for 8.1 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_1_3Ga      980   /*!< magnetometer Z axis sensitivity for 1.3 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_1_9Ga      760   /*!< magnetometer Z axis sensitivity for 1.9 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_2_5Ga      600   /*!< magnetometer Z axis sensitivity for 2.5 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_4Ga        400   /*!< magnetometer Z axis sensitivity for 4 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_4_7Ga      355   /*!< magnetometer Z axis sensitivity for 4.7 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_5_6Ga      295   /*!< magnetometer Z axis sensitivity for 5.6 Ga full scale [LSB/Ga] */
#define LSM_Magn_Sensitivity_Z_8_1Ga      205   /*!< magnetometer Z axis sensitivity for 8.1 Ga full scale [LSB/Ga] */

/**
 * @}
 */


/**
 * @defgroup Temperature_Sensitivity            Temperature Sensitivity
 * @{
 */
#define LSM_Temp_Sensitivity     8  /*!< temperature sensitivity [LSB/deg] */

/**
 * @}
 */


/**
 * @}
 */


/**
 * @defgroup Magnetometer_Exported_Macros       Magnetometer Exported Macros
 * @{
 */

/** @defgroup Magnetometer_I2C_Communication        Magnetometer I2C Communication
 * @{
 */
#define Lsm303dlhcMagI2CBufferRead(pVal,cAddress,nBytes)      Lsm303dlhcI2CBufferRead(LSM_M_I2C_ADDRESS,pVal,cAddress,nBytes)
#define Lsm303dlhcMagI2CBufferWrite(pVal,cAddress,nBytes)     Lsm303dlhcI2CBufferWrite(LSM_M_I2C_ADDRESS,pVal,cAddress,nBytes)

#define Lsm303dlhcMagI2CByteRead(pVal,cAddress)               Lsm303dlhcMagI2CBufferRead(pVal,cAddress,1)
#define Lsm303dlhcMagI2CByteWrite(pVal,cAddress)              Lsm303dlhcMagI2CBufferWrite(pVal,cAddress,1)

/**
 * @}
 */

/**
 *@}
 */

/**
 * @defgroup Magnetometer_Register_Mapping        Magnetometer Register Mapping
 * @{
 */

/**
 * @brief Magnetometer I2C Slave Address
 */
#define LSM_M_I2C_ADDRESS         0x3D


/**
 * @brief Magnetometer Control Register A
 * \code
 * Read Write
 * Default value: 0x10
 * 7 TEMP_EN: Temperature sensor enabling. 0: temperature sensor disabled; 1: temperature sensor enabled
 * 6:5 0: They shall be zero for correct working of the device
 * 4:2 DO2-DO0: Data Output Rate Bits
 *     DO2 |  DO1 |  DO0 |   Minimum Data Output Rate (Hz)
 *     ------------------------------------------------------
 *      0  |  0   |  0   |      0.75
 *      0  |  0   |  1   |      1.5
 *      0  |  1   |  0   |      3.0
 *      0  |  1   |  1   |      7.5
 *      1  |  0   |  0   |      15(default)
 *      1  |  0   |  1   |      30
 *      1  |  1   |  0   |      75
 *      1  |  1   |  1   |      220
 * 1:0 0: They shall be zero for correct working of the device
 * \endcode
 */
#define LSM_M_CRA_REG_ADDR     0x00


/**
 * @brief Magnetometer Control Register B
 * \code
 * Read/Write
 * Default value: 0x20
 * 7:5 GN2-GN0: Gain Configuration Bits
 *     GN2 |  GN1 |  GN0 |   Mag Input   | Gain X, Y   | Gain Z      |     Output Range
 *         |      |      |  Range[Ga]    | [LSB/Gauss] | [LSB/Gauss] |
 *     -------------------------------------------------------------------------------------------
 *      0  |  0   |  0   |      NA       |    NA       |     NA      |  0xF800–0x07FF (-2048:2047)
 *      0  |  0   |  1   |    ±1.3       |   1100      |    980      |       ""
 *      0  |  1   |  0   |    ±1.9       |   855       |    760      |       ""
 *      0  |  1   |  1   |    ±2.5       |   670       |    600      |       ""
 *      1  |  0   |  0   |    ±4.0       |   450       |    400      |       ""
 *      1  |  0   |  1   |    ±4.7       |   400       |    355      |       ""
 *      1  |  1   |  0   |    ±5.6       |   330       |    295      |       ""
 *      1  |  1   |  1   |    ±8.1       |   230       |    205      |       ""
 * 4:0 0: They shall be zero for correct working of the device
 * \endcode
 */
#define LSM_M_CRB_REG_ADDR     (uint8_t) 0x01


/**
 * @brief Magnetometer  Mode Register
 * \code
 * Read/Write
 * Default value: 0x02
 * 7:2 0: They shall be zero for correct working of the device
 * 1:0 MD1-MD0: Mode Select Bits
 *     MD1 | MD0  |   MODE
 *    ------------------------------
 *      0  |  0   |  Continuous-Conversion Mode.
 *      0  |  1   |  Single-Conversion Mode
 *      1  |  0   |  Sleep Mode. Device is placed in sleep mode.
 *      1  |  1   |  Sleep Mode. Device is placed in sleep mode.
 * \endcode
 */
#define LSM_M_MR_REG_ADDR       (uint8_t) 0x02


/**
 * @brief Magnetometer X-axis Magnetic Field Data MSB register.
 *        The value (MSB+LSB) is expressed as 16bit two’s complement
 * \code
 * Read
 * Default value:
 * endcode
 */
#define LSM_M_OUT_X_H_ADDR  0x03


/**
 * @brief Magnetometer X-axis Magnetic Field Data LSB register
 * The value (MSB+LSB) is expressed as 16bit two’s complement
 * \code
 * Read
 * Default value:
 * \endcode
 */
#define LSM_M_OUT_X_L_ADDR  0x04


/**
 * @brief Magnetometer Z-axis Magnetic Field Data MSB register
 * The value (MSB+LSB) is expressed as 16bit two’s complement
 * \code
 * Read
 * Default value:
 * \endcode
 */
#define LSM_M_OUT_Z_H_ADDR  0x05


/**
 * @brief Magnetometer Z-axis Magnetic Field Data LSB register
 * The value (MSB+LSB) is expressed as 16bit two’s complement
 * \code
 * Read
 * Default value:
 * \endcode
 */
#define LSM_M_OUT_Z_L_ADDR  0x06


/**
 * @brief Magnetometer Y-axis Magnetic Field Data MSB register
 * The value (MSB+LSB) is expressed as 16bit two’s complement
 * \code
 * Read
 * Default value:
 * \endcode
 */
#define LSM_M_OUT_Y_H_ADDR  0x07


/**
 * @brief Magnetometer Y-axis Magnetic Field Data LSB register
 * The value (MSB+LSB) is expressed as 16bit two’s complement
 * \code
 * Read
 * Default value:
 * \endcode
 */
#define LSM_M_OUT_Y_L_ADDR  0x08


/**
 * @brief Magnetometer Status Register
 * \code
 * Read Only
 * Default value: 0x00
 * 7:2 0:  They shall be zero for correct working of the device
 * 1 LOCK: Data output register lock.Once a new set of measurements is available, this bit is set when the first magnetic file data register has been read.
 * 0 RDY:  Data ready bit. This bit is set when a new set of measurements are available
 * \endcode
 */
#define LSM_M_SR_REG_ADDR  0x09


/**
 * \brief Magnetometer Identification Register A
 * \code
 * Read
 * Default value: 0x48
 * \endcode
 */
#define LSM_M_IRA_REG_ADDR  0x0A


/**
 * @brief Magnetometer Identification Register B
 * \code
 * Read
 * Default value: 0x34
 * \endcode
 */
#define LSM_M_IRB_REG_ADDR  0x0B


/**
 * @brief Magnetometer Identification Register C
 * \code
 * Read
 * Default value: 0x33
 * \endcode
 */
#define LSM_M_IRC_REG_ADDR  0x0C


/**
 * @brief Temperature Data MSB register
 * The value (MSB+LSB) is expressed as 12bit two’s complement with a sensitivity of 8LSB/deg
 * \code
 * Read
 * Default value:
 * 7:0 TEMP11-TEMP4: MSB byte of temperature data.
 * \endcode
 */
#define LSM_M_TEMP_H_REG_ADDR   0x31


/**
 * @brief Temperature Data LSB register
 * The value (MSB+LSB) is expressed as 12bit two’s complement with a sensitivity of 8LSB/deg
 * \code
 * Read
 * Default value:
 * 7:4 TEMP3-TEMP0: LSB byte of temperature data.
 * 3-0 -: Do not consider.
 * \endcode
 */
#define LSM_M_TEMP_L_REG_ADDR   0x32


/**
 * @}
 */


/** @defgroup Magnetometer_Exported_Functions             Magnetometer Exported Functions
 * @{
 */


void Lsm303dlhcMagConfig(LSMMagInit* pxLSMMagInitStruct);
void Lsm303dlhcMagGetInfo(LSMMagInit* pxLSMMagInitStruct);
void Lsm303dlhcMagSetFullScale(MagFullScale xFullScale);
MagFullScale Lsm303dlhcMagGetEnumFullScale(void);
void Lsm303dlhcMagGetSensitivity(float *pfSensitivityXYZ);
float Lsm303dlhcMagGetFullScale(void);
void Lsm303dlhcMagSetDataRate(MagOutputDataRate xDataRate);
MagOutputDataRate Lsm303dlhcMagGetEnumDataRate(void);
float Lsm303dlhcMagGetDataRate(void);
void Lsm303dlhcMagReadMag(float* pfData);
void Lsm303dlhcMagReadRawData(int16_t* pnRawData);
int16_t Lsm303dlhcMagReadRawDataTemp(void);
float Lsm303dlhcMagReadTemp(void);
LSMMDataStatus Lsm303dlhcMagGetDataStatus(void);
void Lsm303dlhcMagDataReadyIrqConfig(LSMFunctionalState xFunctionalState);


/**
 * @}
 */

/**
 * @}
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

#endif /* __LSM303DLHC_H */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/

