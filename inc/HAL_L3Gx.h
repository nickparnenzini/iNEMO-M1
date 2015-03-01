/**
  * @file    HAL_L3GX.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    12 April 2012
  * @brief   Hardware Abstraction Layer for a L3Gx class gyro.
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
#ifndef __HAL_L3GX_H
#define __HAL_L3GX_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @addtogroup iNemo_Sensor_Drivers           iNemo Sensor Drivers
 * @{
 */

/**
 * @addtogroup HAL_L3GX        HAL L3GX
 * @brief This is an adapter header to join the platform indipendent @ref Sensor_Libraries for the L3Gx gyro
 *        to the low level driver on the microcontroller side.
 * @{
 */

/**
 * @addtogroup  HAL_L3GX_Exported_Constants      HAL L3Gx Exported Constants
 * @{
 */

//#define  USE_I2C



#define L3g4200dCommInit        L3gxCommInit
#define L3gd20CommInit          L3gxCommInit
   
/**
 *@}
 */


/**
 * @addtogroup  HAL_L3GX_Exported_Macros       	HAL L3Gx Exported Macros
 * @{
 */


#ifdef USE_I2C
#include "iNEMO_I2C_Driver.h"

   // put here the macro definition for i2c drivers

#else
#include "iNEMO_SPI_Driver.h"

#define L3G_SPI                 SPI2

#define L3gxCommInit()                             iNemoSPIInit(L3G_SPI)
#define L3gxBufferRead(pVal,cAddress,nBytes)       iNemoSPIBufferRead(L3G_SPI,pVal,cAddress,nBytes)
#define L3gxBufferWrite(pVal,cAddress,nBytes)      iNemoSPIBufferWrite(L3G_SPI,pVal,cAddress,nBytes)

#endif

/**
 *@}
 */

/**
 *@}
 */

/**
 *@}
 */

#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
