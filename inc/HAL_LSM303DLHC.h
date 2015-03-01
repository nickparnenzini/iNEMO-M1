/**
  * @file    HAL_LSM303DLHC.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    12 April 2012
  * @brief   Hardware Abstraction Layer for LSM303DLHC.
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


/* Define to prevent recursive inclusion*/
#ifndef __HAL_LSM303DLHC_H
#define __HAL_LSM303DLHC_H

/* Includes */
#include "stm32f10x.h"
#include "iNEMO_I2C_Driver.h"


#ifdef __cplusplus
 extern "C" {
#endif

/**
* @addtogroup iNemo_Sensor_Drivers           iNemo Sensor Drivers
* @{
*/   
   
/**
 * @addtogroup HAL_LSM303DLHC      HAL LSM303DLHC
 * @brief This is an adapter header to join the platform indipendent @ref Sensor_Libraries for the LSM303DLHC 6 axis sensor
 *        to the low level driver on the microcontroller side.
 * @{
 */

/**
 * @addtogroup  HAL_LSM303DLHC_Exported_Constants      HAL LSM303DLHC Exported Constants
 * @{
 */

   
#define LSM_I2C                  I2C2
   
#define LSM_I2C_Speed            400000



/**
 * @addtogroup HAL_LSM303DLHC_Interrupt_Pin_Define       HAL LSM303DLHC Interrupt Pin Define
 * @{
 */

#define LSM_A_INT1_Pin           GPIO_Pin_2
#define LSM_A_INT1_Port          GPIOD
#define LSM_A_INT1_RCC_Port      RCC_APB2Periph_GPIOD

#define LSM_A_INT2_Pin           GPIO_Pin_5
#define LSM_A_INT2_Port          GPIOB
#define LSM_A_INT2_RCC_Port      RCC_APB2Periph_GPIOB


/** 
 *@}
 */
   
/**
 *@}
 */

/**
 * @addtogroup  HAL_LSM303DLHC_Exported_Macros       HAL LSM303DLHC Exported Macros
 * @{
 */
          
#define Lsm303dlhcI2CInit()                                             iNemoI2C2Init(LSM_I2C_Speed)
#define Lsm303dlhcI2CBufferRead(cDevAddress,pVal,cAddress,nBytes)       iNemoI2C2BufferRead(cDevAddress,pVal,cAddress,nBytes)
#define Lsm303dlhcI2CBufferWrite(cDevAddress,pVal,cAddress,nBytes)      iNemoI2C2BufferWrite(cDevAddress,pVal,cAddress,nBytes)

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
