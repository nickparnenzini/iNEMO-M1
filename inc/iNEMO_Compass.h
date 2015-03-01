/**
  * @file    iNEMO_Compass.h
  * @author  ART Team IMS-Systems Lab
  * @version V1.2.0  [FW V2.3.0]
  * @date    02 September 2012
  * @brief   This file is the header of the iNEMO Compass algorithm. This is the interface from your application.
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


/* Define to prevent recursive inclusion  */
#ifndef __INEMO_COMPASS
#define __INEMO_COMPASS

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup iNemo_Compass					
  * @{
  */




/** @defgroup iNemo_Compass_Functions			        iNEMO Tilted Compass Functions
* @{
*/


void iNEMO_TiltedCompass(float* pfMagXYZ, float* pfAccXYZ, float* pfRPH);
void iNEMO_MagSensorCalibrationInit(void);
uint8_t iNEMO_MagSensorCalibrationRun(float* pfMagXYZ, float* pfGain, float* pfOffset);

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

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/


