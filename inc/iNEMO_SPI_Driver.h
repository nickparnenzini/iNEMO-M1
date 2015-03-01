/**
 * @file    iNEMO_SPI_Driver.h
 * @author  ART Team IMS-Systems Lab
 * @version V2.3.0
 * @date    12 April 2011
 * @brief   Header for iNEMO_SPI_Driver.c file
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
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 */



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __iNemoSPIDriver_H
#define __iNemoSPIDriver_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_spi.h"

#ifdef __cplusplus
 extern "C" {
#endif



/**
* @addtogroup iNemo_Sensor_Drivers           iNemo Sensor Drivers
* @{
*/

/**
 * @addtogroup iNemoSPIDriver
 * @{
 */
#define MAKE_SPI_CRITICAL_SEC
   
#define SPI1_USE_DMA_TX               DISABLE
#define SPI1_USE_DMA_RX               DISABLE

#define SPI2_USE_DMA_TX               DISABLE
#define SPI2_USE_DMA_RX               DISABLE

/**
 * @defgroup iNemoSPIDriver_Exported_Types        iNemoSPIDriver Exported Types
 * @{
 */

/**
 * @}
 */


/** @defgroup iNemoSPIDriver_Exported_Macros     iNemoSPIDriver Exported Macros
 * @{
 */
#define SPI_CS_LOW(SPIx)         ((SPIx==SPI2) ? GPIO_ResetBits(GPIOB, GPIO_Pin_12) : GPIO_ResetBits(GPIOA, GPIO_Pin_4))
#define SPI_CS_HIGH(SPIx)        ((SPIx==SPI2) ? GPIO_SetBits(GPIOB, GPIO_Pin_12) : GPIO_SetBits(GPIOA, GPIO_Pin_4))

#ifdef MAKE_SPI_CRITICAL_SEC
  #ifdef USE_FREERTOS
    #include "FreeRTOS.h"
    #include "task.h"
    #define SpiEnterCritical()      taskENTER_CRITICAL();
    #define SpiExitCritical()       taskEXIT_CRITICAL();
  #else
    #define SpiEnterCritical()      __disable_irq();
    #define SpiExitCritical()       __enable_irq();
  #endif
#else
  #define SpiEnterCritical()
  #define SpiExitCritical()
#endif
/**
 * @}
 */

/** @defgroup iNemoSPIDriver_Exported_Functions     iNemoSPIDriver Exported Functions
 * @{
 */

void iNemoSPIInit(SPI_TypeDef* SPIx);
void iNemoSPIBufferRead(SPI_TypeDef* SPIx, uint8_t* pcBuffer, uint8_t cReadAddr, uint8_t cNumByteToRead);
void iNemoSPIBufferWrite(SPI_TypeDef* SPIx, uint8_t* pcBuffer, uint8_t cWriteAddr, uint8_t cNumByteToWrite);
uint8_t iNemoSPISendByte(SPI_TypeDef* SPIx, uint8_t cByte);


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

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

