/**
  * @file    iNEMO_SPI_Driver.c
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    12 April 2011
  * @brief   This file provides a set of functions needed to manage the
  *          communication between STM32 SPI master and a generic SPI slave sensor on iNemo-M1.
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


/* Includes */
#include "iNEMO_SPI_Driver.h"
#include <string.h>

/**
* @addtogroup iNemo_Sensor_Drivers           iNemo Sensor Drivers
* @{
*/

/**
 * @defgroup iNemoSPIDriver
 * @brief SPI low level driver for STM32F10x.
 * @{
 */


/** @defgroup iNemoSPIDriver_Private_Functions        iNemoSPIDriver Private Functions
 * @{
 */

/**
 * @brief  Initializes the SPI peripheral used to drive the device.
 * @param  SPIx: The SPI peripheral to be configured.
 *         This parameter is a pointer to @ref SPI_TypeDef.
 * @retval None.
 */
void iNemoSPIInit(SPI_TypeDef* SPIx)
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable AFIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* Enable SPIx and GPIO clocks */
    if(SPIx == SPI2)
    {
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_13;
    }
    else
    {
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_5;
    }

    /* Configure SPI2 pins: SCK, MISO and MOSI */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

     if(SPIx == SPI2)
    {
      GPIO_Init(GPIOB, &GPIO_InitStructure);

      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    }
    else
    {
      GPIO_Init(GPIOA, &GPIO_InitStructure);
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    }

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

    if(SPIx == SPI2)
    {
      GPIO_Init(GPIOB, &GPIO_InitStructure);
    }
    else
    {
      GPIO_Init(GPIOA, &GPIO_InitStructure);
    }

    /* SPI configuration */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPIx, &SPI_InitStructure);

    /* Configure GPIO PIN for Chip select */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    if(SPIx == SPI2)
    {
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
      GPIO_Init(GPIOB, &GPIO_InitStructure);

      /* Deselect : Chip Select high */
      GPIO_SetBits(GPIOB, GPIO_Pin_12);
    }
    else
    {
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      /* Deselect : Chip Select high */
      GPIO_SetBits(GPIOA, GPIO_Pin_4);
    }

    /* Enable SPI */
    SPI_Cmd(SPIx, ENABLE);

}



/**
 * @brief  Sends/reads a byte to/from the SPI device.
 * @brief  SPIx: SPI peripherial to use.
 * @param  cByte: byte to send.
 * @retval uint8_t: byte read.
 */
uint8_t iNemoSPISendByte(SPI_TypeDef* SPIx, uint8_t cByte)
{

  SpiEnterCritical();
  
  /* Loop while DR register in not empty */
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);

  /* Send Half Word through the SPI1 peripheral */
  SPI_I2S_SendData(SPIx, cByte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);

  SpiExitCritical();
      
  /* Return the Half Word read from the SPI bus */
  return (uint8_t)SPI_I2S_ReceiveData(SPIx);


}


/**
 * @brief  Reads a block of data from the device.
 * @brief  SPIx: SPI peripherial to use.
 * @param  pcBuffer : pointer to the buffer that receives the data read.
 * @param  cReadAddr : register internal address to read from.
 * @param  nNumByteToRead : number of bytes to read.
 * @retval None
 */
void iNemoSPIBufferRead(SPI_TypeDef* SPIx, uint8_t* pcBuffer, uint8_t cReadAddr, uint8_t cNumByteToRead)
{
  /* reset the nW/R bit in order to make a read */
  cReadAddr |= 0xC0;

  /* mantain or not the multiple byte bit */
  if(cNumByteToRead<=1)
    cReadAddr &= 0xBF;

  /* Select : SPIx Chip Select low */
  SPI_CS_LOW(SPIx);

  iNemoSPISendByte(SPIx,cReadAddr);

  while(cNumByteToRead >=1)
  {
    (*pcBuffer)=iNemoSPISendByte(SPIx,0xFF);

    cNumByteToRead--;
    pcBuffer++;
  }

  /* Deselect : SPIx Chip Select high */
  SPI_CS_HIGH(SPIx);

}


/**
 * @brief  Writes a block of data to the device.
 * @param  pcBuffer : pointer to the buffer  containing the data to be written.
 * @param  cWriteAddr : register internal address to write to.
 * @param  cNumByteToWrite : number of bytes to write.
 * @retval None
 */
void iNemoSPIBufferWrite(SPI_TypeDef* SPIx, uint8_t* pcBuffer, uint8_t cWriteAddr, uint8_t cNumByteToWrite)
{
  /* reset the nW/R bit in order to make a write */
  cWriteAddr &= 0x3F;

  /* set or not the multiple byte bit */
  if(cNumByteToWrite>1)
    cWriteAddr |= 0x40;

  /* Select : SPIx Chip Select low */
  SPI_CS_LOW(SPIx);

  iNemoSPISendByte(SPIx,cWriteAddr);

  while(cNumByteToWrite >=1)
  {
    iNemoSPISendByte(SPIx,*pcBuffer);
    cNumByteToWrite--;
    pcBuffer++;
  }

  /* Deselect : SPIx Chip Select high */
  SPI_CS_HIGH(SPIx);

}

/**
 * @}
 */

/**
* @}
*/

/**
*@}
*/

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
