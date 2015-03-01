/**
  * @file    iNEMO_I2C_Driver.c
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    12 April 2011
  * @brief   This file provides a set of functions needed to manage the
  *          communication between STM32 I2C master and a generic I2C slave sensor on iNemo-M1.
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
#include "iNEMO_I2C_Driver.h"
#include <string.h>

/**
 * @addtogroup iNemo_Sensor_Drivers           iNemo Sensor Drivers
 * @brief A low level driver to talk with some ST MEMS sensors using a STM32F10x platform.
 * @{
 */ 

/**
 * @defgroup iNemoI2CDriver
 * @brief I2C low level driver for STM32F10x.
 * @{
 */

#define FORCE_CRITICAL_SEC

#define I2C1_DR_Address               0x40005410
#define I2C2_DR_Address               0x40005810

#define I2C1_DMA_CHANNEL_TX           DMA1_Channel6
#define I2C1_DMA_CHANNEL_RX           DMA1_Channel7
#define I2C2_DMA_CHANNEL_TX           DMA1_Channel4
#define I2C2_DMA_CHANNEL_RX           DMA1_Channel5

#define I2C_DIRECTION_TX 0
#define I2C_DIRECTION_RX 1

#define DMA_BUFFER_SIZE       196


DMA_InitTypeDef  I2CDMA_InitStructure;
__IO uint32_t I2CDirection = I2C_DIRECTION_TX;

/** @defgroup iNemoI2CDriver_I2C_Private_Functions          iNemoI2CDriver I2C Private Functions
 * @{
 */

/**
 * @brief  Initializes the I2C peripheral used to drive the device.
 * @param  I2Cx: The I2C peripheral to be configured.
 *         This parameter is a pointer to @ref I2C_TypeDef.
 * @param  I2CxSpeed: I2C speed.
 *         This parameter is an uint32_t.
 * @retval None.
 */
void iNemoI2CInit(I2C_TypeDef* I2Cx, uint32_t I2CxSpeed)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clocks */ 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
  
  /* Configure I2C pins: SCL and SDA */
  if(I2Cx==I2C2)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
  }
  else
  {
    GPIO_PinRemapConfig(GPIO_Remap_I2C1,ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
  }
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2CxSpeed;
  
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2Cx, &I2C_InitStructure);
  
  /* I2C Peripheral Enable */
  I2C_Cmd(I2Cx, ENABLE);
   
  /* Enable DMA if required */
#if (defined(I2C1_USE_DMA_TX) || defined(I2C1_USE_DMA_RX))
 if (I2Cx==I2C1)
    iNemoI2CDMAInit(I2C1);
#endif
 
#if (defined(I2C2_USE_DMA_TX) || defined(I2C2_USE_DMA_RX))
 if (I2Cx==I2C2)
    iNemoI2CDMAInit(I2C2);
#endif 
       
      
}



/**
 * @brief  Initializes the DMA for the used I2C peripheral.
 * @param  I2Cx: The I2C peripheral to configure with DMA.
 * @retval None.
 */
void iNemoI2CDMAInit(I2C_TypeDef* I2Cx)
{
  /* Enable the DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
  /* I2C TX DMA Channel configuration */    
  I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;   /* This parameter will be configured durig communication */
  I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    /* This parameter will be configured durig communication */
  I2CDMA_InitStructure.DMA_BufferSize = 0xFFFF;            /* This parameter will be configured durig communication */
  I2CDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  I2CDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  I2CDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
  I2CDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  I2CDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  I2CDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  I2CDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  if(I2Cx==I2C2)
  {
    I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C2_DR_Address;
    
#ifdef I2C2_USE_DMA_TX
      DMA_DeInit(I2C2_DMA_CHANNEL_TX);
      DMA_Init(I2C2_DMA_CHANNEL_TX, &I2CDMA_InitStructure);
#endif
    
#ifdef I2C2_USE_DMA_RX
      /* I2C2 RX DMA Channel configuration */
      DMA_DeInit(I2C2_DMA_CHANNEL_RX);
      DMA_Init(I2C2_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
#endif
  }

  if(I2Cx==I2C1)
  {
    I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
    
#ifdef I2C1_USE_DMA_TX
      DMA_DeInit(I2C1_DMA_CHANNEL_TX);
      DMA_Init(I2C1_DMA_CHANNEL_TX, &I2CDMA_InitStructure);
#endif
    
#ifdef I2C1_USE_DMA_RX
      /* I2C1 RX DMA Channel configuration */
      DMA_DeInit(I2C1_DMA_CHANNEL_RX);
      DMA_Init(I2C1_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
#endif

  }


}


/**
 * @brief  DMA configuration.
 * @param  I2Cx: The I2C peripheral to configure with DMA.
 *         This parameter is a pointer to @ref I2C_TypeDef.
 * @param  pBuffer: DMA memory buffer.
 *         This parameter is a pointer to uint8_t.
 * @param  lBufferSize: size of the pBuffer.
 *         This parameter is an uint32_t.
 * @param  lDirection: Tx or Rx direction.
 *         This parameter can be I2C_DIRECTION_TX or I2C_DIRECTION_RX.
 * @retval None.
 */
void iNemoI2CDMAConfig(I2C_TypeDef* I2Cx, uint8_t* pBuffer, uint32_t lBufferSize, uint32_t lDirection)
{
  /* Initialize the DMA with the new parameters */
  if (lDirection == I2C_DIRECTION_TX)
  {
    /* Configure the DMA Tx Channel with the buffer address and the buffer size */
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    I2CDMA_InitStructure.DMA_BufferSize = (uint32_t)lBufferSize;
    if(I2Cx==I2C2)
    {
      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C2_DR_Address;
      DMA_Cmd(I2C2_DMA_CHANNEL_TX, DISABLE);
      DMA_Init(I2C2_DMA_CHANNEL_TX, &I2CDMA_InitStructure);
      DMA_Cmd(I2C2_DMA_CHANNEL_TX, ENABLE);
    }
    else
    {
      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
      DMA_Cmd(I2C1_DMA_CHANNEL_TX, DISABLE);
      DMA_Init(I2C1_DMA_CHANNEL_TX, &I2CDMA_InitStructure);
      DMA_Cmd(I2C1_DMA_CHANNEL_TX, ENABLE);
    }
  }
  else /* Reception */
  {
    /* Configure the DMA Rx Channel with the buffer address and the buffer size */
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    I2CDMA_InitStructure.DMA_BufferSize = (uint32_t)lBufferSize;
    
    if(I2Cx==I2C2)
    {
      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C2_DR_Address;
      DMA_Cmd(I2C2_DMA_CHANNEL_RX, DISABLE);
      DMA_Init(I2C2_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
      DMA_Cmd(I2C2_DMA_CHANNEL_RX, ENABLE);
    }
    else
    {
      I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
      DMA_Cmd(I2C1_DMA_CHANNEL_RX, DISABLE);
      DMA_Init(I2C1_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
      DMA_Cmd(I2C1_DMA_CHANNEL_RX, ENABLE);
    }
  }
}




/**
 * @brief  Reads a block of data from the device by DMA.
 * @brief  I2Cx: I2C peripherial to use.
 * @param  cAddr: slave address.
 * @param  pcBuffer: pointer to the buffer that receives the data read.
 * @param  cReadAddr: register internal address to read from.
 * @param  nNumByteToRead: number of bytes to read.
 * @retval None
 */
void iNemoI2CBufferReadDma(I2C_TypeDef* I2Cx, uint8_t cAddr, uint8_t* pcBuffer, uint8_t cReadAddr, uint8_t cNumByteToRead)
{
  
    __IO uint32_t temp = 0;
    __IO uint32_t Timeout = 0;
    
    /* Enable I2C errors interrupts */
    I2Cx->CR2 |= I2C_IT_ERR;
    
    /* Set the MSb of the register address in case of multiple readings */
    if(cNumByteToRead>1)
      cReadAddr |= 0x80;
    
#ifdef FORCE_CRITICAL_SEC
    __disable_irq();
#endif    
    
    /* While the bus is busy */
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    
    /* Send START condition */
    I2C_GenerateSTART(I2Cx, ENABLE);
    
    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    
    /* Send LSM303DLH address for read */
    I2C_Send7bitAddress(I2Cx, cAddr, I2C_Direction_Transmitter);
    
    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(I2Cx, ENABLE);
    
    /* Send the LSM303DLH_Magn's internal address to write to */
    I2C_SendData(I2Cx, cReadAddr);
    
    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    /* Configure I2Cx DMA channel */
    iNemoI2CDMAConfig(I2Cx, pcBuffer, cNumByteToRead, I2C_DIRECTION_RX);
    
    /* Set Last bit to have a NACK on the last received byte */
    I2Cx->CR2 |= 0x1000;
    
    /* Enable I2C DMA requests */
    I2C_DMACmd(I2Cx, ENABLE);
    Timeout = 0xFFFF;
    
    /* Send START condition */
    I2C_GenerateSTART(I2Cx, ENABLE);
    
    /* Wait until SB flag is set: EV5  */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
    {
      if (Timeout-- == 0)
        return;
    }
    Timeout = 0xFFFF;
    
    /* Send LSM303DLH address for read */
    I2C_Send7bitAddress(I2Cx, cAddr, I2C_Direction_Receiver);
    
    /* Wait until ADDR is set: EV6 */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
      if (Timeout-- == 0)
        return;
    }
    /* Clear ADDR flag by reading SR2 register */
    temp = I2Cx->SR2;
    
    
    
    if(I2Cx == I2C2)
    {
      /* Wait until DMA end of transfer */
      while (!DMA_GetFlagStatus(DMA1_FLAG_TC5));
      /* Disable DMA Channel */
      DMA_Cmd(I2C2_DMA_CHANNEL_RX, DISABLE);
      
      /* Clear the DMA Transfer Complete flag */
      DMA_ClearFlag(DMA1_FLAG_TC5);
    }
    else
    {
      /* Wait until DMA end of transfer */
      while (!DMA_GetFlagStatus(DMA1_FLAG_TC7));
      /* Disable DMA Channel */
      DMA_Cmd(I2C1_DMA_CHANNEL_RX, DISABLE);
      
      /* Clear the DMA Transfer Complete flag */
      DMA_ClearFlag(DMA1_FLAG_TC7);
    }
    
       
    /* Disable Ack for the last byte */
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    
    /* Send STOP Condition */
    I2C_GenerateSTOP(I2Cx, ENABLE);
    
    /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
    while ((I2Cx->CR1 & 0x0200) == 0x0200);
    
    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    
#ifdef FORCE_CRITICAL_SEC
    __enable_irq();
#endif
    
}


/**
 * @brief  Reads a block of data from the device by polling.
 * @brief  I2Cx: I2C peripherial to use.
 * @param  cAddr: slave address.
 * @param  pcBuffer: pointer to the buffer that receives the data read.
 * @param  cReadAddr: register internal address to read from.
 * @param  nNumByteToRead: number of bytes to read.
 * @retval None
 */
void iNemoI2CBufferReadPolling(I2C_TypeDef* I2Cx, uint8_t cAddr, uint8_t* pcBuffer, uint8_t cReadAddr, uint8_t cNumByteToRead)
{
    /* Set the MSb of the register address in case of multiple readings */
    if(cNumByteToRead>1)
      cReadAddr |= 0x80;
    
#ifdef FORCE_CRITICAL_SEC
    __disable_irq();
#endif
    
    /* While the bus is busy */
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    
    /* Send START condition */
    I2C_GenerateSTART(I2Cx, ENABLE);
    
    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    
    /* Send LSM303DLH address for write */
    I2C_Send7bitAddress(I2Cx, cAddr, I2C_Direction_Transmitter);
    
    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(I2Cx, ENABLE);
    
    /* Send the LSM303DLH_Magn's internal address to write to */
    I2C_SendData(I2Cx, cReadAddr);
    
    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    /* Send START condition a second time */
    I2C_GenerateSTART(I2Cx, ENABLE);
    
    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    
    /* Send LSM303DLH address for read */
    I2C_Send7bitAddress(I2Cx, cAddr, I2C_Direction_Receiver);
    
    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    
    /* While there is data to be read */
    while(cNumByteToRead)
    {
      if(cNumByteToRead == 1)
      {
        /* Disable Acknowledgement */
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        
        /* Send STOP Condition */
        I2C_GenerateSTOP(I2Cx, ENABLE);
      }
      
      /* Test on EV7 and clear it */
      if(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
      {
        /* Read a byte from the LSM303DLH */
        *pcBuffer = I2C_ReceiveData(I2Cx);
        
        /* Point to the next location where the byte read will be saved */
        pcBuffer++;
        
        /* Decrement the read bytes counter */
        cNumByteToRead--;
      }
    }
    
    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    
#ifdef FORCE_CRITICAL_SEC
    __enable_irq();
#endif
  
}
      

/**
 * @brief  Writes a block of data to the device by DMA.
 * @brief  I2Cx: I2C peripherial to use.
 * @param  cAddr : slave address.
 * @param  pcBuffer : pointer to the buffer  containing the data to be written.
 * @param  cWriteAddr : register internal address to write to.
 * @param  nNumByteToWrite : number of bytes to write.
 * @retval None
 */
void iNemoI2CBufferWriteDma(I2C_TypeDef* I2Cx, uint8_t cAddr, uint8_t* pcBuffer, uint8_t cWriteAddr, uint8_t cNumByteToWrite)
{
    
  __IO uint32_t temp = 0;
  __IO uint32_t Timeout = 0;
  
  static uint8_t pcDmaBuffer[DMA_BUFFER_SIZE+1];
  
  /* Set to 1 the MSb of the register address in case of multiple byte writing */
  if(cNumByteToWrite>1)
    cWriteAddr |= 0x80;
  
  pcDmaBuffer[0]=cWriteAddr;
  memcpy(&pcDmaBuffer[1],pcBuffer,cNumByteToWrite);
  
  /* Enable Error IT  */
  I2Cx->CR2 |= I2C_IT_ERR;
  
  Timeout = 0xFFFF;
  /* Configure the DMA channel for I2Cx transmission */
  iNemoI2CDMAConfig(I2Cx, pcDmaBuffer, cNumByteToWrite+1, I2C_DIRECTION_TX);
  
  /* Enable DMA for I2C */
  I2C_DMACmd(I2Cx, ENABLE);
   
  /* Send START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);
  
  
  /* Wait until SB flag is set: EV5 */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if (Timeout-- == 0)
      return;
  }
  
  Timeout = 0xFFFF;
  
  /* Send LSM303DLH address for write */
  I2C_Send7bitAddress(I2Cx, cAddr, I2C_Direction_Transmitter);
  
  /* Wait until ADDR is set: EV6 */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if (Timeout-- == 0)
      return;
  }
  
  /* Clear ADDR flag by reading SR2 register */
  temp = I2Cx->SR2;
  
  
  /* Disable the DMA1 channel */
  if(I2Cx == I2C2)
  {
    /* Wait until DMA end of transfer */
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC4));
    /* Disable DMA Channel */
    DMA_Cmd(I2C2_DMA_CHANNEL_TX, DISABLE);
    
    /* Clear the DMA Transfer complete flag */
    DMA_ClearFlag(DMA1_FLAG_TC4);
  }
  else
  {
    /* Wait until DMA end of transfer */
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC6));
    /* Disable DMA Channel */
    DMA_Cmd(I2C1_DMA_CHANNEL_TX, DISABLE);
    
    /* Clear the DMA Transfer complete flag */
    DMA_ClearFlag(DMA1_FLAG_TC6);
  }
 
  
  /* EV8_2: Wait until BTF is set before programming the STOP */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
   
  /* Send STOP Condition */
  I2C_GenerateSTOP(I2Cx, ENABLE);
  
  /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
  while ((I2Cx->CR1 & 0x0200) == 0x0200);
  
}


/**
 * @brief  Writes a block of data to the device by polling.
 * @brief  I2Cx: I2C peripherial to use.
 * @param  cAddr : slave address.
 * @param  pcBuffer : pointer to the buffer  containing the data to be written.
 * @param  cWriteAddr : register internal address to write to.
 * @param  nNumByteToWrite : number of bytes to write.
 * @retval None
 */
void iNemoI2CBufferWritePolling(I2C_TypeDef* I2Cx, uint8_t cAddr, uint8_t* pcBuffer, uint8_t cWriteAddr, uint8_t cNumByteToWrite)
{   
    /* Set to 1 the MSb of the register address in case of multiple byte writing */
    if(cNumByteToWrite>1)
      cWriteAddr |= 0x80;
    
#ifdef FORCE_CRITICAL_SEC
    __disable_irq();
#endif
    
    /* While the bus is busy */
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    
    /* Send START condition */
    I2C_GenerateSTART(I2Cx, ENABLE);
    
    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    
    /* Send LSM303DLH address for write */
    I2C_Send7bitAddress(I2Cx, cAddr, I2C_Direction_Transmitter);
    
    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    /* Send the LSM303DLHC_internal register address to write */
    I2C_SendData(I2Cx, cWriteAddr);
    
    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    for(uint8_t i=0 ; i<cNumByteToWrite ; i++)
    {
      /* Send the byte to be written */
      I2C_SendData(I2Cx, pcBuffer[i]);
      
      /* Test on EV8 and clear it */
      while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }
    
    /* Send STOP condition */
    I2C_GenerateSTOP(I2Cx, ENABLE);
    
#ifdef FORCE_CRITICAL_SEC
    __enable_irq();
#endif
  
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
