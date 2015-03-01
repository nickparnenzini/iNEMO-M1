/**
  * @file    L3GX.c
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    12 April 2012
  * @brief   This file provides a set of functions needed to manage the L3Gx slave.
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


#include "L3Gx.h"


/**
 * @addtogroup Sensor_Libraries           Sensor Libraries
 * @{
 */


/**
* @defgroup L3Gx
* @{
*/


/**
 * @defgroup L3GX_Private_TypesDefinitions      L3Gx Private TypesDefinitions
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup L3GX_Private_Defines               L3Gx Private Defines
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup L3GX_Private_Macros               L3Gx Private Macros
 * @{
 */

#define L3G_ABS(a)              (a>0?(a):-(a))

/**
 *@}
 */


/**
 * @defgroup L3GX_Private_FunctionPrototypes    L3Gx Private FunctionPrototypes
 * @{
 */

/**
 *@}
 */


/**
* @defgroup L3GX_Private_Functions            L3Gx Private Functions
* @{
*/


/**
* @brief Set configuration of angular rate of L3GX pointer to a L3GInit structure that contains
*                  the configuration setting for the L3GX.
* @param pxL3GInitStruct : pointer to a L3GInit structure that contains the configuration setting.
*         This parameter is a pointer to @ref L3GInit .
* @retval None.
* @details
* <b>Example:</b>
* @code
*  L3GInit L3GInitStructure;
*
*  L3GInitStructure.xPowerMode = L3G_NORMAL_SLEEP_MODE;
*  L3GInitStructure.xOutputDataRate = L3G_ODR_100_HZ_CUTOFF_12_5;
*  L3GInitStructure.xEnabledAxes = L3G_ALL_AXES_EN;
*  L3GInitStructure.xFullScale = L3G_FS_500_DPS;
*  L3GInitStructure.xDataUpdate = L3G_CONTINUOS_UPDATE;
*  L3GInitStructure.xEndianness = L3G_LITTLE_ENDIAN;
*
*  L3gxConfig(&L3GInitStructure);
* @endcode
*/
void L3gxConfig(L3GInit *pxL3GInitStruct)
{
  uint8_t CTRL1 = 0x00, CTRL4 = 0x00;

  CTRL1 |= ((uint8_t)pxL3GInitStruct->xPowerMode | (uint8_t)pxL3GInitStruct->xOutputDataRate | (uint8_t)pxL3GInitStruct->xEnabledAxes);
  CTRL4 |= ((uint8_t)pxL3GInitStruct->xFullScale | (uint8_t)pxL3GInitStruct->xDataUpdate | (uint8_t)pxL3GInitStruct->xEndianness);

  L3gxByteWrite(&CTRL1,L3G_CTRL_REG1);
  L3gxByteWrite(&CTRL4,L3G_CTRL_REG4);

}


/**
* @brief  Gets the general configuration of L3GX for the magnetometer.
* @param  pxL3GInitStruct : pointer to a L3GInit structure that will
*         contain the configuration setting read from the L3GX registers.
* @retval None
*/
void L3gxGetInfo(L3GInit *pxL3GInitStruct)
{
  uint8_t CTRL1, CTRL4;

  /* Read the registers content */
  L3gxByteRead(&CTRL4, L3G_CTRL_REG4);
  L3gxByteRead(&CTRL1, L3G_CTRL_REG1);

  /* Fill the structure fields from CTRL1 reg info */
  pxL3GInitStruct->xPowerMode = (GyroPowerMode)(CTRL1 & 0x08);
  pxL3GInitStruct->xOutputDataRate = (GyroOutputDataRate)(CTRL1 & 0xF0);
  pxL3GInitStruct->xEnabledAxes = (GyroAxesEnabling)(CTRL1 & 0x07);

  /* Fill the structure fields from CTRL4 reg info */
  pxL3GInitStruct->xFullScale = (GyroFullScale)(CTRL4 & 0x30);
  pxL3GInitStruct->xDataUpdate = (GyroBlockDataUpdate)(CTRL4 & 0x80);
  pxL3GInitStruct->xEndianness = (GyroEndianness)(CTRL4 & 0x40);

}


/**
* @brief  Set configuration of angular rate of L3GX pointer to a L3GFilterInit structure
*             that contains the filter configuration setting for the L3GX.
* @param  pxL3GFilterInitStruct: pointer to a L3GInit structure that contains the filter configuration setting.
*           This parameter is a pointer to @ref L3GFilterInit .
* @retval None.
* @details
* <b>Example:</b>
* @code
*  L3GFilterInit FilterInitStr;
*
*  FilterInitStr.xHPF = L3G_NORMAL_SLEEP_MODE;
*  FilterInitStr.cHPFCutOff = 5;
*  FilterInitStr.xHPF_Mode = L3G_HPFM_NORMAL;
*  FilterInitStr.cHPFReference = 0;
*
*  L3gxFilterConfig(&FilterInitStr);
* @endcode
*/
void L3gxFilterConfig(L3GFilterInit *pxL3GFilterInitStruct)
{
  uint8_t CTRL2 = 0x00, CTRL5 = 0x00;

  L3gxByteRead(&CTRL5,L3G_CTRL_REG5);
  if(pxL3GFilterInitStruct->xHPF == L3G_ENABLE)
  {
    CTRL5 |= 0x10;
  }
  else
  {
    CTRL5 &= 0xCF;
  }

  CTRL2 |= (uint8_t)(pxL3GFilterInitStruct->xHPF_Mode | pxL3GFilterInitStruct->cHPFCutOff);

  L3gxByteWrite(&CTRL5,L3G_CTRL_REG5);
  L3gxByteWrite(&CTRL2,L3G_CTRL_REG2);
  L3gxByteWrite(&pxL3GFilterInitStruct->cHPFReference,L3G_REFERENCE);
}

/**
* @brief  Get configuration of Internal High Pass Filter of  L3GX for the linear acceleration
* @param  L3GFilterInit : pointer to a L3GFilterInit structure that will
*         contain the configuration setting read from the L3GX registers.
* @retval None
*/
void L3gxFilterGetInfo(L3GFilterInit* pxL3GFilterInitStruct)
{
  uint8_t CTRL2 = 0x00, CTRL5 = 0x00;

  /* Read the sensor registers */
  L3gxByteRead(&CTRL5,L3G_CTRL_REG5);
  L3gxByteRead(&CTRL2,L3G_CTRL_REG2);

  /* check if enabled */
  if(CTRL5 & 0x10)
    pxL3GFilterInitStruct->xHPF=L3G_ENABLE;
  else
    pxL3GFilterInitStruct->xHPF=L3G_DISABLE;

  /* Get mode and cutoff */
  pxL3GFilterInitStruct->xHPF_Mode = (GyroHPFMode)(CTRL2 & 0x30);
  pxL3GFilterInitStruct->cHPFCutOff = (CTRL2 & 0x0F);

  /* Get reference */
  L3gxByteRead(&pxL3GFilterInitStruct->cHPFReference,L3G_REFERENCE);
}


/**
* @brief Set the L3GX Power Mode.
* @param xPowerMode: Power Mode selection.
*         This parameter is a value of @ref GyroPowerMode .
* @retval None.
*/
void L3gxLowpower(GyroPowerMode xPowerMode)
{
  uint8_t tmpreg;
  L3gxByteRead(&tmpreg,L3G_CTRL_REG1);
  tmpreg &= 0xF7;

  tmpreg |= (uint8_t) xPowerMode;
  L3gxByteWrite(&tmpreg,L3G_CTRL_REG1);
}


/**
* @brief Set the L3GX Data Rate.
* @param xDataRate: Datarate value.
*         This parameter can be any value of @ref GyroOutputDataRate .
* @retval None.
*/
void L3gxSetDataRate(GyroOutputDataRate xDataRate)
{
  uint8_t tmpreg;

  /* Read the register value */
  L3gxByteRead(&tmpreg,L3G_CTRL_REG1);

  /* mask and set it */
  tmpreg &= 0x0F;
  tmpreg |= (uint8_t)xDataRate;

  /* write value on the register */
  L3gxByteWrite(&tmpreg,L3G_CTRL_REG1);

}


/**
* @brief  Returns the output data rate as a GyroOutputDataRate enumerative value.
* @param  None.
* @retval GyroOutputDataRate: GyroOutputDataRate enumerative value.
*/
GyroOutputDataRate L3gxGetEnumDataRate(void)
{
  uint8_t tmpReg;

  /* Read the register value */
  L3gxByteRead(&tmpReg,L3G_CTRL_REG1);

  /* return the masked value */
  return ((GyroOutputDataRate)(tmpReg & 0xF0));

}


/**
* @brief Set the L3GX Full Scale register
* @param  xFullScale: new full scale value.
*         This parameter can be any value of @ref GyroFullScale .
* @retval None.
*/
void L3gxSetFullScale(GyroFullScale xFullScale)
{
  uint8_t tmpreg;

  /* Read the register value */
  L3gxByteRead(&tmpreg,L3G_CTRL_REG4);

  /* mask and set it */
  tmpreg &= 0xCF;
  tmpreg |= (uint8_t)xFullScale;

  /* write value on the register */
  L3gxByteWrite(&tmpreg,L3G_CTRL_REG4);
}



/**
* @brief  Returns the full scale as a GyroFullScale enumerative value.
* @param  None.
* @retval GyroFullScale: GyroFullScale enumerative value.
*/
GyroFullScale L3gxGetEnumFullScale(void)
{
  uint8_t tmpReg;

  /* Read the register value */
  L3gxByteRead(&tmpReg,L3G_CTRL_REG4);

  /* return the masked value */
  return ((GyroFullScale)(tmpReg & 0x30));
}



/**
* @brief  Returns the Full Scale of Gyro expressed in LSB/dps.
* @param  pfSensitivityXYZ: pointer to 3 elements array in which the sensitivity values have to be stored.
*         This parameter is a pointer to a float array.
* @retval None.
*/
void L3gxGetSensitivity(float *pfSensitivityXYZ)
{
  uint8_t tmpReg;

  /* Read the register content */
  L3gxByteRead(&tmpReg,L3G_CTRL_REG4);
  tmpReg &= 0x30;

  /* return the correspondent value */
  switch(tmpReg)
  {
  case L3G_FS_250_DPS:
    pfSensitivityXYZ[0]=L3G_Sensitivity_250dps;
    pfSensitivityXYZ[1]=L3G_Sensitivity_250dps;
    pfSensitivityXYZ[2]=L3G_Sensitivity_250dps;
    
    break;
  case L3G_FS_500_DPS:
    pfSensitivityXYZ[0]=L3G_Sensitivity_500dps;
    pfSensitivityXYZ[1]=L3G_Sensitivity_500dps;
    pfSensitivityXYZ[2]=L3G_Sensitivity_500dps;
    
    break;
  case L3G_FS_2000_DPS:
    pfSensitivityXYZ[0]=L3G_Sensitivity_2000dps;
    pfSensitivityXYZ[1]=L3G_Sensitivity_2000dps;
    pfSensitivityXYZ[2]=L3G_Sensitivity_2000dps;
    
    break;
  }
  
   
}

/**
* @brief  Reboots the L3GX.
* @param  None.
* @retval None.
*/
void L3gxReboot(void)
{
  uint8_t tmpreg, value;
  L3gxByteRead(&tmpreg,L3G_CTRL_REG5);

  /* Take memory of the register content */
  tmpreg &= 0x7F;

  /* Put the BOOT field to 1 and make the reboot */
  value = 0x80;
  L3gxByteWrite(&value,L3G_CTRL_REG5);
  /* Rewrite the old content of the register */
  L3gxByteWrite(&tmpreg,L3G_CTRL_REG5);
}



/**
* @brief Read L3GX output register, and calculate the raw angular
*                  rate [LSB] AngRate= (out_h*256+out_l)/16 (12 bit rappresentation)
* @param pnRawData: pointer to the uint16_t array where the read data must be stored.
* @retval None.
*/
void L3gxReadRawData(int16_t* pnRawData)
{
  uint8_t buffer[6];
  uint8_t reg;
  L3gxByteRead(&reg,L3G_CTRL_REG4);
  L3gxBufferRead(buffer,L3G_OUT_X_L, 6);

  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(reg & 0x40))
    {
      for(int i=0; i<3; i++)
        pnRawData[i]=(int16_t)(((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]);
    }
  else
    {
      for(int i=0; i<3; i++)
        pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1]);
    }
}


/**
* @brief Read L3GX output register, and calculate the angular rate expressed in dps.
* @param pfData: pointer to the uint16_t array where the read data must be stored.
* @retval None.
*/
void L3gxReadAngRate(float* pfData)
{
  int16_t buffer[3];
  uint8_t reg;
  float fSensitivity;

  /* read the register values */
  L3gxReadRawData(buffer);
  L3gxByteRead(&reg,L3G_CTRL_REG4);

  /* switch the sensitivity value set in the CRTL4 */
  switch(reg & 0x30)
  {
  case 0x00:
    fSensitivity=L3G_Sensitivity_250dps;
    break;

  case 0x10:
    fSensitivity=L3G_Sensitivity_500dps;
    break;

  case 0x30:
    fSensitivity=L3G_Sensitivity_2000dps;
    break;
  }

  /* divide by sensitivity */
  for(uint8_t i=0; i<3; i++)
  {
    pfData[i]=(float)buffer[i]/fSensitivity;
  }

}


/**
* @brief Read L3GX output register when FIFO mode is active.
* @param pfData: pointer to float buffer where to store data.
*        This parameter is a float array pointer.
* @param cDataToRead: number of samples to read. Each sample is made up of the three axis readings.
*        This parameter is an uint8_t .
* @retval None
*/
void L3gxReadFifo(float* pfData, uint8_t cDataToRead)
{
  uint8_t *pcBuffer=(uint8_t*)pfData;
  uint8_t j=0,reg;
  float fSensitivity;
  uint16_t aux;

  L3gxByteRead(&reg,L3G_CTRL_REG4);

  /* switch the sensitivity value set in the CRTL4 */
  switch(reg & 0x30)
  {
  case 0x00:
    fSensitivity=L3G_Sensitivity_250dps;
    break;

  case 0x10:
    fSensitivity=L3G_Sensitivity_500dps;
    break;

  case 0x30:
    fSensitivity=L3G_Sensitivity_2000dps;
    break;
  }

  /* Read the register content */
  L3gxBufferRead(pcBuffer, L3G_OUT_X_L, cDataToRead*6);

  /* convert all data to float */
  for(uint16_t i=0 ; i<cDataToRead*3 ; i++)
  {
    aux=(((uint16_t)pcBuffer[cDataToRead*6-j-1]<<8)+(uint16_t)pcBuffer[cDataToRead*6-j-2]);
    pfData[cDataToRead*3-i-1] = (float)(*(int16_t*)&aux)/fSensitivity;
    j+=2;
  }

}


/**
* @brief  Gets status for gyro data.
* @param  None.
* @retval L3GDataStatus: Data status in a L3GDataStatus bitfields structure.
*/
L3GDataStatus L3gxGetDataStatus(void)
{
  L3GDataStatus xStatus;

  /* Read the register content */
  L3gxByteRead((uint8_t*)&xStatus, L3G_STATUS_REG);

  return xStatus;

}


/**
* @brief Enable or disable the IRQ1 line.
* @param xNewState: Enable or disable I1.
*        This parameter can be L3G_ENABLE or L3G_DISABLE.
* @retval None
*/
void L3gxIrq1Config(L3GFunctionalState xNewState)
{
  uint8_t tempReg;

  /* Read the register content */
  L3gxByteRead(&tempReg, L3G_CTRL_REG3);

  /* Enable or disable I1 */
  if(xNewState)
    tempReg |= 0x80;
  else
    tempReg &= 0x7F;

  /* Write byte on register */
  L3gxByteWrite(&tempReg, L3G_CTRL_REG3);

}


/**
* @brief Sets the IRQ2 line to be raised on a specific event.
* @param xIrq2Config: Interrupt mask to enable.
*        This parameter is a @ref L3GIrq2List
* @param xNewState: Enable or disable I2.
*        This parameter can be L3G_ENABLE or L3G_DISABLE.
* @retval None.
* @details
* <b>Example:</b>
* @code
*  ...
*  ExtiConfiguration();                       // set the micro exti before init
*  L3gxIrq2Config(L3G_I2_DRDY,L3G_ENABLE);  // for example enable the data_ready IRQ
*  ...
* @endcode
*/
void L3gxIrq2Config(L3GIrq2List xIrq2Config, L3GFunctionalState xNewState)
{
  uint8_t tempReg;

  /* Read the register content */
  L3gxByteRead(&tempReg, L3G_CTRL_REG3);

  /* Unmask the selected IRQ */
  if(xNewState)
    tempReg |= (uint8_t)xIrq2Config;
  else
    tempReg &= ~(uint8_t)xIrq2Config;

  /* Write byte on register */
  L3gxByteWrite(&tempReg, L3G_CTRL_REG3);

}


/**
* @brief Configures the sensor FIFO.
* @param L3GFifoInit: pointer to the Fifo initialization structure.
*        This parameter is a pointer to a @ref L3GFifoInit.
* @note  This function won't enable the FIFO. Please call the @ref Lsm303dlhcAccFifo in order to enable this mode.
* @retval None
*/
void L3gxFifoInit(L3GFifoInit* pxFifoInit)
{
  /* Built the byte to be written */
  uint8_t tempReg = (pxFifoInit->xFifoMode | (pxFifoInit->cWtm & 0x1F));

  /* Write the built value on the FIFO_CTRL register */
  L3gxByteWrite(&tempReg, L3G_FIFO_CTRL_REG);

}


/**
* @brief Configures the sensor FIFO.
* @param xNewState: New state for the FIFO use.
*        This parameter is a pointer to a @ref L3gxFifo
* @retval None
*/
void L3gxFifo(L3GFunctionalState xNewState)
{
  /* Built the byte to be written */
  uint8_t tempReg;

  /* Read the value on the CTRL5 register */
  L3gxByteRead(&tempReg, L3G_CTRL_REG5);

  /* Build the value to write */
  if(xNewState == L3G_ENABLE)
    tempReg |= 0x40;
  else
    tempReg &= (~0x40);

  /* Write the built value on the CTRL5 register */
  L3gxByteWrite(&tempReg, L3G_CTRL_REG5);

}


/**
* @brief Gets the FIFO status flags and unread data.
* @param  None.
* @retval L3GFifoStatus Fifo status descriptor.
*/
L3GFifoStatus L3gxFifoGetStatus(void)
{
  L3GFifoStatus xL3GFifoStatus;

  /* Read the register content */
  L3gxByteRead((uint8_t*)&xL3GFifoStatus, L3G_FIFO_SRC_REG);

  /* Return its value */
  return xL3GFifoStatus;

}


/**
* @brief Sets the IRQs on Axis.
* @param xIrqCombinations: Combination of events on axis.
*        This parameter is a @ref L3GIrqOnaxisCombination .
* @param pxAxisEvents: Events on axis structure.
*        This parameter is a pointer to a @ref L3GAxisEvents .
* @param xLatched: Latched interrupt.
*        This parameter can be L3G_ENABLE or L3G_DISABLE .
* @retval None
* @details
* <b>Example:</b>
* @code
*   L3GAxisEvents xAxisEvents;
*
*   xAxisEvents.X_LOW=0;
*   xAxisEvents.X_HIGH=0;
*   xAxisEvents.Y_LOW=1;
*   xAxisEvents.Y_HIGH=0;
*   xAxisEvents.Z_LOW=0;
*   xAxisEvents.Z_HIGH=1;
*
*   L3gxSetAxisIrqs(L3G_OR_COMBINATION,&xAxisEvents,L3G_DISABLE);
* @endcode
*/
void L3gxSetAxisIrqs(L3GIrqOnaxisCombination xIrqCombinations, L3GAxisEvents* pxAxisEvents, L3GFunctionalState xLatched)
{
  /* Build the value to build on register */
  uint8_t tempReg = ((uint8_t)xIrqCombinations) | (*(uint8_t*)pxAxisEvents);

  /* if required, set the IRQ latched bit */
  if(xLatched)
    tempReg |= 0x40;

  /* Write the built value on the irq1 configuration register */
  L3gxByteWrite(&tempReg, L3G_INT1_CFG_REG);

}


/**
* @brief Gets the IRQs on Axis status.
* @param pxAxisEvents: Events on axis structure.
*        This parameter is a pointer to a @ref L3GAxisEvents .
* @retval None
*/
L3GFunctionalState L3gxGetAxisIrqs(L3GAxisEvents* pxAxisEvents)
{
  uint8_t tempReg;

  /* Read register */
  L3gxByteRead(&tempReg, L3G_INT1_SCR_REG);

  uint8_t tempRet = tempReg & 0x3F;

  /* Take the MSb to return */
  (*pxAxisEvents)=*(L3GAxisEvents*)(&tempRet);

  return (L3GFunctionalState)((tempReg & 0x40)>>6);

}


/**
* @brief Sets threshold for the angular rate.
* @param pfData: Angular rate threshold ordered by axis (XYZ) and expressed in dps.
*        This parameter is a float array.
* @retval None
*/
void L3gxSetThreshold(float *pfData)
{
  uint8_t reg, j=0, pcData[6];
  float fSensitivity,aux;
  int16_t pnData[3];

  /* X and Y thresholds and flags are inverted.. here a workaround */
  aux=pfData[1];
  pfData[1]=pfData[0];
  pfData[0]=aux;

  /* Read the CTRL4 register */
  L3gxByteRead(&reg,L3G_CTRL_REG4);

  /* switch the sensitivity value set in the CRTL4*/
  switch(reg & 0x30)
  {
  case 0x00:
    fSensitivity=L3G_Sensitivity_250dps;
    break;

  case 0x10:
    fSensitivity=L3G_Sensitivity_500dps;
    break;

  case 0x30:
    fSensitivity=L3G_Sensitivity_2000dps;
    break;
  }

  /* Convert the dps values in register values */
  for(uint8_t i=0;i<3;i++)
  {
    pnData[i] = (int16_t)(pfData[i]*fSensitivity);

    pcData[j++] = (uint8_t)(pnData[i]>>8);
    pcData[j++] = (uint8_t)pnData[i];
  }

  /* Write the values on registers */
  L3gxBufferWrite(pcData,L3G_INT1_THS_XH_REG,6);


}


/**
* @brief  Read L3G4300D temperature output register.
* @param  None.
* @retval float: Temperature (C/LSB).
*/
float L3gxReadTemp(void)
{
  int8_t buffer;

  /* Read register */
  L3gxByteRead((uint8_t*)&buffer, L3G_OUT_TEMP);

  /* convert to float */
  return (float)buffer;

}


/**
*@}
*/


/**
 * @}
 */


/**
 * @}
 */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
