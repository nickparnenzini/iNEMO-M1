/**
  * @file    LSM303DLH.c
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    12 April 2012
  * @brief   This file provides a set of functions needed to manage the LSM303DLH slave.
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
#include "LSM303DLHC.h"



/**
 * @addtogroup Sensor_Libraries           Sensor Libraries
 * @{
 */

/**
* @defgroup LSM303DLHC
* @{
*/

/**
 * @addtogroup Accelerometer
 * @{
 */

/**
 * @defgroup Accelerometer_Private_TypesDefinitions      Accelerometer Private TypesDefinitions
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup Accelerometer_Private_Defines               Accelerometer Private Defines
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Accelerometer_Private_Macros               Accelerometer Private Macros
 * @{
 */

#define LSM_ABS(a)              (a>0?(a):-(a))
/**
 *@}
 */


/**
 * @defgroup Accelerometer_Private_Variables             Accelerometer Private Variables
 * @{
 */



/**
 *@}
 */



/**
 * @defgroup Accelerometer_Private_FunctionPrototypes    Accelerometer Private FunctionPrototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Accelerometer_Private_Functions             Accelerometer Private Functions
 * @{
 */

/**
* @brief  Set configuration of Linear Acceleration measurement of LSM303DLHC
* @param  pxLSMAccInitStruct : pointer to a LSMAccInit structure that contains the configuration setting for the Accelerometer LSM303DLH.
* @retval None.
* @details
* <b>Example:</b>
* @code
*  LSMAccInit LSMAccInitStructure;
*
*  LSMAccInitStructure.xPowerMode = LSM_NORMAL_MODE;
*  LSMAccInitStructure.xOutputDataRate = LSM_ODR_400_HZ;
*  LSMAccInitStructure.xEnabledAxes= LSM_ALL_AXES_EN;
*  LSMAccInitStructure.xFullScale = LSM_FS_2G;
*  LSMAccInitStructure.xDataUpdate = LSM_CONTINUOS_UPDATE;
*  LSMAccInitStructure.xEndianness = LSM_BIG_ENDIAN;
*  LSMAccInitStructure.xHighResolution = LSM_ENABLE;
*
*  Lsm303dlhcAccConfig(&LSMAccInitStructure);
* @endcode
*/
void Lsm303dlhcAccConfig(LSMAccInit* pxLSMAccInitStruct)
{
  uint8_t CTRL1 = 0x00, CTRL4  =  0x00;

  /* Read the CTRL4 register content */
  Lsm303dlhcAccI2CByteRead(&CTRL4, LSM_A_CTRL4_REG_ADDR);

  /* Compute the register values */
  CTRL1 |= (uint8_t) ((uint8_t)pxLSMAccInitStruct->xPowerMode | (uint8_t)pxLSMAccInitStruct->xOutputDataRate | (uint8_t)pxLSMAccInitStruct->xEnabledAxes);
  CTRL4 |= (uint8_t) ((uint8_t)pxLSMAccInitStruct->xFullScale | (uint8_t)pxLSMAccInitStruct->xDataUpdate | (uint8_t)pxLSMAccInitStruct->xEndianness);
  if(pxLSMAccInitStruct->xHighResolution == LSM_ENABLE)
  {
    CTRL4 |= 0x08;
  }
  else
  {
    CTRL4 &= 0xF0;
  }

  /* Write the computed values on registers */
  Lsm303dlhcAccI2CByteWrite(&CTRL1, LSM_A_CTRL1_REG_ADDR);
  Lsm303dlhcAccI2CByteWrite(&CTRL4, LSM_A_CTRL4_REG_ADDR);

}


/**
* @brief  Gets the general configuration of LSM303DLHC for the linear acceleration.
* @param  pxLSMAccInitStruct : pointer to a LSMAccInit structure that will
*         contain the configuration setting read from the LSM303DLHC registers.
* @retval None
*/
void Lsm303dlhcAccGetInfo(LSMAccInit* pxLSMAccInitStruct)
{
  uint8_t CTRL1, CTRL4;

  /* Read the registers content */
  Lsm303dlhcAccI2CByteRead(&CTRL4, LSM_A_CTRL4_REG_ADDR);
  Lsm303dlhcAccI2CByteRead(&CTRL1, LSM_A_CTRL1_REG_ADDR);

  /* Fill the structure fields from CTRL1 reg info */
  pxLSMAccInitStruct->xPowerMode = (AccPowerMode)(CTRL1 & 0x08);
  pxLSMAccInitStruct->xOutputDataRate = (AccOutputDataRate)(CTRL1 & 0xF0);
  pxLSMAccInitStruct->xEnabledAxes = (AccAxesEnabling)(CTRL1 & 0x07);

  /* Fill the structure fields from CTRL4 reg info */
  pxLSMAccInitStruct->xFullScale = (AccFullScale)(CTRL4 & 0x30);
  pxLSMAccInitStruct->xDataUpdate = (AccBlockDataUpdate)(CTRL4 & 0x80);
  pxLSMAccInitStruct->xEndianness = (AccEndianness)(CTRL4 & 0x40);

  if(CTRL4 & 0x08)
    pxLSMAccInitStruct->xHighResolution = LSM_ENABLE;
  else
    pxLSMAccInitStruct->xHighResolution = LSM_DISABLE;

}


/**
* @brief  Set configuration of  Internal High Pass Filter of  LSM303DLHC for the linear acceleration
* @param  pxLSMAccFilterInitStruct : pointer to a LSMAccFilterInit structure that
*         contains the configuration setting for the LSM303DLHC.
* @retval None
* @details
* <b>Example:</b>
* @code
*  LSMAccFilterInit LSMAccFilterInitStructure;
*
*  LSMAccFilterInitStructure.xHPF=LSM_DISABLE;
*  LSMAccFilterInitStructure.xHPF_Mode=LSM_HPFM_NORMAL;
*  LSMAccFilterInitStructure.cHPFReference=0x00;
*  LSMAccFilterInitStructure.xHPFCutOff=LSM_HPCF_16;
*  LSMAccFilterInitStructure.xHPFClick=LSM_DISABLE;
*  LSMAccFilterInitStructure.xHPFAOI2=LSM_DISABLE;
*  LSMAccFilterInitStructure.xHPFAOI1=LSM_DISABLE;
*
*  Lsm303dlhcAccFilterConfig(&LSMAccFilterInitStructure);
* @endcode
*/
void Lsm303dlhcAccFilterConfig(LSMAccFilterInit* pxLSMAccFilterInitStruct)
{
  uint8_t CTRL2 = 0x00;
  uint8_t REF  =  0x00;

  /* Compute the register values */
  CTRL2 |= (uint8_t) ((uint8_t)pxLSMAccFilterInitStruct->xHPF_Mode| (uint8_t)pxLSMAccFilterInitStruct->xHPFCutOff);
  if(pxLSMAccFilterInitStruct->xHPF == LSM_ENABLE)
  {
    CTRL2 |= 0x08;
  }
  else
  {
    CTRL2 &= 0xF7;
  }
  if(pxLSMAccFilterInitStruct->xHPFClick == LSM_ENABLE)
  {
    CTRL2 |= 0x04;
  }
  else
  {
    CTRL2 &= 0xFB;
  }
  if(pxLSMAccFilterInitStruct->xHPFAOI2 == LSM_ENABLE)
  {
    CTRL2 |= 0x02;
  }
  else
  {
    CTRL2 &= 0xFD;
  }
  if(pxLSMAccFilterInitStruct->xHPFAOI1 == LSM_ENABLE)
  {
    CTRL2 |= 0x01;
  }
  else
  {
    CTRL2 &= 0xFE;
  }

  REF |= (uint8_t) (pxLSMAccFilterInitStruct->cHPFReference);

  /* Write the computed values on registers */
  Lsm303dlhcAccI2CByteWrite(&CTRL2, LSM_A_CTRL2_REG_ADDR);
  Lsm303dlhcAccI2CByteWrite(&REF, LSM_A_REFERENCE_REG_ADDR);
}


/**
* @brief  Get configuration of Internal High Pass Filter of  LSM303DLHC for the linear acceleration
* @param  pxLSMAccFilterInitStruct : pointer to a LSMAccFilterInit structure that will
*         contain the configuration setting read from the LSM303DLHC registers.
* @retval None
*/
void Lsm303dlhcAccFilterGetInfo(LSMAccFilterInit* pxLSMAccFilterInitStruct)
{
  uint8_t ctrl2, ref;

  Lsm303dlhcAccI2CByteRead(&ctrl2, LSM_A_CTRL2_REG_ADDR);
  Lsm303dlhcAccI2CByteRead(&ref, LSM_A_REFERENCE_REG_ADDR);

  /* Get the filter mode and the cut-off frequency */
  pxLSMAccFilterInitStruct->xHPF_Mode = (AccHPFMode)(ctrl2 & 0xC0);
  pxLSMAccFilterInitStruct->xHPFCutOff =  (AccHPFCutOff)(ctrl2 & 0x30);

  /* Get the enable/disable filter bit */
  if(ctrl2 & 0x08)
    pxLSMAccFilterInitStruct->xHPF = LSM_ENABLE;
  else
    pxLSMAccFilterInitStruct->xHPF = LSM_DISABLE;

  /* Get the enable/disable filter for click bit */
  if(ctrl2 & 0x04)
    pxLSMAccFilterInitStruct->xHPFClick = LSM_ENABLE;
  else
    pxLSMAccFilterInitStruct->xHPFClick = LSM_DISABLE;

  /* Get the enable/disable int2 bit */
  if(ctrl2 & 0x02)
    pxLSMAccFilterInitStruct->xHPFAOI2 = LSM_ENABLE;
  else
    pxLSMAccFilterInitStruct->xHPFAOI2 = LSM_DISABLE;

  /* Get the enable/disable int1 bit */
  if(ctrl2 & 0x01)
    pxLSMAccFilterInitStruct->xHPFAOI1 = LSM_ENABLE;
  else
    pxLSMAccFilterInitStruct->xHPFAOI1 = LSM_DISABLE;

  /* Get the reference value */
  pxLSMAccFilterInitStruct->cHPFReference=ref;

}


/**
* @brief  Enable or disable the lowpower mode for Accelerometer of LSM303DLHC
* @param  xFunctionalState : new state for the lowpower mode. This parameter can be:  LSM_ENABLE or LSM_DISABLE
* @retval None
*/
void Lsm303dlhcAccLowPowerMode(LSMFunctionalState xFunctionalState)
{
  uint8_t tmpreg;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead(&tmpreg, LSM_A_CTRL1_REG_ADDR);

  /* modify the specified bit */
  if(xFunctionalState == LSM_ENABLE)
  {
    tmpreg |= 0x08;
  }
  else
  {
    tmpreg &= 0xF7;
  }

  /* Write the computed values on registers */
  Lsm303dlhcAccI2CByteWrite(&tmpreg, LSM_A_CTRL1_REG_ADDR);
}


/**
* @brief  Change the ODR(Output data rate) for Acceleromter of LSM303DLH
* @param  xDataRate : new ODR value. This parameter can be one of the AccOutputDataRate value
* @retval None
*/
void Lsm303dlhcAccSetDataRate(AccOutputDataRate xDataRate)
{
  uint8_t tmpreg;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead(&tmpreg, LSM_A_CTRL1_REG_ADDR);
  tmpreg &= 0x0F;
  tmpreg |= (uint8_t) xDataRate;

  /* Write computed byte onto register */
  Lsm303dlhcAccI2CByteWrite(&tmpreg, LSM_A_CTRL1_REG_ADDR);
}


/**
* @brief  Returns the output data rate as a AccOutputDataRate enumerative value.
* @param  None.
* @retval AccOutputDataRate: AccOutputDataRate enumerative value.
*/
AccOutputDataRate Lsm303dlhcAccGetEnumDataRate(void)
{
  uint8_t tmpReg;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead(&tmpReg, LSM_A_CTRL1_REG_ADDR);

  /* mask and return it */
  return((AccOutputDataRate)(tmpReg & 0xF0));

}


/**
* @brief  Returns the output data rate.
* @param  None.
* @retval Datarate in Hz.
*         This parameter is an uint16_t.
*/
uint16_t Lsm303dlhcAccGetDataRate(void)
{
  uint8_t tmpReg;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead(&tmpReg, LSM_A_CTRL1_REG_ADDR);

  /* ..mask it */
  tmpReg &= 0xF0;

  /* return the correspondent value */
  switch(tmpReg){
  case LSM_ODR_1_HZ:
    return 1;
  case LSM_ODR_10_HZ:
    return 10;
  case LSM_ODR_25_HZ:
    return 25;
  case LSM_ODR_50_HZ:
    return 50;
  case LSM_ODR_100_HZ:
    return 100;
  case LSM_ODR_200_HZ:
    return 200;
  case LSM_ODR_400_HZ:
    return 400;
  case LSM_ODR_1620_HZ:
    return 1620;
  case LSM_ODR_1344_HZ:
    return 1344;
  }

  return 0;
}

/**
* @brief  Change the Full Scale of LSM303DLH
* @param  xFullScale : new full scale value. This parameter can be one of the AccFullScale value
* @retval None
*/
void Lsm303dlhcAccSetFullScale(AccFullScale xFullScale)
{
  uint8_t tmpreg;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead(&tmpreg, LSM_A_CTRL4_REG_ADDR);

  /* Compute the value */
  tmpreg &= 0xCF;
  tmpreg |= (uint8_t) xFullScale;

  /* Write the computed value */
  Lsm303dlhcAccI2CByteWrite(&tmpreg, LSM_A_CTRL4_REG_ADDR);
}


/**
* @brief  Returns the Full Scale of LSM303DLH as a AccFullScale enumerative value.
* @param  None.
* @retval AccFullScale: Abs value of Fullscale typdef.
*/
AccFullScale Lsm303dlhcAccGetEnumFullScale(void)
{
  uint8_t tmpReg;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead(&tmpReg, LSM_A_CTRL4_REG_ADDR);

  /* Return the enumerative type value */
  return((AccFullScale)(tmpReg & 0x30));

}


/**
* @brief  Returns the Full Scale of LSM303DLH expressed in g.
* @param  None.
* @retval uint8_t: Abs value of Fullscale expressed in g.
*/
uint8_t Lsm303dlhcAccGetFullScale(void)
{
  uint8_t tmpReg;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead(&tmpReg, LSM_A_CTRL4_REG_ADDR);
  tmpReg &= 0x30;

  /* return the correspondent value */
  switch(tmpReg)
  {
  case LSM_FS_2G:
    return 2;
  case LSM_FS_4G:
    return 4;
  case LSM_FS_8G:
    return 8;
  case LSM_FS_16G:
    return 16;
  }

  return 0;
}


/**
* @brief  Returns the Full Scale of LSM303DLH expressed in LSB/mg.
* @param  pfSensitivityXYZ: pointer to 3 elements array in which the sensitivity values have to be stored.
*         This parameter is a pointer to a float array.
* @retval None.
*/
void Lsm303dlhcAccGetSensitivity(float *pfSensitivityXYZ)
{
  uint8_t tmpReg;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead(&tmpReg, LSM_A_CTRL4_REG_ADDR);
  tmpReg &= 0x30;

  /* return the correspondent value */
  switch(tmpReg)
  {
  case LSM_FS_2G:
    pfSensitivityXYZ[0]=LSM_Acc_Sensitivity_2g;
    pfSensitivityXYZ[1]=LSM_Acc_Sensitivity_2g;
    pfSensitivityXYZ[2]=LSM_Acc_Sensitivity_2g;
    
    break;
  case LSM_FS_4G:
    pfSensitivityXYZ[0]=LSM_Acc_Sensitivity_4g;
    pfSensitivityXYZ[1]=LSM_Acc_Sensitivity_4g;
    pfSensitivityXYZ[2]=LSM_Acc_Sensitivity_4g;
    
    break;
  case LSM_FS_8G:
    pfSensitivityXYZ[0]=LSM_Acc_Sensitivity_8g;
    pfSensitivityXYZ[1]=LSM_Acc_Sensitivity_8g;
    pfSensitivityXYZ[2]=LSM_Acc_Sensitivity_8g;
    
    break;
  case LSM_FS_16G:
    pfSensitivityXYZ[0]=LSM_Acc_Sensitivity_16g;
    pfSensitivityXYZ[1]=LSM_Acc_Sensitivity_16g;
    pfSensitivityXYZ[2]=LSM_Acc_Sensitivity_16g;
    
    break;
  }
  
   
}

/**
* @brief  Reboot memory content of LSM303DLH
* @param  None
* @retval None
*/
void Lsm303dlhcAccRebootCmd(void)
{
  uint8_t tmpreg;
  Lsm303dlhcAccI2CByteRead(&tmpreg, LSM_A_CTRL2_REG_ADDR);
  tmpreg |= 0x80;
  Lsm303dlhcAccI2CByteWrite(&tmpreg, LSM_A_CTRL2_REG_ADDR);
}


/**
* @brief  Read LSM303DLHC linear acceleration output register
* @param  out : buffer to store data
* @retval None
*/
void Lsm303dlhcAccReadOutReg(uint8_t* pcReg)
{
  /* Read the register content */
  Lsm303dlhcAccI2CBufferRead(pcReg, LSM_A_OUT_X_L_REG_ADDR, 6);
}


/**
* @brief Read LSM303DLHC output register, and calculate the raw  acceleration [LSB] ACC= (out_h*256+out_l)/16 (12 bit rappresentation)
* @param pnRawData: pointer to signed 16-bit data buffer where to store data
* @retval None
*/
void Lsm303dlhcAccReadRawData(int16_t* pnRawData)
{
  uint8_t buffer[6], ctrlx[2], cDivider;

  /* Read the register content */
  Lsm303dlhcAccI2CBufferRead(ctrlx, LSM_A_CTRL4_REG_ADDR,2);
  Lsm303dlhcAccReadOutReg(&buffer[0]);


  if(ctrlx[1]&0x40)
    cDivider=64;
  else
    cDivider=16;

  /* check in the control register4 the data alignment*/
  if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
  {
    for(int i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
    }
  }
  else /* Big Endian Mode */
  {
    for(uint8_t i=0; i<3; i++)
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
  }
}


/**
* @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit rappresentation)
* @param pnData: pointer to float buffer where to store data
* @retval None
*/
void Lsm303dlhcAccReadAcc(float* pfData)
{
  int16_t buffer[3];
  uint8_t ctrlx[2];
  float LSM_Acc_Sensitivity;

  /* Read the raw data */
  Lsm303dlhcAccReadRawData(buffer);

  /* Read the register content */
  Lsm303dlhcAccI2CBufferRead(ctrlx, LSM_A_CTRL4_REG_ADDR,2);


  if(ctrlx[1]&0x40){
    /* FIFO mode */
    LSM_Acc_Sensitivity = 0.25;
  }
  else
  {
    /* normal mode */
    /* switch the sensitivity value set in the CRTL4*/
    switch(ctrlx[0] & 0x30)
    {
    case LSM_FS_2G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
      break;
    case LSM_FS_4G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
      break;
    case LSM_FS_8G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
      break;
    case LSM_FS_16G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
      break;
    }
  }

  /* Obtain the mg value for the three axis */
  for(uint8_t i=0; i<3; i++)
  {
    pfData[i]=(float)buffer[i]/LSM_Acc_Sensitivity;
  }

}


/**
* @brief Read LSM303DLHC output register when FIFO mode is active.
* @param pnData: pointer to signed integer buffer where to store data.
*        This parameter is an int16_t array pointer.
* @param cDataToRead: number of samples to read. Each sample is made up of the three axis readings.
*        This parameter is an uint8_t .
* @retval None
*/
void Lsm303dlhcAccReadAccFifo(int16_t* pnData, uint8_t cDataToRead)
{
  uint8_t *pcBuffer=(uint8_t*)pnData;
  uint8_t j=0;

  /* Read the register content */
  Lsm303dlhcAccI2CBufferRead(pcBuffer, LSM_A_OUT_X_L_REG_ADDR, cDataToRead*6);

  /* convert all data to signed int16 */
  for(uint16_t i=0 ; i<cDataToRead*3 ; i++)
  {
    pnData[i] = (int16_t)(((uint16_t)pcBuffer[j+1]<<8)+(uint16_t)pcBuffer[j])/16;
    j+=2;
  }

}


/**
* @brief Sets the IRQ1 line to be raised on a specific event.
* @param xLSMAIrq1Config: Interrupt mask to enable.
*        This parameter is a @ref LSMAIrq1List .
* @param xNewState: Enable or disable I1.
*        This parameter can be LSM_ENABLE or LSM_DISABLE.
* @retval None
* @details
* <b>Example:</b>
* @code
*  ...
*  ExtiConfiguration();           // set the micro exti before init
*  Lsm303dlhcAccIrq1Config(LSM_I1_DRDY1,LSM_ENABLE);      // for example enable the data_ready IRQ on line1
*  ...
* @endcode
*/
void Lsm303dlhcAccIrq1Config(LSMAIrq1List xLSMAIrq1Config, LSMFunctionalState xNewState)
{
  uint8_t tempReg;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead(&tempReg, LSM_A_CTRL3_REG_ADDR);

  /* Unmask the selected IRQ */
  if(xNewState)
    tempReg |= (uint8_t)xLSMAIrq1Config;
  else
    tempReg &= ~(uint8_t)xLSMAIrq1Config;

  /* Write byte on register */
  Lsm303dlhcAccI2CByteWrite(&tempReg, LSM_A_CTRL3_REG_ADDR);

}


/**
* @brief Sets the IRQ2 line to be raised on a specific event.
* @param xLSMAIrq2Config: Interrupt mask to enable.
*        This parameter is a @ref LSMAIrq2List .
* @param xNewState: Enable or disable I2.
*        This parameter can be LSM_ENABLE or LSM_DISABLE.
* @retval None
*/
void Lsm303dlhcAccIrq2Config(LSMAIrq2List xLSMAIrq2Config, LSMFunctionalState xNewState)
{
  uint8_t tempReg;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead(&tempReg, LSM_A_CTRL6_REG_ADDR);

  /* Unmask the selected IRQ */
  if(xNewState)
    tempReg |= (uint8_t)xLSMAIrq2Config;
  else
    tempReg &= ~(uint8_t)xLSMAIrq2Config;

  /* Write byte on register */
  Lsm303dlhcAccI2CByteWrite(&tempReg, LSM_A_CTRL6_REG_ADDR);

}


/**
* @brief Configures the sensor FIFO.
* @param xNewState: New state for the FIFO use.
*        This parameter is a @ref LSMFunctionalState
* @retval None
*/
void Lsm303dlhcAccFifo(LSMFunctionalState xNewState)
{
  /* Built the byte to be written */
  uint8_t tempReg;

  /* Read the value on the CTRL5 register */
  Lsm303dlhcAccI2CByteWrite(&tempReg, LSM_A_CTRL5_REG_ADDR);

  /* Build the value to write */
  if(xNewState == LSM_ENABLE)
    tempReg |= 0x40;
  else
    tempReg &= (~0x40);

  /* Write the built value on the CTRL5 register */
  Lsm303dlhcAccI2CByteWrite(&tempReg, LSM_A_CTRL5_REG_ADDR);

}


/**
* @brief Configures the sensor FIFO.
* @param pxLSMAccFifoInit: pointer to the Fifo initialization structure.
*        This parameter is a pointer to a @ref LSMAccFifoInit.
* @note  This function won't enable the FIFO. Please call the @ref Lsm303dlhcAccFifo in order to enable this mode.
* @retval None
*/
void Lsm303dlhcAccFifoInit(LSMAccFifoInit* pxLSMAccFifoInit)
{
  /* Built the byte to be written */
  uint8_t tempReg = (pxLSMAccFifoInit->xFifoMode | (pxLSMAccFifoInit->cWtm & 0x1F));

  /* Select the INT line */
  if(pxLSMAccFifoInit->xTriggerSel == LSM_INT2_LINE)
    tempReg |= 0x20;

  /* Write the built value on the FIFO_CTRL register */
  Lsm303dlhcAccI2CByteWrite(&tempReg, LSM_A_FIFO_CTRL_REG_ADDR);

}


/**
* @brief Gets the FIFO status flags and unread data.
* @param pxLSMAccFifoStatus: pointer to the Fifo status structure.
*        This parameter is a pointer to a @ref LSMAccFifoStatus
* @retval None
*/
LSMAccFifoStatus Lsm303dlhcAccFifoGetStatus(void)
{
  LSMAccFifoStatus xLSMAccFifoStatus;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead((uint8_t*)&xLSMAccFifoStatus, LSM_A_FIFO_STATUS_REG_ADDR);

  /* Return its value */
  return xLSMAccFifoStatus;
}


/**
* @brief Sets the IRQs on Axis.
* @param xIrqCombinations: Combination of events on axis.
*        This parameter is a @ref LSMAccIrqOnaxisCombination .
* @param pxAxisEvents: Events on axis structure.
*        This parameter is a pointer to a @ref LSMAccAxisEvents .
* @param xIRQLine: IRQ line to be set.
*        This parameter is a @ref LSMAIrqLine .
* @param xLatched: specifies if an IRQ is latched.
*        This parameter can be LSM_ENABLE or LSM_DISABLE .
* @retval None
*/
void Lsm303dlhcAccSetAxisIrqs(LSMAccIrqOnaxisCombination xIrqCombinations, LSMAccAxisEvents* pxAxisEvents, LSMAIrqLine xIRQLine, LSMFunctionalState xLatched)
{

  /* Build the value to build on register */
  uint8_t tempReg = ((uint8_t)xIrqCombinations) | (*(uint8_t*)pxAxisEvents);
  uint8_t tempLatch;

  /* Read for latch conf */
  Lsm303dlhcAccI2CByteRead(&tempLatch, LSM_A_CTRL5_REG_ADDR);

  /* Interrupt line selection */
  if(xIRQLine == LSM_INT1_LINE)
  {
    /* Write the built value on the LSM_A_INT1_CFG_REG register */
    Lsm303dlhcAccI2CByteWrite(&tempReg, LSM_A_INT1_CFG_REG_ADDR);

    if(xLatched)
      tempLatch |= 0x08;
    else
      tempLatch &= 0xF7;
  }
  else
  {
    /* Write the built value on the LSM_A_INT2_CFG_REG register */
    Lsm303dlhcAccI2CByteWrite(&tempReg, LSM_A_INT2_CFG_REG_ADDR);

    if(xLatched)
      tempLatch |= 0x02;
    else
      tempLatch &= 0xFD;
  }

  /* Write for latch conf */
  Lsm303dlhcAccI2CByteWrite(&tempLatch, LSM_A_CTRL5_REG_ADDR);

}


/**
* @brief Gets the IRQs on Axis status.
* @param pxAxisEvents: Events on axis structure.
*        This parameter is a pointer to a @ref LSMAccAxisEvents .
* @param xIRQLine: IRQ line mask to be get.
*        This parameter is a @ref LSMAIrqLine .
* @retval None
*/
LSMFunctionalState Lsm303dlhcAccGetAxisIrqs(LSMAccAxisEvents* pxAxisEvents, LSMAIrqLine xIRQLine)
{
  uint8_t tempReg;

  /* Interrupt line selection */
  if(xIRQLine == LSM_INT1_LINE)
  {
    /* Read the register for the LINE1 */
    Lsm303dlhcAccI2CByteRead(&tempReg, LSM_A_INT1_SRC_REG_ADDR);
  }
  else
  {
    /* Read the register for the LINE2 */
    Lsm303dlhcAccI2CByteRead(&tempReg, LSM_A_INT2_SRC_REG_ADDR);
  }

  uint8_t tempRet = tempReg & 0x7F;

  /* Take the MSb to return */
  (*pxAxisEvents)=*(LSMAccAxisEvents*)(&tempRet);

  return (LSMFunctionalState)(tempRet>>6);

}


/**
* @brief Sets threshold for acceleration.
* @param nData: Acceleration threshold.
*        This parameter is a uint16_t.
* @param xIRQLine: IRQ threshold mask to be set.
*        This parameter is a @ref LSMAIrqLine .
* @retval None
*/
void Lsm303dlhcAccSetThreshold(int16_t nData, LSMAIrqLine xIRQLine)
{
  uint8_t ctrl4, accThs;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead(&ctrl4, LSM_A_CTRL4_REG_ADDR);

  /* switch the sensitivity value set in the CRTL4*/
  switch(ctrl4 & 0x30)
  {
  case LSM_FS_2G:
      accThs = (uint8_t)((float)(nData/16)*LSM_Acc_Sensitivity_2g);
  break;
  case LSM_FS_4G:
      accThs = (uint8_t)((float)(nData/16)*LSM_Acc_Sensitivity_4g);
  break;
  case LSM_FS_8G:
      accThs = (uint8_t)((float)(nData/16)*LSM_Acc_Sensitivity_8g);
  break;
  case LSM_FS_16G:
      accThs = (uint8_t)((float)(nData/16)*LSM_Acc_Sensitivity_16g);
  break;
  }

  /* Ensure that the MSb is 0 */
  accThs &= 0x7F;

  /* Interrupt line selection */
  if(xIRQLine == LSM_INT1_LINE)
  {
    /* Write byte on register */
    Lsm303dlhcAccI2CByteWrite(&accThs, LSM_A_INT1_THS_REG_ADDR);
  }
  else
  {
    /* Write byte on register */
    Lsm303dlhcAccI2CByteWrite(&accThs, LSM_A_INT2_THS_REG_ADDR);
  }

}


/**
* @brief Sets the minimum duration of an IRQ to be recognized.
* @param cDuration: Duration expressed in ms.
*        This parameter is a uint8_t .
* @param xIRQLine: IRQ duration to be set.
*        This parameter is a @ref LSMAIrqLine .
* @retval None
*/
void Lsm303dlhcAccSetIrqDuration(uint8_t cDuration, LSMAIrqLine xIRQLine)
{
  uint8_t tempReg;

  /* Get datarate for time register value computation */
  uint16_t nDatarate = Lsm303dlhcAccGetDataRate();

  /* Compute the duration register value */
  tempReg=(uint8_t)((float)cDuration/1000*nDatarate);

  if(xIRQLine == LSM_INT1_LINE)
  {
    /* Write byte on register */
    Lsm303dlhcAccI2CByteWrite(&tempReg, LSM_A_INT1_DURATION_REG_ADDR);
  }
  else
  {
    /* Write byte on register */
    Lsm303dlhcAccI2CByteWrite(&tempReg, LSM_A_INT2_DURATION_REG_ADDR);
  }

}


/**
* @brief  Configures the Click or Double click recognition parameters.
* @param  xClickInit: Click configuration structure.
*         This parameter is a pointer to
* @retval None.
*/
void Lsm303dlhcAccClickInit(LSMAccClickInit* pxClickInit)
{
  uint8_t tempReg[5];

  /* Get fullscale for threshold register value computation */
  uint8_t cFullscale = Lsm303dlhcAccGetFullScale();

  /* Get datarate for time register value computation */
  uint16_t nDatarate = Lsm303dlhcAccGetDataRate();

  /* Read values on register */
  Lsm303dlhcI2CBufferRead(LSM_A_CLICK_SRC_REG_ADDR, tempReg, LSM_A_CTRL4_REG_ADDR, 5);

  /* Sign setting */
  tempReg[0] &= 0x08;
  tempReg[0] |= (pxClickInit->xNegativeDetection)<<3;

  /* Threshold value computation */
  tempReg[1] = (uint8_t)((float)pxClickInit->nClickThreshold/1000*(128/cFullscale));

  /* time limit computation */
  tempReg[2] = (uint8_t)((float)pxClickInit->cClickTimeLimit/1000*nDatarate);

  /* time latency computation */
  tempReg[3] = (uint8_t)((float)pxClickInit->cDClickTimeLatency/1000*nDatarate);

  /* time window computation */
  tempReg[4] = (uint8_t)((float)pxClickInit->cDClickTimeWindow/1000*nDatarate);

  /* Write values on register */
  Lsm303dlhcI2CBufferWrite(LSM_A_CLICK_SRC_REG_ADDR, tempReg, LSM_A_CTRL4_REG_ADDR, 5);

}


/**
* @brief  Gets status for accelerometer data.
* @param  None.
* @retval LSMADataStatus: Data status in a LSMADataStatus bitfields structure.
*/
LSMADataStatus Lsm303dlhcAccGetDataStatus(void)
{
  LSMADataStatus xStatus;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead((uint8_t*)&xStatus, LSM_A_STATUS_REG_ADDR);

  return xStatus;

}


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
 * @defgroup Magnetometer_Private_TypesDefinitions      Magnetometer Private TypesDefinitions
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup Magnetometer_Private_Defines               Magnetometer Private Defines
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Magnetometer_Private_Macros               Magnetometer Private Macros
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Magnetometer_Private_Variables             Magnetometer Private Variables
 * @{
 */

/**
 *@}
 */



/**
 * @defgroup Magnetometer_Private_FunctionPrototypes    Magnetometer Private FunctionPrototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Magnetometer_Private_Functions             Magnetometer Private Functions
 * @{
 */

/**
* @brief  Set configuration of Magnetic field measurement of LSM303DLHC.
* @param  pxLSMMagInitStruct:  pointer to LSMMagInit structure that
*         contains the configuration setting for the LSM303DLHC Magnetometer part.
* @retval None.
* @details
* <b>Example:</b>
* @code
*  LSMMagInit LSMMagInitStructure;
*
*  LSMMagInitStructure.xOutputDataRate = LSM_ODR_30_HZ;
*  LSMMagInitStructure.xFullScale = LSM_FS_1_3_GA;
*  LSMMagInitStructure.xWorkingMode = LSM_CONTINUOS_CONVERSION;
*  LSMMagInitStructure.xTemperatureSensor = LSM_ENABLE;
*
*  Lsm303dlhcMagConfig(&LSMMagInitStructure);
* @endcode
*/
void Lsm303dlhcMagConfig(LSMMagInit* pxLSMMagInitStruct)
{
  uint8_t CTRLx[2] = {0x00,0x00};
  uint8_t MODE = 0x00;

  CTRLx[0] |= (uint8_t) (pxLSMMagInitStruct->xOutputDataRate);
  if(pxLSMMagInitStruct->xTemperatureSensor == LSM_ENABLE)
    CTRLx[0] |= 0x80;

  CTRLx[1] |= (uint8_t) (pxLSMMagInitStruct->xFullScale);

  MODE  |= (uint8_t) (pxLSMMagInitStruct->xWorkingMode);

  Lsm303dlhcMagI2CBufferWrite(CTRLx, LSM_M_CRA_REG_ADDR, 2);  //CRTL_REGA and B
  Lsm303dlhcMagI2CByteWrite(&MODE, LSM_M_MR_REG_ADDR);       //Mode register

}


/**
* @brief  Gets the general configuration of LSM303DLHC for the magnetometer.
* @param  pxLSMMagInitStruct : pointer to a LSMMagInit structure that will
*         contain the configuration setting read from the LSM303DLHC registers.
* @retval None
*/
void Lsm303dlhcMagGetInfo(LSMMagInit* pxLSMMagInitStruct)
{
  uint8_t CTRLx[2];
  uint8_t MODE;

  Lsm303dlhcMagI2CBufferRead(CTRLx, LSM_M_CRA_REG_ADDR, 2);
  Lsm303dlhcMagI2CByteRead(&MODE, LSM_M_MR_REG_ADDR);

  /* Get the CTRL[0] info */
  pxLSMMagInitStruct->xOutputDataRate = (MagOutputDataRate)(CTRLx[0] & 0x1C);

  if(CTRLx[0] & 0x80)
    pxLSMMagInitStruct->xTemperatureSensor = LSM_ENABLE;
  else
    pxLSMMagInitStruct->xTemperatureSensor = LSM_DISABLE;

  /* Get the CTRL[1] info */
  pxLSMMagInitStruct->xFullScale = (MagFullScale)(CTRLx[1] & 0xE0);

  /* Get the MODE info */
  pxLSMMagInitStruct->xWorkingMode = (MagWorkingMode)(MODE & 0x03);

}


/**
* @brief  Set the full scale of the Magnetic field sensor.
* @param  xFullScale:  FullScale value.
*         This parameter must be a value of @ref MagFullScale .
* @retval None
*/
void Lsm303dlhcMagSetFullScale(MagFullScale xFullScale)
{
  uint8_t tempReg=(uint8_t) xFullScale;

  /* Write value on register */
  Lsm303dlhcMagI2CByteWrite(&tempReg, LSM_M_CRB_REG_ADDR);

}


/**
* @brief  Get the full scale of the Magnetic field sensor expressed as a MagFullScale enumerative value.
* @param  None
* @retval MagFullScale: full scale value expressed as a value of the enum typdef.
*/
MagFullScale Lsm303dlhcMagGetEnumFullScale(void)
{
  uint8_t tempReg;

  /* Read value on register */
  Lsm303dlhcMagI2CByteRead(&tempReg, LSM_M_CRB_REG_ADDR);

  /* Mask and return it */
  return((MagFullScale)(tempReg & 0xE0));

}

/**
* @brief  Get the full scale of the Magnetic field sensor.
* @param  None
* @retval float: full scale value expressed in gauss.
*/
float Lsm303dlhcMagGetFullScale(void)
{
  uint8_t tempReg;

  /* Read value on register */
  Lsm303dlhcMagI2CByteRead(&tempReg, LSM_M_CRB_REG_ADDR);

  switch(tempReg)
  {
  case LSM_FS_1_3_GA:
    return 1.3;
  case LSM_FS_1_9_GA:
    return 1.9;
  case LSM_FS_2_5_GA:
    return 2.5;
  case LSM_FS_4_0_GA:
    return 4.0;
  case LSM_FS_4_7_GA:
    return 4.7;
  case LSM_FS_5_6_GA:
    return 5.6;
  case LSM_FS_8_1_GA:
    return 8.1;
  }

  return 0.0;
}


/**
* @brief  Returns the Full Scale of LSM303DLH expressed in LSB/mgauss.
* @param  pfSensitivityXYZ: pointer to 3 elements array in which the sensitivity values have to be stored.
*         This parameter is a pointer to a float array.
* @retval None.
*/
void Lsm303dlhcMagGetSensitivity(float *pfSensitivityXYZ)
{
  uint8_t tempReg;

  /* Read value on register */
  Lsm303dlhcMagI2CByteRead(&tempReg, LSM_M_CRB_REG_ADDR);

  switch(tempReg & 0xE0)
  {
  case LSM_FS_1_3_GA:
    pfSensitivityXYZ[0]=LSM_Magn_Sensitivity_XY_1_3Ga;
    pfSensitivityXYZ[1]=LSM_Magn_Sensitivity_XY_1_3Ga;
    pfSensitivityXYZ[2]=LSM_Magn_Sensitivity_Z_1_3Ga;
    break;
  case LSM_FS_1_9_GA:
    pfSensitivityXYZ[0]=LSM_Magn_Sensitivity_XY_1_9Ga;
    pfSensitivityXYZ[1]=LSM_Magn_Sensitivity_XY_1_9Ga;
    pfSensitivityXYZ[2]=LSM_Magn_Sensitivity_Z_1_9Ga;
    break;
  case LSM_FS_2_5_GA:
    pfSensitivityXYZ[0]=LSM_Magn_Sensitivity_XY_2_5Ga;
    pfSensitivityXYZ[1]=LSM_Magn_Sensitivity_XY_2_5Ga;
    pfSensitivityXYZ[2]=LSM_Magn_Sensitivity_Z_2_5Ga;
    break;
  case LSM_FS_4_0_GA:
    pfSensitivityXYZ[0]=LSM_Magn_Sensitivity_XY_4Ga;
    pfSensitivityXYZ[1]=LSM_Magn_Sensitivity_XY_4Ga;
    pfSensitivityXYZ[2]=LSM_Magn_Sensitivity_Z_4Ga;
    break;
  case LSM_FS_4_7_GA:
    pfSensitivityXYZ[0]=LSM_Magn_Sensitivity_XY_4_7Ga;
    pfSensitivityXYZ[1]=LSM_Magn_Sensitivity_XY_4_7Ga;
    pfSensitivityXYZ[2]=LSM_Magn_Sensitivity_Z_4_7Ga;
    break;
  case LSM_FS_5_6_GA:
    pfSensitivityXYZ[0]=LSM_Magn_Sensitivity_XY_5_6Ga;
    pfSensitivityXYZ[1]=LSM_Magn_Sensitivity_XY_5_6Ga;
    pfSensitivityXYZ[2]=LSM_Magn_Sensitivity_Z_5_6Ga;
    break;
  case LSM_FS_8_1_GA:
    pfSensitivityXYZ[0]=LSM_Magn_Sensitivity_XY_8_1Ga;
    pfSensitivityXYZ[1]=LSM_Magn_Sensitivity_XY_8_1Ga;
    pfSensitivityXYZ[2]=LSM_Magn_Sensitivity_Z_8_1Ga;
    break;
  }
}
/**
* @brief  Set the data rate of the Magnetic field sensor.
* @param  xFullScale:  Datarate value.
*         This parameter must be a value of @ref MagOutputDataRate .
* @retval None
*/
void Lsm303dlhcMagSetDataRate(MagOutputDataRate xDataRate)
{
  uint8_t tempReg;

  /* Read value on register */
  Lsm303dlhcMagI2CByteRead(&tempReg, LSM_M_CRA_REG_ADDR);

  tempReg &= 0x80;
  tempReg |= (uint8_t)xDataRate;

  /* Write value on register */
  Lsm303dlhcMagI2CByteWrite(&tempReg, LSM_M_CRA_REG_ADDR);

}


/**
* @brief  Get the data rate of the Magnetic field sensor in the MagOutputDataRate enumerative value.
* @param  None
* @retval MagOutputDataRate: data rate value expressed as enum value.
*/
MagOutputDataRate Lsm303dlhcMagGetEnumDataRate(void)
{
  uint8_t tempReg;

  /* Read value on register */
  Lsm303dlhcMagI2CByteRead(&tempReg, LSM_M_CRA_REG_ADDR);

  /* Mask and return it */
  return((MagOutputDataRate)(tempReg & 0x1C));

}


/**
* @brief  Get the data rate of the Magnetic field sensor.
* @param  None
* @retval float: data rate value expressed in gauss.
*/
float Lsm303dlhcMagGetDataRate(void)
{
  uint8_t tempReg;

  /* Read value on register */
  Lsm303dlhcMagI2CByteRead(&tempReg, LSM_M_CRA_REG_ADDR);

  switch((MagOutputDataRate)(tempReg & 0x1C))
  {
  case LSM_ODR_0_75_HZ:
    return 0.75;
  case LSM_ODR_1_5_HZ:
    return 1.5;
  case LSM_ODR_7_5_HZ:
    return 7.5;
  case LSM_ODR_15_HZ:
    return 15.0;
  case LSM_ODR_30_HZ:
    return 30.0;
  case LSM_ODR_75_HZ:
    return 75.0;
  case LSM_ODR_220_HZ:
    return 220.0;
  }

  return 0.0;
}



/**
* @brief  Read LSM303DLHC output register, and calculate the magnetic field Magn[mGa]=(out_h*256+out_l)*1000/ SENSITIVITY
* @param  pnData: pointer to signed 16-bit buffer where to store data
* @note   Despite the datasheet indications, the read axis will be stored in the passed
* 		  array in the order X,Y,Z.
* @retval None
*/
void Lsm303dlhcMagReadMag(float* pfData)
{
  uint8_t buffer[6];
  uint8_t CTRLB;
  uint16_t LSM_Magn_Sensitivity_XY, LSM_Magn_Sensitivity_Z;
  uint8_t aux[2];

  Lsm303dlhcMagI2CByteRead(&CTRLB, LSM_M_CRB_REG_ADDR);
  Lsm303dlhcMagI2CBufferRead(buffer, LSM_M_OUT_X_H_ADDR, 6);

  /* exchange Z with Y */
  aux[0]=buffer[2];
  aux[1]=buffer[3];

  buffer[2]=buffer[4];
  buffer[3]=buffer[5];

  buffer[4]=aux[0];
  buffer[5]=aux[1];

  /** switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM_FS_1_3_GA:
    LSM_Magn_Sensitivity_XY = LSM_Magn_Sensitivity_XY_1_3Ga;
    LSM_Magn_Sensitivity_Z = LSM_Magn_Sensitivity_Z_1_3Ga;
    break;
  case LSM_FS_1_9_GA:
    LSM_Magn_Sensitivity_XY = LSM_Magn_Sensitivity_XY_1_9Ga;
    LSM_Magn_Sensitivity_Z = LSM_Magn_Sensitivity_Z_1_9Ga;
    break;
  case LSM_FS_2_5_GA:
    LSM_Magn_Sensitivity_XY = LSM_Magn_Sensitivity_XY_2_5Ga;
    LSM_Magn_Sensitivity_Z = LSM_Magn_Sensitivity_Z_2_5Ga;
    break;
  case LSM_FS_4_0_GA:
    LSM_Magn_Sensitivity_XY = LSM_Magn_Sensitivity_XY_4Ga;
    LSM_Magn_Sensitivity_Z = LSM_Magn_Sensitivity_Z_4Ga;
    break;
  case LSM_FS_4_7_GA:
    LSM_Magn_Sensitivity_XY = LSM_Magn_Sensitivity_XY_4_7Ga;
    LSM_Magn_Sensitivity_Z = LSM_Magn_Sensitivity_Z_4_7Ga;
    break;
  case LSM_FS_5_6_GA:
    LSM_Magn_Sensitivity_XY = LSM_Magn_Sensitivity_XY_5_6Ga;
    LSM_Magn_Sensitivity_Z = LSM_Magn_Sensitivity_Z_5_6Ga;
    break;
  case LSM_FS_8_1_GA:
    LSM_Magn_Sensitivity_XY = LSM_Magn_Sensitivity_XY_8_1Ga;
    LSM_Magn_Sensitivity_Z = LSM_Magn_Sensitivity_Z_8_1Ga;
    break;
  }

  for(int i=0; i<2; i++)
  {
    pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/LSM_Magn_Sensitivity_XY;
  }
  pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/LSM_Magn_Sensitivity_Z;

}


/**
* @brief  Read LSM303DLH magnetic field output register and compute the int16_t value.
* @param  pnRawData: pointer to signed 16-bit buffer where to store data.
* @note   Despite the datasheet indications, the read axis will be stored in the passed
* 		  array in the order X,Y,Z.
* @retval None
*/
void Lsm303dlhcMagReadRawData(int16_t* pnRawData)
{
  uint8_t buffer[6];
  uint8_t aux[2];

  Lsm303dlhcMagI2CBufferRead(buffer, LSM_M_OUT_X_H_ADDR, 6);

  /* excange Z with Y */
  aux[0]=buffer[2];
  aux[1]=buffer[3];

  buffer[2]=buffer[4];
  buffer[3]=buffer[5];

  buffer[4]=aux[0];
  buffer[5]=aux[1];

  for(uint8_t i=0; i<3; i++)
      pnRawData[i]=(int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1]);
}


/**
* @brief  Reads LSM303DLHC temperature output register, and calculate the Temperature=(out_h*256+out_l)*10/16* SENSITIVITY.
*         The temperature is expressed in decimal of degC. The sensitivity is 8 LSB/degC
* @param  pnData: pointer to a float where to store data.
* @retval None
*/
float Lsm303dlhcMagReadTemp(void)
{
  uint8_t buffer[2];

  /* Read register */
  Lsm303dlhcMagI2CBufferRead(buffer, LSM_M_TEMP_H_REG_ADDR, 2);

  /* convert to float */
  return ((float)((int16_t)(((uint16_t)buffer[0]<<8)+buffer[1])/16)/LSM_Temp_Sensitivity);

}


/**
* @brief  Reads LSM303DLHC temperature output register and compute the int16_t value
* @param  pnRawData: pointer to signed 16-bit buffer where to store data
* @retval None
*/
int16_t Lsm303dlhcMagReadRawDataTemp(void)
{
  uint8_t buffer[2];

  /* Read register */
  Lsm303dlhcMagI2CBufferRead(buffer, LSM_M_TEMP_H_REG_ADDR, 2);

  /* Return the int16 value */
  return ((int16_t)(((uint16_t)buffer[0]<<8)+buffer[1])/16);

}


/**
* @brief  Gets status for magnetometer data.
* @param  None.
* @retval LSMADataStatus: Data status in a LSMADataStatus bitfields structure.
*/
LSMMDataStatus Lsm303dlhcMagGetDataStatus(void)
{
  LSMMDataStatus xStatus;

  /* Read the register content */
  Lsm303dlhcAccI2CByteRead((uint8_t*)&xStatus, LSM_M_SR_REG_ADDR);

  return xStatus;
}


/**
* @brief Configures the DATA_READY IRQ of the magnetometer.
* @param xNewState: New state for DATA_READY.
*        This parameter is a @ref LSMFunctionalState
* @retval None
*/
void Lsm303dlhcMagDataReadyIrqConfig(LSMFunctionalState xFunctionalState)
{
  uint8_t cTempReg;
  
  Lsm303dlhcAccI2CByteRead(&cTempReg, LSM_M_SR_REG_ADDR);
  
  if(xFunctionalState)
    cTempReg |= 0x01;
  else
    cTempReg &= 0xFE;
  
  Lsm303dlhcAccI2CByteWrite(&cTempReg, LSM_M_SR_REG_ADDR);
  
}


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

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
