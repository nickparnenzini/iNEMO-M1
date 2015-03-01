/**
  * @file    iNEMO_Compass.c
  * @author  ART Team IMS-Systems Lab
  * @version V1.2.0 [FW V2.3.0]
  * @date    02 September 2012
  * @brief   Implementation file of the iNEMO Compass algorithm.
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



#include "iNEMO_Compass.h"
#include <math.h>

/** @addtogroup iNemo_Compass					iNemo Tilted Compass
  * @{
  */



/** @defgroup iNemo_Compass_Macro				iNEMO Tilted Compass Macro
  * @{
  */

#define abs(x)  (x>0.0?x:-x)

/**
 * @}
 */


/** @defgroup iNemo_Compass_Variables			        iNEMO Tilted Compass Variables
  * @{
  */

static float s_fMagMin[3],s_fMagMax[3];
static uint8_t s_cExtrReachMask;


/**
 * @}
 */



/** @defgroup iNemo_Compass_Functions			        iNEMO Tilted Compass Functions
* @{
*/

/**
 * @brief  Tilted compass algorithm
 * @param  pfMagXYZ: the XYZ magnetometer readings.
 *         This parameter is a pointer to a float array.
 * @param  pfAccXYZ: the XYZ accelerometer readings.
 *         This parameter is a pointer to a float array.
 * @param  pfRPH: Roll, Pitch and Heading angles array(in this order).
 *         This parameter is a pointer to a float array.
 * @retval None.
 */
void iNEMO_TiltedCompass(float* pfMagXYZ, float* pfAccXYZ, float* pfRPH)
{
  float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch;
  float fTiltedX,fTiltedY;
  float fAcc[3];
  int i;
   
  /* Rescale the accelerometer radings (in order to increase accuracy in the following norm computation) */
  for(i=0;i<3;i++)
    fAcc[i] = pfAccXYZ[i]/100.0;
  
  /* Compute the scaled acceleration vector norm */
  fNormAcc = sqrt(pow(fAcc[0],2)+pow(fAcc[1],2)+pow(fAcc[2],2));
  
  /* Compute some useful parameters for the g vector rotation matrix */
  fSinRoll=fAcc[1]/sqrt(pow(fAcc[1],2)+pow(fAcc[2],2));
  fCosRoll=sqrt(1.0-fSinRoll*fSinRoll);
  fSinPitch=-fAcc[0]/fNormAcc;
  fCosPitch=sqrt(1.0-fSinPitch*fSinPitch);
  
  /* Apply the rotation matrix to the magnetic field vector to obtain the X and Y components on the earth plane */
  fTiltedX = pfMagXYZ[0]*fCosPitch+pfMagXYZ[2]*fSinPitch;
  fTiltedY = pfMagXYZ[0]*fSinRoll*fSinPitch + pfMagXYZ[1]*fCosRoll - pfMagXYZ[2]*fSinRoll*fCosPitch;
 
    
  /* return the heading angle expressed in degree */
  pfRPH[2] = -atan2f(fTiltedY, fTiltedX);
  
  /* return the roll and pitch angles */
  pfRPH[0]  = atan2f(fSinRoll,fCosRoll);
  pfRPH[1] = atan2f(fSinPitch,fCosPitch);

  
}


/**
 * @brief  Megnetic sensor calibration initialization.
 * @param  None.
 * @retval None.
 */
void iNEMO_MagSensorCalibrationInit(void)
{
  for(uint8_t j=0;j<3;j++)
  {
    s_fMagMin[j]=1e6;
    s_fMagMax[j]=-s_fMagMin[j];
  }
  
  /* reset the progress indication byte */
  s_cExtrReachMask=0;
  
}


/**
 * @brief  Megnetic sensor calibration run routine.
 * @param  pfMagXYZ: the XYZ magnetometer readings.
 *         This parameter is a pointer to a float array.
 * @param  pfGain: the XYZ magnetometer extimeted gains.
 *         This parameter is a pointer to a float array.
 * @param  pfOffset: the XYZ magnetometer extimeted offsets.
 *         This parameter is a pointer to a float array.
 * @retval None.
 */
uint8_t iNEMO_MagSensorCalibrationRun(float* pfMagXYZ, float* pfGain, float* pfOffset)
{
  uint8_t cMagSensorCalProgress=0;
  uint8_t n,p;
  
#define EPS 110.0f
  
 
  /* max and min assignment */
  for(uint8_t j=0;j<3;j++)
  {
    if(pfMagXYZ[j]<s_fMagMin[j])
      s_fMagMin[j]=pfMagXYZ[j];
    
    if(pfMagXYZ[j]>s_fMagMax[j])
      s_fMagMax[j]=pfMagXYZ[j];
  }
 
  
  /* parameters computation */
  for(uint8_t j=0;j<3;j++)
  {
    pfGain[j]=(s_fMagMax[j]-s_fMagMin[j])/2;
    pfOffset[j]=(s_fMagMax[j]+s_fMagMin[j])/2;
  }
  
  
  /* calibration progress computation: it is a value which express the completeness of the explored space
  for each ellipse max and min a flag is stored in the s_cExtrReachMask variable according to the following 
  notation [0 0 0 MIN_Z_REACH MIN_Y_REACH MIN_X_REACH MAX_Z_REACH MAX_Y_REACH MAX_X_REACH] */
  for(uint8_t j=0;j<3;j++)
  {
    /* compute the next and the previous index */
    n=(j+1)%3; 
    (j==0) ? (p=2) : (p=j-1);
       
    /* check if the other to axes measurements are less than the epsilon threshold */
    if(abs(pfMagXYZ[p])<EPS && abs(pfMagXYZ[n])<EPS)
      /* if yes, take note of the positive or negative direction */
      if(pfMagXYZ[j]>0.0f)
        s_cExtrReachMask |= (0x01<<j);
      else
        s_cExtrReachMask |= (0x01<<(j+3));

  }
  
  /* compute the progress as a number between 0 and 6 (bit sum of the six bits of s_cExtrReachMask)*/
  for(uint8_t j=0;j<6;j++)
    cMagSensorCalProgress += ((s_cExtrReachMask>>j) & 0x01);
  
  /* return it as a normalized value between 0 and 255 */
  return (uint8_t)((float)cMagSensorCalProgress*42.5);
  
}


/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/

