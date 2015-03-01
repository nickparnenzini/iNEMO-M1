/**
*
* \file    iNEMO_math.h
* \author  ART Team IMS-Systems Lab
* \version V1.2.1 [FW v2.0.0]
* \date    13/09/2010
* \brief   Header file of Math Library of iNEMO
*
********************************************************************************
*
* \details
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
*/

#ifndef __INEMO_MATH
#define __INEMO_MATH

#ifdef __cplusplus
 extern "C" {
#endif

#include <stddef.h>

/** @addtogroup iNEMO_Engine_Lite        
  * @{
  */

/**
 * @defgroup iNEMO_Math        iNEMO Math       
 * @{
 */


/**
* @defgroup  iNEMO_Math_Define  iNEMO Math Define
* @{
*/

#define iNEMO_ZERO_MATRIX	0		/*!< Null Matrix */
#define	iNEMO_ONES_MATRIX	1		/*!< Ones Matrix */
#define	iNEMO_IDEN_MATRIX	2		/*!< Identity Matrix */


/**
 * @brief Gravitational force [m/s2].
 */
#define	  G	9.81f
/**
 * @brief Square Gravitational force [m/s2]2.
 */
#define   G2   19.62f

/**
 * @brief Pi-Greek.
 */
#define   PI      3.141592f

/**
 * @brief Square Pi-Greek.
 */
#define   PI2     6.283184f

/** Memory Allocation Functions */
extern void *iNEMO_Malloc(size_t size);
extern void iNEMO_Free(void *p);

/**
*@}
*/


/* Exported types ------------------------------------------------------------*/

/**
 * @addtogroup iNEMO_Math_Data_Structures iNEMO Math Data Structures
 * @details 
 *  The following Structures are used by the mathematical functions
 * @{
*/

/**
 * @struct iNEMO_fMATRIX_TYPE
 * @brief iNEMO Float Matrix Struct
 */

typedef struct
{

  short int m_nRow;      /*!< Number of Rows in the Matrix */
  short int m_nCol;      /*!< Number of Columns in the Matrix */
  float **m_pData;       /*!< Pointer to Data in the Matrix */  
} iNEMO_fMATRIX_TYPE;

/**
 * @struct iNEMO_sMATRIX_TYPE
 * @brief iNEMO Short Int Matrix Struct
 */


typedef struct
{
  short int m_nRow;      /*!< Number of Rows in the Matrix */ 
  short int m_nCol;      /*!< Number of Columns in the Matrix */
  short int **m_pData;   /*!< Pointer to Data in the Matrix */
} iNEMO_sMATRIX_TYPE;


/**
*@}
*/


/** @defgroup iNEMO_Math_Macro      iNEMO Math Macro
* @{
*/

/**
* @brief  Get the number of Rows in the Matrix
* @param  a: pointer to a Matrix in iNEMO_fMATRIX_TYPE or iNEMO_sMATRIX_TYPE format
* @retval a short int value 
*/
#define iNEMO_MatRow(a)	  (a->m_nRow)

/**
* @brief  Get the number of Column in the Matrix
* @param  a: pointer to a Matrix in iNEMO_fMATRIX_TYPE or iNEMO_sMATRIX_TYPE format
* @retval a short int value 
*/
#define iNEMO_MatCol(a)	  (a->m_nCol)

/**
* @brief  Get the Data in the Matrix
* @param  a: pointer to a Matrix in iNEMO_fMATRIX_TYPE or iNEMO_sMATRIX_TYPE format
* @retval a pointer to float or short int 
*/
#define iNEMO_MatData(a)  (a->m_pData)

/**
* @brief  Get the Roll Angle
* @param  a: pointer to iNEMO_EULER_ANGLES
* @retval a pointer to float
*/
#define iNEMO_Roll(a)     (a->m_fRoll)

/**
* @brief  Get the Pitch Angle
* @param  a: pointer to iNEMO_EULER_ANGLES
* @retval a pointer to float
*/
#define iNEMO_Pitch(a)    (a->m_fPitch)

/**
* @brief  Get the Yaw Angle
* @param  a: pointer to iNEMO_EULER_ANGLES
* @retval a pointer to float
*/
#define iNEMO_Yaw(a)      (a->m_fYaw)

/**
*@}
*/

/** @defgroup iNEMO_Math_Function_Prototypes      iNEMO Math Function Prototypes
* @{
*/


iNEMO_fMATRIX_TYPE* iNEMO_fMatCreate(int nRow, int nCol);
iNEMO_sMATRIX_TYPE* iNEMO_sMatCreate(int nRow, int nCol);
iNEMO_fMATRIX_TYPE* iNEMO_fMatCreateInit(int nRow, int nCol, int nType);
iNEMO_fMATRIX_TYPE* iNEMO_fMatFill(iNEMO_fMATRIX_TYPE *pMat, float fValue);
int iNEMO_fMatFree(iNEMO_fMATRIX_TYPE *pMat);
int iNEMO_sMatFree(iNEMO_sMATRIX_TYPE *pMat);
iNEMO_fMATRIX_TYPE* iNEMO_fMatCopy(iNEMO_fMATRIX_TYPE* pSource, iNEMO_fMATRIX_TYPE* pDest);

iNEMO_fMATRIX_TYPE* iNEMO_fMatAdd(iNEMO_fMATRIX_TYPE* pTerm1,
                                iNEMO_fMATRIX_TYPE* pTerm2,
                                iNEMO_fMATRIX_TYPE* pAdd);

iNEMO_fMATRIX_TYPE* iNEMO_fMatSub(iNEMO_fMATRIX_TYPE* pTerm1,
                                iNEMO_fMATRIX_TYPE* pTerm2,
                                iNEMO_fMATRIX_TYPE* pSub);

iNEMO_fMATRIX_TYPE* iNEMO_fMatMulMat(iNEMO_fMATRIX_TYPE* pTerm1,
                                   iNEMO_fMATRIX_TYPE* pTerm2,
                                   iNEMO_fMATRIX_TYPE* pMul);

iNEMO_fMATRIX_TYPE* iNEMO_fMatMulMatMT(iNEMO_fMATRIX_TYPE* pTerm1,
                                      iNEMO_fMATRIX_TYPE* pTerm2,
                                      iNEMO_fMATRIX_TYPE* pMul);

/*
iNEMO_fMATRIX_TYPE* iNEMO_fMatMulScal(iNEMO_fMATRIX_TYPE* pTerm,
                                    float fScalar,
                                    iNEMO_fMATRIX_TYPE* pMul);
*/

iNEMO_fMATRIX_TYPE* iNEMO_fMatInv(iNEMO_fMATRIX_TYPE* pSource,
                                iNEMO_fMATRIX_TYPE* pDest);

int iNEMO_MatLUP(iNEMO_fMATRIX_TYPE* pSourceDestLU, iNEMO_sMATRIX_TYPE* pPerm);

iNEMO_fMATRIX_TYPE* iNEMO_MatBackSubs(iNEMO_fMATRIX_TYPE* pSourceLU,
                                   iNEMO_fMATRIX_TYPE* pSourceDestColumn,
                                   iNEMO_sMATRIX_TYPE* pPerm,
                                   iNEMO_fMATRIX_TYPE* pDest,
                                   int iResultCol);

float iNEMO_WrapAround(float fInput);

/**
*@}
*/

/** @defgroup iNEMO_Math_Function_Macro      iNEMO Math Function Macro
* @{
*/

/**
* @brief  Create a Matrix filled of Zero
* @param  R : number of Rows
* @param  C : number of Columns
* @retval a pointer to iNEMO_fMATRIX_TYPE
*/
#define iNEMO_fMatCreateZero(R, C) (iNEMO_fMatCreateInit((R), (C), iNEMO_ZERO_MATRIX))

/**
* @brief  Create a Matrix filled of One
* @param  R : number of Rows
* @param  C : number of Columns
* @retval a pointer to iNEMO_fMATRIX_TYPE
*/
#define iNEMO_fMatCreateOne(R, C) (iNEMO_fMatCreateInit((R), (C), iNEMO_ONES_MATRIX))

/**
* @brief  Create an Identity Matrix
* @param  R : number of Rows
* @param  C : number of Columns
* @retval a pointer to iNEMO_fMATRIX_TYPE
*/
#define iNEMO_fMatCreateUnit(R, C) (iNEMO_fMatCreateInit((R), (C), iNEMO_IDEN_MATRIX))

#ifdef __cplusplus
}
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

#endif

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
