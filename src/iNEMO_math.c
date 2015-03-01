/**
*
* \file    iNEMO_math.c
* \author  ART Team IMS-Systems Lab
* \version V1.2.1 [FW v2.0.0]
* \date    13/09/2010
* \brief   Implementation file of the Math Library of iNEMO
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
********************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "iNEMO_math.h"
#include <stdlib.h>

/** @addtogroup iNEMO_Engine_Lite        
  * @{
  */

/**
 * @defgroup iNEMO_Math        iNEMO Math       
 * @{
 */

/** @defgroup iNEMO_Math_Function      iNEMO Math Function
* @{
*/


/**
********************************************************************************
* @brief Create an Empty Matrix
* @param  nRow : Number of Rows
* @param  nCol : Number of columns
* @retval Pointer to iNEMO_fMATRIX_TYPE
* @par Functions called:
* None
*/
iNEMO_fMATRIX_TYPE *iNEMO_fMatCreate(int nRow, int nCol)
{
  int i;

  iNEMO_fMATRIX_TYPE *pTmp = (iNEMO_fMATRIX_TYPE*)iNEMO_Malloc(sizeof(iNEMO_fMATRIX_TYPE));

  if (pTmp != NULL)
  {
       // Check on null values
    	if(nRow==0 || nCol==0)
		return(NULL);

	pTmp->m_nRow = nRow;
	pTmp->m_nCol = nCol;

	// Allocate memory for data
	iNEMO_MatData(pTmp) = iNEMO_Malloc(nRow * sizeof(float*));
	for (i=0; i < nRow; ++i)
		iNEMO_MatData(pTmp)[i] = iNEMO_Malloc(nCol * sizeof(float));
  }

  return pTmp;
}

/**
********************************************************************************
* @brief Create an Empty Matrix of Short Int
* @param  nRow : Number of Rows
* @param  nCol : Number of columns
* @retval Pointer to iNEMO_sMATRIX_TYPE
* @par Functions called:
* None
*/
iNEMO_sMATRIX_TYPE *iNEMO_sMatCreate(int nRow, int nCol)
{
  int i;
  iNEMO_sMATRIX_TYPE *pTmp = (iNEMO_sMATRIX_TYPE*)iNEMO_Malloc(sizeof(iNEMO_sMATRIX_TYPE));

  if (pTmp != NULL)
  {
       // Check on null values
    	if(nRow==0 || nCol==0)
		return(NULL);

	pTmp->m_nRow = nRow;
	pTmp->m_nCol = nCol;

	// Allocate memory for data
	iNEMO_MatData(pTmp) = iNEMO_Malloc(nRow * sizeof(short int*));
	for (i=0; i < nRow; ++i)
                iNEMO_MatData(pTmp)[i] = iNEMO_Malloc(nCol * sizeof(short int));
  }

  return pTmp;
}


/**
********************************************************************************
* @brief Create an Empty Matrix of Short Int
* @param  nRow  : Number of Rows
* @param  nCol  : Number of columns
* @param  nType : specifies as to fill the Matrix
* @retval Pointer to iNEMO_sMATRIX_TYPE
* @par Functions called:
* None
*/
iNEMO_fMATRIX_TYPE *iNEMO_fMatCreateInit(int nRow, int nCol, int nType)
{
  iNEMO_fMATRIX_TYPE* pTmp;
  int 	i, j;

  if(nRow==0 || nCol==0)
	return(NULL);

  if ((pTmp = iNEMO_fMatCreate( nRow, nCol )) != NULL)
  {
      switch (nType)
      {
        case iNEMO_ZERO_MATRIX:
        case iNEMO_IDEN_MATRIX:
        case iNEMO_ONES_MATRIX:
	  for (i=0; i < nRow; i++)
        for (j=0; j < nCol; j++)
	    {
          /* Fills of ones */
		  if (nType == iNEMO_ONES_MATRIX)
		  {
            iNEMO_MatData(pTmp)[i][j] = 1.0f;
		    continue;
          }
          /* Fills as Identity Matrix */
		  if (nType == iNEMO_IDEN_MATRIX)
		  {
		    if (i==j)
		    {
              iNEMO_MatData(pTmp)[i][j] = 1.0f;
		      continue;
		    }
		  }
                  /* Fills of zeros */
                  iNEMO_MatData(pTmp)[i][j] = 0.0f;
	      }
	  break;
      }
     return (pTmp);
  }
  else
     return (NULL);
}

/**
********************************************************************************
* @brief Fill the Matrix with a specific value
* @param  pMat : Pointer to iNEMO_fMATRIX_TYPE
* @param  fValue : the float value to fill the Matrix
* @retval Pointer to iNEMO_fMATRIX_TYPE
* @par Functions called:
* None
*/
iNEMO_fMATRIX_TYPE *iNEMO_fMatFill(iNEMO_fMATRIX_TYPE *pMat, float fValue)
{
  int 	i, j;

  for (i=0; i<iNEMO_MatRow(pMat); i++)
      for (j=0; j<iNEMO_MatCol(pMat); j++)
          iNEMO_MatData(pMat)[i][j] = (float) (fValue);
  return (pMat);
}


/**
********************************************************************************
* @brief Free the Matrix from Memory
* @param pMat : the Matrix to delete
* @retval 0 if NULL pMat is passed, 1 if Normal Exit
* @par Functions called:
* None
*/
int iNEMO_fMatFree(iNEMO_fMATRIX_TYPE *pMat)
{
	int i;

	if (pMat == NULL)
		return (0);
	for (i=0; i<iNEMO_MatRow(pMat); i++)
	{
		iNEMO_Free( iNEMO_MatData(pMat)[i] );
	}
	iNEMO_Free(iNEMO_MatData(pMat));

	iNEMO_Free( pMat );

	return (1);
}

/**
********************************************************************************
* @brief Free the Matrix from Memory
* @param pMat : the short int Matrix to delete
* @retval 0 if NULL pMat is passed, 1 if Normal Exit
* @par Functions called:
* None
*/
int iNEMO_sMatFree(iNEMO_sMATRIX_TYPE *pMat)
{
 int i;

  if (pMat == NULL)
     return (0);
  for (i=0; i<iNEMO_MatRow(pMat); i++)
  {
      iNEMO_Free( iNEMO_MatData(pMat)[i] );
  }
  iNEMO_Free(iNEMO_MatData(pMat));
  iNEMO_Free( pMat );
  return (1);
}

/**
********************************************************************************
* @brief Copy pSource to pDest
* @param pSource : the source Matrix 
* @param pDest : the destination Matrix 
* @retval the resulting Matrix 
* @par Functions called:
* None
*/
iNEMO_fMATRIX_TYPE* iNEMO_fMatCopy(iNEMO_fMATRIX_TYPE* pSource, iNEMO_fMATRIX_TYPE* pDest)
{
  int		i, j;

  // Check the dimensions
  if ( iNEMO_MatRow(pSource) != iNEMO_MatRow(pDest) ||
      iNEMO_MatCol(pSource) != iNEMO_MatCol(pDest) )
    return NULL;
  else
  {
    for (i=0; i < iNEMO_MatRow(pSource); i++)
	for (j=0; j < iNEMO_MatCol(pSource); j++)
	   iNEMO_MatData(pDest)[i][j] = iNEMO_MatData(pSource)[i][j];
    return(pDest);
  }
}


/**
*******************************************************************************
* @brief Add two Matrix iNEMO_fMATRIX_TYPE 
* @param pTerm1 : the first Addend source Matrix 
* @param pTerm2 : the second Addend source Matrix 
* @param pAdd   : the resulting Matrix
* @retval the resulting Matrix 
* @par Functions called:
* None
*/
iNEMO_fMATRIX_TYPE* iNEMO_fMatAdd(iNEMO_fMATRIX_TYPE* pTerm1,
                                  iNEMO_fMATRIX_TYPE* pTerm2,
                                  iNEMO_fMATRIX_TYPE* pAdd)
{
   int	i, j;

   /* Check if dimensions are wrong */
   if ( iNEMO_MatRow(pAdd) != iNEMO_MatRow(pTerm1) || iNEMO_MatCol(pAdd) != iNEMO_MatCol(pTerm2) )
     return NULL;
   else
   {
	for (i=0; i < iNEMO_MatRow(pTerm1); ++i)
	    for (j=0; j < iNEMO_MatCol(pTerm1); ++j)
		iNEMO_MatData(pAdd)[i][j] = iNEMO_MatData(pTerm1)[i][j] +
                  iNEMO_MatData(pTerm2)[i][j];
   }
   return(pAdd);
}

/**
*******************************************************************************
* @brief Sub two Matrix iNEMO_fMATRIX_TYPE 
* @param pTerm1 : the first source Matrix 
* @param pTerm2 : the second source Matrix 
* @param pSub   : the resulting Matrix
* @retval the resulting Matrix 
* @par Functions called:
* None
*/
iNEMO_fMATRIX_TYPE* iNEMO_fMatSub(iNEMO_fMATRIX_TYPE* pTerm1,
                                  iNEMO_fMATRIX_TYPE* pTerm2,
                                  iNEMO_fMATRIX_TYPE* pSub)
{
  int	i, j;

  /* Check if dimensions are wrong */
  if ( iNEMO_MatRow(pTerm1) != iNEMO_MatRow(pSub) ||
      iNEMO_MatCol(pTerm1) != iNEMO_MatCol(pSub) )
    return NULL;
  else
  {
    for (i=0; i < iNEMO_MatRow(pTerm1); ++i)
	for (j=0; j < iNEMO_MatCol(pTerm1); ++j)
		iNEMO_MatData(pSub)[i][j] = iNEMO_MatData(pTerm1)[i][j] -
                  iNEMO_MatData(pTerm2)[i][j];
   }
   return(pSub);

}

/**
********************************************************************************
* @brief Multiply two Matrixes
* @param  pTerm1 : the first source Matrix
* @param  pTerm2 : the second source Matrix
* @param  pMul   : the destination Matrix
* @retval the resulting Matrix
* @par Functions called:
* None
*/
iNEMO_fMATRIX_TYPE* iNEMO_fMatMulMat(iNEMO_fMATRIX_TYPE* pTerm1,
                                     iNEMO_fMATRIX_TYPE* pTerm2,
                                     iNEMO_fMATRIX_TYPE* pMul)
{
   int	i, j, k;

  // if dimensions are wrong
  if ( iNEMO_MatRow(pMul) != iNEMO_MatRow(pTerm1) ||
      iNEMO_MatCol(pMul) != iNEMO_MatCol(pTerm2) )
    return NULL;
  else
  {
    for (i=0; i < iNEMO_MatRow(pTerm1); i++)
	for (j=0; j < iNEMO_MatCol(pTerm2); j++)
	    for (k=0, iNEMO_MatData(pMul)[i][j]=0.0; k < iNEMO_MatCol(pTerm1); k++)
               iNEMO_MatData(pMul)[i][j] += iNEMO_MatData(pTerm1)[i][k]
                 * iNEMO_MatData(pTerm2)[k][j];
  }
  return(pMul);
}

/**
********************************************************************************
* @brief Multiply one Matrix for the Transpose of another one
* @param pTerm1 : the first source Matrix
* @param pTerm2 : the second source Matrix (MT)
* @param pMul : the destination Matrix
* @retval the resulting Matrix
* @par Functions called:
* None
*/
iNEMO_fMATRIX_TYPE* iNEMO_fMatMulMatMT(iNEMO_fMATRIX_TYPE* pTerm1,
                                       iNEMO_fMATRIX_TYPE* pTerm2,
                                       iNEMO_fMATRIX_TYPE* pMul)
{
   int	i, j, k;

  // if dimensions are wrong
  if ( iNEMO_MatRow(pMul) != iNEMO_MatRow(pTerm1) ||
      iNEMO_MatCol(pMul) != iNEMO_MatRow(pTerm2) )
    return NULL;
  else
  {
    for (i=0; i < iNEMO_MatRow(pTerm1); i++)
	for (j=0; j < iNEMO_MatRow(pTerm2); j++)
	    for (k=0, iNEMO_MatData(pMul)[i][j]=0.0; k < iNEMO_MatCol(pTerm1); k++)
               iNEMO_MatData(pMul)[i][j] += iNEMO_MatData(pTerm1)[i][k]
                 * iNEMO_MatData(pTerm2)[j][k];
  }
  return(pMul);
}


/**
********************************************************************************
* @brief Find Inverse of a Matrix
* @param pSource : the source Matrix
* @param pDest   : square inverse matrix of pSource, NULL in case of fail 
* @retval the resulting Matrix
* @par Functions called:
* @ref iNEMO_fMatCreate
* @ref iNEMO_fMatCopy
* @ref iNEMO_MatLUP 
* @ref iNEMO_fMatFree 
*/
iNEMO_fMATRIX_TYPE* iNEMO_fMatInv(iNEMO_fMATRIX_TYPE* pSource,
                                iNEMO_fMATRIX_TYPE* pDest)
{
  iNEMO_fMATRIX_TYPE* A;
  iNEMO_fMATRIX_TYPE* B;
  iNEMO_sMATRIX_TYPE* P;
  int i, nCol, nRow;

  nCol = iNEMO_MatCol(pSource);
  nRow = iNEMO_MatRow(pSource);

  if (nCol != nRow)
    /* The matrix is not Square */
    return (NULL);

  A = iNEMO_fMatCreate(nRow, nCol);
  if (A == NULL)
    return (NULL);

  B = iNEMO_fMatCreate(nRow, nCol);
  if (B == NULL)
    return (NULL);

  /* P is a vector matrix */
  P = iNEMO_sMatCreate(nRow, 1);
  if (P == NULL)
    return (NULL);

  /* It is to avoid to modify pSource Matrix */
  iNEMO_fMatCopy(pSource, A);

  /* LU Decomposition and check for Singular Matrix */
  if (iNEMO_MatLUP(A, P) == -1)
  {
    iNEMO_fMatFree(A);
    iNEMO_fMatFree(B);
    iNEMO_sMatFree(P);

    return (NULL);
  }

  for (i=0; i<nCol; ++i)
  {
    iNEMO_fMatFill(B, 0.0f);

    iNEMO_MatData(B)[i][0] = 1.0f;
    iNEMO_MatBackSubs(A, B, P, pDest, i);
  }
  iNEMO_fMatFree(A);
  iNEMO_fMatFree(B);
  iNEMO_sMatFree(P);

  if (pDest == NULL)
  {
    return(NULL);
  }
  else
  {
    return (pDest);
  }

}

/**
********************************************************************************
* @brief LU decomposition with partial pivoting
* @param pSourceDestLU : the source and destination Matrix
* @param pPerm         : the destination permutation Matrix 
* @retval the number of permutation performed, -1 means suspected singular matrix
* @par Functions called:
* None
*/

/*******************************************************************************
* Algorithm:
*     Given a squared input matrix (n x n)
*      the algorithm returns a modified matrix and
*            a permutation vector P (n x 1)
*
* --------------------------------------------------------------------------- */
int iNEMO_MatLUP(iNEMO_fMATRIX_TYPE* pSourceDestLU, iNEMO_sMATRIX_TYPE* pPerm)
{
  int	i, j, k, iC;
  int	iMax;
  int	retNumPerm = 0;
  short int sTmp;
  float fP1, fP2; /* Pivot Variables */

  iC = iNEMO_MatCol(pSourceDestLU);

  for (i=0; i < iC; ++i)
    iNEMO_MatData(pPerm)[i][0] = (short int) (i);

  /* Partial Pivoting */
  for (k=0; k < iC; ++k)
  {
    for (i=k, iMax=k, fP1=0.0f; i < iC; ++i)
    {
      /* Local ABS */
      if (iNEMO_MatData(pSourceDestLU)[iNEMO_MatData(pPerm)[i][0]][k] > 0)
        fP2 = iNEMO_MatData(pSourceDestLU)[iNEMO_MatData(pPerm)[i][0]][k];
      else
        fP2 = - iNEMO_MatData(pSourceDestLU)[iNEMO_MatData(pPerm)[i][0]][k];
      if (fP2 > fP1)
      {
        fP1 = fP2;
        iMax = i;
      }
    }
    /* Row exchange, update permutation vector */
    if (k != iMax)
    {
      retNumPerm++;
      sTmp = iNEMO_MatData(pPerm)[k][0];
      iNEMO_MatData(pPerm)[k][0] = iNEMO_MatData(pPerm)[iMax][0];
      iNEMO_MatData(pPerm)[iMax][0] = sTmp;
    }

    /* Suspected Singular Matrix */
    if (iNEMO_MatData(pSourceDestLU)[iNEMO_MatData(pPerm)[k][0]][k] == 0.0f)
      return (-1);

    for (i=k+1; i < iC; ++i)
    {
      /* Calculate Mat [i][j] */
      iNEMO_MatData(pSourceDestLU)[iNEMO_MatData(pPerm)[i][0]][k] =
        iNEMO_MatData(pSourceDestLU)[iNEMO_MatData(pPerm)[i][0]][k] /
                      iNEMO_MatData(pSourceDestLU)[iNEMO_MatData(pPerm)[k][0]][k];

      /* Elimination */
      for (j=k+1; j < iC; ++j)
      {
        iNEMO_MatData(pSourceDestLU)[iNEMO_MatData(pPerm)[i][0]][j] -=
          iNEMO_MatData(pSourceDestLU)[iNEMO_MatData(pPerm)[i][0]][k] *
                        iNEMO_MatData(pSourceDestLU)[iNEMO_MatData(pPerm)[k][0]][j];
      }


    }
  }
  return (retNumPerm);
}

/**
********************************************************************************
* @brief Back Substitution
* @param pSourceLU : the source Matrix (already LU composite)
* @param pSourceDestColumn : the column Matrix
* @param pPerm : the Permutation Vector
* @param pDest : the Destination Matrix
* @param iResultCol : the column of pSourceDestColumn
* @retval Pointer to iNEMO_fMATRIX_TYPE
* @par Functions called:
* None
*/

iNEMO_fMATRIX_TYPE* iNEMO_MatBackSubs(iNEMO_fMATRIX_TYPE* pSourceLU,
                                   iNEMO_fMATRIX_TYPE* pSourceDestColumn,
                                   iNEMO_sMATRIX_TYPE* pPerm,
                                   iNEMO_fMATRIX_TYPE* pDest,
                                   int iResultCol)
{
  int i, j, k, iC;
  float fSum, fTmp;

  iC = iNEMO_MatCol(pSourceLU);

  for (k=0; k < iC; ++k)
    for (i=k+1; i < iC; ++i)
      iNEMO_MatData(pSourceDestColumn)[iNEMO_MatData(pPerm)[i][0]][0] -=
        iNEMO_MatData(pSourceLU)[iNEMO_MatData(pPerm)[i][0]][k] *
          iNEMO_MatData(pSourceDestColumn)[iNEMO_MatData(pPerm)[k][0]][0];

  iNEMO_MatData(pDest)[iC-1][iResultCol] =
    iNEMO_MatData(pSourceDestColumn)[iNEMO_MatData(pPerm)[iC-1][0]][0] /
       iNEMO_MatData(pSourceLU)[iNEMO_MatData(pPerm)[iC-1][0]][iC-1];


  for (k=iC-2; k >= 0; k--)
  {
    fSum = 0.0f;

    for (j=k+1; j < iC; ++j)
      fSum += iNEMO_MatData(pSourceLU)[iNEMO_MatData(pPerm)[k][0]][j] *
        iNEMO_MatData(pDest)[j][iResultCol];

    fTmp = iNEMO_MatData(pSourceDestColumn)[iNEMO_MatData(pPerm)[k][0]][0] - fSum;
    iNEMO_MatData(pDest)[k][iResultCol] = fTmp /
      iNEMO_MatData(pSourceLU)[iNEMO_MatData(pPerm)[k][0]][k];
  }

  return pDest;

}

/**
********************************************************************************
* @brief cover the +/-180 rotation
* @param fInput : the input m_angle
* @retval the resulting m_angle
* @par Functions called:
* None
*/
float iNEMO_WrapAround(float fInput)
{
  if (fInput > PI)
    fInput -= PI2;
  if (fInput < -PI)
    fInput += PI2;
  return (fInput);
}


/**
*@}
*/

/**
*@}
*/

/**
*@}
*/

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
