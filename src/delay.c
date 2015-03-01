//file:
#include "stm32f10x.h"

#if 1

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(vu32 nCount)
{
         for(; nCount != 0; nCount--);
}

/*******************************************************************************
* Function Name  : Delaynus
* Description    : Inserts a delay time abort nus.
* Input          :nus
* Output         : None
* Return         : None
*******************************************************************************/
void Delaynus(vu32 nus) 
{
        u8 nCount ;
        while(nus--)
        {
                for(nCount = 6 ; nCount != 0; nCount--);
        }
}

#endif //#if DELAY_EN>0