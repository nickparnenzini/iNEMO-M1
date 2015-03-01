/****************************************************************************/
/****************************************************************************/
/* author: Valeria Parnenzini                                               */
/****************************************************************************/
/****************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "L3Gx.h"
#include "stm32f10x_usart.h"
#include "LSM303DLHC.h"
#include "iNEMO_Compass.h"
#include <math.h>
#include "delay.h"
#include "MadgwickAHRS.h"

/* macros used to send data via serial communication */
#define GYRO 1
#define ACC 2
#define MAG 3
#define RPH 4
#define QUAT 5
#define MAG_ORIG 6

/* macros used for Madwick filter algorithm */
#define GYRO_FACT 0.0174
#define ACC_FACT 107.0


void GPIO_Configuration(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void RCC_Configuration(void);

/* arrays for magnetometer calibration */
float pfGain[3], pfOffset[3];

/* Accelerometer sensor init structure */
LSMAccInit LSMAccInitStructure;

/*Accelerometer high pass filter init structure */
LSMAccFilterInit LSMAccFilterInitStructure;

/* Magnetometer sensor init structure */
LSMMagInit LSMMagInitStructure;

/* Acceletometer DataReady flag */
FlagStatus xAccDataReady = RESET;

/* Gyro DataReady flag */
FlagStatus xGyroDataReady = RESET;

/* Acceleration and Magnetic field values */
float fAccXYZ[3], fMagXZY[3];

/* Temperature sensor filed value */
float fTemperature;

/* Accelerometer status */
LSMADataStatus xAccStatus;

/* Magnetometer status */
LSMMDataStatus xMagStatus;

/* Gyroscopic sensor init structure */
L3GInit L3GInitStructure;

/* Gyroscope data */
float fGyroXYZ[3];

/* Roll, Pitch and Heading data for Tilted Compass algorithm */
float pfRPH[3];

/* variable used for quaternion initialization */
int init_quat;  

/**
 * @brief  This configures the Exit line associated with LSM303DLH data ready line.
 * @param  None
 * @retval None
 */
void ExtiConfigurationAcc(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;

  EXTI_Init(&EXTI_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line5);

  /* Generate software interrupt: simulate a rising edge applied on  EXTI line 5*/
  EXTI_GenerateSWInterrupt(EXTI_Line5);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}


void ExtiConfigurationGyro(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);

  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;

  EXTI_Init(&EXTI_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line6);

  /* Generate software interrupt: simulate a rising edge applied on  EXTI line 6*/
   EXTI_GenerateSWInterrupt(EXTI_Line6);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

void PwmConfig(void)
{

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 24000000) - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC2Init(TIM2, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}


/**
 * @brief  This function handles External interrupt request (associated with LSM303DLHC data ready line).
 * @param  None
 * @retval None
 */
void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
    /* set the DataReady flag */
    xAccDataReady = SET;

    /* Clear the pending bit */
    EXTI_ClearITPendingBit(EXTI_Line5);
  }

  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
        /* set the DataReady flag */
        xGyroDataReady = SET;

        /* Clear the pending bit */
        EXTI_ClearITPendingBit(EXTI_Line6);
   }

}

   /**
    * @brief This function is used for number conversion before sending it via serial communication 
    * @param float number
    * @retval uint32 number to send via serial communication
    */
  uint32_t convert_num(float num)
  {
  	  /* number representation */
      uint32_t rappr = 0;

      /* numeber is multiplied by 1000 to have more decimal values */
      num = 1000.0 * num;

      /* conversion from float to uint32 */
      if(num < 0)
      {
      	rappr = pow(2,32) + num;

      }
      else
      {
  		  rappr = num;
  	  }

      rappr = (uint32_t)rappr;

      return rappr;

  }

  /**
  * @brief Function used to initialize quaternions for Madgwick filter 
  * @param 
  * @retval None
  */
  void quat_initialization()
  {
      /* XYZ rotation reference 
         heading angle from TiltedCompass algorithm is used 
         pfRPH[2] is heading angle */

      /* roll angle */  
    	float phi = pfRPH[0]; 
      /* pitch angle*/
    	float theta = pfRPH[1]; 
      /* heading angle */
    	float psi = pfRPH[2]; 

    	/* angles are expressed in radians */
    	/* we suppose a XYZ rotation */
    	/* quaternion q computation: q= q0 + iq1 + jq2 + kq3 */

    	q0 = - sin(phi/2.0)*sin(theta/2.0)*sin(psi/2.0) + cos(phi/2.0)*cos(theta/2.0)*cos(psi/2.0);
    	q1 = sin(phi/2.0)*cos(theta/2.0)*cos(psi/2.0) + sin(theta/2.0)*sin(psi/2.0)*cos(phi/2.0);
    	q2 = -sin(phi/2.0)*sin(psi/2.0)*cos(theta/2.0) + sin(theta/2.0)*cos(phi/2.0)*cos(psi/2.0);
    	q3 = sin(phi/2.0)*sin(theta/2.0)*cos(psi/2.0) + sin(psi/2.0)*cos(phi/2.0)*cos(theta/2.0);

  }


int main(void)
{

        /*variables used for bias computation */
        int init = 0;
        float bx = 0;
        float by = 0;
        float bz = 0;

        /*indexes used for loops */
        int i = 0;
        int n = 3;

        /* index variable used to send data via serial communication */
        int index = GYRO;

        /* structure used to send data via serial communication */
        USART_InitTypeDef USART_InitStructure;

        /* array used to send data via serial communication */
        uint32_t val_int[4];

        uint8_t data_mr = 0x00;
        uint8_t val = 0x00;

        /* gyro array for Madwick filter */
        float gyro[3];

        /* accelerometer array for Madgwick filter */
        float acc[3];

        /* magnetometer array for Madgwick filter */
        float mag[3];

        /* vector used for original magnetometer data (not calibrated) */
        float orig_mag[3];

        /* quaternion array */
        float quat[4];

        init_quat = 0;

        /* global variables initialization to calibrate magnetometer */
        iNEMO_MagSensorCalibrationInit();

        /*************************************************************************/
        /*           MAGNETOMETER AND ACCELEROMETER CONFIGURATION                */
        /*************************************************************************/ 

        /* I2C bus deinitialization */
        I2C_DeInit(I2C2);

        /* I2C bus must be free before starting communication with accelerometer */
        I2C_ClearFlag(I2C2, I2C_FLAG_BUSY);

       /* Initialize the MCU digital interface to communicate with the sensor */
        Lsm303dlhcI2CInit();

        /* Fill the accelerometer structure */
        LSMAccInitStructure.xPowerMode = LSM_NORMAL_MODE;
        LSMAccInitStructure.xOutputDataRate =   LSM_ODR_100_HZ;
        LSMAccInitStructure.xEnabledAxes= LSM_ALL_AXES_EN;
        LSMAccInitStructure.xFullScale = LSM_FS_2G;
        LSMAccInitStructure.xDataUpdate = LSM_BLOCK_UPDATE;
        LSMAccInitStructure.xEndianness = LSM_BIG_ENDIAN;
        LSMAccInitStructure.xHighResolution=LSM_ENABLE;

        /* Fill the accelerometer LPF structure */
        LSMAccFilterInitStructure.xHPF=LSM_DISABLE;
        LSMAccFilterInitStructure.xHPF_Mode=LSM_HPFM_NORMAL;
        LSMAccFilterInitStructure.cHPFReference=0x00;
        LSMAccFilterInitStructure.xHPFCutOff=LSM_HPCF_16;
        LSMAccFilterInitStructure.xHPFClick=LSM_DISABLE;
        LSMAccFilterInitStructure.xHPFAOI2=LSM_DISABLE;
        LSMAccFilterInitStructure.xHPFAOI1=LSM_DISABLE;

        /* Fill the magnetometer structure */
        LSMMagInitStructure.xOutputDataRate = LSM_ODR_30_HZ;
        LSMMagInitStructure.xFullScale = LSM_FS_1_3_GA;
        LSMMagInitStructure.xWorkingMode = LSM_CONTINUOS_CONVERSION;
        LSMMagInitStructure.xTemperatureSensor = LSM_ENABLE ;

        /* External Interrupts configuration */
        ExtiConfigurationAcc();

        /* Configure the sensor IRQ */
        Lsm303dlhcAccIrq1Config(LSM_I1_DRDY1, LSM_ENABLE);

        /* Configure the accelerometer main parameters */
        Lsm303dlhcAccConfig(&LSMAccInitStructure);

         /* Configure the accelerometer LPF main parameters */
         Lsm303dlhcAccFilterConfig(&LSMAccFilterInitStructure);

         /* Configure the magnetometer main parameters */
         Lsm303dlhcMagConfig(&LSMMagInitStructure);

        /*****************************************************************/
        /*                    GYRO                                       */
        /*****************************************************************/

         /* Initialize the MCU digital interface to communicate with the sensor */
         L3gd20CommInit();

         /* Fill the gyro structure */
         L3GInitStructure.xPowerMode = L3G_NORMAL_SLEEP_MODE;
         L3GInitStructure.xOutputDataRate = L3G_ODR_200_HZ_CUTOFF_25; //L3G_ODR_380_HZ_CUTOFF_20  ;
         L3GInitStructure.xEnabledAxes = L3G_ALL_AXES_EN;
         L3GInitStructure.xFullScale = L3G_FS_500_DPS;
         L3GInitStructure.xDataUpdate = L3G_BLOCK_UPDATE;
         L3GInitStructure.xEndianness = L3G_BIG_ENDIAN;

         /* Configure the gyro main parameters */
         L3gd20Config(&L3GInitStructure);

         /* Configure the MCU exti */
         ExtiConfigurationGyro();

         /* Configure the sensor data ready */
         L3gd20Irq2Config(L3G_I2_DRDY, L3G_ENABLE);

         /******************************************************/
         /*              USART CONFIGURATION                   */
         /******************************************************/

      	 /* System Clocks Configuration */
      	 RCC_Configuration();
      	 /* Configure the GPIO ports */
      	 GPIO_Configuration();

      	 /*USART1 Initialization*/
      	 USART_InitStructure.USART_BaudRate = 256000;
      	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      	 USART_InitStructure.USART_StopBits = USART_StopBits_1;
      	 USART_InitStructure.USART_Parity = USART_Parity_No;
      	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      	 USART_InitStructure.USART_Mode = USART_Mode_Tx| USART_Mode_Rx;

      	 USART_DeInit(USART1);

      	 /* Configure the USART1 */
      	 USART_Init(USART1, &USART_InitStructure);

      	 /* Enable USART1 to receive interrupt */
      	 USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);

      	 /* Enable the USART1 */
      	 USART_Cmd(USART1, ENABLE);


    	   /******************************************************/
    	   /*            MAGNETOMETER OPERATIONS                 */
    	   /******************************************************/

         /* write in Magnetometer Status Register */
    	   Lsm303dlhcMagI2CByteWrite( &val, LSM_M_SR_REG_ADDR);

         /* Continuous conversion setting for magnetometer */
    	   data_mr = LSM_CONTINUOS_CONVERSION;



  		   while(1)
  		  {
                 
                  /* executed the first time */
        			    if (!init)  
        			   	{

            			   		/* Bias Computation */
                        
                        /* counter variable */
            			   		int cont;

            			   		for (cont = 0; cont <= 1000; cont++)
            			   		{
                              /* read data from gyro */
                			   			L3gd20ReadAngRate(fGyroXYZ);

                              /* bias update */
                			   			bx = bx + fGyroXYZ[0];
                			   			by = by + fGyroXYZ[1];
                			   			bz = bz + fGyroXYZ[2];

                              /* bias computed for 1000 iterations */
                			   			if (cont == 1000)
                			   			{
                  			   				bx = bx/1000;
                  			   				by = by/1000;
                  			   				bz = bz/1000;
                			   			}
            			   		}

            			   		init = 1;

        			   	}
        			    else
        			    {
                         /* gyro data correction */
              				   fGyroXYZ[0] = fGyroXYZ[0] - bx;
              				   fGyroXYZ[1] = fGyroXYZ[1] - by;
              				   fGyroXYZ[2] = fGyroXYZ[2] - bz;


              				   /******************************************************/
              				   /*              MAGNETOMETER CONFIGURATION            */
              				   /******************************************************/

              				   /* each step we must change magnetometer configuration (otherwise it does not work) */
              				   if(data_mr == LSM_CONTINUOS_CONVERSION)
              				   {
              						    data_mr = LSM_SINGLE_CONVERSION;
              					 }
              					 else
              					 {
              						    data_mr = LSM_CONTINUOS_CONVERSION;
              					 }

        					       /* Configure the magnetometer MR register */
        					       Lsm303dlhcMagI2CByteWrite( &data_mr, LSM_M_MR_REG_ADDR);

        					       /*************************************************/
        					       /*          READ DATA FROM ACCELEROMETER         */
        					       /*************************************************/

        					       /* Wait for data ready (set by the proper ISR) */
        					       while(!xAccDataReady);
        					       xAccDataReady = RESET;

        					       /* read data from accelerometer */
        					       Lsm303dlhcAccReadAcc(fAccXYZ);

        					       /**********************************************/
        					       /*        READ DATA FROM MAGNETOMETER         */
        					       /**********************************************/

        					       /* read data from magnetometer */
        					       Lsm303dlhcMagReadMag(fMagXZY);

        					       /* save original data from magnetometer, to compare them with calibrated ones */
        					       for(i=0; i<3; i++)
        					       {
        						          orig_mag[i] = fMagXZY[i];
      					         }

        					       /* read data from temperature sensor */
        					       fTemperature = Lsm303dlhcMagReadTemp();


        					       /*************************************************/
        					       /*          READ DATA FROM GYRO                  */
        					       /*************************************************/

                         /* Wait for data ready (set by the proper ISR) */
        					       while(!xGyroDataReady);
        					       xGyroDataReady = RESET;

        					       /* Read data from gyroscope */
        					       L3gd20ReadAngRate(fGyroXYZ);

        					       /************************************************/
        					       /*            TILTED COMPASS                    */
        					       /************************************************/

        					       /* magnetometer calibration, with pfGain and pfOffset update */
        					       iNEMO_MagSensorCalibrationRun(fMagXZY, pfGain, pfOffset );

        				   	     for(i=0; i<3; i++)
        					       {
        						          fMagXZY[i] = (fMagXZY[i] - pfOffset[i])*pfGain[i];
        					       }
        
        					       /* Tilted Compass algorithm for roll, pitch and yaw angles */
        					       iNEMO_TiltedCompass(fMagXZY, fAccXYZ, pfRPH);

        					       /*****************************************************/
        					       /*              AHRS  update                         */
        					       /*****************************************************/

        					       /* gyro measurements are in degrees per second (dps) *
        					         and must be converted in rad/s */

        					       for(i =0; i<3; i++)
        					       {
        						          gyro[i] = fGyroXYZ[i]*GYRO_FACT;
        					            acc[i] =  fAccXYZ[i]/ACC_FACT ;
        						          mag[i] = fMagXZY[i]; 
        					       }

                         /* quaternion initialization */
        					       /* values to initialize quaternion: angles computed during 2nd iteration*/
        					       if(init_quat == 1)
        					       {
                                quat_initialization();
        					       }

                         /* init_quat is 1 in the 1st iteration */
        					       init_quat++;

        					       /* Madgwick filter algorithm for attitude estimation */
        					       /* angles estimation after quaternion initialization */
        					       if(init_quat > 1)
        					       {
        			               MadgwickAHRSupdate( -gyro[1], gyro[0], gyro[2], acc[0], acc[1], acc[2] , mag[0], mag[1], mag[2]);
        					       }

        					       /* quat is an auxiliar array used to save quaternion data */
        					       quat[0] = q0;
        					       quat[1] = q1;
        					       quat[2] = q2;
        					       quat[3] = q3;


        			   }//end else

				         /************************************************/
				         /*                SEND  DATA                    */
				         /************************************************/

                 /* 'index' is used as a reference to different data */
                 while(index <= 6)
                 {
                	   switch(index)
                	   {
                	         /* gyro data */
                	         case GYRO:
                  	        	  n = 3;
                                for(i = 0; i<n; i++)
                                {
                                	  val_int[i] = convert_num(fGyroXYZ[i]);
                                }
                                break;

                           /* acc data */
                	         case ACC:
                  	        	  n = 3;
                  	        	  for(i = 0; i<n; i++)
                  	        	  {
                	        		       val_int[i] = convert_num(fAccXYZ[i]);
                  	        	  }
                	              break;

                	         /* magnetometer calibrated data */
                	         case MAG:
                  	        	 n = 3;
                  	        	 for(i=0; i<n; i++)
                  	        	 {
                  	        		     val_int[i] = convert_num(fMagXZY[i]);
                  	        	  }
                	             break;

                	         /* Roll, Pitch and Heading data */
                	         case RPH:
                  	        	 n = 3;
                  	        	 for(i=0; i<n; i++)
                  	        	 {
                  	        		     val_int[i] = convert_num(pfRPH[i]);
                  	        	 }
                	             break;

                	         /* quaternion data */
                	         case QUAT:
                  	        	 n = 4;
                  	        	 for(i=0; i<n; i++)
                  	        	 {
                  	        		     val_int[i] = convert_num(quat[i]);
                  	        	 }
                               break;

                             /* magnetometer data (not calibrated) */
                	         case MAG_ORIG:
                  	        	 n = 3;
                  	        	 for(i=0; i<n; i++)
                  	        	 {
                  	        		     val_int[i] = convert_num(orig_mag[i]);
                  	        	 }
                	             break;

                	         default: break;

                	  }

                	   /* send data via Serial Communication */
                	  for (i=0; i<n; i++)
                	  {
                	   	     while( !(USART1->SR & 0x00000040) );
                	   	     USART_SendData(USART1, (uint16_t)(val_int[i] >> 24 ));

                	   	     while( !(USART1->SR & 0x00000040) );
                	   	     USART_SendData(USART1, (uint16_t)(val_int[i] >> 16 ));

                	   	     while( !(USART1->SR & 0x00000040) );
                	   	     USART_SendData(USART1, (uint16_t)(val_int[i] >> 8));

                	   	     while( !(USART1->SR & 0x00000040) );
                	   	     USART_SendData(USART1, (uint16_t)(val_int[i]));

                	   }

                     /* increase index value */
                	   index++;

                 }//end(while(index<=6))

                 /* reset index value */
				         index = GYRO;

		  }//end while(1)

}//end main




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}

#endif


void RCC_Configuration(void)
{

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);

}



void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure1, GPIO_InitStructure2;

  /* Configure USART Rx as input floating */
  GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure1);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure2.GPIO_Pin =GPIO_Pin_9;
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure2);



}
