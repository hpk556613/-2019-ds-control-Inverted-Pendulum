#include "motor.h"


/**
  * @brief  ���������ʼ��
  * @param  ��
  * @retval ��
  */
void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
//		AIN1=0;
//		AIN2=1;
}


/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=99;    //===PWM������100  ������99
	  if(Moto<-Amplitude) Moto=-Amplitude;	
		if(Moto>Amplitude)  Moto=Amplitude;		
		
		int Amplitude2=1040;    //===�����һȦ��1080�����岨
		int circle=0;
			
//		if(Encoder_Balance<0)
//		{
//			Encoder_Balance%=1040;
//			Encoder_Balance+=1040;
//		}
//		if((Encoder_Balance>=Amplitude2)&&(Encoder_Balance<(Amplitude2*2)))
//		{
//			circle=Encoder_Balance/1040;
//			Encoder_Balance-=circle*1040;	
//			Encoder_Balance=-Encoder_Balance+1040;
//		}	
}


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto)
{
//    	if(moto<0)		{	AIN2=0;	 	AIN1=1;}
//			else 	        { AIN2=1;		AIN1=0;}
			PWMA=myabs(moto);
}



/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


