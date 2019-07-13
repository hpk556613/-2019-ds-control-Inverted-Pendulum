#include "motor.h"


/**
  * @brief  电机驱动初始化
  * @param  无
  * @retval 无
  */
void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
//		AIN1=0;
//		AIN2=1;
}


/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=99;    //===PWM满幅是100  限制在99
	  if(Moto<-Amplitude) Moto=-Amplitude;	
		if(Moto>Amplitude)  Moto=Amplitude;		
		
		int Amplitude2=1040;    //===电机跑一圈是1080个脉冲波
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
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto)
{
//    	if(moto<0)		{	AIN2=0;	 	AIN1=1;}
//			else 	        { AIN2=1;		AIN1=0;}
			PWMA=myabs(moto);
}



/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


