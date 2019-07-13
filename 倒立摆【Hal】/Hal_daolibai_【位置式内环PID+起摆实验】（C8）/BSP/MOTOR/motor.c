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
int Xianfu_Pwm(int Moto)
{	
	  int Amplitude=99;    //===PWM满幅是100  限制在99
	  if(Moto<-Amplitude) Moto=-Amplitude;	
		if(Moto>Amplitude)  Moto=Amplitude;		
		return Moto;
}


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto)
{
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


