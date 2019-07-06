#include "MiniBalance.h"
#include "math.h"
#include "led.h"
#include "mpu6050.h"
#define PI 3.14159265
/**************************************************************************
作者：Mini Balance 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/

/**************************************************************************
函数功能：5MS定时中断函数 5MS控制周期
入口参数：无
返回  值：无
作    者：Mini Balance
**************************************************************************/
int zhongzhi=1750;
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
void TIM1_UP_TIM16_IRQHandler(void)  
{    
	if(TIM1->SR&0X0001)//5ms定时中断
	{   
		  TIM1->SR&=~(1<<0);                                       //===清除定时器1中断标志位		 
			readEncoder();                                           //===读取编码器的值
			adc=Get_Adc(0);
  		Led_Flash(400);                                          //===LED闪烁;	
  		Get_battery_volt();                                      //===获取电池电压	          
			key(100);                                                 //===扫描按键状态
 		  Balance_Pwm=balance(adc,zhongzhi);
		  Velocity_Pwm=velocity(Encoder_Left);
 		  Moto1=Balance_Pwm-Velocity_Pwm;                 //===计算左轮电机最终PWM
   		Xianfu_Pwm();                                            //===PWM限幅
     if(Turn_Off(adc,Voltage)==0)                   //===如果不存在异常
 			Set_Pwm(Moto1);                                    //===赋值给PWM寄存器    		
	}       
} 

/**************************************************************************
函数功能：直立PID控制
入口参数：角度、角速度
返回  值：直立控制PWM
作    者：Mini Balance
**************************************************************************/
int balance(int adc, int target)
{  
	 static int Last_Bias;
	 int balance,Bias;
	 Bias=adc-target;                                          //===求出平衡的角度中值 和机械相关
	 balance=15*Bias+50*(Bias-Last_Bias);                              //===计算平衡控制的电机PWM
	 Last_Bias=Bias;
	 return balance;
}

/**************************************************************************
函数功能：速度PI控制
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
作    者：Mini Balance
**************************************************************************/
int velocity(int encoder_left)
{  
	  static int Velocity,Encoder_Least,Encoder,Movement;
	  static long Encoder_Integral;
	  //=============遥控前进后退部分=======================//
		if(1==Flag_Qian)	Movement=-900;	             //===如果前进标志位置1 位移为负
		else if(1==Flag_Hou)	  Movement=900;        //===如果后退标志位置1 位移为正
	  else  Movement=0;	
   //=============速度PI控制器=======================//	
		Encoder_Least =Encoder_Left; //===获取最新速度偏差
		Encoder *= 0.4;		                         //===一阶低通滤波器       
		Encoder += Encoder_Least*0.6;	             //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                     //===积分出位移 积分时间：5ms
		Encoder_Integral=Encoder_Integral-Movement;                     //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>500)  	Encoder_Integral=500;          //===积分限幅
		if(Encoder_Integral<-500)	Encoder_Integral=-500;         //===积分限幅	
		Velocity=Encoder*300+Encoder_Integral*10; //===速度控制	
		if(Turn_Off(Angle_Balance,Voltage)==1)   Encoder_Integral=0;    //===电机关闭后清除积分
	  return Velocity;
}


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
作    者：Mini Balance
**************************************************************************/
void Set_Pwm(int moto1)
{
			PBout(3)=1;//===电机使能打开
			if(moto1<0)			PBout(5)=1,			PBout(4)=0;
			else 	          PBout(5)=0,			PBout(4)=1;
			TIM4->CCR2=myabs(moto1);

	
}
/**************************************************************************
函数功能：读取编码器的数据并进行数据类型转换
入口参数：无
返回  值：无
作    者：Mini Balance
**************************************************************************/
void readEncoder(void)
{
	  u16 Encoder_L;       //===左右编码器的脉冲计数	
	  Encoder_L= TIM3 -> CNT;        //===获取正交解码2数据	
	  TIM3 -> CNT=0;	               //===计数器清零
		if(Encoder_L>32768)  Encoder_Left=Encoder_L-65000; else  Encoder_Left=Encoder_L;  //===数据类型转换
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
作    者：Mini Balance
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=7100;    //===PWM满幅是7200 限制在7100
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
作    者：Mini Balance
**************************************************************************/
u8 Turn_Off(int adc, int voltage)
{
	    u8 temp;
			if(adc<(zhongzhi-500)||adc>(zhongzhi+500)||1==Flag_Stop||Voltage<1110)//===电压低于11.1V 关闭电机
			{	                                             
      temp=1;                                           
			PBout(3)=0;
			PBout(12)=0;
			PAout(15)=0;
			PBout(4)=0;	
			PBout(5)=0;
      }
			else
      temp=0;
      return temp;			
}
	
