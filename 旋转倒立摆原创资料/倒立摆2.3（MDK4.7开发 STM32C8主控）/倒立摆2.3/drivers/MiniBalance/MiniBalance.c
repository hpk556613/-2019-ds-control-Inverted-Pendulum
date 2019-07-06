#include "MiniBalance.h"
#include "math.h"
#include "led.h"
#include "mpu6050.h"
#define PI 3.14159265
/**************************************************************************
���ߣ�Mini Balance 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/

/**************************************************************************
�������ܣ�5MS��ʱ�жϺ��� 5MS��������
��ڲ�������
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
int zhongzhi=1750;
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
void TIM1_UP_TIM16_IRQHandler(void)  
{    
	if(TIM1->SR&0X0001)//5ms��ʱ�ж�
	{   
		  TIM1->SR&=~(1<<0);                                       //===�����ʱ��1�жϱ�־λ		 
			readEncoder();                                           //===��ȡ��������ֵ
			adc=Get_Adc(0);
  		Led_Flash(400);                                          //===LED��˸;	
  		Get_battery_volt();                                      //===��ȡ��ص�ѹ	          
			key(100);                                                 //===ɨ�谴��״̬
 		  Balance_Pwm=balance(adc,zhongzhi);
		  Velocity_Pwm=velocity(Encoder_Left);
 		  Moto1=Balance_Pwm-Velocity_Pwm;                 //===�������ֵ������PWM
   		Xianfu_Pwm();                                            //===PWM�޷�
     if(Turn_Off(adc,Voltage)==0)                   //===����������쳣
 			Set_Pwm(Moto1);                                    //===��ֵ��PWM�Ĵ���    		
	}       
} 

/**************************************************************************
�������ܣ�ֱ��PID����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
��    �ߣ�Mini Balance
**************************************************************************/
int balance(int adc, int target)
{  
	 static int Last_Bias;
	 int balance,Bias;
	 Bias=adc-target;                                          //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=15*Bias+50*(Bias-Last_Bias);                              //===����ƽ����Ƶĵ��PWM
	 Last_Bias=Bias;
	 return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI����
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
��    �ߣ�Mini Balance
**************************************************************************/
int velocity(int encoder_left)
{  
	  static int Velocity,Encoder_Least,Encoder,Movement;
	  static long Encoder_Integral;
	  //=============ң��ǰ�����˲���=======================//
		if(1==Flag_Qian)	Movement=-900;	             //===���ǰ����־λ��1 λ��Ϊ��
		else if(1==Flag_Hou)	  Movement=900;        //===������˱�־λ��1 λ��Ϊ��
	  else  Movement=0;	
   //=============�ٶ�PI������=======================//	
		Encoder_Least =Encoder_Left; //===��ȡ�����ٶ�ƫ��
		Encoder *= 0.4;		                         //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.6;	             //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                     //===���ֳ�λ�� ����ʱ�䣺5ms
		Encoder_Integral=Encoder_Integral-Movement;                     //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>500)  	Encoder_Integral=500;          //===�����޷�
		if(Encoder_Integral<-500)	Encoder_Integral=-500;         //===�����޷�	
		Velocity=Encoder*300+Encoder_Integral*10; //===�ٶȿ���	
		if(Turn_Off(Angle_Balance,Voltage)==1)   Encoder_Integral=0;    //===����رպ��������
	  return Velocity;
}


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
void Set_Pwm(int moto1)
{
			PBout(3)=1;//===���ʹ�ܴ�
			if(moto1<0)			PBout(5)=1,			PBout(4)=0;
			else 	          PBout(5)=0,			PBout(4)=1;
			TIM4->CCR2=myabs(moto1);

	
}
/**************************************************************************
�������ܣ���ȡ�����������ݲ�������������ת��
��ڲ�������
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
void readEncoder(void)
{
	  u16 Encoder_L;       //===���ұ��������������	
	  Encoder_L= TIM3 -> CNT;        //===��ȡ��������2����	
	  TIM3 -> CNT=0;	               //===����������
		if(Encoder_L>32768)  Encoder_Left=Encoder_L-65000; else  Encoder_Left=Encoder_L;  //===��������ת��
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=7100;    //===PWM������7200 ������7100
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
��    �ߣ�Mini Balance
**************************************************************************/
u8 Turn_Off(int adc, int voltage)
{
	    u8 temp;
			if(adc<(zhongzhi-500)||adc>(zhongzhi+500)||1==Flag_Stop||Voltage<1110)//===��ѹ����11.1V �رյ��
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
	
