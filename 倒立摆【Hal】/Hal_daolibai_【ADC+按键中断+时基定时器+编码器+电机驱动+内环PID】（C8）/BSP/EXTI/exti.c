#include "exti.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
�������ܣ��ⲿ�жϳ�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/


//�ⲿ�ж�9~5�������
void EXTI9_5_IRQHandler(void)
{			
	HAL_Delay(5);   //����			 
   if(KEY5==0)	// 
	{
		Flag_Stop=!Flag_Stop;
	}		
	if(KEY7==0)	///Menu 
	{
		if(Menu++==4) Menu=1;
	}		
 		EXTI->PR=1<<5;     //���LINE5�ϵ��жϱ�־λ  
		EXTI->PR=1<<7;     //���LINE7�ϵ��жϱ�־λ
}





////�ⲿ�ж�15~10�������
//void EXTI15_10_IRQHandler(void)
//{			
//	delay_ms(5);   //����			 
//  if(KEY11==0)	//PID-
//	{
//		if(Menu==1)        Balance_KP-=Amplitude1;
//	  else	if(Menu==2)  Balance_KD-=Amplitude2;
//		else  if(Menu==3)  Position_KP-=Amplitude3;
//		else  if(Menu==4)  Position_KD-=Amplitude4;
//	}		
//	 if(KEY12==0)	//PID+ 
//	{
//			    if(Menu==1)  Balance_KP+=Amplitude1;
//	  else	if(Menu==2)  Balance_KD+=Amplitude2;
//		else  if(Menu==3)  Position_KP+=Amplitude3;
//		else  if(Menu==4)  Position_KD+=Amplitude4;
//	}		
//	if(Balance_KP<=0) Balance_KP=0;
//	if(Balance_KD<=0) Balance_KD=0;
//	if(Position_KP<=0) Position_KP=0;
//	if(Position_KD<=0) Position_KD=0;
//  EXTI->PR=1<<11; //���LINE11�ϵ��жϱ�־λ  
//	EXTI->PR=1<<12; //���LINE12�ϵ��жϱ�־λ 
//}





