#include "exti.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
函数功能：外部中断初始化
入口参数：无
返回  值：无 
**************************************************************************/


//外部中断9~5服务程序
void EXTI9_5_IRQHandler(void)
{			
	HAL_Delay(5);   //消抖			 
   if(KEY5==0)	// 
	{
		Flag_Stop=!Flag_Stop;
	}		
	if(KEY7==0)	///Menu 
	{
		if(Menu++==4) Menu=1;
	}		
 		EXTI->PR=1<<5;     //清除LINE5上的中断标志位  
		EXTI->PR=1<<7;     //清除LINE7上的中断标志位
}





////外部中断15~10服务程序
//void EXTI15_10_IRQHandler(void)
//{			
//	delay_ms(5);   //消抖			 
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
//  EXTI->PR=1<<11; //清除LINE11上的中断标志位  
//	EXTI->PR=1<<12; //清除LINE12上的中断标志位 
//}





