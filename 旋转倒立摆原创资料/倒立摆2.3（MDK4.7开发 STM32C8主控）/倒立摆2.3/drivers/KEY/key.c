#include "key.h"
/**************************************************************************
作者：Mini Balance 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟	   	 
	GPIOA->CRL&=0XFFF0FFFF; 
	GPIOA->CRL|=0X00080000;//PA4 上拉输入（因为外接了上拉电阻） 
} 
/**************************************************************************
函数功能：按键扫描
入口参数：双击等待时间
返回  值：无
作    者：Mini Balance
**************************************************************************/
void key(u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single;
		if(0==KEY1&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
			if(flag_key==1) double_key++,count_key=1;	
			if(double_key==2) Flag_Show=!Flag_Show,double_key=0,count_single=0;
		}
		if(1==KEY1)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			if(++count_single>time)	Flag_Stop=!Flag_Stop,double_key=0,count_single=0;	
		}	
}
