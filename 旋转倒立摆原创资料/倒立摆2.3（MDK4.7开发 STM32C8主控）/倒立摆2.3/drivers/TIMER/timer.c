#include "timer.h"
#include "mpu6050.h"
#include "led.h"
#include "math.h"
/**************************************************************************
作者：Mini Balance 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
作    者：Mini Balance
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：编码器读取中断
入口参数：无
返回  值：无
作    者：Mini Balance
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//溢出中断
	{
			    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//清除中断标志位 	    
}

/**************************************************************************
函数功能：编码器读取中断
入口参数：无
返回  值：无
作    者：Mini Balance
**************************************************************************/
u8  TIM2CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM2CH1_CAPTURE_VAL;	//输入捕获值
void TIM3_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM3->SR;	
			if(tsr&0X0001)//溢出中断
			{
																						
			}				   
			TIM3->SR&=~(1<<0);//清除中断标志位 	 
}
/**************************************************************************
函数功能：编码器初始化
入口参数：无
返回  值：无
作    者：Mini Balance
**************************************************************************/
void Encoder_Init2(void)
{
	/* TIM3 clock source enable */ 
	RCC->APB1ENR|=1<<0;       //TIM2时钟使能
	/* Enable GPIOA, clock */
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟

	/* Configure PA.06,07 as encoder input */
	GPIOA->CRL&=0XFFFFFFF0;//PA0
	GPIOA->CRL|=0X00000004;//浮空输入
	GPIOA->CRL&=0XFFFFFF0F;//PA1
	GPIOA->CRL|=0X00000040;//浮空输入

	/* Enable the TIM3 Update Interrupt */
	//这两个东东要同时设置才可以使用中断
	TIM2->DIER|=1<<0;   //允许更新中断				
	TIM2->DIER|=1<<6;   //允许触发中断
	MY_NVIC_Init(1,3,TIM2_IRQChannel,1);

	/* Timer configuration in Encoder mode */ 
	TIM2->PSC = 0x0;//预分频器
	TIM2->ARR = ENCODER_TIM_PERIOD-1;//设定计数器自动重装值 
	TIM2->CR1 &=~(3<<8);// 选择时钟分频：不分频
	TIM2->CR1 &=~(3<<5);// 选择计数模式:边沿对齐模式
		
	TIM2->CCMR1 |= 1<<0; //CC1S='01' IC1FP1映射到TI1
	TIM2->CCMR1 |= 1<<8; //CC2S='01' IC2FP2映射到TI2
	TIM2->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1不反相，IC1FP1=TI1
	TIM2->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2不反相，IC2FP2=TI2
	TIM2->CCMR1 |= 3<<4; //	IC1F='1000' 输入捕获1滤波器
	TIM2->SMCR |= 3<<0;	 //SMS='011' 所有的输入均在上升沿和下降沿有效
	TIM2->CNT = COUNTER_RESET;
	TIM2->CR1 |= 0x01;    //CEN=1，使能定时器
}

/**************************************************************************
函数功能：编码器初始化
入口参数：无
返回  值：无
作    者：Mini Balance
**************************************************************************/
void Encoder_Init(void)
{
	/* TIM3 clock source enable */ 
	RCC->APB1ENR|=1<<1;       //TIM3时钟使能
	/* Enable GPIOA, clock */
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟

	/* Configure PA.06,07 as encoder input */
	GPIOA->CRL&=0XF0FFFFFF;//PA6
	GPIOA->CRL|=0X08000000;//浮空输入
	GPIOA->CRL&=0X0FFFFFFF;//PA7
	GPIOA->CRL|=0X40000000;//浮空输入

	/* Enable the TIM3 Update Interrupt */
	//这两个东东要同时设置才可以使用中断
	TIM3->DIER|=1<<0;   //允许更新中断				
	TIM3->DIER|=1<<6;   //允许触发中断
	MY_NVIC_Init(1,3,TIM3_IRQChannel,1);

	/* Timer configuration in Encoder mode */ 
	TIM3->PSC = 0x0;//预分频器
	TIM3->ARR = ENCODER_TIM_PERIOD-1;//设定计数器自动重装值 
	TIM3->CR1 &=~(3<<8);// 选择时钟分频：不分频
	TIM3->CR1 &=~(3<<5);// 选择计数模式:边沿对齐模式
		
	TIM3->CCMR1 |= 1<<0; //CC1S='01' IC1FP1映射到TI1
	TIM3->CCMR1 |= 1<<8; //CC2S='01' IC2FP2映射到TI2
	TIM3->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1不反相，IC1FP1=TI1
	TIM3->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2不反相，IC2FP2=TI2
	TIM3->CCMR1 |= 3<<4; //	IC1F='1000' 输入捕获1滤波器
	TIM3->SMCR |= 3<<0;	 //SMS='011' 所有的输入均在上升沿和下降沿有效
	TIM3->CNT = COUNTER_RESET;
	TIM3->CR1 |= 0x01;    //CEN=1，使能定时器
}


/**************************************************************************
函数功能：定时中断初始化
入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
作    者：Mini Balance
**************************************************************************/
void Timer1_Init(u16 arr,u16 psc)  
{  
	RCC->APB2ENR|=1<<11;//TIM2时钟使能    
 	TIM1->ARR=arr;  //设定计数器自动重装值//刚好1ms    
	TIM1->PSC=psc;  //预分频器7200,得到10Khz的计数时钟
	TIM1->DIER|=1<<0;   //允许更新中断				
	TIM1->DIER|=1<<6;   //允许触发中断	   
	TIM1->CR1|=0x01;    //使能定时器
	MY_NVIC_Init(1,3,TIM1_UP_IRQChannel,1);
}  

/**************************************************************************
函数功能：PWM 以及电机控制的IO初始化
入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
作    者：Mini Balance
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	RCC->APB1ENR|=1<<2;       //TIM4时钟使能    
	RCC->APB2ENR|=1<<3;       //PORTB时钟使能     
	RCC->APB2ENR|=1<<2;       //PORTA时钟使能   
	GPIOB->CRL&=0X00000FFF;//
	GPIOB->CRL|=0XBB333000;//
	GPIOB->CRH&=0XFFF0FFFF;//
	GPIOB->CRH|=0X00030000;//
	GPIOB->ODR|=1<<6;//PB6上拉	
	GPIOB->ODR|=1<<7;//PB7上拉
	GPIOA->CRH&=0X0FFFFFFF;//
	GPIOA->CRH|=0X30000000;//	
	TIM4->ARR=arr;//设定计数器自动重装值 
	TIM4->PSC=psc;//预分频器不分频
	TIM4->CCMR1|=6<<12;  //CH2 PWM2模式	
	TIM4->CCMR1|=6<<4;  //CH1 PWM2模式	
	TIM4->CCMR2|=6<<12;  //CH2 PWM2模式	
	TIM4->CCMR2|=6<<4;  //CH1 PWM2模式	
	TIM4->CCMR1|=1<<11; //CH2预装载使能	 
	TIM4->CCMR1|=1<<3; //CH1预装载使能	  
	TIM4->CCMR2|=1<<11; //CH2预装载使能	 
	TIM4->CCMR2|=1<<3; //CH1预装载使能	  
	TIM4->CCER|=1<<4;   //Oencoder_right 输出使能	   
	TIM4->CCER|=1;   //Oencoder_right 输出使能	
	TIM4->CCER|=1<<12;   //Oencoder_right 输出使能	   
	TIM4->CCER|=1<<8;   //Oencoder_right 输出使能	
	TIM4->CR1=0x8000;   //ARPE使能 
	TIM4->CR1|=0x01;    //使能定时器2 										  
} 

void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)	 
{ 
	u32 temp;	
	u8 IPROFFSET=NVIC_Channel%4;//在组内的偏移
	IPROFFSET=IPROFFSET*8+4;    //得到偏移的确切位置
	MY_NVIC_PriorityGroupConfig(NVIC_Group);//设置分组
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf;//取低四位
	if(NVIC_Channel<32)NVIC->ISER[0]|=1<<NVIC_Channel;//使能中断位(要清除的话,相反操作就OK)
	else NVIC->ISER[1]|=1<<(NVIC_Channel-32);       	    	  				   
}
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
	u32 temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//取后三位
	temp1<<=8;
	temp=SCB->AIRCR;  //读取先前的设置
	temp&=0X0000F8FF; //清空先前分组
	temp|=0X05FA0000; //写入钥匙
	temp|=temp1;	   
	SCB->AIRCR=temp;  //设置分组	    	  				   
}
