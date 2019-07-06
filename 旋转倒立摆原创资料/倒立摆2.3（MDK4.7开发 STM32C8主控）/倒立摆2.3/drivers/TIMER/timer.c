#include "timer.h"
#include "mpu6050.h"
#include "led.h"
#include "math.h"
/**************************************************************************
���ߣ�Mini Balance 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
��    �ߣ�Mini Balance
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
�������ܣ���������ȡ�ж�
��ڲ�������
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//����ж�
	{
			    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//����жϱ�־λ 	    
}

/**************************************************************************
�������ܣ���������ȡ�ж�
��ڲ�������
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
u8  TIM2CH1_CAPTURE_STA=0;	//���벶��״̬		    				
u16	TIM2CH1_CAPTURE_VAL;	//���벶��ֵ
void TIM3_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM3->SR;	
			if(tsr&0X0001)//����ж�
			{
																						
			}				   
			TIM3->SR&=~(1<<0);//����жϱ�־λ 	 
}
/**************************************************************************
�������ܣ���������ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
void Encoder_Init2(void)
{
	/* TIM3 clock source enable */ 
	RCC->APB1ENR|=1<<0;       //TIM2ʱ��ʹ��
	/* Enable GPIOA, clock */
	RCC->APB2ENR|=1<<2;    //ʹ��PORTAʱ��

	/* Configure PA.06,07 as encoder input */
	GPIOA->CRL&=0XFFFFFFF0;//PA0
	GPIOA->CRL|=0X00000004;//��������
	GPIOA->CRL&=0XFFFFFF0F;//PA1
	GPIOA->CRL|=0X00000040;//��������

	/* Enable the TIM3 Update Interrupt */
	//����������Ҫͬʱ���òſ���ʹ���ж�
	TIM2->DIER|=1<<0;   //��������ж�				
	TIM2->DIER|=1<<6;   //�������ж�
	MY_NVIC_Init(1,3,TIM2_IRQChannel,1);

	/* Timer configuration in Encoder mode */ 
	TIM2->PSC = 0x0;//Ԥ��Ƶ��
	TIM2->ARR = ENCODER_TIM_PERIOD-1;//�趨�������Զ���װֵ 
	TIM2->CR1 &=~(3<<8);// ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM2->CR1 &=~(3<<5);// ѡ�����ģʽ:���ض���ģʽ
		
	TIM2->CCMR1 |= 1<<0; //CC1S='01' IC1FP1ӳ�䵽TI1
	TIM2->CCMR1 |= 1<<8; //CC2S='01' IC2FP2ӳ�䵽TI2
	TIM2->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1�����࣬IC1FP1=TI1
	TIM2->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2�����࣬IC2FP2=TI2
	TIM2->CCMR1 |= 3<<4; //	IC1F='1000' ���벶��1�˲���
	TIM2->SMCR |= 3<<0;	 //SMS='011' ���е�������������غ��½�����Ч
	TIM2->CNT = COUNTER_RESET;
	TIM2->CR1 |= 0x01;    //CEN=1��ʹ�ܶ�ʱ��
}

/**************************************************************************
�������ܣ���������ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
void Encoder_Init(void)
{
	/* TIM3 clock source enable */ 
	RCC->APB1ENR|=1<<1;       //TIM3ʱ��ʹ��
	/* Enable GPIOA, clock */
	RCC->APB2ENR|=1<<2;    //ʹ��PORTAʱ��

	/* Configure PA.06,07 as encoder input */
	GPIOA->CRL&=0XF0FFFFFF;//PA6
	GPIOA->CRL|=0X08000000;//��������
	GPIOA->CRL&=0X0FFFFFFF;//PA7
	GPIOA->CRL|=0X40000000;//��������

	/* Enable the TIM3 Update Interrupt */
	//����������Ҫͬʱ���òſ���ʹ���ж�
	TIM3->DIER|=1<<0;   //��������ж�				
	TIM3->DIER|=1<<6;   //�������ж�
	MY_NVIC_Init(1,3,TIM3_IRQChannel,1);

	/* Timer configuration in Encoder mode */ 
	TIM3->PSC = 0x0;//Ԥ��Ƶ��
	TIM3->ARR = ENCODER_TIM_PERIOD-1;//�趨�������Զ���װֵ 
	TIM3->CR1 &=~(3<<8);// ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM3->CR1 &=~(3<<5);// ѡ�����ģʽ:���ض���ģʽ
		
	TIM3->CCMR1 |= 1<<0; //CC1S='01' IC1FP1ӳ�䵽TI1
	TIM3->CCMR1 |= 1<<8; //CC2S='01' IC2FP2ӳ�䵽TI2
	TIM3->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1�����࣬IC1FP1=TI1
	TIM3->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2�����࣬IC2FP2=TI2
	TIM3->CCMR1 |= 3<<4; //	IC1F='1000' ���벶��1�˲���
	TIM3->SMCR |= 3<<0;	 //SMS='011' ���е�������������غ��½�����Ч
	TIM3->CNT = COUNTER_RESET;
	TIM3->CR1 |= 0x01;    //CEN=1��ʹ�ܶ�ʱ��
}


/**************************************************************************
�������ܣ���ʱ�жϳ�ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
void Timer1_Init(u16 arr,u16 psc)  
{  
	RCC->APB2ENR|=1<<11;//TIM2ʱ��ʹ��    
 	TIM1->ARR=arr;  //�趨�������Զ���װֵ//�պ�1ms    
	TIM1->PSC=psc;  //Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��
	TIM1->DIER|=1<<0;   //��������ж�				
	TIM1->DIER|=1<<6;   //�������ж�	   
	TIM1->CR1|=0x01;    //ʹ�ܶ�ʱ��
	MY_NVIC_Init(1,3,TIM1_UP_IRQChannel,1);
}  

/**************************************************************************
�������ܣ�PWM �Լ�������Ƶ�IO��ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	RCC->APB1ENR|=1<<2;       //TIM4ʱ��ʹ��    
	RCC->APB2ENR|=1<<3;       //PORTBʱ��ʹ��     
	RCC->APB2ENR|=1<<2;       //PORTAʱ��ʹ��   
	GPIOB->CRL&=0X00000FFF;//
	GPIOB->CRL|=0XBB333000;//
	GPIOB->CRH&=0XFFF0FFFF;//
	GPIOB->CRH|=0X00030000;//
	GPIOB->ODR|=1<<6;//PB6����	
	GPIOB->ODR|=1<<7;//PB7����
	GPIOA->CRH&=0X0FFFFFFF;//
	GPIOA->CRH|=0X30000000;//	
	TIM4->ARR=arr;//�趨�������Զ���װֵ 
	TIM4->PSC=psc;//Ԥ��Ƶ������Ƶ
	TIM4->CCMR1|=6<<12;  //CH2 PWM2ģʽ	
	TIM4->CCMR1|=6<<4;  //CH1 PWM2ģʽ	
	TIM4->CCMR2|=6<<12;  //CH2 PWM2ģʽ	
	TIM4->CCMR2|=6<<4;  //CH1 PWM2ģʽ	
	TIM4->CCMR1|=1<<11; //CH2Ԥװ��ʹ��	 
	TIM4->CCMR1|=1<<3; //CH1Ԥװ��ʹ��	  
	TIM4->CCMR2|=1<<11; //CH2Ԥװ��ʹ��	 
	TIM4->CCMR2|=1<<3; //CH1Ԥװ��ʹ��	  
	TIM4->CCER|=1<<4;   //Oencoder_right ���ʹ��	   
	TIM4->CCER|=1;   //Oencoder_right ���ʹ��	
	TIM4->CCER|=1<<12;   //Oencoder_right ���ʹ��	   
	TIM4->CCER|=1<<8;   //Oencoder_right ���ʹ��	
	TIM4->CR1=0x8000;   //ARPEʹ�� 
	TIM4->CR1|=0x01;    //ʹ�ܶ�ʱ��2 										  
} 

void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)	 
{ 
	u32 temp;	
	u8 IPROFFSET=NVIC_Channel%4;//�����ڵ�ƫ��
	IPROFFSET=IPROFFSET*8+4;    //�õ�ƫ�Ƶ�ȷ��λ��
	MY_NVIC_PriorityGroupConfig(NVIC_Group);//���÷���
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf;//ȡ����λ
	if(NVIC_Channel<32)NVIC->ISER[0]|=1<<NVIC_Channel;//ʹ���ж�λ(Ҫ����Ļ�,�෴������OK)
	else NVIC->ISER[1]|=1<<(NVIC_Channel-32);       	    	  				   
}
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
	u32 temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//ȡ����λ
	temp1<<=8;
	temp=SCB->AIRCR;  //��ȡ��ǰ������
	temp&=0X0000F8FF; //�����ǰ����
	temp|=0X05FA0000; //д��Կ��
	temp|=temp1;	   
	SCB->AIRCR=temp;  //���÷���	    	  				   
}
