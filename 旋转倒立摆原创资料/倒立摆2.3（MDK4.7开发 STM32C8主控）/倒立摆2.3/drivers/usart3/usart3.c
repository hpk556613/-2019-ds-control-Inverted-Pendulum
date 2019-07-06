#include "main.h"
#include "usart3.h"
/**************************************************************************
���ߣ�Mini Balance 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
 u8 mode_data[8];
 u8 six_data_1[4]={6,5,4,0};
 u8 six_data_2[4]={4,5,6,0};

void uart3_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000/2)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
  mantissa<<=4;
	mantissa+=fraction; 
	//AFIO->MAPR &= ~AFIO_MAPR_USART1_REMAP;
	RCC->APB2ENR|=1<<3;   //ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<18;  //ʹ�ܴ���ʱ�� 
	GPIOB->CRH&=0XFFFF00FF; 
	GPIOB->CRH|=0X00008B00;//IO״̬����
	GPIOB->ODR|=1<<10;	  
	RCC->APB1RSTR|=1<<18;   //��λ����1
	RCC->APB1RSTR&=~(1<<18);//ֹͣ��λ	   	   
	//����������
 	USART3->BRR=mantissa; // ����������	 
	USART3->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART3->CR1|=1<<8;    //PE�ж�ʹ��
	USART3->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(1,3,USART3_IRQChannel,2);//��2��������ȼ� 
}

/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
void USART3_IRQHandler(void)
{	
	if(USART3->SR&(1<<5))//���յ�����
	{	  
	  static	int uart_receive=0;//����������ر���
		uart_receive=USART3->DR; 
		if(uart_receive<10)    mode_data[0]=uart_receive;

			if((mode_data[0]==six_data_2[0]
			&&mode_data[1]==six_data_2[1]
			&&mode_data[2]==six_data_2[2]
			&&mode_data[3]==six_data_2[3])
			||(mode_data[0]==six_data_1[0]
			&&mode_data[1]==six_data_1[1]
			&&mode_data[2]==six_data_1[2]
			&&mode_data[3]==six_data_1[3]))
		{	
			Flag_Stop=!Flag_Stop;
			mode_data[0]=0;	mode_data[1]=0;	mode_data[2]=0;	mode_data[3]=0;
		}
		if(uart_receive==0x00)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
		if(uart_receive==0x01)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ǰ
		if(uart_receive==0x05)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////��
		else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)	
													Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;
		else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	
													Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;

		mode_data[7]=mode_data[6];
		mode_data[6]=mode_data[5];
		mode_data[5]=mode_data[4];
		mode_data[4]=mode_data[3];
		mode_data[3]=mode_data[2];
		mode_data[2]=mode_data[1];
		mode_data[1]=mode_data[0];
	}  											 
} 

