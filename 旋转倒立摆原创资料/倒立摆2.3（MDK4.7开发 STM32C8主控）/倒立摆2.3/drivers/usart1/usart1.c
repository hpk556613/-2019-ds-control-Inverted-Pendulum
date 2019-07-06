/**************************************************************************
���ߣ�Mini Balance 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
#include "main.h"


uint8_t buffer1[14];
/************************************************************/
/*����printf�ر�����*/
/************************************************************/
int fputc(int ch, FILE *f)
{
	USART1->DR=(u8)ch;
	while((USART1->SR&0X40)==0);
	return ch;
}
/************************************************************/
/**************************ʵ�ֺ���**********************************************
*��    ��:		����usart1
*���������     ������
*********************************************************************************/
void usart1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//ʹ�� UART1 ģ���ʱ��  ʹ�� UART1��Ӧ�����Ŷ˿�PA��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	//����UART1 �ķ�������   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//����PA9 Ϊ�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//ˢ��Ƶ��50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);    
  //����UART1 �Ľ������� 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//����PA10Ϊ��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//UART1������:
	USART_InitStructure.USART_BaudRate = 115200; //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ����
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No ;//����żЧ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��ʹ��Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ʹ�ܷ��ͺͽ��չ���
	//Ӧ�����õ�UART1
	USART_Init(USART1, &USART_InitStructure);
	USART_ClearFlag(USART1,USART_FLAG_TC);//�����1���ֽڶ�ʧ
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //�򿪽����ж�  
	//����UART1
  	USART_Cmd(USART1, ENABLE);
	//��usart1�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1����һ���ֽ�
*********************************************************************************/
u8 usart1_receive(void)
{
	while((USART1->SR&0x20)==0);
	return USART1->DR;
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1����һ���ֽ�
*********************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1�����ж�
*********************************************************************************/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  	{
		u8 rxTemp;
		rxTemp=usart1_receive();
		if(rxTemp==0xa5)
		{
			rxTemp=usart1_receive(); 
			if(rxTemp==0x5a)
			{
				switch (usart1_receive())
				{
					case 0xa1:	command1();
					      		break;
					case 0xa2:	command2();
					      		break;
					case 0xa3:	command3();
					      		break;
					//default:	usart1_send(0xff);
				} 
			}
		}
	}
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1ָ��1
*********************************************************************************/
void command1(void)
{
	 
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1ָ��2
*********************************************************************************/
void command2(void)
{
	 
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1ָ��3
*********************************************************************************/
void command3(void)
{
	 
}
/**************************�������*********************************************/
