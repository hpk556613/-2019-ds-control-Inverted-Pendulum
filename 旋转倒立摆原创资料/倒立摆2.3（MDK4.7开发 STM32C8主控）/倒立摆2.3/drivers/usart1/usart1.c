/**************************************************************************
作者：Mini Balance 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
#include "main.h"


uint8_t buffer1[14];
/************************************************************/
/*串口printf必备程序*/
/************************************************************/
int fputc(int ch, FILE *f)
{
	USART1->DR=(u8)ch;
	while((USART1->SR&0X40)==0);
	return ch;
}
/************************************************************/
/**************************实现函数**********************************************
*功    能:		定义usart1
*输入参数：     波特率
*********************************************************************************/
void usart1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//使能 UART1 模块的时钟  使能 UART1对应的引脚端口PA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	//配置UART1 的发送引脚   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//配置PA9 为复用输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//刷新频率50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);    
  //配置UART1 的接收引脚 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//配置PA10为浮地输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//UART1的配置:
	USART_InitStructure.USART_BaudRate = 115200; //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No ;//无奇偶效验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//不使用硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//使能发送和接收功能
	//应用配置到UART1
	USART_Init(USART1, &USART_InitStructure);
	USART_ClearFlag(USART1,USART_FLAG_TC);//避免第1个字节丢失
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //打开接收中断  
	//启动UART1
  	USART_Cmd(USART1, ENABLE);
	//打开usart1中断
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/**************************实现函数**********************************************
*功    能:		usart1接收一个字节
*********************************************************************************/
u8 usart1_receive(void)
{
	while((USART1->SR&0x20)==0);
	return USART1->DR;
}
/**************************实现函数**********************************************
*功    能:		usart1发送一个字节
*********************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************实现函数**********************************************
*功    能:		usart1接收中断
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
/**************************实现函数**********************************************
*功    能:		usart1指令1
*********************************************************************************/
void command1(void)
{
	 
}
/**************************实现函数**********************************************
*功    能:		usart1指令2
*********************************************************************************/
void command2(void)
{
	 
}
/**************************实现函数**********************************************
*功    能:		usart1指令3
*********************************************************************************/
void command3(void)
{
	 
}
/**************************程序结束*********************************************/
