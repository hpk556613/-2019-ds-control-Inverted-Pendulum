#include "led.h"


void LED_Init(void)
{
RCC->APB2ENR|=1<<2;      //ʹ�� PORTA ʱ��  
GPIOA->CRL&=0XFFF0FFFF;
GPIOA->CRL|=0X00030000;  //PA4 �������
GPIOA->ODR|=1<<4;        //PA4 �����
}






