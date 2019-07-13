#include "led.h"


void LED_Init(void)
{
RCC->APB2ENR|=1<<2;      //使能 PORTA 时钟  
GPIOA->CRL&=0XFFF0FFFF;
GPIOA->CRL|=0X00030000;  //PA4 推挽输出
GPIOA->ODR|=1<<4;        //PA4 输出高
}






