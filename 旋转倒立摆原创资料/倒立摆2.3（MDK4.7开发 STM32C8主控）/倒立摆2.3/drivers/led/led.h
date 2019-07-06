#ifndef __LED_H
#define __LED_H 
/**************************************************************************
作者：Mini Balance 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
#include "main.h"
//LED端口定义
#define LED1 PBout(9)// PD2	
void led_init(void);
void Led_Flash(u16 time);
#endif

