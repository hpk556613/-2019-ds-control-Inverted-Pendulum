#ifndef __DELAY_H
#define __DELAY_H 

#include "main.h"
/**************************************************************************
×÷Õß£ºMini Balance 
ÌÔ±¦µêÆÌ£ºhttp://shop114407458.taobao.com/
**************************************************************************/
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void get_ms(unsigned long *count);
#endif

//------------------End of File----------------------------
