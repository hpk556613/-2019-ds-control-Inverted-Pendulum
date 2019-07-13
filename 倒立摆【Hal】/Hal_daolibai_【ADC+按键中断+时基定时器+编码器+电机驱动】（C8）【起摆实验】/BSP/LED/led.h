#ifndef __LED_H
#define __LED_H
#include "main.h"
#include "stm32f1xx_hal.h"
 /**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/

#define LED_ON                       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)    // 输出高电平
#define LED_OFF                      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)  // 输出低电平
#define LED_TOGGLE                   HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4)                // 输出反转


//LED 端口定义
//#define LED PAout(4) // PA4
void LED_Init(void);  //初始化
void Led_Flash(u16 time);
#endif
