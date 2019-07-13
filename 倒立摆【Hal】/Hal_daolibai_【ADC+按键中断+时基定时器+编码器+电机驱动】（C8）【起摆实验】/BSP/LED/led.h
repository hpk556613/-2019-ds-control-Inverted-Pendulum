#ifndef __LED_H
#define __LED_H
#include "main.h"
#include "stm32f1xx_hal.h"
 /**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/

#define LED_ON                       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)    // ����ߵ�ƽ
#define LED_OFF                      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)  // ����͵�ƽ
#define LED_TOGGLE                   HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4)                // �����ת


//LED �˿ڶ���
//#define LED PAout(4) // PA4
void LED_Init(void);  //��ʼ��
void Led_Flash(u16 time);
#endif
