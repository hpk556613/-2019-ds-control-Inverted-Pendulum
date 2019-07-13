#ifndef __SHOW_H
#define __SHOW_H
#include "main.h"
#include "stm32f1xx_hal.h"
#include "control.h"		
#include "oled.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/

extern __IO float ADC_ConvertedValueLocal;			// 用于保存转换计算后的电压值	 
extern __IO int Angle_Balance;    
extern __IO int Encoder_Balance;
extern __IO int Balance_Pwm;
extern __IO int Position_Pwm;
extern __IO int Position_Zero;
extern __IO int Flag_Target;
extern __IO int Position_Target;
extern __IO int Moto;
extern __IO int Para1;
extern __IO int Para2;
extern __IO int Error_temp;
extern __IO int Error_Target;
extern __IO uint8_t Flag_Stop;
extern __IO uint8_t Position_Key;
extern __IO uint16_t ADC_ConvertedValue;			// AD转换结果值
extern __IO uint16_t timer_count;
extern __IO uint16_t timer_count2;
extern __IO uint16_t Times_up;

void oled_show(void);
void DataScope(void);
#endif
