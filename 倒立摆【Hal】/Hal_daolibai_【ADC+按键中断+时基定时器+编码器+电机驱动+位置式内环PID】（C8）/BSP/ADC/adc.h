#ifndef __ADC_H
#define __ADC_H	
#include "main.h"
#include "stm32f1xx_hal.h"

 /**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
//#define Battery_Ch 6
u16 Get_Adc(u8 ch);
int Get_Adc_Average(int ch,int times);
void ADC_Delay(__IO uint32_t i);

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
extern __IO uint8_t Flag_Stop;
extern __IO uint8_t Position_Key;
extern __IO uint16_t ADC_ConvertedValue;			// AD转换结果值


#endif 















