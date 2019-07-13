#ifndef __KEY_H
#define __KEY_H	 
#include "main.h"
#include "stm32f1xx_hal.h"
 /**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
#define KEY5 PAin(5)   
#define KEY2 PAin(2)
#define KEY7 PAin(7)
#define KEY11 PAin(11)
#define KEY12 PAin(12)

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
extern __IO uint8_t Swing_Key;
extern __IO uint16_t ADC_ConvertedValue;			// AD转换结果值
extern __IO uint16_t timer_count;


void KEY_Init(void);          //按键初始化
u8 click_N_Double (u8 time);  //单击按键扫描和双击按键扫描
u8 click(void);               //单击按键扫描
u8 Long_Press(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#endif 

