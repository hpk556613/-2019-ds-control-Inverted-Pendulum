#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "tim.h"


#define PWMA   TIM3->CCR4    //倒立摆仅使用一路驱动 使用的A路
#define AIN2   PBout(12)     //倒立摆仅使用一路驱动 使用的A路
#define AIN1   PBout(13)     //倒立摆仅使用一路驱动 使用的A路

extern __IO int Moto;
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

void Set_Pwm(int moto);
int myabs(int a);
void Motor_Init(void);
	


#endif
