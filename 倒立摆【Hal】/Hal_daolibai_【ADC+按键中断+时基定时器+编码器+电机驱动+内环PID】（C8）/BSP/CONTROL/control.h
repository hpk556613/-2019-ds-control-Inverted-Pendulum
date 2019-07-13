#ifndef __CONTROL_H
#define __CONTROL_H
#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "adc.h"
#include "motor.h"
#include "timer.h"

  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/

#define PI 			3.14159265
#define ZHONGZHI 			3085

//#define Angle_KP 			1
//#define Angle_KI			0
//#define	Angle_KD 			0
//#define	Position_KP 		0.1
//#define	Position_KI 		0
//#define Position_KD 		0	 //PID系数

typedef volatile struct __PIDStruct
{
	float SetPoint;
	float Proportion;
	float Integral;
	float Derivative;
	float LastError;
	float PrevError;
	float LastErrorSum;
}PIDStruct;  

extern PIDStruct Motor_AnglePID;
extern PIDStruct Motor_PostionPID;

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


int TIM1_UP_IRQHandler(void);
int balance(float angle);
int Position(int Encoder);
void Set_Pwm(int moto);
void Key(void);
void Xianfu_Pwm(void);
void PID_Init(PIDStruct * pidStuc, float kp, float ki, float kd);
int Inc_PID_Calc1(PIDStruct * pidStuc,int NextPoint);
int Inc_PID_Calc2(PIDStruct * pidStuc,int NextPoint);
void PID_SetPoint(PIDStruct * pidStuc, int SetPoint);
u8 Turn_Off(void);

#endif

