#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "tim.h"


#define PWMA   TIM3->CCR4    //�����ڽ�ʹ��һ·���� ʹ�õ�A·
#define AIN2   PBout(12)     //�����ڽ�ʹ��һ·���� ʹ�õ�A·
#define AIN1   PBout(13)     //�����ڽ�ʹ��һ·���� ʹ�õ�A·

extern __IO int Moto;

void Set_Pwm(int moto);
int myabs(int a);
void Motor_Init(void);
	


#endif
