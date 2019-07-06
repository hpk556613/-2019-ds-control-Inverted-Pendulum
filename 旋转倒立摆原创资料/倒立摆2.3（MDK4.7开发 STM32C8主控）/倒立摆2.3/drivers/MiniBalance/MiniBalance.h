#ifndef __MINIBALANCE_H
#define __MINIBALANCE_H
#include "main.h"
#include "filter.h"
/**************************************************************************
×÷Õß£ºMini Balance 
ÌÔ±¦µêÆÌ£ºhttp://shop114407458.taobao.com/
**************************************************************************/
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
void TIM1_UP_TIM16_IRQHandler(void);  
int balance(int adc, int target);
int velocity(int encoder_left);
int turn(int encoder_left,int encoder_right,float gyro);
void Set_Pwm(int moto1);

void readEncoder(void);

void Xianfu_Pwm(void);
u8 Turn_Off(int adc, int voltage);
void Get_Angle(u8 way);
#endif
