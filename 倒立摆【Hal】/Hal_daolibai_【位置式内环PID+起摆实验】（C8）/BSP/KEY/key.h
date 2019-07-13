#ifndef __KEY_H
#define __KEY_H	 
#include "main.h"
#include "stm32f1xx_hal.h"
 /**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
#define KEY5 PAin(5)   
#define KEY2 PAin(2)
#define KEY7 PAin(7)
#define KEY11 PAin(11)
#define KEY12 PAin(12)

extern __IO float ADC_ConvertedValueLocal;			// ���ڱ���ת�������ĵ�ѹֵ	 
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
extern __IO uint16_t ADC_ConvertedValue;			// ADת�����ֵ
extern __IO uint16_t timer_count;


void KEY_Init(void);          //������ʼ��
u8 click_N_Double (u8 time);  //��������ɨ���˫������ɨ��
u8 click(void);               //��������ɨ��
u8 Long_Press(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#endif 

