#include "encoder.h"

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
	 {
	   case 2:  Encoder_TIM= (short)TIM2 -> CNT; 	break;
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT; 	break;	
		 default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}

