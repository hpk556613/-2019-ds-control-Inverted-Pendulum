#include "timer.h"


/**
  * ��������: ������ģʽ�¶�ʱ���Ļص�����
  * �������: htim����ʱ�����
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim1))
	{
		Angle_Balance=Get_Adc_Average(3,15);               //===������̬	
			
		timer_coun_1ms++;
		if(timer_coun_1ms==10)
		{
			timer_coun_1ms=0;
			timer_count_50ms++;
			timer_count_100ms++;
			if(timer_count_50ms==5)											//50ms����һ��PID
			{
				timer_count_50ms=0;
				Times_up=1;															//��PID����
				Times_up2=1;													//����ڹ���
				LED_TOGGLE;
			}
		}

	}
}



