#include "timer.h"


/**
  * 函数功能: 非阻塞模式下定时器的回调函数
  * 输入参数: htim：定时器句柄
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim1))
	{
		Angle_Balance=Get_Adc_Average(3,15);               //===更新姿态	
			
		timer_coun_1ms++;
		if(timer_coun_1ms==10)
		{
			timer_coun_1ms=0;
			timer_count_50ms++;
			timer_count_100ms++;
			if(timer_count_50ms==5)											//50ms运算一次PID
			{
				timer_count_50ms=0;
				Times_up=1;															//打开PID运算
				Times_up2=1;													//打开起摆功能
				LED_TOGGLE;
			}
		}

	}
}



