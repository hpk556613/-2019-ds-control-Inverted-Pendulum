#include "key.h"

/**
  * ��������: �����ⲿ�жϷ�����
  * �������: GPIO_Pin���ж�����
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_11)
  {
    HAL_Delay(10);/* ��ʱһС��ʱ�䣬�������� */
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==0)
    {
			Moto-=100;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
  }
	
		if(GPIO_Pin==GPIO_PIN_12)
  {
    HAL_Delay(10);/* ��ʱһС��ʱ�䣬�������� */
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)==0)
    {
			Moto+=100;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
  }
	
	 if(GPIO_Pin==GPIO_PIN_5)
  {
    HAL_Delay(10);/* ��ʱһС��ʱ�䣬�������� */
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0)
    {
			Position_Key=!Position_Key;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
  }
}



