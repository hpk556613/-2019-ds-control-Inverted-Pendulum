#include "key.h"

/**
  * 函数功能: 按键外部中断服务函数
  * 输入参数: GPIO_Pin：中断引脚
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_11)
  {
    HAL_Delay(10);/* 延时一小段时间，消除抖动 */
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==0)
    {
			Moto-=100;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
  }
	
		if(GPIO_Pin==GPIO_PIN_12)
  {
    HAL_Delay(10);/* 延时一小段时间，消除抖动 */
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)==0)
    {
			Moto+=100;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
  }
	
	 if(GPIO_Pin==GPIO_PIN_5)
  {
    HAL_Delay(10);/* 延时一小段时间，消除抖动 */
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0)
    {
			Position_Key=!Position_Key;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
  }
}



