/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2015-10-04
  * 功    能: 单通道ADC电压采集实现
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usart/bsp_debug_usart.h"
#include "adc/bsp_adc.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
// 用于保存转换计算后的电压值	 
float ADC_ConvertedValueLocal;
// AD转换结果值
__IO uint16_t ADC_ConvertedValue;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 9倍频，得到72MHz主时钟
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：72MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟：72MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;               // APB1时钟：36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;               // APB2时钟：72MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  /* ADC时钟配置 */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6; // 6分频，12MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);  // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{  
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();

  /* 初始化串口并配置串口中断优先级 */
  MX_DEBUG_USART_Init();  
  printf("----这是一个ADC单通道电压采集实验-----\n"); 
  
  /* ADC 初始化 */
  MX_ADCx_Init();
  HAL_ADCEx_Calibration_Start(&hadcx);
  
  /* 启动AD转换并使能AD中断 */
  HAL_ADC_Start_IT(&hadcx);
  
  /* 无限循环 */
  while (1)
  {
    HAL_Delay(500);
    /* 3.3为AD转换的参考电压值，stm32的AD转换为12bit，2^12=4096，
       即当输入为3.3V时，AD转换结果为4096 */
    ADC_ConvertedValueLocal =(float)ADC_ConvertedValue*3.3/4096; 	
		printf("AD转换原始值 = 0x%04X \r\n", ADC_ConvertedValue); 
		printf("计算得出电压值 = %f V \r\n",ADC_ConvertedValueLocal); 
  }
}

/**
  * 函数功能: AD转换结束回调函数
  * 输入参数: hadc：AD设备类型句柄
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  ADC_ConvertedValue=HAL_ADC_GetValue(&hadcx);
}  

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
