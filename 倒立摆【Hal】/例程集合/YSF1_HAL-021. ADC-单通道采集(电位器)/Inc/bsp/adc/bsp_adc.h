#ifndef __ADC_H__
#define	__ADC_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
// 注意：用作ADC采集的IO必须没有复用，否则采集电压会有影响
/********************ADC输入通道（引脚）配置**************************/
#define ADCx_RCC_CLK_ENABLE()            __HAL_RCC_ADC1_CLK_ENABLE()
#define ADCx_RCC_CLK_DISABLE()           __HAL_RCC_ADC1_CLK_DISABLE()
#define ADCx                             ADC1
#define ADCx_IRQ                         ADC1_2_IRQn
#define ADCx_IRQHandler                  ADC1_2_IRQHandler

//#define ADCx_RCC_CLK_ENABLE()            __HAL_RCC_ADC2_CLK_ENABLE()
//#define ADCx_RCC_CLK_DISABLE()           __HAL_RCC_ADC2_CLK_DISABLE()
//#define ADCx                             ADC2
//#define ADCx_IRQ                         ADC1_2_IRQn
//#define ADCx_IRQHandler                  ADC1_2_IRQHandler

//#define ADCx_RCC_CLK_ENABLE()            __HAL_RCC_ADC3_CLK_ENABLE()
//#define ADCx_RCC_CLK_DISABLE()           __HAL_RCC_ADC3_CLK_DISABLE()
//#define ADCx                             ADC3
//#define ADCx_IRQ                         ADC3_IRQn
//#define ADCx_IRQHandler                  ADC3_IRQHandler


#define ADC_GPIO_ClK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADC_GPIO                         GPIOC
#define ADC_GPIO_PIN                     GPIO_PIN_0        // 连接至板载精密可调电阻(需加跳帽)
#define ADC_CHANNEL                      ADC_CHANNEL_10    // 连接至板载精密可调电阻(需加跳帽)
//#define    ADC_GPIO_PIN                  GPIO_PIN_1        // 连接至板载光敏电阻(需加跳帽)
//#define    ADC_CHANNEL                   ADC_CHANNEL_11    // 连接至板载光敏电阻(需加跳帽)
                                      
/* 扩展变量 ------------------------------------------------------------------*/
extern ADC_HandleTypeDef hadcx;

/* 函数声明 ------------------------------------------------------------------*/
void MX_ADCx_Init(void);

#endif /* __ADC_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
