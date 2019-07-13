#ifndef __ADC_H__
#define	__ADC_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
// ע�⣺����ADC�ɼ���IO����û�и��ã�����ɼ���ѹ����Ӱ��
/********************ADC����ͨ�������ţ�����**************************/
#define ADCx_RCC_CLK_ENABLE()            __HAL_RCC_ADC1_CLK_ENABLE()
#define ADCx_RCC_CLK_DISABLE()           __HAL_RCC_ADC1_CLK_DISABLE()
#define DMAx_RCC_CLK_ENABLE()            __HAL_RCC_DMA1_CLK_ENABLE()
#define ADCx                             ADC1
#define ADC_DMAx_CHANNELn                DMA1_Channel1
#define ADC_DMAx_CHANNELn_IRQn           DMA1_Channel1_IRQn
#define ADC_DMAx_CHANNELn_IRQHANDLER     DMA1_Channel1_IRQHandler

//#define ADCx_RCC_CLK_ENABLE()            __HAL_RCC_ADC3_CLK_ENABLE()
//#define ADCx_RCC_CLK_DISABLE()           __HAL_RCC_ADC3_CLK_DISABLE()
//#define DMAx_RCC_CLK_ENABLE()            __HAL_RCC_DMA2_CLK_ENABLE()
//#define ADCx                             ADC3
//#define ADC_DMAx_CHANNELn                DMA2_Channel5
//#define ADC_DMAx_CHANNELn_IRQn           DMA2_Channel4_5_IRQn
//#define ADC_DMAx_CHANNELn_IRQHANDLER     DMA2_Channel4_5_IRQHandler

#define ADC_GPIO_ClK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADC_GPIO                         GPIOC

#define ADC_GPIO_PIN1                    GPIO_PIN_0        // ���������ؾ��ܿɵ�����(�����ñ)
#define ADC_CHANNEL1                     ADC_CHANNEL_10    // ���������ؾ��ܿɵ�����(�����ñ)
#define ADC_GPIO_PIN2                    GPIO_PIN_1        // ���������ع�������(�����ñ)
#define ADC_CHANNEL2                     ADC_CHANNEL_11    // ���������ع�������(�����ñ)
#define ADC_GPIO_PIN3                    GPIO_PIN_2
#define ADC_CHANNEL3                     ADC_CHANNEL_12
#define ADC_GPIO_PIN4                    GPIO_PIN_3
#define ADC_CHANNEL4                     ADC_CHANNEL_12          

#define ADC_NUMOFCHANNEL                 4

/* ��չ���� ------------------------------------------------------------------*/
extern ADC_HandleTypeDef hadcx;
extern DMA_HandleTypeDef hdma_adcx;
/* �������� ------------------------------------------------------------------*/
void MX_ADCx_Init(void);

#endif /* __ADC_H__ */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
