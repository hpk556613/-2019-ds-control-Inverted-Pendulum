/**
  ******************************************************************************
  * �ļ�����: bsp_adc.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: ����ADC��ѹ�ɼ��ײ���������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "adc/bsp_adc.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
ADC_HandleTypeDef hadcx;
DMA_HandleTypeDef hdma_adcx;
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ADת����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void MX_ADCx_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

  // ADC��������
  hadcx.Instance = ADCx;
  hadcx.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadcx.Init.ContinuousConvMode = ENABLE;
  hadcx.Init.DiscontinuousConvMode = DISABLE;
  hadcx.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadcx.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadcx.Init.NbrOfConversion = ADC_NUMOFCHANNEL;
  HAL_ADC_Init(&hadcx);

  // ���ò���ͨ��
  sConfig.Channel = ADC_CHANNEL1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  HAL_ADC_ConfigChannel(&hadcx, &sConfig);

  // ���ò���ͨ��
  sConfig.Channel = ADC_CHANNEL2;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadcx, &sConfig);

  // ���ò���ͨ��
  sConfig.Channel = ADC_CHANNEL3;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadcx, &sConfig);
  
  // ���ò���ͨ��
  sConfig.Channel = ADC_CHANNEL4;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadcx, &sConfig);
}

/**
  * ��������: ADC�����ʼ������
  * �������: hadc��AD����������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADCx)
  {
    /* ����ʱ��ʹ�� */
    ADCx_RCC_CLK_ENABLE();
    
    /* ADת��ͨ������ʱ��ʹ�� */
    ADC_GPIO_ClK_ENABLE();
    
    /* DMAʱ��ʹ�� */
    DMAx_RCC_CLK_ENABLE();
      
    /* ADת��ͨ�����ų�ʼ�� */
    GPIO_InitStruct.Pin = ADC_GPIO_PIN1|ADC_GPIO_PIN2|ADC_GPIO_PIN3|ADC_GPIO_PIN4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(ADC_GPIO, &GPIO_InitStruct);

    /* DMA�����ʼ������ */  
    hdma_adcx.Instance = ADC_DMAx_CHANNELn;
    hdma_adcx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adcx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adcx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adcx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adcx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adcx.Init.Mode = DMA_CIRCULAR;
    hdma_adcx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adcx);
    /* ����DMA */
    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adcx);
    
    /* �����ж����ȼ����ú�ʹ���ж� */
    HAL_NVIC_SetPriority(ADC_DMAx_CHANNELn_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_DMAx_CHANNELn_IRQn);
  }
}

/**
  * ��������: ADC���跴��ʼ������
  * �������: hadc��AD����������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADCx)
  {
    /* ����ADC����ʱ�� */
    ADCx_RCC_CLK_DISABLE();
  
    /* ADת��ͨ�����ŷ���ʼ�� */
    HAL_GPIO_DeInit(ADC_GPIO, ADC_GPIO_PIN1|ADC_GPIO_PIN2|ADC_GPIO_PIN3|ADC_GPIO_PIN4);

    /* DMA���跴��ʼ��*/
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }
} 

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
