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
  hadcx.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadcx.Init.ContinuousConvMode = ENABLE;
  hadcx.Init.DiscontinuousConvMode = DISABLE;
  hadcx.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadcx.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadcx.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadcx);

  // ���ò���ͨ��
  sConfig.Channel = ADC_CHANNEL;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
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
    
    /* ADת��ͨ�����ų�ʼ�� */
    GPIO_InitStruct.Pin = ADC_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(ADC_GPIO, &GPIO_InitStruct);

    /* �����ж����ȼ����ú�ʹ���ж� */
    HAL_NVIC_SetPriority(ADCx_IRQ, 1, 0);
    HAL_NVIC_EnableIRQ(ADCx_IRQ);
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
    HAL_GPIO_DeInit(ADC_GPIO, ADC_GPIO_PIN);

    /* ���������ж� */
    HAL_NVIC_DisableIRQ(ADCx_IRQ);

  }
} 

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
