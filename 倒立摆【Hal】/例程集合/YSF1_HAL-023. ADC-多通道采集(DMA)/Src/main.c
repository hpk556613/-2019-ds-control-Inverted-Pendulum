/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: ʹ��DMA����ADCת���������
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
#include "stm32f1xx_hal.h"
#include "usart/bsp_debug_usart.h"
#include "adc/bsp_adc.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
// ���ڱ���ת�������ĵ�ѹֵ	 
__IO float ADC_ConvertedValueLocal[ADC_NUMOFCHANNEL];
// ADת�����ֵ
uint32_t ADC_ConvertedValue[ADC_NUMOFCHANNEL];
uint32_t DMA_Transfer_Complete_Count=0;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 9��Ƶ���õ�72MHz��ʱ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�72MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ�72MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;               // APB1ʱ�ӣ�36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;               // APB2ʱ�ӣ�72MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  /* ADCʱ������ */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6; // 6��Ƶ��12MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);  // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{  
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();

  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init();  
  printf("----����һ��ADC��ͨ����ѹ�ɼ�ʵ��(DMA����)------\n"); 
  
  /* ADC ��ʼ�� */
  MX_ADCx_Init();  
  HAL_ADCEx_Calibration_Start(&hadcx);
  
  /* ����ADת����ʹ��DMA������ж� */
  HAL_ADC_Start_DMA(&hadcx,ADC_ConvertedValue,ADC_NUMOFCHANNEL);  
  
  /* ����ѭ�� */
  while (1)
  {
    HAL_Delay(1000);
    /* 3.3ΪADת���Ĳο���ѹֵ��stm32��ADת��Ϊ12bit��2^12=4096��
       ��������Ϊ3.3Vʱ��ADת�����Ϊ4096 */    
    ADC_ConvertedValueLocal[0] =(float)(ADC_ConvertedValue[0]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]ֻȡ���12��Ч����
	  ADC_ConvertedValueLocal[1] =(float)(ADC_ConvertedValue[1]&0xFFF)*3.3/4096; // ADC_ConvertedValue[1]ֻȡ���12��Ч����
    ADC_ConvertedValueLocal[2] =(float)(ADC_ConvertedValue[2]&0xFFF)*3.3/4096; // ADC_ConvertedValue[2]ֻȡ���12��Ч����
    ADC_ConvertedValueLocal[3] =(float)(ADC_ConvertedValue[3]&0xFFF)*3.3/4096; // ADC_ConvertedValue[3]ֻȡ���12��Ч����
    
	  printf("CH1_PC0 value = %d -> %fV\n",ADC_ConvertedValue[0]&0xFFF,ADC_ConvertedValueLocal[0]);
    printf("CH2_PC1 value = %d -> %fV\n",ADC_ConvertedValue[1]&0xFFF,ADC_ConvertedValueLocal[1]);
    printf("CH3_PC2 value = %d -> %fV\n",ADC_ConvertedValue[2]&0xFFF,ADC_ConvertedValueLocal[2]);
    printf("CH4_PC3 value = %d -> %fV\n",ADC_ConvertedValue[3]&0xFFF,ADC_ConvertedValueLocal[3]);

    printf("�Ѿ����ADת��������%d\n",DMA_Transfer_Complete_Count);
    DMA_Transfer_Complete_Count=0;
    printf("\n");   
  }
}

/**
  * ��������: ADCת����ɻص�����
  * �������: hadc��ADC�����豸���
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void  HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  DMA_Transfer_Complete_Count++; 
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
