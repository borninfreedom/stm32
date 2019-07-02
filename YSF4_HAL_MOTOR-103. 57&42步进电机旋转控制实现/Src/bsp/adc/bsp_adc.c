/**
  ******************************************************************************
  * �ļ�����: bsp_adc.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ����ADC��ѹ�ɼ��ײ���������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "adc/bsp_adc.h"
#include "usart/bsp_debug_usart.h"

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

  hadcx.Instance = ADCx;
  hadcx.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadcx.Init.Resolution = ADC_RESOLUTION_12B;
  hadcx.Init.ScanConvMode = ENABLE;
  hadcx.Init.ContinuousConvMode = ENABLE;
  hadcx.Init.DiscontinuousConvMode = DISABLE;
  hadcx.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadcx.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadcx.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadcx.Init.NbrOfConversion = ADC_CHANNEL_NUMBER;
  hadcx.Init.DMAContinuousRequests = ENABLE;
  hadcx.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  HAL_ADC_Init(&hadcx);
  
  // ���ò���ͨ��
  sConfig.Channel = ADC_CHANNEL1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;  
  HAL_ADC_ConfigChannel(&hadcx, &sConfig);
  
  sConfig.Channel = ADC_CHANNEL2;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadcx, &sConfig);
  
  sConfig.Channel = ADC_CHANNEL3;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadcx, &sConfig);

//  sConfig.Channel = ADC_CHANNEL4;
//  sConfig.Rank = 4;
//  HAL_ADC_ConfigChannel(&hadcx, &sConfig);
//  
//  sConfig.Channel = ADC_CHANNEL5;
//  sConfig.Rank = 5;
//  HAL_ADC_ConfigChannel(&hadcx, &sConfig);  
//  
//  sConfig.Channel = ADC_CHANNEL6;
//  sConfig.Rank = 6;
//  HAL_ADC_ConfigChannel(&hadcx, &sConfig);
//  
//  sConfig.Channel = ADC_CHANNEL7;
//  sConfig.Rank = 7;
//  HAL_ADC_ConfigChannel(&hadcx, &sConfig);   
//  
//  sConfig.Channel = ADC_CHANNEL8;
//  sConfig.Rank = 8;
//  HAL_ADC_ConfigChannel(&hadcx, &sConfig);   
  
}

void MX_DMA_Init(void) 
{
  
  DMAx_RCC_CLK_ENABLE();
  
  /* �����ж����ȼ����ú�ʹ���ж� */
  HAL_NVIC_SetPriority(DMAx_Streamn_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAx_Streamn_IRQn); 
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
    GPIO_InitStruct.Pin = ADC_GPIO_PIN1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    
    HAL_GPIO_Init(ADC_GPIO1, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_GPIO_PIN2;
    HAL_GPIO_Init(ADC_GPIO2, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = ADC_GPIO_PIN3;
    HAL_GPIO_Init(ADC_GPIO3, &GPIO_InitStruct);
//    
//    GPIO_InitStruct.Pin = ADC_GPIO_PIN4;
//    HAL_GPIO_Init(ADC_GPIO4, &GPIO_InitStruct);
//    
//    GPIO_InitStruct.Pin = ADC_GPIO_PIN5;
//    HAL_GPIO_Init(ADC_GPIO5, &GPIO_InitStruct);
//    
//    GPIO_InitStruct.Pin = ADC_GPIO_PIN6;
//    HAL_GPIO_Init(ADC_GPIO6, &GPIO_InitStruct);
//    
//    GPIO_InitStruct.Pin = ADC_GPIO_PIN7;
//    HAL_GPIO_Init(ADC_GPIO7, &GPIO_InitStruct);
//    
//    GPIO_InitStruct.Pin = ADC_GPIO_PIN8;
//    HAL_GPIO_Init(ADC_GPIO8, &GPIO_InitStruct);    
    
    hdma_adcx.Instance = DMAx_Streamn;
    hdma_adcx.Init.Channel = DMA_CHANNEL;
    hdma_adcx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adcx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adcx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adcx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adcx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adcx.Init.Mode = DMA_CIRCULAR;
    hdma_adcx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adcx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_adcx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_adcx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_adcx.Init.PeriphBurst = DMA_PBURST_SINGLE;    
    HAL_DMA_Init(&hdma_adcx);
    
    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adcx);
  }
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
