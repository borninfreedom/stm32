/**
  ******************************************************************************
  * �ļ�����: bsp_STEPMOTOR.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-05-31
  * ��    ��: �����������������ʵ��
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
#include "StepMotor/bsp_STEPMOTOR.h" 

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_STEPMOTOR;
__IO uint16_t Toggle_Pulse[2] = {500, 500};         // �Ƚ�������ڣ�ֵԽС���Ƶ��Խ��

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ���������GPIO��ʼ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void STEPMOTOR_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct; 
  
  /* ���Ŷ˿�ʱ��ʹ�� */
	//���1
  STEPMOTOR_TIM_PUL1_GPIO_CLK_ENABLE();
  STEPMOTOR_DIR1_GPIO_CLK_ENABLE();
  STEPMOTOR_ENA1_GPIO_CLK_ENABLE();
  
	//���2 
  STEPMOTOR_TIM_PUL2_GPIO_CLK_ENABLE();
  STEPMOTOR_DIR2_GPIO_CLK_ENABLE();
  STEPMOTOR_ENA2_GPIO_CLK_ENABLE();
	
	//���1
  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFx_TIMx;        // GPIO��������TIM���ù���
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL1_PORT, &GPIO_InitStruct);
  
  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_DIR1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_DIR1_PORT, &GPIO_InitStruct);
  
  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_ENA1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_ENA1_PORT, &GPIO_InitStruct);
  
	//motor 2
	 /* �����������������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFx_TIMx;        // GPIO��������TIM���ù���
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL2_PORT, &GPIO_InitStruct);
  
  /* ��������������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_DIR2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_DIR2_PORT, &GPIO_InitStruct);
  
  /* ���������ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_ENA2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_ENA2_PORT, &GPIO_InitStruct);
	
	
  STEPMOTOR_DIR1_FORWARD();
  STEPMOTOR_OUTPUT1_ENABLE();
	
	 STEPMOTOR_DIR2_FORWARD();
  STEPMOTOR_OUTPUT2_ENABLE();
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void STEPMOTOR_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // ��ʱ��ʱ��
  TIM_MasterConfigTypeDef sMasterConfig;                 // ��ʱ����ģʽ����
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;   // ɲ��������ʱ������
  TIM_OC_InitTypeDef sConfigOC;                          // ��ʱ��ͨ���Ƚ����
  
  /* ��ʱ�������������� */
  htimx_STEPMOTOR.Instance = STEPMOTOR_TIMx;                                 // ��ʱ�����
  htimx_STEPMOTOR.Init.Prescaler = STEPMOTOR_TIM_PRESCALER;                  // ��ʱ��Ԥ��Ƶ��
  htimx_STEPMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;                     // �����������ϼ���
  htimx_STEPMOTOR.Init.Period = STEPMOTOR_TIM_PERIOD;                        // ��ʱ������
  htimx_STEPMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;                 // ʱ�ӷ�Ƶ
  htimx_STEPMOTOR.Init.RepetitionCounter = STEPMOTOR_TIM_REPETITIONCOUNTER;  // �ظ�������
  HAL_TIM_Base_Init(&htimx_STEPMOTOR);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;                 // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htimx_STEPMOTOR, &sClockSourceConfig);

  /* ��ʼ����ʱ���Ƚ�������� */
  HAL_TIM_OC_Init(&htimx_STEPMOTOR);
  
  /* ��ʱ�������ģʽ */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_STEPMOTOR, &sMasterConfig);
  
  /* ɲ��������ʱ������ */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_STEPMOTOR, &sBreakDeadTimeConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;                // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = Toggle_Pulse[0];                      // ������
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;          // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;         // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_NO1_TIM_CHANNEL_x);

  sConfigOC.Pulse = Toggle_Pulse[1];                   // ������
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_NO2_TIM_CHANNEL_x);
  /* STEPMOTOR���GPIO��ʼ������ */
  STEPMOTOR_GPIO_Init();
  
  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(STEPMOTOR_TIMx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(STEPMOTOR_TIMx_IRQn);

}

/**
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIMx)
  {
    /* ������ʱ������ʱ��ʹ�� */
    STEPMOTOR_TIM_RCC_CLK_ENABLE();
  }
}

/**
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIMx)
  {
    /* ������ʱ������ʱ�ӽ��� */
    STEPMOTOR_TIM_RCC_CLK_DISABLE();
		
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL1_PORT,STEPMOTOR_TIM_PUL1_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_DIR1_PORT,STEPMOTOR_DIR1_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_ENA1_PORT,STEPMOTOR_ENA1_PIN);
    
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL2_PORT,STEPMOTOR_TIM_PUL2_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_DIR2_PORT,STEPMOTOR_DIR2_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_ENA2_PORT,STEPMOTOR_ENA2_PIN);
    
    HAL_NVIC_DisableIRQ(STEPMOTOR_TIMx_IRQn);
  }
} 

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
