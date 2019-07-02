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
#include <math.h>
TIM_HandleTypeDef htimx_STEPMOTOR, htimx;
__IO uint16_t Toggle_Pulse[2] = {500, 500};         // �Ƚ�������ڣ�ֵԽС���Ƶ��Խ��
speedRampData srd               = {STOP,CW,0,0,0,0,0};         // �Ӽ������߱���
__IO int32_t  step_position     = 0;           // ��ǰλ��
__IO uint8_t  MotionStatus      = 0;           //�Ƿ����˶���0��ֹͣ��1���˶�


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
  * ��������: ������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void BASIC_TIMx_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htimx.Instance = BASIC_TIMx;
  htimx.Init.Prescaler = BASIC_TIMx_PRESCALER;
  htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx.Init.Period = BASIC_TIMx_PERIOD;
  HAL_TIM_Base_Init(&htimx);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htimx, &sMasterConfig);
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
	
	if(htim_base->Instance==BASIC_TIMx)
  {
    /* ������ʱ������ʱ��ʹ�� */
    BASIC_TIM_RCC_CLK_ENABLE();

    /* �����ж����� */
    HAL_NVIC_SetPriority(BASIC_TIM_IRQ, 1, 0);
    HAL_NVIC_EnableIRQ(BASIC_TIM_IRQ);
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
	
	if(htim_base->Instance==BASIC_TIMx)
  {
    /* ������ʱ������ʱ�ӽ��� */
    BASIC_TIM_RCC_CLK_DISABLE();

    /* �ر������ж� */
    HAL_NVIC_DisableIRQ(BASIC_TIM_IRQ);
  }
} 

/**
  * ��������: ���λ���˶����˶������Ĳ���
  * �������: step���ƶ��Ĳ��� (����Ϊ˳ʱ�룬����Ϊ��ʱ��).
              accel  ���ٶ�,ʵ��ֵΪaccel*0.1*rad/sec^2
              decel  ���ٶ�,ʵ��ֵΪdecel*0.1*rad/sec^2
              speed  ����ٶ�,ʵ��ֵΪspeed*0.1*rad/sec
  * �� �� ֵ: ��
  * ˵    ��: �Ը����Ĳ����ƶ�����������ȼ��ٵ�����ٶȣ�Ȼ���ں���λ�ÿ�ʼ
  *           ������ֹͣ��ʹ�������˶�����Ϊָ���Ĳ���������Ӽ��ٽ׶κ̲ܶ���
  *           �ٶȺ������ǻ�û�ﵽ����ٶȾ�Ҫ��ʼ����
  */
void STEPMOTOR_AxisMoveRel( int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{  
  __IO uint16_t tim_count;
  // �ﵽ����ٶ�ʱ�Ĳ���
  __IO uint32_t max_s_lim;
  // ����Ҫ��ʼ���ٵĲ������������û�дﵽ����ٶȣ�
  __IO uint32_t accel_lim;

  if(MotionStatus != STOP)    // ֻ�����������ֹͣ��ʱ��ż���
    return;
  if(step < 0) // ����Ϊ����
  {
    srd.dir = CCW; // ��ʱ�뷽����ת
    STEPMOTOR_DIR1_REVERSAL();
		STEPMOTOR_DIR2_REVERSAL();
    step =-step;   // ��ȡ��������ֵ
  }
  else
  {
    srd.dir = CW; // ˳ʱ�뷽����ת
    STEPMOTOR_DIR1_FORWARD();
		STEPMOTOR_DIR2_FORWARD();
  }
  
  if(step == 1)    // ����Ϊ1
  {
    srd.accel_count = -1;   // ֻ�ƶ�һ��
    srd.run_state = DECEL;  // ����״̬.
    srd.step_delay = 1000;	// ����ʱ	
  }
  else if(step != 0)  // ���Ŀ���˶�������Ϊ0
  {
    // ���ǵĵ������ר��ָ���ֲ�����ϸ�ļ��㼰�Ƶ�����

    // ��������ٶȼ���, ����õ�min_delay���ڶ�ʱ���ļ�������ֵ��
    // min_delay = (alpha / tt)/ w
    srd.min_delay = (int32_t)(A_T_x10/speed);

    // ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶȣ�����accel��λΪ0.1rad/sec^2
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
    srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);

    // ������ٲ�֮��ﵽ����ٶȵ�����
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
    // ����ﵽ����ٶ�С��0.5�������ǽ���������Ϊ0
    // ��ʵ�����Ǳ����ƶ�����һ�����ܴﵽ��Ҫ���ٶ�
    if(max_s_lim == 0){
      max_s_lim = 1;
    }

    // ������ٲ�֮�����Ǳ��뿪ʼ����
    // n1 = (n1+n2)decel / (accel + decel)
    accel_lim = (uint32_t)(step*decel/(accel+decel));
    // ���Ǳ����������1�����ܲ��ܿ�ʼ����.
    if(accel_lim == 0){
      accel_lim = 1;
    }

    // ʹ�������������ǿ��Լ�������ٽ׶β���
    if(accel_lim <= max_s_lim){
      srd.decel_val = accel_lim - step;
    }
    else{
      srd.decel_val = -(max_s_lim*accel/decel);
    }
    // ��ֻʣ��һ�����Ǳ������
    if(srd.decel_val == 0){
      srd.decel_val = -1;
    }

    // ���㿪ʼ����ʱ�Ĳ���
    srd.decel_start = step + srd.decel_val;

    // �������ٶȺ��������ǾͲ���Ҫ���м����˶�
    if(srd.step_delay <= srd.min_delay){
      srd.step_delay = srd.min_delay;
      srd.run_state = RUN;
    }
    else{
      srd.run_state = ACCEL;
    }    
    // ��λ���ٶȼ���ֵ
    srd.accel_count = 0;
  }
  MotionStatus = 1; // ���Ϊ�˶�״̬
  tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_NO1_TIM_CHANNEL_x | STEPMOTOR_NO2_TIM_CHANNEL_x,tim_count+srd.step_delay); // ���ö�ʱ���Ƚ�ֵ
  TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x | STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_ENABLE);// ʹ�ܶ�ʱ��ͨ�� 
  STEPMOTOR_OUTPUT1_ENABLE();
	STEPMOTOR_OUTPUT2_ENABLE();
}


/**
  * ��������: ��ʱ���жϷ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ּӼ��ٹ���
  */
//void STEPMOTOR_TIMx_IRQHandler(void)//��ʱ���жϴ���
//{ 
//	HAL_TIM_IRQHandler(&htimx_STEPMOTOR);
//  __IO uint32_t tim_count=0;
//  __IO uint32_t tmp = 0;
//  // �����£��£�һ����ʱ����
//  uint16_t new_step_delay=0;
//  // ���ٹ��������һ����ʱ���������ڣ�.
//  __IO static uint16_t last_accel_delay=0;
//  // ���ƶ�����������
//  __IO static uint32_t step_count = 0;
//  // ��¼new_step_delay�е������������һ������ľ���
//  __IO static int32_t rest = 0;
//  //��ʱ��ʹ�÷�תģʽ����Ҫ���������жϲ����һ����������
//  __IO static uint8_t i=0;
//  
//  if(__HAL_TIM_GET_IT_SOURCE(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx) !=RESET)
//  {
//    // �����ʱ���ж�
//    __HAL_TIM_CLEAR_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
//    
//    // ���ñȽ�ֵ
//    tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
//    tmp = tim_count+srd.step_delay;
//    __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_NO1_TIM_CHANNEL_x | STEPMOTOR_NO2_TIM_CHANNEL_x,tmp);

//    i++;     // ��ʱ���жϴ�������ֵ
//    if(i==2) // 2�Σ�˵���Ѿ����һ����������
//    {
//      i=0;   // ���㶨ʱ���жϴ�������ֵ
//      switch(srd.run_state) // �Ӽ������߽׶�
//      {
//        case STOP:
//          step_count = 0;  // ���㲽��������
//          rest = 0;        // ������ֵ
//          // �ر�ͨ��
//          TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x | STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_DISABLE);        
//          __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
//          STEPMOTOR_OUTPUT1_DISABLE();
//				 STEPMOTOR_OUTPUT2_DISABLE();
//          MotionStatus = 0;  //  ���Ϊֹͣ״̬     
//          break;

//        case ACCEL:
//          step_count++;      // ������1
//          if(srd.dir==CW)
//          {	  	
//            step_position++; // ����λ�ü�1
//          }
//          else
//          {
//            step_position--; // ����λ�ü�1
//          }
//          srd.accel_count++; // ���ټ���ֵ��1
//          new_step_delay = srd.step_delay - (((2 *srd.step_delay) + rest)/(4 * srd.accel_count + 1));//������(��)һ����������(ʱ����)
//          rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// �����������´μ��㲹���������������
//          if(step_count >= srd.decel_start)// ����ǹ�Ӧ�ÿ�ʼ����
//          {
//            srd.accel_count = srd.decel_val; // ���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
//            srd.run_state = DECEL;           // �¸����������ٽ׶�
//          }
//          else if(new_step_delay <= srd.min_delay) // ����Ƿ񵽴�����������ٶ�
//          {
//            last_accel_delay = new_step_delay; // ������ٹ��������һ����ʱ���������ڣ�
//            new_step_delay = srd.min_delay;    // ʹ��min_delay����Ӧ����ٶ�speed��
//            rest = 0;                          // ������ֵ
//            srd.run_state = RUN;               // ����Ϊ��������״̬
//          }
//          break;

//        case RUN:
//          step_count++;  // ������1
//          if(srd.dir==CW)
//          {	  	
//            step_position++; // ����λ�ü�1
//          }
//          else
//          {
//            step_position--; // ����λ�ü�1
//          }
//          new_step_delay = srd.min_delay;     // ʹ��min_delay����Ӧ����ٶ�speed��
//          if(step_count >= srd.decel_start)   // ��Ҫ��ʼ����
//          {
//            srd.accel_count = srd.decel_val;  // ���ٲ�����Ϊ���ټ���ֵ
//            new_step_delay = last_accel_delay;// �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
//            srd.run_state = DECEL;            // ״̬�ı�Ϊ����
//          }
//          break;

//        case DECEL:
//          step_count++;  // ������1
//          if(srd.dir==CW)
//          {	  	
//            step_position++; // ����λ�ü�1
//          }
//          else
//          {
//            step_position--; // ����λ�ü�1
//          }
//          srd.accel_count++;
//          new_step_delay = srd.step_delay - (((2 * srd.step_delay) + rest)/(4 * srd.accel_count + 1)); //������(��)һ����������(ʱ����)
//          rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// �����������´μ��㲹���������������
//          
//          //����Ƿ�Ϊ���һ��
//          if(srd.accel_count >= 0)
//          {
//            srd.run_state = STOP;
//          }
//          break;
//      }      
//      srd.step_delay = new_step_delay; // Ϊ�¸�(�µ�)��ʱ(��������)��ֵ
//    }
//  }
//}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
