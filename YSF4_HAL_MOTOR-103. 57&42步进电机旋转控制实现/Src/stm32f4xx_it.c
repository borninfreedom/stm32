/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "StepMotor/bsp_STEPMOTOR.h"
#include "usart/bsp_debug_usart.h"
#include "adc/bsp_adc.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim8;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
void DMAx_Streamn_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adcx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}
void FPU_IRQHandler(void)
{
  /* USER CODE BEGIN FPU_IRQn 0 */

  /* USER CODE END FPU_IRQn 0 */
  /* USER CODE BEGIN FPU_IRQn 1 */

  /* USER CODE END FPU_IRQn 1 */
}

/* USER CODE END 1 */

void BASIC_TIM_INT_FUN(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htimx);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}
/**
* @brief This function handles TIM8 capture compare interrupt.
*/
void STEPMOTOR_TIMx_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htimx_STEPMOTOR);
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
//          STEPMOTOR_OUTPUT_DISABLE();
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
///******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
