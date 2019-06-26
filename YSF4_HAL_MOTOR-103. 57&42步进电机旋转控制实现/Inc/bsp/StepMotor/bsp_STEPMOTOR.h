#ifndef __STEPMOTOR_TIM_H__
#define __STEPMOTOR_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define STEPMOTOR_TIMx                        TIM8
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIMx_IRQn                   TIM8_CC_IRQn
#define STEPMOTOR_TIMx_IRQHandler             TIM8_CC_IRQHandler

//���1
#define STEPMOTOR_NO1_TIM_CHANNEL_x                TIM_CHANNEL_1
#define STEPMOTOR_TIM_PUL1_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOI_CLK_ENABLE()     // ���������������������
#define STEPMOTOR_TIM_PUL1_PORT             		   GPIOI                            // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define STEPMOTOR_TIM_PUL1_PIN               		   GPIO_PIN_5                       // ��PLU+ֱ�ӽӿ������VCC
#define GPIO_AFx_TIMx                     		     GPIO_AF3_TIM8

#define STEPMOTOR_DIR1_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_DIR1_PORT                    GPIOD                            // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_DIR1_PIN                     GPIO_PIN_3                       // ��DIR+ֱ�ӽӿ������VCC
#define GPIO_PIN_AF_AS_SYS                    GPIO_AF0_RTC_50Hz                // ���Ų���Ϊ���ù���ʹ��

#define STEPMOTOR_ENA1_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_ENA1_PORT                    GPIOD                            // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_ENA1_PIN                     GPIO_PIN_7                       // ��ENA+ֱ�ӽӿ������VCC

#define STEPMOTOR_DIR1_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR1_PORT,STEPMOTOR_DIR1_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_DIR1_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR1_PORT,STEPMOTOR_DIR1_PIN,GPIO_PIN_SET)

#define STEPMOTOR_OUTPUT1_ENABLE()             HAL_GPIO_WritePin(STEPMOTOR_ENA1_PORT,STEPMOTOR_ENA1_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT1_DISABLE()            HAL_GPIO_WritePin(STEPMOTOR_ENA1_PORT,STEPMOTOR_ENA1_PIN,GPIO_PIN_SET)

//���2
#define STEPMOTOR_NO2_TIM_CHANNEL_x            TIM_CHANNEL_2
#define STEPMOTOR_TIM_PUL2_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOI_CLK_ENABLE()     // ���������������������
#define STEPMOTOR_TIM_PUL2_PORT                GPIOI                            // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define STEPMOTOR_TIM_PUL2_PIN                 GPIO_PIN_6                       // ��PLU+ֱ�ӽӿ������VCC

#define STEPMOTOR_DIR2_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_DIR2_PORT                    GPIOD                            // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_DIR2_PIN                     GPIO_PIN_11                       // ��DIR+ֱ�ӽӿ������VCC

#define STEPMOTOR_ENA2_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()     // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_ENA2_PORT                    GPIOF                            // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_ENA2_PIN                     GPIO_PIN_11                       // ��ENA+ֱ�ӽӿ������VCC

#define STEPMOTOR_DIR2_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR2_PORT,STEPMOTOR_DIR2_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_DIR2_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR2_PORT,STEPMOTOR_DIR2_PIN,GPIO_PIN_SET)

#define STEPMOTOR_OUTPUT2_ENABLE()             HAL_GPIO_WritePin(STEPMOTOR_ENA2_PORT,STEPMOTOR_ENA2_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT2_DISABLE()            HAL_GPIO_WritePin(STEPMOTOR_ENA2_PORT,STEPMOTOR_ENA2_PIN,GPIO_PIN_SET)


// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��STEPMOTOR_TIMx_PRESCALER+1��
#define STEPMOTOR_TIM_PRESCALER               5  // �������������ϸ������Ϊ��   32  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               9  // �������������ϸ������Ϊ��   16  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               19  // �������������ϸ������Ϊ��   8  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               39  // �������������ϸ������Ϊ��   4  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               79  // �������������ϸ������Ϊ��   2  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               159 // �������������ϸ������Ϊ��   1  ϸ��


// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define STEPMOTOR_TIM_PERIOD                   0xFFFF
// ����߼���ʱ���ظ������Ĵ���ֵ
#define STEPMOTOR_TIM_REPETITIONCOUNTER       0

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR;
extern __IO uint16_t Toggle_Pulse[2];
/* �������� ------------------------------------------------------------------*/

void STEPMOTOR_TIMx_Init(void);

#endif	/* __STEPMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
