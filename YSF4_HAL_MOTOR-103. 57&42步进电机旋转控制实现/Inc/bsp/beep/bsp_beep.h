#ifndef __BSP_BEEP_H__
#define __BSP_BEEP_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef enum
{
  BEEPState_OFF = 0,
  BEEPState_ON,
}BEEPState_TypeDef;

#define IS_BEEP_STATE(STATE)           (((STATE) == BEEPState_OFF) || ((STATE) == BEEPState_ON))

/* �궨�� --------------------------------------------------------------------*/
#define BEEP_RCC_CLK_ENABLE()         __HAL_RCC_GPIOI_CLK_ENABLE()
#define BEEP_GPIO_PIN                 GPIO_PIN_10
#define BEEP_GPIO                     GPIOI

#define BEEP_ON                       HAL_GPIO_WritePin(BEEP_GPIO,BEEP_GPIO_PIN,GPIO_PIN_SET)    // ����ߵ�ƽ
#define BEEP_OFF                      HAL_GPIO_WritePin(BEEP_GPIO,BEEP_GPIO_PIN,GPIO_PIN_RESET)  // ����͵�ƽ
#define BEEP_TOGGLE                   HAL_GPIO_TogglePin(BEEP_GPIO,BEEP_GPIO_PIN)                // �����ת


/* ��չ���� ------------------------------------------------------------------*/
/* �������� ------------------------------------------------------------------*/
void BEEP_GPIO_Init(void);
void BEEP_StateSet(BEEPState_TypeDef state);
  
#endif  // __BSP_BEEP_H__

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
