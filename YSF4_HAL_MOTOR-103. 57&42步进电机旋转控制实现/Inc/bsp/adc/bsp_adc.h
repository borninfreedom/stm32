#ifndef __ADC_H__
#define	__ADC_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
// 注意：用作ADC采集的IO必须没有复用，否则采集电压会有影响
/********************ADC输入通道（引脚）配置**************************/
//#define ADCx_RCC_CLK_ENABLE()            __HAL_RCC_ADC1_CLK_ENABLE()
//#define ADCx_RCC_CLK_DISABLE()           __HAL_RCC_ADC1_CLK_DISABLE()
//#define DMAx_RCC_CLK_ENABLE()            __HAL_RCC_DMA2_CLK_ENABLE()
//#define ADC_DMAx_Streamn_IRQn            DMA2_Stream0_IRQn
//#define ADC_DMAx_Streamn                 DMA2_Stream0
//#define ADC_DMA_CHANNEL                  DMA_CHANNEL_0
//#define ADCx                             ADC1
//
//#define ADC_GPIO_ClK_ENABLE()            {__HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE();}
//#define ADC_GPIO_PIN1                    GPIO_PIN_0    
//#define ADC_GPIO1                        GPIOA
//#define ADC_CHANNEL1                     ADC_CHANNEL_0    
//
//#define ADC_GPIO_PIN2                    GPIO_PIN_3    
//#define ADC_GPIO2                        GPIOA
//#define ADC_CHANNEL2                     ADC_CHANNEL_3 
//
//#define ADC_GPIO_PIN3                    GPIO_PIN_4    
//#define ADC_GPIO3                        GPIOA
//#define ADC_CHANNEL3                     ADC_CHANNEL_4    
//
//#define ADC_GPIO_PIN4                    GPIO_PIN_5     
//#define ADC_GPIO4                        GPIOA
//#define ADC_CHANNEL4                     ADC_CHANNEL_5    
//
//#define ADC_GPIO_PIN5                    GPIO_PIN_6   
//#define ADC_GPIO5                        GPIOA
//#define ADC_CHANNEL5                     ADC_CHANNEL_6    
// 
//#define ADC_GPIO_PIN6                    GPIO_PIN_0    
//#define ADC_GPIO6                        GPIOB
//#define ADC_CHANNEL6                     ADC_CHANNEL_8    
//  
//#define ADC_GPIO_PIN7                    GPIO_PIN_1        
//#define ADC_GPIO7                        GPIOB
//#define ADC_CHANNEL7                     ADC_CHANNEL_9    
// 
//#define ADC_GPIO_PIN8                    GPIO_PIN_0   
//#define ADC_GPIO8                        GPIOC
//#define ADC_CHANNEL8                     ADC_CHANNEL_10  


#define ADCx_RCC_CLK_ENABLE()            __HAL_RCC_ADC3_CLK_ENABLE()
#define ADCx_RCC_CLK_DISABLE()           __HAL_RCC_ADC3_CLK_DISABLE()
#define DMAx_RCC_CLK_ENABLE()            __HAL_RCC_DMA2_CLK_ENABLE()
#define DMAx_Streamn_IRQn                DMA2_Stream0_IRQn
#define DMAx_Streamn_IRQHandler          DMA2_Stream0_IRQHandler
#define DMAx_Streamn                     DMA2_Stream0
#define DMA_CHANNEL                      DMA_CHANNEL_2
#define ADCx                             ADC3
#define ADC_GPIO_ClK_ENABLE()            __HAL_RCC_GPIOF_CLK_ENABLE();

#define ADC_GPIO_PIN1                    GPIO_PIN_6        
#define ADC_GPIO1                        GPIOF
#define ADC_CHANNEL1                     ADC_CHANNEL_4    

#define ADC_GPIO_PIN2                    GPIO_PIN_7        
#define ADC_GPIO2                        GPIOF
#define ADC_CHANNEL2                     ADC_CHANNEL_5

//#define ADC_GPIO_PIN3                    GPIO_PIN_8       
//#define ADC_GPIO3                        GPIOF
//#define ADC_CHANNEL3                     ADC_CHANNEL_6    

//#define ADC_GPIO_PIN4                    GPIO_PIN_9
//#define ADC_GPIO4                        GPIOF
//#define ADC_CHANNEL4                     ADC_CHANNEL_7    

//#define ADC_GPIO_PIN5                    GPIO_PIN_10
//#define ADC_GPIO5                        GPIOF
//#define ADC_CHANNEL5                     ADC_CHANNEL_8    
// 
//#define ADC_GPIO_PIN6                    GPIO_PIN_3
//#define ADC_GPIO6                        GPIOF
//#define ADC_CHANNEL6                     ADC_CHANNEL_9    
//  
//#define ADC_GPIO_PIN7                    GPIO_PIN_4        
//#define ADC_GPIO7                        GPIOF
//#define ADC_CHANNEL7                     ADC_CHANNEL_14    
// 
//#define ADC_GPIO_PIN8                    GPIO_PIN_5
//#define ADC_GPIO8                        GPIOF
//#define ADC_CHANNEL8                     ADC_CHANNEL_15

#define ADC_CHANNEL_NUMBER               2

/* 扩展变量 ------------------------------------------------------------------*/
extern ADC_HandleTypeDef hadcx;
extern DMA_HandleTypeDef hdma_adcx;

/* 函数声明 ------------------------------------------------------------------*/
void MX_ADCx_Init(void);
void MX_DMA_Init(void) ;
#endif /* __ADC_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
