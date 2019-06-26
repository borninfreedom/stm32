#include "stm32f4xx_hal.h"
#include "StepMotor/bsp_STEPMOTOR.h"
#include "key/bsp_key.h"
#include "usart/bsp_debug_usart.h"
#include "adc/bsp_adc.h"
#include "lcd/bsp_lcd.h"
#include "stdlib.h"
#include "spiflash/bsp_spiflash.h"

#define STEPMOTOR_MICRO_STEP      32 																				 // �������������ϸ�֣�������������ʵ�����ö�Ӧ

uint8_t dir=0; 																																// 0 ��˳ʱ��   1����ʱ�� 
uint8_t ena=0; 																															// 0 ���������� 1��ͣ��

__IO float ADC_ConvertedValueLocal[ADC_CHANNEL_NUMBER];											// ���ڱ���ת�������ĵ�ѹֵ	 

uint32_t ADC_ConvertedValue[ADC_CHANNEL_NUMBER];														// ADת�����ֵ
uint32_t DMA_Transfer_Complete_Count=0;	

extern __IO uint16_t Toggle_Pulse[2]; 																					/* ��������ٶȿ��ƣ��ɵ��ڷ�ΧΪ 300 -- 3500 ��ֵԽС�ٶ�Խ�� */

/*
*    ���������������ϸ������Ϊ1ʱ��ÿ200�����岽�������תһ��
*                          Ϊ32ʱ��ÿ6400�����岽�������תһ��
*    ����������Ϊ32ʱΪ�����⣺
*    pulse_count���ڼ�¼�������������pulse_countΪ��������������
*    ���統pulse_count=12800ʱ��ʵ�����6400���������塣
*    �������Էǳ����㲽�������ʵ��ת��Ȧ����������Ƕȶ��а취���������
*    ������������������ϸ������Ϊ����ֵ��pulse_countҲҪ����Ӧ����
*
*/

__IO uint32_t pulse_count[2]; 																								/*  ���������һ�����������������2 */

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     // ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // ���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        // ��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    // ��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            // PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 // 8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               // 336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     // 2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 // USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                 // ���ò�����ϵͳ�δ�ʱ��
 
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);								 /* ϵͳ�δ�ʱ��ʱ��Դ */

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);															  /* ϵͳ�δ�ʱ���ж����ȼ����� */
}

int main(void)
{  
	uint8_t i;
	uint32_t lcdid;
	char str[50];  
	uint16_t count;
	float pulse_temp1, pulse_temp2;
	float temp1, temp_raw1, temp2, temp_raw2;
 
  HAL_Init();																							 /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  SystemClock_Config();																			  /* ����ϵͳʱ�� */
	
  KEY_GPIO_Init();
  MX_DEBUG_USART_Init();															  /* ��ʼ�����ڲ����ô����ж����ȼ� */ 
  lcdid=BSP_LCD_Init(); 														 /* ��ʼ��3.5��TFTҺ��ģ�飬һ�������ڵ��Դ��ڳ�ʼ�� */
  srand(0xffff);   /* ��ʼ��������� */
	MX_DMA_Init();
  MX_ADCx_Init();															 /* ADC ��ʼ�� */

  HAL_ADC_Start_DMA(&hadcx,ADC_ConvertedValue,ADC_CHANNEL_NUMBER);  					/* ����ADת����ʹ��DMA������ж� */
  STEPMOTOR_TIMx_Init();									 /* �߼����ƶ�ʱ����ʼ��������PWM������� */
  HAL_TIM_Base_Start(&htimx_STEPMOTOR);												 /* ȷ����ʱ�� */
  HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_NO1_TIM_CHANNEL_x);									/* ʹ���ж� �رձȽ����*/
  TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_DISABLE);
	
	HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_NO2_TIM_CHANNEL_x);									/* ʹ���ж� �رձȽ����*/
  TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_DISABLE);

	LCD_Clear(0,0,LCD_DEFAULT_WIDTH,LCD_DEFAULT_HEIGTH,BLACK);
  LCD_BK_ON();	 /* ������ */
	
//  /* ��ʾӢ���ַ� */
//  LCD_DispChar_EN(50,50,'Y',BLUE,YELLOW,USB_FONT_16);
//  LCD_DispChar_EN(70,50,'S',BLUE,YELLOW,USB_FONT_16);
//  LCD_DispChar_EN(50,70,'Y',YELLOW,BLUE,USB_FONT_24);
//  LCD_DispChar_EN(70,70,'S',YELLOW,BLUE,USB_FONT_24);
//  
//  /* ��ʾӢ���ַ��� */
//  LCD_DispString_EN(50,100,"Hellow World!!!",RED,GREEN,USB_FONT_16);
//  LCD_DispString_EN(50,120,"Hellow World!!!",RED,GREEN,USB_FONT_24);
//  LCD_DispString_EN(10,200,"Welcome to use YS_F1Pro STM32 development board",BLACK,YELLOW,USB_FONT_24);
//  
  while (1)
  {		
		HAL_Delay(10);
		count=rand();
    LCD_Clear(178,150,120,24,BLACK);
    LCD_Clear(198,190,120,24,BLACK);    
		
		ADC_ConvertedValueLocal[0] =(double)(ADC_ConvertedValue[0]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]ֻȡ���12��Ч����
	  ADC_ConvertedValueLocal[1] =(double)(ADC_ConvertedValue[1]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]ֻȡ���12��Ч����
		
		//stop
		if (ADC_ConvertedValueLocal[0] > 1.5 && ADC_ConvertedValueLocal[0] < 1.8 && ADC_ConvertedValueLocal[1] > 1.5 && ADC_ConvertedValueLocal[1] < 1.8) {
			
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			
			temp_raw1 = 1.6 - ADC_ConvertedValueLocal[0];
			temp1 = (int)(temp_raw1 * 10.0 + 0.5) / 10.0;
			
			temp_raw2 = 1.6 - ADC_ConvertedValueLocal[1];
			temp2 = (int)(temp_raw2 * 10.0 + 0.5) / 10.0;
			
			sprintf(str,"ADC0-X��%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1��%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1��%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1��%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y��%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2��%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2��%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2��%d",Toggle_Pulse[1]);  
			LCD_DispString_EN_CH(10,240,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			LCD_DispString_EN(10,300,"status: STOP    ",RED,GREEN,USB_FONT_24);
		}
		
		//forward
		else if (ADC_ConvertedValueLocal[0] < 1.5 && ADC_ConvertedValueLocal[1] > 1.5 && ADC_ConvertedValueLocal[1] < 1.8) {
			
	//		if (dir == 1) {
				 STEPMOTOR_DIR1_FORWARD();   // ��ת
				STEPMOTOR_DIR2_FORWARD();
	//			 dir = 0;
	//		}
			
			temp_raw1 = 1.6 - ADC_ConvertedValueLocal[0];
			temp1 = (int)(temp_raw1 * 10.0 + 0.5) / 10.0;
			
			temp_raw2 = 1.6 - ADC_ConvertedValueLocal[0];
			temp2 = (int)(temp_raw2 * 10.0 + 0.5) / 10.0;
			
			Toggle_Pulse[0] = 800 - temp1 * 200;
			Toggle_Pulse[1] = 800 - temp2 * 200;

			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			
			sprintf(str,"ADC0-X��%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1��%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1��%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1��%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y��%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2��%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2��%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2��%d",Toggle_Pulse[1]);  
			LCD_DispString_EN_CH(10,240,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			LCD_DispString_EN(10,300,"status: FOEWARD ",GREEN,BLACK,USB_FONT_24);
		}
		
		//backward
		else if (ADC_ConvertedValueLocal[0] >= 1.8 && ADC_ConvertedValueLocal[1] > 1.5 && ADC_ConvertedValueLocal[1] < 1.8) {
			
//			dir = 1;
			STEPMOTOR_DIR1_REVERSAL();  // ��ת
			STEPMOTOR_DIR2_REVERSAL();  // ��ת
			
			temp_raw1 = ADC_ConvertedValueLocal[0] - 1.6;
			temp1 = (int)(temp_raw1 * 10.0 + 0.5) / 10.0;
			
			temp_raw2 = ADC_ConvertedValueLocal[0] - 1.6;
			temp2 = (int)(temp_raw2 * 10.0 + 0.5) / 10.0;
			
			Toggle_Pulse[0] = 800 - temp1 * 200;
			Toggle_Pulse[1] = 800 - temp2 * 200;
			
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			
			sprintf(str,"ADC0-X��%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1��%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1��%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1��%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y��%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2��%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2��%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2��%d",Toggle_Pulse[1]);  
			LCD_DispString_EN_CH(10,240,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			LCD_DispString_EN(10,300,"status: BACKWARD",GREEN,BLACK,USB_FONT_24);
		}
		
		//��ת
		else if (ADC_ConvertedValueLocal[0] > 1.5 && ADC_ConvertedValueLocal[0] < 1.8 && ADC_ConvertedValueLocal[1] >= 1.8) {
			
			STEPMOTOR_DIR1_REVERSAL();   // ��ת
			STEPMOTOR_DIR2_FORWARD();
			
			temp_raw1 = ADC_ConvertedValueLocal[1] - 1.6;
			temp1 = (int)(temp_raw1 * 10.0 + 0.5) / 10.0;
			
			temp_raw2 = ADC_ConvertedValueLocal[1] - 1.6;
			temp2 = (int)(temp_raw2 * 10.0 + 0.5) / 10.0;
			
			Toggle_Pulse[0] = 800 - temp1 * 200;
			Toggle_Pulse[1] = 800 - temp2 * 200;

			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			
			sprintf(str,"ADC0-X��%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1��%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1��%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1��%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y��%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2��%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2��%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2��%d",Toggle_Pulse[1]);  
			LCD_DispString_EN_CH(10,240,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			LCD_DispString_EN(10,300,"status: LEFT    ",GREEN,BLACK,USB_FONT_24);
		}
		
		//��ת
		else if (ADC_ConvertedValueLocal[0] > 1.5 && ADC_ConvertedValueLocal[0] < 1.8 && ADC_ConvertedValueLocal[1] <= 1.5) {
			
			STEPMOTOR_DIR2_REVERSAL();   // ��ת
			STEPMOTOR_DIR1_FORWARD();
			
			temp_raw1 = 1.6 - ADC_ConvertedValueLocal[1];
			temp1 = (int)(temp_raw1 * 10.0 + 0.5) / 10.0;
			
			temp_raw2 = 1.6 - ADC_ConvertedValueLocal[1];
			temp2 = (int)(temp_raw2 * 10.0 + 0.5) / 10.0;
			
			Toggle_Pulse[0] = 800 - temp1 * 200;
			Toggle_Pulse[1] = 800 - temp2 * 200;

			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			
			sprintf(str,"ADC0-X��%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1��%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1��%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1��%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y��%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2��%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2��%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2��%d",Toggle_Pulse[1]);  
			LCD_DispString_EN_CH(10,240,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			LCD_DispString_EN(10,300,"status: RIGHT   ",GREEN,BLACK,USB_FONT_24);
		}
		else {
			
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			
		  sprintf(str,"ADC0-X��%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1��%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1��%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1��%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y��%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2��%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2��%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2��%d",Toggle_Pulse[1]);  
			LCD_DispString_EN_CH(10,240,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			LCD_DispString_EN(10,300,"status: NONE   ",GREEN,BLACK,USB_FONT_24);
			
		}
		
		
		
		
		
		
//		pulse_temp = (int)(abs(ADC_ConvertedValueLocal[0] - 1.6) * 10.0 + 0.5) / 10.0 * 200 + 400;
//		
//	  
//		
//		if (ADC_ConvertedValueLocal[0] >= 1.5 && ADC_ConvertedValueLocal[0] <= 1.8){ 	
//			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_TIM_CHANNEL_x,TIM_CCx_DISABLE);
//			
//			sprintf(str,"ADC[0]-X axis��%f",ADC_ConvertedValueLocal[0]);  
//			LCD_DispString_EN_CH(10,50,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
//			sprintf(str,"pulse_temp��%f",pulse_temp);  
//			LCD_DispString_EN_CH(10,100,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
//		  
//			LCD_DispString_EN(10,150,"status: STOP",RED,GREEN,USB_FONT_24);
//		}
//		else {
//			if (ADC_ConvertedValueLocal[0] >= 0 && ADC_ConvertedValueLocal[0] <= 1.5){
//				if(pulse_temp < 320)		pulse_temp = 320;
//		
//				sprintf(str,"ADC[0]-X axis��%f",ADC_ConvertedValueLocal[0]);  
//				LCD_DispString_EN_CH(10,50,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
//				sprintf(str,"pulse_temp��%f",pulse_temp);  
//				LCD_DispString_EN_CH(10,100,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
//		
//				LCD_DispString_EN(10,150,"status: RUN ",GREEN,WHITE,USB_FONT_24);
//				Toggle_Pulse = pulse_temp;
//				pulse_count=0;
//				ena=0;  
//				TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_TIM_CHANNEL_x,TIM_CCx_ENABLE);
//			}
//			
//			if (ADC_ConvertedValueLocal[0] >= 1.7){
//				sprintf(str,"ADC[0]-X axis��%f",ADC_ConvertedValueLocal[0]);  
//				LCD_DispString_EN_CH(10,50,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
//				sprintf(str,"pulse_temp��%f",pulse_temp);  
//				LCD_DispString_EN_CH(10,100,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
//		
//				LCD_DispString_EN(10,150,"status: RUN ",GREEN,WHITE,USB_FONT_24);
//				
//				 STEPMOTOR_DIR_REVERSAL();  // ��ת
//				Toggle_Pulse = pulse_temp;
//				pulse_count=0;
//				ena=0;  
//				TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_TIM_CHANNEL_x,TIM_CCx_ENABLE);
//			}
//		}
 
//		 if(KEY1_StateRead() == KEY_DOWN){
////			 while(1){
//				 HAL_Delay(500);
//				TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_TIM_CHANNEL_x,TIM_CCx_ENABLE);
//				if(Toggle_Pulse >= 150) 
//						Toggle_Pulse -= 2;
//				else
//						Toggle_Pulse = 150;
//			}
//		}
//    for(i=0;i<ADC_CHANNEL_NUMBER;i++)
//    {
//      /* 3.3ΪADת���Ĳο���ѹֵ��stm32��ADת��Ϊ12bit��2^12=4096��
//         ��������Ϊ3.3Vʱ��ADת�����Ϊ4096 */    
//      ADC_ConvertedValueLocal[i] =(double)(ADC_ConvertedValue[i]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]ֻȡ���12��Ч����
//    }
//    
//    for(i=0;i<ADC_CHANNEL_NUMBER;i++)
//    {
//      printf("CH%d value = %d -> %.1fV\r\n",i,ADC_ConvertedValue[i]&0xFFF,(int)(ADC_ConvertedValueLocal[i] * 10.0 + 0.5) / 10.0);
//    }   
//    
//    printf("�Ѿ����ADת��������%d\r\n",DMA_Transfer_Complete_Count);
//    DMA_Transfer_Complete_Count=0;
//    printf("\n");   
//    
		
		
//    if(KEY1_StateRead() == KEY_DOWN)  
//    {
//      pulse_count=0;            
//      ena=0;          
//      TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_TIM_CHANNEL_x,TIM_CCx_ENABLE);
//    }
//    if(KEY2_StateRead() == KEY_DOWN)  // ���ܵ���
//    {
//      Toggle_Pulse-=10;
//			printf("%d",Toggle_Pulse);
//    //  if(Toggle_Pulse<50)  // ����ٶ�����
//    //    Toggle_Pulse=50;
//    }
//    if(KEY3_StateRead() == KEY_DOWN)
//    {
//      Toggle_Pulse+=100;
//      if(Toggle_Pulse>3500)         // �����ٶ�����
//        Toggle_Pulse=3500;
//    }
//    if(KEY4_StateRead() == KEY_DOWN)
//    {
//      if(dir==0)
//      {
//        STEPMOTOR_DIR_REVERSAL();  // ��ת
//        dir=1;
//      }
//      else
//      {
//        STEPMOTOR_DIR_FORWARD();   // ��ת
//        dir=0;
//      }
//    }
//    if(KEY5_StateRead() == KEY_DOWN)
//    {
//      if(ena==1)
//      {
//        STEPMOTOR_OUTPUT_ENABLE();   // ��������
//        HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
//        ena=0;
//      }
//      else
//      {
//        STEPMOTOR_OUTPUT_DISABLE();// ͣ��
//        HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
//        ena=1;
//      }
//    }
 //   if(pulse_count >= STEPMOTOR_MICRO_STEP*200*2*10)  // ת��10Ȧ��ͣ�� 
 //   {
  //    TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_TIM_CHANNEL_x,TIM_CCx_DISABLE);
 //   }
  }
}

/**
  * ��������: ��ʱ���Ƚ�����жϻص�����
  * �������: htim����ʱ�����ָ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  __IO uint32_t count;
  __IO uint32_t tmp;
  count =__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
	
  if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
  {
    tmp = STEPMOTOR_TIM_PERIOD & (count+Toggle_Pulse[0]);
    __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_NO1_TIM_CHANNEL_x,tmp);
    pulse_count[0]++;
  }
  if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
  {
    tmp = STEPMOTOR_TIM_PERIOD & (count+Toggle_Pulse[1]);
    __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_NO2_TIM_CHANNEL_x,tmp);
    pulse_count[1]++;
  }
}

/**
  * ��������: ADCת����ɻص�����
  * �������: hadc��ADC�����豸���
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  DMA_Transfer_Complete_Count++; 
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
