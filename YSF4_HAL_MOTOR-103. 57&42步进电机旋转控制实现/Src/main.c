#include "stm32f4xx_hal.h"
#include "StepMotor/bsp_STEPMOTOR.h"
#include "key/bsp_key.h"
#include "usart/bsp_debug_usart.h"
#include "adc/bsp_adc.h"
#include "lcd/bsp_lcd.h"
#include "stdlib.h"
#include "spiflash/bsp_spiflash.h"

#define STEPMOTOR_MICRO_STEP      32 																				 // 步进电机驱动器细分，必须与驱动器实际设置对应

uint8_t dir=0; 																																// 0 ：顺时针   1：逆时针 
uint8_t ena=0; 																															// 0 ：正常运行 1：停机

__IO float ADC_ConvertedValueLocal[ADC_CHANNEL_NUMBER];											// 用于保存转换计算后的电压值	 

uint32_t ADC_ConvertedValue[ADC_CHANNEL_NUMBER];														// AD转换结果值
uint32_t DMA_Transfer_Complete_Count=0;	

extern __IO uint16_t Toggle_Pulse[2]; 																					/* 步进电机速度控制，可调节范围为 300 -- 3500 ，值越小速度越快 */

/*
*    当步进电机驱动器细分设置为1时，每200个脉冲步进电机旋转一周
*                          为32时，每6400个脉冲步进电机旋转一周
*    下面以设置为32时为例讲解：
*    pulse_count用于记录输出脉冲数量，pulse_count为脉冲数的两倍，
*    比如当pulse_count=12800时，实际输出6400个完整脉冲。
*    这样可以非常方便步进电机的实际转动圈数，就任意角度都有办法控制输出。
*    如果步进电机驱动器的细分设置为其它值，pulse_count也要做相应处理
*
*/

__IO uint32_t pulse_count[2]; 																								/*  脉冲计数，一个完整的脉冲会增加2 */

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     // 使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // 设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        // 打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    // 打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            // PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 // 8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               // 336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     // 2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 // USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                 // 配置并启动系统滴答定时器
 
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);								 /* 系统滴答定时器时钟源 */

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);															  /* 系统滴答定时器中断优先级配置 */
}

int main(void)
{  
	uint8_t i;
	uint32_t lcdid;
	char str[50];  
	uint16_t count;
	float pulse_temp1, pulse_temp2;
	float temp1, temp_raw1, temp2, temp_raw2;
 
  HAL_Init();																							 /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  SystemClock_Config();																			  /* 配置系统时钟 */
	
  KEY_GPIO_Init();
  MX_DEBUG_USART_Init();															  /* 初始化串口并配置串口中断优先级 */ 
  lcdid=BSP_LCD_Init(); 														 /* 初始化3.5寸TFT液晶模组，一般优先于调试串口初始化 */
  srand(0xffff);   /* 初始化随机种子 */
	MX_DMA_Init();
  MX_ADCx_Init();															 /* ADC 初始化 */

  HAL_ADC_Start_DMA(&hadcx,ADC_ConvertedValue,ADC_CHANNEL_NUMBER);  					/* 启动AD转换并使能DMA传输和中断 */
  STEPMOTOR_TIMx_Init();									 /* 高级控制定时器初始化并配置PWM输出功能 */
  HAL_TIM_Base_Start(&htimx_STEPMOTOR);												 /* 确定定时器 */
  HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_NO1_TIM_CHANNEL_x);									/* 使能中断 关闭比较输出*/
  TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_DISABLE);
	
	HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_NO2_TIM_CHANNEL_x);									/* 使能中断 关闭比较输出*/
  TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_DISABLE);

	LCD_Clear(0,0,LCD_DEFAULT_WIDTH,LCD_DEFAULT_HEIGTH,BLACK);
  LCD_BK_ON();	 /* 开背光 */
	
//  /* 显示英文字符 */
//  LCD_DispChar_EN(50,50,'Y',BLUE,YELLOW,USB_FONT_16);
//  LCD_DispChar_EN(70,50,'S',BLUE,YELLOW,USB_FONT_16);
//  LCD_DispChar_EN(50,70,'Y',YELLOW,BLUE,USB_FONT_24);
//  LCD_DispChar_EN(70,70,'S',YELLOW,BLUE,USB_FONT_24);
//  
//  /* 显示英文字符串 */
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
		
		ADC_ConvertedValueLocal[0] =(double)(ADC_ConvertedValue[0]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]只取最低12有效数据
	  ADC_ConvertedValueLocal[1] =(double)(ADC_ConvertedValue[1]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]只取最低12有效数据
		
		//stop
		if (ADC_ConvertedValueLocal[0] > 1.5 && ADC_ConvertedValueLocal[0] < 1.8 && ADC_ConvertedValueLocal[1] > 1.5 && ADC_ConvertedValueLocal[1] < 1.8) {
			
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			
			temp_raw1 = 1.6 - ADC_ConvertedValueLocal[0];
			temp1 = (int)(temp_raw1 * 10.0 + 0.5) / 10.0;
			
			temp_raw2 = 1.6 - ADC_ConvertedValueLocal[1];
			temp2 = (int)(temp_raw2 * 10.0 + 0.5) / 10.0;
			
			sprintf(str,"ADC0-X：%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1：%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1：%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1：%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y：%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2：%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2：%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2：%d",Toggle_Pulse[1]);  
			LCD_DispString_EN_CH(10,240,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			LCD_DispString_EN(10,300,"status: STOP    ",RED,GREEN,USB_FONT_24);
		}
		
		//forward
		else if (ADC_ConvertedValueLocal[0] < 1.5 && ADC_ConvertedValueLocal[1] > 1.5 && ADC_ConvertedValueLocal[1] < 1.8) {
			
	//		if (dir == 1) {
				 STEPMOTOR_DIR1_FORWARD();   // 正转
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
			
			sprintf(str,"ADC0-X：%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1：%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1：%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1：%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y：%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2：%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2：%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2：%d",Toggle_Pulse[1]);  
			LCD_DispString_EN_CH(10,240,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			LCD_DispString_EN(10,300,"status: FOEWARD ",GREEN,BLACK,USB_FONT_24);
		}
		
		//backward
		else if (ADC_ConvertedValueLocal[0] >= 1.8 && ADC_ConvertedValueLocal[1] > 1.5 && ADC_ConvertedValueLocal[1] < 1.8) {
			
//			dir = 1;
			STEPMOTOR_DIR1_REVERSAL();  // 反转
			STEPMOTOR_DIR2_REVERSAL();  // 反转
			
			temp_raw1 = ADC_ConvertedValueLocal[0] - 1.6;
			temp1 = (int)(temp_raw1 * 10.0 + 0.5) / 10.0;
			
			temp_raw2 = ADC_ConvertedValueLocal[0] - 1.6;
			temp2 = (int)(temp_raw2 * 10.0 + 0.5) / 10.0;
			
			Toggle_Pulse[0] = 800 - temp1 * 200;
			Toggle_Pulse[1] = 800 - temp2 * 200;
			
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			
			sprintf(str,"ADC0-X：%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1：%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1：%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1：%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y：%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2：%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2：%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2：%d",Toggle_Pulse[1]);  
			LCD_DispString_EN_CH(10,240,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			LCD_DispString_EN(10,300,"status: BACKWARD",GREEN,BLACK,USB_FONT_24);
		}
		
		//左转
		else if (ADC_ConvertedValueLocal[0] > 1.5 && ADC_ConvertedValueLocal[0] < 1.8 && ADC_ConvertedValueLocal[1] >= 1.8) {
			
			STEPMOTOR_DIR1_REVERSAL();   // 正转
			STEPMOTOR_DIR2_FORWARD();
			
			temp_raw1 = ADC_ConvertedValueLocal[1] - 1.6;
			temp1 = (int)(temp_raw1 * 10.0 + 0.5) / 10.0;
			
			temp_raw2 = ADC_ConvertedValueLocal[1] - 1.6;
			temp2 = (int)(temp_raw2 * 10.0 + 0.5) / 10.0;
			
			Toggle_Pulse[0] = 800 - temp1 * 200;
			Toggle_Pulse[1] = 800 - temp2 * 200;

			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			
			sprintf(str,"ADC0-X：%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1：%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1：%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1：%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y：%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2：%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2：%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2：%d",Toggle_Pulse[1]);  
			LCD_DispString_EN_CH(10,240,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			LCD_DispString_EN(10,300,"status: LEFT    ",GREEN,BLACK,USB_FONT_24);
		}
		
		//右转
		else if (ADC_ConvertedValueLocal[0] > 1.5 && ADC_ConvertedValueLocal[0] < 1.8 && ADC_ConvertedValueLocal[1] <= 1.5) {
			
			STEPMOTOR_DIR2_REVERSAL();   // 正转
			STEPMOTOR_DIR1_FORWARD();
			
			temp_raw1 = 1.6 - ADC_ConvertedValueLocal[1];
			temp1 = (int)(temp_raw1 * 10.0 + 0.5) / 10.0;
			
			temp_raw2 = 1.6 - ADC_ConvertedValueLocal[1];
			temp2 = (int)(temp_raw2 * 10.0 + 0.5) / 10.0;
			
			Toggle_Pulse[0] = 800 - temp1 * 200;
			Toggle_Pulse[1] = 800 - temp2 * 200;

			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			
			sprintf(str,"ADC0-X：%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1：%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1：%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1：%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y：%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2：%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2：%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2：%d",Toggle_Pulse[1]);  
			LCD_DispString_EN_CH(10,240,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			LCD_DispString_EN(10,300,"status: RIGHT   ",GREEN,BLACK,USB_FONT_24);
		}
		else {
			
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			
		  sprintf(str,"ADC0-X：%f",ADC_ConvertedValueLocal[0]);  
			LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw1：%f",temp_raw1);  
			LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp1：%f",temp1);  
			LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse1：%d",Toggle_Pulse[0]);  
			LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"ADC1-Y：%f",ADC_ConvertedValueLocal[1]);  
			LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp_raw2：%f",temp_raw2);  
			LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"temp2：%f",temp2);  
			LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
			sprintf(str,"pulse2：%d",Toggle_Pulse[1]);  
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
//			sprintf(str,"ADC[0]-X axis：%f",ADC_ConvertedValueLocal[0]);  
//			LCD_DispString_EN_CH(10,50,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
//			sprintf(str,"pulse_temp：%f",pulse_temp);  
//			LCD_DispString_EN_CH(10,100,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
//		  
//			LCD_DispString_EN(10,150,"status: STOP",RED,GREEN,USB_FONT_24);
//		}
//		else {
//			if (ADC_ConvertedValueLocal[0] >= 0 && ADC_ConvertedValueLocal[0] <= 1.5){
//				if(pulse_temp < 320)		pulse_temp = 320;
//		
//				sprintf(str,"ADC[0]-X axis：%f",ADC_ConvertedValueLocal[0]);  
//				LCD_DispString_EN_CH(10,50,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
//				sprintf(str,"pulse_temp：%f",pulse_temp);  
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
//				sprintf(str,"ADC[0]-X axis：%f",ADC_ConvertedValueLocal[0]);  
//				LCD_DispString_EN_CH(10,50,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
//				sprintf(str,"pulse_temp：%f",pulse_temp);  
//				LCD_DispString_EN_CH(10,100,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
//		
//				LCD_DispString_EN(10,150,"status: RUN ",GREEN,WHITE,USB_FONT_24);
//				
//				 STEPMOTOR_DIR_REVERSAL();  // 反转
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
//      /* 3.3为AD转换的参考电压值，stm32的AD转换为12bit，2^12=4096，
//         即当输入为3.3V时，AD转换结果为4096 */    
//      ADC_ConvertedValueLocal[i] =(double)(ADC_ConvertedValue[i]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]只取最低12有效数据
//    }
//    
//    for(i=0;i<ADC_CHANNEL_NUMBER;i++)
//    {
//      printf("CH%d value = %d -> %.1fV\r\n",i,ADC_ConvertedValue[i]&0xFFF,(int)(ADC_ConvertedValueLocal[i] * 10.0 + 0.5) / 10.0);
//    }   
//    
//    printf("已经完成AD转换次数：%d\r\n",DMA_Transfer_Complete_Count);
//    DMA_Transfer_Complete_Count=0;
//    printf("\n");   
//    
		
		
//    if(KEY1_StateRead() == KEY_DOWN)  
//    {
//      pulse_count=0;            
//      ena=0;          
//      TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_TIM_CHANNEL_x,TIM_CCx_ENABLE);
//    }
//    if(KEY2_StateRead() == KEY_DOWN)  // 功能调节
//    {
//      Toggle_Pulse-=10;
//			printf("%d",Toggle_Pulse);
//    //  if(Toggle_Pulse<50)  // 最快速度限制
//    //    Toggle_Pulse=50;
//    }
//    if(KEY3_StateRead() == KEY_DOWN)
//    {
//      Toggle_Pulse+=100;
//      if(Toggle_Pulse>3500)         // 最慢速度限制
//        Toggle_Pulse=3500;
//    }
//    if(KEY4_StateRead() == KEY_DOWN)
//    {
//      if(dir==0)
//      {
//        STEPMOTOR_DIR_REVERSAL();  // 反转
//        dir=1;
//      }
//      else
//      {
//        STEPMOTOR_DIR_FORWARD();   // 正转
//        dir=0;
//      }
//    }
//    if(KEY5_StateRead() == KEY_DOWN)
//    {
//      if(ena==1)
//      {
//        STEPMOTOR_OUTPUT_ENABLE();   // 正常运行
//        HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
//        ena=0;
//      }
//      else
//      {
//        STEPMOTOR_OUTPUT_DISABLE();// 停机
//        HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
//        ena=1;
//      }
//    }
 //   if(pulse_count >= STEPMOTOR_MICRO_STEP*200*2*10)  // 转动10圈后停机 
 //   {
  //    TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_TIM_CHANNEL_x,TIM_CCx_DISABLE);
 //   }
  }
}

/**
  * 函数功能: 定时器比较输出中断回调函数
  * 输入参数: htim：定时器句柄指针
  * 返 回 值: 无
  * 说    明: 无
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
  * 函数功能: ADC转换完成回调函数
  * 输入参数: hadc：ADC外设设备句柄
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  DMA_Transfer_Complete_Count++; 
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
