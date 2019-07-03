#include "stm32f4xx_hal.h"
#include "StepMotor/bsp_STEPMOTOR.h"
#include "key/bsp_key.h"
#include "usart/bsp_debug_usart.h"
#include "adc/bsp_adc.h"
#include "lcd/bsp_lcd.h"
#include "stdlib.h"
#include "spiflash/bsp_spiflash.h"
#include "led/bsp_led.h"
#include "beep/bsp_beep.h"
#include "bsp/bmp/bsp_bmp.h"
#include "bsp/bmp/pic8.h"


#define STEPMOTOR_MICRO_STEP      32											// 步进电机驱动器细分，必须与驱动器实际设置对应
#define SPEED_SCALE				530
#define SPEED_SCALE_ROT		870
#define SPEED_SCALE_JUMP	200

#define w	'w'
#define s 's'
#define a 'a'
#define d 'd'
#define q 'q'
#define n 'n'
#define ALPHA_SCALE 0.08

#define STATUS_STOP	0
#define STATUS_FORWARD 1
#define STATUS_BACKWARD	2
#define STATUS_LEFT	3
#define STATUS_RIGHT	4
uint8_t status = 0;
uint8_t status_pre = 0;

uint8_t stop_flag = 0;
uint16_t pulse_store[2];
char str[50]; 	//lcd显示字符串
__IO float adc_final[2];	//对adc采集值进行处理去漂后的值
__IO float ADC_ConvertedValueLocal[ADC_CHANNEL_NUMBER];		// 用于保存转换计算后的电压值	 
__IO float adc_filter;
__IO float adc_now, adc_pre;
__IO uint16_t tim6_count=0;

float rotation;	//摇杆旋转值
uint32_t count_stop = 0;
uint32_t count_forward = 0;
uint32_t count_backward = 0;
uint32_t count_left = 0;
uint32_t count_right = 0;

uint32_t ADC_ConvertedValue[ADC_CHANNEL_NUMBER];					// AD转换结果值
uint32_t DMA_Transfer_Complete_Count=0;	

extern __IO uint16_t Toggle_Pulse[2]; 										/* 步进电机速度控制，可调节范围为 300 -- 3500 ，值越小速度越快 */
uint16_t Toggle_Pulse_Temp[2];

__IO uint32_t pulse_count[2]; 														/*  脉冲计数，一个完整的脉冲会增加2 */

void display(char display_flag);
	
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

float filter(float alpha, float adc_now, float adc_pre){
	return alpha * adc_now + (1 - alpha) * adc_pre;
}

float __abs(float value){
	return value > 0 ? value : -value;
}

int main(void)
{  
	uint16_t count;		//lcd显示随机变量使用
	uint8_t led_count;	//流水灯使用
		
  HAL_Init();																					 /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  SystemClock_Config();																			  /* 配置系统时钟 */
	
	/* 板载LED初始化 */
  LED_GPIO_Init();
  
  BASIC_TIMx_Init();
	
//	HAL_TIM_Base_Start_IT(&htimx);
	
  /* 板载蜂鸣器初始化 */
  BEEP_GPIO_Init();
	
  KEY_GPIO_Init();
  MX_DEBUG_USART_Init();															  /* 初始化串口并配置串口中断优先级 */ 
  BSP_LCD_Init(); 														 /* 初始化3.5寸TFT液晶模组，一般优先于调试串口初始化 */
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
	
	//开机各种启动
  LCD_Fill_Pic(0,0,320,480,gImage_gg);
	LED1_ON;
	LED2_ON;
	LED3_ON;
	BEEP_ON;
	HAL_Delay(800);
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;

	BEEP_StateSet(BEEPState_OFF); 
  LCD_Clear(0,0,LCD_DEFAULT_WIDTH,LCD_DEFAULT_HEIGTH,BLACK);
 
  while (1)
  {		
		HAL_Delay(10);
		
		//开机先开启流水灯
		if (led_count > 150)		led_count = 0;
		switch(led_count){
			case 0: LED3_OFF; LED1_ON; break;
			case 50: LED1_OFF; LED2_ON; break;
			case 100: LED2_OFF; LED3_ON; break;
		}
		led_count ++;
		
		//lcd显示随机变量
		count=rand();
	
		//获取adc的值
		ADC_ConvertedValueLocal[0] =(double)(ADC_ConvertedValue[0]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]只取最低12有效数据
	  ADC_ConvertedValueLocal[1] =(double)(ADC_ConvertedValue[1]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]只取最低12有效数据
		ADC_ConvertedValueLocal[2] =(double)(ADC_ConvertedValue[2]&0xFFF)*3.3/4096; // ADC_ConvertedValue[0]只取最低12有效数据
		
		//对adc的原始数据进行去漂处理，得到的值是稳定的
		adc_final[0] = (int)(ADC_ConvertedValueLocal[0] * 10.0 + 0.5) / 10.0;		
		adc_final[1] = (int)(ADC_ConvertedValueLocal[1] * 10.0 + 0.5) / 10.0;	
		rotation = (int)(ADC_ConvertedValueLocal[2] * 10.0 + 0.5) / 10.0;		
				
		adc_filter = filter(ALPHA_SCALE, adc_now, adc_pre);
		
		Toggle_Pulse[0] = Toggle_Pulse[1] = (SPEED_SCALE - __abs(adc_filter - 1.5) * SPEED_SCALE_JUMP) > 225 ? (SPEED_SCALE - __abs(adc_filter - 1.5) * SPEED_SCALE_JUMP) : 225;
		
#ifndef DEBUG		
		
		//停止状态
		if (adc_final[0] >= 1.25 && adc_final[0] <= 1.7 && adc_final[1] >= 1.4 && adc_final[1] <= 1.8) {
		  //关闭电机
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			status = 0;
			status_pre = 0;
			count_stop = 0;
			count_forward = 0;
			count_backward = 0;
			count_left = 0;
			count_right = 0;
			adc_now = adc_pre = adc_final[0];
			
			display(q);
		}
		
		//前进
		else if (ADC_ConvertedValueLocal[0] >= 2.0 && ADC_ConvertedValueLocal[1] >= 0.18 && ADC_ConvertedValueLocal[1] <= 3.0) {
			
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			
			STEPMOTOR_DIR2_FORWARD();	
			STEPMOTOR_DIR1_REVERSAL();
			
			count_stop = 0;
			count_backward = 0;
			count_left = 0;
			count_right = 0;
			count_forward++;
			if(count_forward == 1){
				//status_pre = status;
			//	status = STATUS_FORWARD;
				adc_now = adc_pre = 1.5;		
				BEEP_ON;
			//HAL_Delay(50);
			}
			else{  
				adc_pre = adc_filter;
				adc_now = adc_final[0];
			}
			BEEP_OFF;
			display(w);
		}
			
		
		//后退
		else if (adc_final[0] <= 1.2 && adc_final[1] >= 0.3 && adc_final[1] <= 3.0) {
				status_pre = status;
				status = STATUS_BACKWARD;
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			
			STEPMOTOR_DIR1_FORWARD();	
			STEPMOTOR_DIR2_REVERSAL();
			count_backward++;
			count_stop = 0;
		//	count_backward = 0;
			count_left = 0;
			count_right = 0;
			count_forward = 0;
			if(count_backward == 1){
				//status_pre = status;
			//	status = STATUS_FORWARD;
				adc_now = adc_pre = 1.5;	
				BEEP_ON;	
//HAL_Delay(50);			
			}
			else{  
				adc_pre = adc_filter;
				adc_now = adc_final[0];
			}
			BEEP_OFF;
			display(s);
	}
				
		//左转
		else if (ADC_ConvertedValueLocal[0] >= 0.17 && ADC_ConvertedValueLocal[0] <= 3.1 && ADC_ConvertedValueLocal[1] <= 1.2) {
			status_pre = status;
				status = STATUS_LEFT;
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			count_left++;
			STEPMOTOR_DIR2_FORWARD();	
			STEPMOTOR_DIR1_FORWARD();
		  count_stop = 0;
			count_backward = 0;
		//	count_left = 0;
			count_right = 0;
			count_forward = 0;
			if(count_left == 1){
				//status_pre = status;
			//	status = STATUS_FORWARD;
				adc_now = adc_pre = 1.5;	
			BEEP_ON;	//HAL_Delay(50);	
			}
			else{  
				adc_pre = adc_filter;
				adc_now = adc_final[0];
			}
			BEEP_OFF;
			HAL_TIM_Base_Start_IT(&htimx);
			display(a);
		}
			
		
		//右转
		else if (adc_final[0] >= 0.2 && adc_final[0] <= 3.0 && adc_final[1] >= 1.9) {
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_ENABLE);
			status_pre = status;
				status = STATUS_RIGHT;
		  STEPMOTOR_DIR1_REVERSAL();	
			STEPMOTOR_DIR2_REVERSAL();
			count_right++;
			count_stop = 0;
			count_backward = 0;
			count_left = 0;
		//	count_right = 0;
			count_forward = 0;
			if(count_right == 1){
				//status_pre = status;
			//	status = STATUS_FORWARD;
				adc_now = adc_pre = 1.5;	
				BEEP_ON;//HAL_Delay(50);
			}
			else{  
				adc_pre = adc_filter;
				adc_now = adc_final[0];
			}
			BEEP_OFF;
			display(d);
		}
		
		//未知方向，用来处理不同方向之间的过渡	
		else {
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO1_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx,STEPMOTOR_NO2_TIM_CHANNEL_x,TIM_CCx_DISABLE);
			//if(adc_final[0] > 2.)
			adc_now = adc_pre = 1.5;
			count_stop = 0;
			count_forward = 0;
			count_backward = 0;
			count_left = 0;
			count_right = 0;
			display(n);
		
		}
#endif

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  tim6_count++;
	if(tim6_count == 0){
			pulse_store[0] = Toggle_Pulse_Temp[0];
	}
	if(tim6_count == 100){
		tim6_count = 0;
		pulse_store[1] = Toggle_Pulse_Temp[0];
	}
//	if(pulse_store[1] - pulse_store[0] < 0)	BEEP_ON;
//	else	BEEP_OFF;
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

void display(char display_flag){
		sprintf(str,"adc_final[0]：%f",adc_final[0]);  
		LCD_DispString_EN_CH(10,0,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
		
		sprintf(str,"Toggle_Pulse_Temp[0]：%d     ",Toggle_Pulse_Temp[0]);  
		LCD_DispString_EN_CH(10,60,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
		
		sprintf(str,"Toggle_Pulse[0]：%d     ",Toggle_Pulse[0]);  
		LCD_DispString_EN_CH(10,90,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
	
		sprintf(str,"adc_final[1]：%f",adc_final[1]);  
		LCD_DispString_EN_CH(10,150,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
			
		sprintf(str,"Toggle_Pulse_Temp[1]：%d     ",Toggle_Pulse_Temp[1]);  
		LCD_DispString_EN_CH(10,180,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
		
		sprintf(str,"Toggle_Pulse[1]：%d     ",Toggle_Pulse[1]);  
		LCD_DispString_EN_CH(10,210,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
	
		sprintf(str,"count_forward：%d     ",count_forward);  
		LCD_DispString_EN_CH(10,270,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
	
		sprintf(str,"adc_pre：%f     ",adc_pre);  
		LCD_DispString_EN_CH(10,300,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
	
		sprintf(str,"adc_now：%f     ",adc_now);  
		LCD_DispString_EN_CH(10,330,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 

		sprintf(str,"adc_filter：%f     ",adc_filter);  
		LCD_DispString_EN_CH(10,30,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
		
		sprintf(str,"count_right：%d     ",count_right);  
		LCD_DispString_EN_CH(10,360,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
		sprintf(str,"status：%d     ",status);  
		LCD_DispString_EN_CH(10,390,(uint8_t *)str,BLACK,YELLOW,USB_FONT_24); 
		
		switch(display_flag){
			case 'w' : 	LCD_DispString_EN_CH(10,420,(uint8_t *)"状态：前进",GREEN,BLACK,USB_FONT_24); break;
			case 's' : 	LCD_DispString_EN_CH(10,420,(uint8_t *)"状态：后退",GREEN,BLACK,USB_FONT_24); break;
			case 'a' : 	LCD_DispString_EN_CH(10,420,(uint8_t *)"状态：左转",GREEN,BLACK,USB_FONT_24); break;
			case 'd' : 	LCD_DispString_EN_CH(10,420,(uint8_t *)"状态：右转",GREEN,BLACK,USB_FONT_24); break;
			case 'q' : 	LCD_DispString_EN_CH(10,420,(uint8_t *)"状态：停止",RED,WHITE,USB_FONT_24); break;
			case 'n' : 	LCD_DispString_EN_CH(10,420,(uint8_t *)"状态：过渡",RED,BLACK,USB_FONT_24); break;
		}
}


//void rotwork(){
//	if(rotation >= 1.4 && rotation <= 1.7){
//				if(Toggle_Pulse_Temp[0] < 250 && tim6_count < 200){
//					for(int i = 500; i > 230; i--){
//						Toggle_Pulse[0] = i;
//						Toggle_Pulse[1] = i;
//						for(int j = 0; j < 20000; j++){}
//					}
//				}
//				else{
//					Toggle_Pulse[0] = Toggle_Pulse_Temp[0] > 250 ? Toggle_Pulse_Temp[0] : 250;
//					Toggle_Pulse[1] = Toggle_Pulse_Temp[1] > 250 ? Toggle_Pulse_Temp[1] : 250;
//				}
//			}
//			else if(rotation > 1.7){
//				Toggle_Pulse[0] = Toggle_Pulse_Temp[0] + (rotation - 1.5) * 200;
//				Toggle_Pulse[1] = Toggle_Pulse_Temp[1] + (rotation - 1.5) * 200;
//			}
//			else{
//				if((Toggle_Pulse_Temp[0] - (1.5 - rotation) * 200) < 250 && tim6_count < 200){
//					for(int i = 500; i > 230; i--){
//						Toggle_Pulse[0] = i;
//						Toggle_Pulse[1] = i;
//						for(int j = 0; j < 20000; j++){}
//					}
//				}
//				else{
//					Toggle_Pulse[0] = (Toggle_Pulse_Temp[0] - (1.5 - rotation) * 200) > 250 ? (Toggle_Pulse_Temp[0] - (1.5 - rotation) * 200) : 250;
//					Toggle_Pulse[1] = (Toggle_Pulse_Temp[1] - (1.5 - rotation) * 200) > 250 ? (Toggle_Pulse_Temp[1] - (1.5 - rotation) * 200) : 250;
//				}
//			}
//}
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
