#ifndef __BSP_LCD_H__
#define	__BSP_LCD_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
  USB_FONT_16=16,
  USB_FONT_24=24,
}USB_FONT_Typdef;
#define IS_USB_FONT(FONT)           (((FONT) == USB_FONT_16) || ((FONT) == USB_FONT_24))

/* 宏定义 --------------------------------------------------------------------*/
/******************************************************************************
2^26 =0X0400 0000 = 64MB,每个 BANK 有4*64MB = 256MB
64MB:FSMC_Bank1_NORSRAM1:0X6000 0000 ~ 0X63FF FFFF
64MB:FSMC_Bank1_NORSRAM2:0X6400 0000 ~ 0X67FF FFFF
64MB:FSMC_Bank1_NORSRAM3:0X6800 0000 ~ 0X6BFF FFFF
64MB:FSMC_Bank1_NORSRAM4:0X6C00 0000 ~ 0X6FFF FFFF

选择BANK1-BORSRAM4 连接 TFT，地址范围为0X6C00 0000 ~ 0X6FFF FFFF
YS-F1P开发板选择 FSMC_A0 接LCD的DC(寄存器/数据选择)脚
寄存器基地址 = 0X6C00 0000
RAM基地址 = 0X6C00 0002 = 0X6C00 0000+(1<<(0+1))
如果电路设计时选择不同的地址线时，地址要重新计算  
*******************************************************************************/
/******************************* ILI9488 显示屏的 FSMC 参数定义 ***************/
#define FSMC_LCD_CMD                   ((uint32_t)0x6C000000)	    //FSMC_Bank1_NORSRAM1用于LCD命令操作的地址
#define FSMC_LCD_DATA                  ((uint32_t)0x6C000002)      //FSMC_Bank1_NORSRAM1用于LCD数据操作的地址      
#define LCD_WRITE_CMD(x)               *(__IO uint16_t *)FSMC_LCD_CMD  = x 
#define LCD_WRITE_DATA(x)              *(__IO uint16_t *)FSMC_LCD_DATA = x
#define LCD_READ_DATA()                *(__IO uint16_t *)FSMC_LCD_DATA

#define FSMC_LCD_BANKx                 FSMC_NORSRAM_BANK4

/************************* ILI9488 显示屏8080通讯引脚定义 *********************/
#define FSMC_LCD_CS_GPIO_ClK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE() 
#define FSMC_LCD_CS_PORT               GPIOG
#define FSMC_LCD_CS_PIN                GPIO_PIN_12

#define FSMC_LCD_DC_GPIO_ClK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define FSMC_LCD_DC_PORT               GPIOF
#define FSMC_LCD_DC_PIN                GPIO_PIN_0

#define FSMC_LCD_BK_GPIO_ClK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()   
#define FSMC_LCD_BK_PORT               GPIOC
#define FSMC_LCD_BK_PIN                GPIO_PIN_9
 
#define LCD_BK_ON()                    HAL_GPIO_WritePin(FSMC_LCD_BK_PORT, FSMC_LCD_BK_PIN, GPIO_PIN_SET);
#define LCD_BK_OFF()                   HAL_GPIO_WritePin(FSMC_LCD_BK_PORT, FSMC_LCD_BK_PIN, GPIO_PIN_RESET);

/**************** 显示方向选择，可选(1,2,3,4)四个方向 *************************/
#define LCD_DIRECTION                  1  // 原点在屏幕左上角 X*Y=320*480
//#define LCD_DIRECTION                  2  // 原点在屏幕右上角 X*Y=480*320
//#define LCD_DIRECTION                  3  // 原点在屏幕右下角 X*Y=320*480
//#define LCD_DIRECTION                  4  // 原点在屏幕左下角 X*Y=480*320

/******** ILI934 显示屏全屏默认（扫描方向为1时）最大宽度和最大高度*************/
#if (LCD_DIRECTION==1)||(LCD_DIRECTION==3)

 #define LCD_DEFAULT_WIDTH		         320  // X轴长度
 #define LCD_DEFAULT_HEIGTH         	 480  // Y轴长度
 
#else

 #define LCD_DEFAULT_WIDTH		         480  // X轴长度
 #define LCD_DEFAULT_HEIGTH         	 320  // Y轴长度
 
#endif

/******************************* 定义 ILI9488 显示屏常用颜色 ********************************/
#define BACKGROUND		                 WHITE     //默认背景颜色


#define BLUE                          (uint16_t)0x001F	   //蓝色 
#define GREEN                         (uint16_t)0x07E0	   //绿色 
#define RED                           (uint16_t)0xF800	   //红色 
#define CYAN                          (uint16_t)0x07FF	   //蓝绿色，青色
#define MAGENTA                       (uint16_t)0xF81F	   //红紫色，洋红色
#define YELLOW                        (uint16_t)0xFFE0	   //黄色 
#define LIGHTBLUE                     (uint16_t)0x841F
#define LIGHTGREEN                    (uint16_t)0x87F0
#define LIGHTRED                      (uint16_t)0xFC10
#define LIGHTCYAN                     (uint16_t)0x87FF
#define LIGHTMAGENTA                  (uint16_t)0xFC1F
#define LIGHTYELLOW                   (uint16_t)0xFFF0
#define DARKBLUE                      (uint16_t)0x0010
#define DARKGREEN                     (uint16_t)0x0400
#define DARKRED                       (uint16_t)0x8000
#define DARKCYAN                      (uint16_t)0x0410
#define DARKMAGENTA                   (uint16_t)0x8010
#define DARKYELLOW                    (uint16_t)0x8400
#define WHITE                         (uint16_t)0xFFFF	   //白色
#define LIGHTGRAY                     (uint16_t)0xD69A
#define GRAY                          (uint16_t)0x8410	   //灰色 
#define DARKGRAY                      (uint16_t)0x4208
#define BLACK                         (uint16_t)0x0000	   //黑色 
#define BROWN                         (uint16_t)0xA145
#define ORANGE                        (uint16_t)0xFD20


#define GBK_HZ1616_ADDR       (10*4096)   // GBK_HZ1616.FON字库文件存放在串行Flash的地址
#define GBK_HZ2424_ADDR       (75*4096)   // GBK_HZ2424.FON字库文件存放在串行Flash的地址

/* 扩展变量 ------------------------------------------------------------------*/
extern SRAM_HandleTypeDef hlcd;

/* 函数声明 ------------------------------------------------------------------*/
uint32_t BSP_LCD_Init(void);
void LCD_SetDirection(uint8_t ucOtion);
void LCD_OpenWindow(uint16_t usX,uint16_t usY,uint16_t usWidth,uint16_t usHeight);
void LCD_Clear(uint16_t usX,uint16_t usY,uint16_t usWidth,uint16_t usHeight,uint16_t usColor);
void LCD_SetPointPixel(uint16_t usX,uint16_t usY,uint16_t usColor);
uint16_t LCD_GetPointPixel(uint16_t usX,uint16_t usY);
void LCD_DrawLine(uint16_t usX1,uint16_t usY1,uint16_t usX2,uint16_t usY2,uint16_t usColor);
void LCD_DrawRectangle(uint16_t usX_Start,uint16_t usY_Start, uint16_t usWidth,uint16_t usHeight,uint16_t usColor,uint8_t ucFilled);
void LCD_DrawCircle(uint16_t usX_Center,uint16_t usY_Center,uint16_t usRadius,uint16_t usColor,uint8_t ucFilled);
void LCD_DispChar_EN(uint16_t usX,uint16_t usY,const char cChar,uint16_t usColor_Background,uint16_t usColor_Foreground,USB_FONT_Typdef font);
void LCD_DispString_EN(uint16_t usX,uint16_t usY,const char *pStr,uint16_t usColor_Background,uint16_t usColor_Foreground,USB_FONT_Typdef font);
void LCD_DispCHAR_CH( uint16_t usX, uint16_t usY,const uint8_t *pstr,uint16_t usColor_Background, uint16_t usColor_Foreground,USB_FONT_Typdef font);
void LCD_DispString_CH(uint16_t usX,uint16_t usY,const uint8_t *pstr,uint16_t usColor_Background, uint16_t usColor_Foreground,USB_FONT_Typdef font);
void LCD_DispString_EN_CH(uint16_t usX,uint16_t usY,const uint8_t *pstr,uint16_t usColor_Background, uint16_t usColor_Foreground,USB_FONT_Typdef font);

#endif /* __BSP_LCD_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
