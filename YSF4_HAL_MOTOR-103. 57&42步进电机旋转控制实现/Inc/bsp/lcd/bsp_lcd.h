#ifndef __BSP_LCD_H__
#define	__BSP_LCD_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef enum
{
  USB_FONT_16=16,
  USB_FONT_24=24,
}USB_FONT_Typdef;
#define IS_USB_FONT(FONT)           (((FONT) == USB_FONT_16) || ((FONT) == USB_FONT_24))

/* �궨�� --------------------------------------------------------------------*/
/******************************************************************************
2^26 =0X0400 0000 = 64MB,ÿ�� BANK ��4*64MB = 256MB
64MB:FSMC_Bank1_NORSRAM1:0X6000 0000 ~ 0X63FF FFFF
64MB:FSMC_Bank1_NORSRAM2:0X6400 0000 ~ 0X67FF FFFF
64MB:FSMC_Bank1_NORSRAM3:0X6800 0000 ~ 0X6BFF FFFF
64MB:FSMC_Bank1_NORSRAM4:0X6C00 0000 ~ 0X6FFF FFFF

ѡ��BANK1-BORSRAM4 ���� TFT����ַ��ΧΪ0X6C00 0000 ~ 0X6FFF FFFF
YS-F1P������ѡ�� FSMC_A0 ��LCD��DC(�Ĵ���/����ѡ��)��
�Ĵ�������ַ = 0X6C00 0000
RAM����ַ = 0X6C00 0002 = 0X6C00 0000+(1<<(0+1))
�����·���ʱѡ��ͬ�ĵ�ַ��ʱ����ַҪ���¼���  
*******************************************************************************/
/******************************* ILI9488 ��ʾ���� FSMC �������� ***************/
#define FSMC_LCD_CMD                   ((uint32_t)0x6C000000)	    //FSMC_Bank1_NORSRAM1����LCD��������ĵ�ַ
#define FSMC_LCD_DATA                  ((uint32_t)0x6C000002)      //FSMC_Bank1_NORSRAM1����LCD���ݲ����ĵ�ַ      
#define LCD_WRITE_CMD(x)               *(__IO uint16_t *)FSMC_LCD_CMD  = x 
#define LCD_WRITE_DATA(x)              *(__IO uint16_t *)FSMC_LCD_DATA = x
#define LCD_READ_DATA()                *(__IO uint16_t *)FSMC_LCD_DATA

#define FSMC_LCD_BANKx                 FSMC_NORSRAM_BANK4

/************************* ILI9488 ��ʾ��8080ͨѶ���Ŷ��� *********************/
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

/**************** ��ʾ����ѡ�񣬿�ѡ(1,2,3,4)�ĸ����� *************************/
#define LCD_DIRECTION                  1  // ԭ������Ļ���Ͻ� X*Y=320*480
//#define LCD_DIRECTION                  2  // ԭ������Ļ���Ͻ� X*Y=480*320
//#define LCD_DIRECTION                  3  // ԭ������Ļ���½� X*Y=320*480
//#define LCD_DIRECTION                  4  // ԭ������Ļ���½� X*Y=480*320

/******** ILI934 ��ʾ��ȫ��Ĭ�ϣ�ɨ�跽��Ϊ1ʱ������Ⱥ����߶�*************/
#if (LCD_DIRECTION==1)||(LCD_DIRECTION==3)

 #define LCD_DEFAULT_WIDTH		         320  // X�᳤��
 #define LCD_DEFAULT_HEIGTH         	 480  // Y�᳤��
 
#else

 #define LCD_DEFAULT_WIDTH		         480  // X�᳤��
 #define LCD_DEFAULT_HEIGTH         	 320  // Y�᳤��
 
#endif

/******************************* ���� ILI9488 ��ʾ��������ɫ ********************************/
#define BACKGROUND		                 WHITE     //Ĭ�ϱ�����ɫ


#define BLUE                          (uint16_t)0x001F	   //��ɫ 
#define GREEN                         (uint16_t)0x07E0	   //��ɫ 
#define RED                           (uint16_t)0xF800	   //��ɫ 
#define CYAN                          (uint16_t)0x07FF	   //����ɫ����ɫ
#define MAGENTA                       (uint16_t)0xF81F	   //����ɫ�����ɫ
#define YELLOW                        (uint16_t)0xFFE0	   //��ɫ 
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
#define WHITE                         (uint16_t)0xFFFF	   //��ɫ
#define LIGHTGRAY                     (uint16_t)0xD69A
#define GRAY                          (uint16_t)0x8410	   //��ɫ 
#define DARKGRAY                      (uint16_t)0x4208
#define BLACK                         (uint16_t)0x0000	   //��ɫ 
#define BROWN                         (uint16_t)0xA145
#define ORANGE                        (uint16_t)0xFD20


#define GBK_HZ1616_ADDR       (10*4096)   // GBK_HZ1616.FON�ֿ��ļ�����ڴ���Flash�ĵ�ַ
#define GBK_HZ2424_ADDR       (75*4096)   // GBK_HZ2424.FON�ֿ��ļ�����ڴ���Flash�ĵ�ַ

/* ��չ���� ------------------------------------------------------------------*/
extern SRAM_HandleTypeDef hlcd;

/* �������� ------------------------------------------------------------------*/
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

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
