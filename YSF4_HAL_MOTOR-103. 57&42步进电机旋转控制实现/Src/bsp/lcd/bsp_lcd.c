/**
  ******************************************************************************
  * �ļ�����: bsp_lcd.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: Һ���ײ���������ʵ��
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "lcd/bsp_lcd.h"
#include "lcd/ascii.h"
#include "spiflash/bsp_spiflash.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
__IO uint32_t lcd_id=0;                // ���浱ǰ��鵽��Һ��ģ��ID
SRAM_HandleTypeDef hlcd;               // SRAM������
static int FSMC_LCD_Initialized = 0;   // FSMC��ʼ����־λ��0��δ��ʼ����1������ɳ�ʼ��
static int FSMC_LCD_DeInitialized = 0; // FSMC����ʼ����־λ��0��δ����ʼ����1������ɷ���ʼ��

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ��ʼ��LCD��IO����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����LCD������ILI9488�൱��һ���ⲿSRAM����
  *           �ú�����HAL_SRAM_MspInit��������
  */
static void HAL_FSMC_LCD_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;	

  /* ����Ѿ���ɳ�ʼ���������ʼ���ڶ��� */
  if(FSMC_LCD_Initialized)
  {
    return;
  }
  FSMC_LCD_Initialized = 1;
  
  /* ʹ����ض˿�ʱ�� */
  FSMC_LCD_CS_GPIO_ClK_ENABLE();
  FSMC_LCD_DC_GPIO_ClK_ENABLE();
  FSMC_LCD_BK_GPIO_ClK_ENABLE();  
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* ʹ��FSMC����ʱ�� */
  __HAL_RCC_FSMC_CLK_ENABLE();

  /** FSMC GPIO Configuration  
  PF0   ------> FSMC_A0
  PE7   ------> FSMC_D4
  PE8   ------> FSMC_D5
  PE9   ------> FSMC_D6
  PE10   ------> FSMC_D7
  PE11   ------> FSMC_D8
  PE12   ------> FSMC_D9
  PE13   ------> FSMC_D10
  PE14   ------> FSMC_D11
  PE15   ------> FSMC_D12
  PD8   ------> FSMC_D13
  PD9   ------> FSMC_D14
  PD10   ------> FSMC_D15
  PD14   ------> FSMC_D0
  PD15   ------> FSMC_D1
  PD0   ------> FSMC_D2
  PD1   ------> FSMC_D3
  PD4   ------> FSMC_NOE
  PD5   ------> FSMC_NWE
  PG12   ------> FSMC_NE4
  */
  GPIO_InitStruct.Pin = FSMC_LCD_DC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;

  HAL_GPIO_Init(FSMC_LCD_DC_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = FSMC_LCD_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FSMC_LCD_CS_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4 
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* ����͵�ƽ�����ⲻ�� */
  HAL_GPIO_WritePin(FSMC_LCD_BK_PORT, FSMC_LCD_BK_PIN, GPIO_PIN_RESET);

  /* Һ������������ų�ʼ�� */
  GPIO_InitStruct.Pin = FSMC_LCD_BK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(FSMC_LCD_BK_PORT, &GPIO_InitStruct);
}

/**
  * ��������: ��ʼ��FSMC��IO����
  * �������: hsram��SRAM������ָ��
  * �� �� ֵ: ��
  * ˵    �����ú�����HAL���ڲ���������
  */	
void HAL_SRAM_MspInit(SRAM_HandleTypeDef* hsram)
{
  HAL_FSMC_LCD_MspInit();
}

/**
  * ��������: ����ʼ��LCD��IO����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����LCD������ILI9488�൱��һ���ⲿSRAM����
  *           �ú�����HAL_SRAM_MspDeInit��������
  */
static void HAL_FSMC_LCD_MspDeInit(void)
{
  /* ����Ѿ���ɷ���ʼ���������ʼ���ڶ��� */
  if(FSMC_LCD_DeInitialized)
  {
    return;
  }
  FSMC_LCD_DeInitialized = 1;
  
  /* ����FSMC����ʱ�� */
  __HAL_RCC_FSMC_CLK_DISABLE();
  
  /** FSMC GPIO Configuration  
  PF0   ------> FSMC_A0
  PE7   ------> FSMC_D4
  PE8   ------> FSMC_D5
  PE9   ------> FSMC_D6
  PE10   ------> FSMC_D7
  PE11   ------> FSMC_D8
  PE12   ------> FSMC_D9
  PE13   ------> FSMC_D10
  PE14   ------> FSMC_D11
  PE15   ------> FSMC_D12
  PD8   ------> FSMC_D13
  PD9   ------> FSMC_D14
  PD10   ------> FSMC_D15
  PD14   ------> FSMC_D0
  PD15   ------> FSMC_D1
  PD0   ------> FSMC_D2
  PD1   ------> FSMC_D3
  PD4   ------> FSMC_NOE
  PD5   ------> FSMC_NWE
  PG12   ------> FSMC_NE4
  */
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15);

  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4 
                          |GPIO_PIN_5);
  HAL_GPIO_DeInit(FSMC_LCD_DC_PORT, FSMC_LCD_DC_PIN);
  HAL_GPIO_DeInit(FSMC_LCD_CS_PORT, FSMC_LCD_CS_PIN);
}

/**
  * ��������: ����ʼ��FSMC��IO����
  * �������: hsram��SRAM������ָ��
  * �� �� ֵ: ��
  * ˵    �����ú�����HAL���ڲ���������
  */	
void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef* hsram)
{
  HAL_FSMC_LCD_MspDeInit();
}

/**
  * ��������: LCD  FSMC ģʽ����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������дʹ����ͬʱ������
  */
void MX_FSMC_Init(void)
{
  FSMC_NORSRAM_TimingTypeDef Timing;

  /* ����FSMC���� */
  hlcd.Instance = FSMC_NORSRAM_DEVICE;
  hlcd.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;

  hlcd.Init.NSBank = FSMC_LCD_BANKx;
  hlcd.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hlcd.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hlcd.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hlcd.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hlcd.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hlcd.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hlcd.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hlcd.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hlcd.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hlcd.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hlcd.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hlcd.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hlcd.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  
  Timing.AddressSetupTime      = 0x02; //��ַ����ʱ��
  Timing.AddressHoldTime       = 0x00; //��ַ����ʱ��
  Timing.DataSetupTime         = 0x05; //���ݽ���ʱ��
  Timing.BusTurnAroundDuration = 0x00;
  Timing.CLKDivision           = 0x00;
  Timing.DataLatency           = 0x00;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  HAL_SRAM_Init(&hlcd, &Timing, &Timing);

  /* Disconnect NADV */
//  __HAL_AFIO_FSMCNADV_DISCONNECTED();
}

/**
  * ��������: ��ʼ��LCD�Ĵ���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������Ҫ������Щ�Ĵ�������Ҫ����ʲôֵ��Һ��������������������أ�
  *           ������Щ�����ɳ����ṩ����ͬ���ҿ��ܲ�ͬ��Ҳ���Ը���ILI9488оƬ
  *           �ֲ����ݲο��޸ġ�
  */
static void ILI9488_REG_Config ( void )
{
  //************* Start Initial Sequence **********//
  /* PGAMCTRL (Positive Gamma Control) (E0h) */
  LCD_WRITE_CMD(0xE0);
  LCD_WRITE_DATA(0x00);
  LCD_WRITE_DATA(0x07);
  LCD_WRITE_DATA(0x10);
  LCD_WRITE_DATA(0x09);
  LCD_WRITE_DATA(0x17);
  LCD_WRITE_DATA(0x0B);
  LCD_WRITE_DATA(0x41);
  LCD_WRITE_DATA(0x89);
  LCD_WRITE_DATA(0x4B);
  LCD_WRITE_DATA(0x0A);
  LCD_WRITE_DATA(0x0C);
  LCD_WRITE_DATA(0x0E);
  LCD_WRITE_DATA(0x18);
  LCD_WRITE_DATA(0x1B);
  LCD_WRITE_DATA(0x0F);

  /* NGAMCTRL (Negative Gamma Control) (E1h)  */
  LCD_WRITE_CMD(0XE1);
  LCD_WRITE_DATA(0x00);
  LCD_WRITE_DATA(0x17);
  LCD_WRITE_DATA(0x1A);
  LCD_WRITE_DATA(0x04);
  LCD_WRITE_DATA(0x0E);
  LCD_WRITE_DATA(0x06);
  LCD_WRITE_DATA(0x2F);
  LCD_WRITE_DATA(0x45);
  LCD_WRITE_DATA(0x43);
  LCD_WRITE_DATA(0x02);
  LCD_WRITE_DATA(0x0A);
  LCD_WRITE_DATA(0x09);
  LCD_WRITE_DATA(0x32);
  LCD_WRITE_DATA(0x36);
  LCD_WRITE_DATA(0x0F);
  
  /* Adjust Control 3 (F7h)  */
  LCD_WRITE_CMD(0XF7);
  LCD_WRITE_DATA(0xA9);
  LCD_WRITE_DATA(0x51);
  LCD_WRITE_DATA(0x2C);
  LCD_WRITE_DATA(0x82);/* DSI write DCS command, use loose packet RGB 666 */

  /* Power Control 1 (C0h)  */
  LCD_WRITE_CMD(0xC0);
  LCD_WRITE_DATA(0x11);
  LCD_WRITE_DATA(0x09);

  /* Power Control 2 (C1h) */
  LCD_WRITE_CMD(0xC1);
  LCD_WRITE_DATA(0x41);

  /* VCOM Control (C5h)  */
  LCD_WRITE_CMD(0XC5);
  LCD_WRITE_DATA(0x00);
  LCD_WRITE_DATA(0x0A);
  LCD_WRITE_DATA(0x80);

  /* Frame Rate Control (In Normal Mode/Full Colors) (B1h) */
  LCD_WRITE_CMD(0xB1);
  LCD_WRITE_DATA(0xB0);
  LCD_WRITE_DATA(0x11);

  /* Display Inversion Control (B4h) */
  LCD_WRITE_CMD(0xB4);
  LCD_WRITE_DATA(0x02);

  /* Display Function Control (B6h)  */
  LCD_WRITE_CMD(0xB6);
  LCD_WRITE_DATA(0x02);
  LCD_WRITE_DATA(0x22);

  /* Entry Mode Set (B7h)  */
  LCD_WRITE_CMD(0xB7);
  LCD_WRITE_DATA(0xc6);

  /* HS Lanes Control (BEh) */
  LCD_WRITE_CMD(0xBE);
  LCD_WRITE_DATA(0x00);
  LCD_WRITE_DATA(0x04);

  /* Set Image Function (E9h)  */
  LCD_WRITE_CMD(0xE9);
  LCD_WRITE_DATA(0x00);
 
  /* ������Ļ����ͳߴ� */
  LCD_SetDirection(LCD_DIRECTION);
  
  /* Interface Pixel Format (3Ah) */
  LCD_WRITE_CMD(0x3A);
  LCD_WRITE_DATA(0x55);/* 0x55 : 16 bits/pixel  */

  /* Sleep Out (11h) */
  LCD_WRITE_CMD(0x11);
  HAL_Delay(120);
  /* Display On */
  LCD_WRITE_CMD(0x29);
}

/**
  * ��������: ��ȡҺ��ģ��ID
  * �������: ��
  * �� �� ֵ: Һ��ģ���ID
  * ˵    ��������ͨ����ȡ04H�Ĵ�����ȡ�õ�Һ��ģ��ID����IDֵ��Һ�����ұ�̣���ͬҺ��
  *           ���ҵ�Һ��ģ��õ���IDֵ���ܲ�ͬ����Ҳ���Էֱ治ͬ�ͺŵ�Һ��ģ�顣
  */
static uint32_t LCD_ReadID(void)
{
	uint16_t buf[4];

	LCD_WRITE_CMD(0x04);  
	buf[0] = LCD_READ_DATA();        // ��һ����ȡ������Ч
	buf[1] = LCD_READ_DATA()&0x00ff; // ֻ�е�8λ������Ч
	buf[2] = LCD_READ_DATA()&0x00ff; // ֻ�е�8λ������Ч
	buf[3] = LCD_READ_DATA()&0x00ff; // ֻ�е�8λ������Ч
	return (buf[1] << 16) + (buf[2] << 8) + buf[3];  
}

/**
  * ��������: Һ��ģ���ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
uint32_t BSP_LCD_Init(void)
{
  MX_FSMC_Init();
  lcd_id=LCD_ReadID();
  if(lcd_id==0x548066 || lcd_id==0x8066)
  {
    ILI9488_REG_Config();
  }
  LCD_Clear(0,0,LCD_DEFAULT_WIDTH,LCD_DEFAULT_HEIGTH,BLACK);
  HAL_Delay(20);
  
  return lcd_id;
}

/**
  * ��������: ����LCD��GRAM��ɨ�跽�� 
  * �������: ucOption ��ѡ��GRAM��ɨ�跽�� 
  *           ��ѡֵ��1 :ԭ������Ļ���Ͻ� X*Y=320*480
  *                   2 :ԭ������Ļ���Ͻ� X*Y=480*320
  *                   3 :ԭ������Ļ���½� X*Y=320*480
  *                   4 :ԭ������Ļ���½� X*Y=480*320
  * �� �� ֵ: ��
  * ˵    ������
  */
void LCD_SetDirection( uint8_t ucOption )
{	
/**
  * Memory Access Control (36h)
  * This command defines read/write scanning direction of the frame memory.
  *
  * These 3 bits control the direction from the MPU to memory write/read.
  *
  * Bit  Symbol  Name  Description
  * D7   MY  Row Address Order     -- ��X�᾵��
  * D6   MX  Column Address Order  -- ��Y�᾵��
  * D5   MV  Row/Column Exchange   -- X����Y�ύ��
  * D4   ML  Vertical Refresh Order  LCD vertical refresh direction control. 
  *
  * D3   BGR RGB-BGR Order   Color selector switch control
  *      (0 = RGB color filter panel, 1 = BGR color filter panel )
  * D2   MH  Horizontal Refresh ORDER  LCD horizontal refreshing direction control.
  * D1   X   Reserved  Reserved
  * D0   X   Reserved  Reserved
  */
	switch ( ucOption )
	{
		case 1:
//   ���Ͻ�->���½� 
//	(0,0)	___ x(320)
//	     |  
//	     |
//       |	y(480) 
			LCD_WRITE_CMD(0x36); 
			LCD_WRITE_DATA(0x08); 
      
			LCD_WRITE_CMD(0x2A); 
			LCD_WRITE_DATA(0x00);	/* x start */	
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x01);  /* x end */	
			LCD_WRITE_DATA(0x3F);

			LCD_WRITE_CMD(0x2B); 
			LCD_WRITE_DATA(0x00);	/* y start */  
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x01);	/* y end */   
			LCD_WRITE_DATA(0xDF);					
		  break;
		
		case 2:
//		���Ͻ�-> ���½�
//		y(320)___ (0,0)            
//		         |
//		         |
//             |x(480)    
			LCD_WRITE_CMD(0x36); 
			LCD_WRITE_DATA(0x68);	
			LCD_WRITE_CMD(0x2A); 
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x01);
			LCD_WRITE_DATA(0xDF);	

			LCD_WRITE_CMD(0x2B); 
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x01);
			LCD_WRITE_DATA(0x3F);				
		  break;
		
		case 3:
//		���½�->���Ͻ�
//		          |y(480)
//		          |           
//		x(320) ___|(0,0)		
			LCD_WRITE_CMD(0x36); 
			LCD_WRITE_DATA(0xC8);	
			LCD_WRITE_CMD(0x2A); 
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x01);
			LCD_WRITE_DATA(0x3F);	

			LCD_WRITE_CMD(0x2B); 
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x01);
			LCD_WRITE_DATA(0x3F);			  
		  break;

		case 4:
//		���½�->���Ͻ�
//		|x(480)
//		|
//		|___ y(320)					  
			LCD_WRITE_CMD(0x36); 
			LCD_WRITE_DATA(0xA8);	
    
			LCD_WRITE_CMD(0x2A); 
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x01);
			LCD_WRITE_DATA(0xDF);	

			LCD_WRITE_CMD(0x2B); 
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x00);
			LCD_WRITE_DATA(0x01);
			LCD_WRITE_DATA(0x3F);				
	    break;		
	}	
	/* ��ʼ��GRAMд������ */
	LCD_WRITE_CMD (0x2C);	
}

/**
  * ��������: ��LCD��ʾ���Ͽ���һ������
  * �������: usX �����ض�ɨ�跽���´��ڵ����X����
  *           usY �����ض�ɨ�跽���´��ڵ����Y����
  *           usWidth �����ڵĿ��
  *           usHeight �����ڵĸ߶�
  * �� �� ֵ: ��
  * ˵    ������
  */
void LCD_OpenWindow(uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight)
{	
	LCD_WRITE_CMD(0x2A ); 				       /* ����X���� */
	LCD_WRITE_DATA(usX>>8);	             /* ������ʼ�㣺�ȸ�8λ */
	LCD_WRITE_DATA(usX&0xff);	           /* Ȼ���8λ */
	LCD_WRITE_DATA((usX+usWidth-1)>>8);  /* ���ý����㣺�ȸ�8λ */
	LCD_WRITE_DATA((usX+usWidth-1)&0xff);/* Ȼ���8λ */

	LCD_WRITE_CMD(0x2B); 			           /* ����Y����*/
	LCD_WRITE_DATA(usY>>8);              /* ������ʼ�㣺�ȸ�8λ */
	LCD_WRITE_DATA(usY&0xff);            /* Ȼ���8λ */
	LCD_WRITE_DATA((usY+usHeight-1)>>8); /* ���ý����㣺�ȸ�8λ */
	LCD_WRITE_DATA((usY+usHeight-1)&0xff);/* Ȼ���8λ */
}

/**
  * ��������: �趨LCD�Ĺ������
  * �������: usX �����ض�ɨ�跽���´��ڵ����X����
  *           usY �����ض�ɨ�跽���´��ڵ����Y����
  * �� �� ֵ: ��
  * ˵    ������
  */
static void LCD_SetCursor(uint16_t usX,uint16_t usY)	
{
	LCD_OpenWindow(usX,usY,1,1);
}

/**
  * ��������: ��LCD��ʾ������ĳһ��ɫ������ص�
  * �������: ulAmout_Point ��Ҫ�����ɫ�����ص������Ŀ
  *           usColor ����ɫ
  * �� �� ֵ: ��
  * ˵    ������
  */
#if defined ( __CC_ARM )  // ʹ��Keil���뻷��
static __inline void LCD_FillColor ( uint32_t ulAmout_Point, uint16_t usColor )
{
	uint32_t i = 0;	
	
	/* ��ʼ��GRAMд������ */
	LCD_WRITE_CMD ( 0x2C );	
	
	for ( i = 0; i < ulAmout_Point; i ++ )
		LCD_WRITE_DATA ( usColor );	
}
#elif defined ( __ICCARM__ ) // ʹ��IAR���뻷��
#pragma inline
static void LCD_FillColor ( uint32_t ulAmout_Point, uint16_t usColor )
{
	uint32_t i = 0;	
	
	/* ��ʼ��GRAMд������ */
	LCD_WRITE_CMD ( 0x2C );	
	
	for ( i = 0; i < ulAmout_Point; i ++ )
		LCD_WRITE_DATA ( usColor );	
}
#endif

/**
  * ��������: ��LCD��ʾ����ĳһ������ĳ����ɫ��������
  * �������: usX �����ض�ɨ�跽���´��ڵ����X����
  *           usY �����ض�ɨ�跽���´��ڵ����Y����
  *           usWidth �����ڵĿ��
  *           usHeight �����ڵĸ߶�
  *           usColor ����ɫ
  * �� �� ֵ: ��
  * ˵    ������
  */
void LCD_Clear(uint16_t usX,uint16_t usY,uint16_t usWidth,uint16_t usHeight,uint16_t usColor)
{	 
#if 0   /* �Ż�����ִ���ٶ� */
  uint32_t i;
	uint32_t n,m;
  /* ��LCD��ʾ���Ͽ���һ������ */
  LCD_OpenWindow(usX,usY,usWidth,usHeight); 
  /* ��ʼ��GRAMд������ */
  LCD_WRITE_CMD(0x2C);
  
  m=usWidth * usHeight;
  n=m/8;
  m=m-8*n;
	for(i=0;i<n;i++)
	{
		LCD_WRITE_DATA(usColor);	
    LCD_WRITE_DATA(usColor);	
    LCD_WRITE_DATA(usColor);	
    LCD_WRITE_DATA(usColor);	
    
    LCD_WRITE_DATA(usColor);	
    LCD_WRITE_DATA(usColor);	
    LCD_WRITE_DATA(usColor);	
    LCD_WRITE_DATA(usColor);	
	}
  for(i=0;i<m;i++)
	{
		LCD_WRITE_DATA(usColor);	
	}
#else
  /* ��LCD��ʾ���Ͽ���һ������ */
  LCD_OpenWindow(usX,usY,usWidth,usHeight);
  /* ��LCD��ʾ������ĳһ��ɫ������ص� */
	LCD_FillColor(usWidth*usHeight,usColor);	
#endif	
}

/**
  * ��������: ��LCD��ʾ����ĳһ����ĳ����ɫ�������
  * �������: usX �����ض�ɨ�跽���´��ڵ����X����
  *           usY �����ض�ɨ�跽���´��ڵ����Y����
  *           usColor ����ɫ
  * �� �� ֵ: ��
  * ˵    ������
  */
void LCD_SetPointPixel(uint16_t usX,uint16_t usY,uint16_t usColor)	
{	
	if((usX<LCD_DEFAULT_WIDTH)&&(usY<LCD_DEFAULT_HEIGTH))
  {
		LCD_OpenWindow(usX,usY,1,1);
		LCD_FillColor(1,usColor);
	}
}

/**
  * ��������: ��LCD��ʾ����ĳһ����ĳ����ɫ�������
  * �������: ��
  * �� �� ֵ: uint16_t:��������RGB565
  * ˵    ������
  */
static uint16_t LCD_Read_PixelData ( void )	
{	
	uint16_t usR=0, usG=0, usB=0 ;
	
	LCD_WRITE_CMD ( 0x2E );   /* ������ */
	usR = LCD_READ_DATA (); 	/*FIRST READ OUT DUMMY DATA*/
	
	usR = LCD_READ_DATA ();  	/*READ OUT RED DATA  */
	usB = LCD_READ_DATA ();  	/*READ OUT BLUE DATA*/
	usG = LCD_READ_DATA ();  	/*READ OUT GREEN DATA*/	
	
  return (((usR>>11)<<11) | ((usG>>10)<<5) | (usB>>11));
	
}

/**
  * ��������: ��ȡ LCD ��ʾ����ĳһ����������������
  * �������: usX �����ض�ɨ�跽���´��ڵ����X����
  *           usY �����ض�ɨ�跽���´��ڵ����Y����
  * �� �� ֵ: uint16_t:��������RGB565
  * ˵    ������
  */
uint16_t LCD_GetPointPixel ( uint16_t usX, uint16_t usY )
{ 
	uint16_t usPixelData;
	
	LCD_SetCursor ( usX, usY );
	
	usPixelData = LCD_Read_PixelData ();
	
	return usPixelData;
	
}

/**
  * ��������: �� LCD ��ʾ����ʹ�� Bresenham �㷨���߶�
  * �������: usX1 �����ض�ɨ�跽���´��ڵ����X����
  *           usY1 �����ض�ɨ�跽���´��ڵ����Y����
  *           usX2 �����ض�ɨ�跽�����߶ε���һ���˵�X����
  *           usY2 �����ض�ɨ�跽�����߶ε���һ���˵�Y����
  *           usColor ���߶ε���ɫ
  * �� �� ֵ: ��
  * ˵    ������
  */
void LCD_DrawLine(uint16_t usX1,uint16_t usY1,uint16_t usX2,uint16_t usY2,uint16_t usColor)
{
	uint16_t us; 
	uint16_t usX_Current, usY_Current;
	int32_t lError_X=0,lError_Y=0,lDelta_X,lDelta_Y,lDistance; 
	int32_t lIncrease_X, lIncrease_Y;
	
	lDelta_X=usX2-usX1; //������������ 
	lDelta_Y=usY2-usY1; 
	usX_Current = usX1; 
	usY_Current = usY1; 
	
	if(lDelta_X>0)
  {
    lIncrease_X=1; //���õ������� 
	}
  else if(lDelta_X==0)
  {
		lIncrease_X=0;//��ֱ�� 
	}
	else 
  { 
    lIncrease_X=-1;
    lDelta_X=-lDelta_X;
  }
  
	if(lDelta_Y>0)
  {
		lIncrease_Y=1;
	}
	else if(lDelta_Y==0)
  {
		lIncrease_Y=0;//ˮƽ�� 
	}
	else
  {
    lIncrease_Y=-1;
    lDelta_Y=-lDelta_Y;
  }
	
	if(lDelta_X>lDelta_Y)
  {
		lDistance=lDelta_X; //ѡȡ�������������� 
	}
	else
  {
		lDistance=lDelta_Y; 
  }
	
	for(us=0;us<=lDistance+1;us++)//������� 
	{
		LCD_SetPointPixel(usX_Current,usY_Current,usColor);//���� 
		lError_X+=lDelta_X; 
		lError_Y+=lDelta_Y;
		if(lError_X>lDistance)
		{ 
			lError_X-=lDistance; 
			usX_Current+=lIncrease_X; 
		}
		if(lError_Y>lDistance) 
		{ 
			lError_Y-=lDistance; 
			usY_Current+=lIncrease_Y; 
		}		
	}
}

/**
  * ��������: ��LCD��ʾ���ϻ�һ������
  * �������: usX_Start �����ض�ɨ�跽���´��ڵ����X����
  *           usY_Start �����ض�ɨ�跽���´��ڵ����Y����
  *           usWidth�����εĿ�ȣ���λ�����أ�
  *           usHeight�����εĸ߶ȣ���λ�����أ�
  *           usColor �����ε���ɫ
  *           ucFilled ��ѡ���Ƿ����þ���
  *             ��ѡֵ��0�����ľ���
  *                     1��ʵ�ľ���
  * �� �� ֵ: ��
  * ˵    ������
  */
void LCD_DrawRectangle ( uint16_t usX_Start, uint16_t usY_Start, uint16_t usWidth, uint16_t usHeight, uint16_t usColor, uint8_t ucFilled )
{
	if(ucFilled)
  {
	  LCD_Clear ( usX_Start, usY_Start, usWidth, usHeight, usColor);
  }
	else
	{
		LCD_DrawLine ( usX_Start, usY_Start, usX_Start + usWidth - 1, usY_Start, usColor );
		LCD_DrawLine ( usX_Start, usY_Start + usHeight - 1, usX_Start + usWidth - 1, usY_Start + usHeight - 1, usColor );
		LCD_DrawLine ( usX_Start, usY_Start, usX_Start, usY_Start + usHeight - 1, usColor );
		LCD_DrawLine ( usX_Start + usWidth - 1, usY_Start, usX_Start + usWidth - 1, usY_Start + usHeight - 1, usColor );		
	}
}

/**
  * ��������: �� LCD ��ʾ����ʹ�� Bresenham �㷨��Բ
  * �������: usX_Center �����ض�ɨ�跽����Բ�ĵ�X����
  *           usY_Center �����ض�ɨ�跽����Բ�ĵ�Y����
  *           usRadius��Բ�İ뾶����λ�����أ�
  *           usColor ��Բ����ɫ
  *           ucFilled ��ѡ���Ƿ�����Բ
  *             ��ѡֵ��0������Բ
  *                     1��ʵ��Բ
  * �� �� ֵ: ��
  * ˵    ������
  */
void LCD_DrawCircle(uint16_t usX_Center,uint16_t usY_Center,uint16_t usRadius,uint16_t usColor,uint8_t ucFilled)
{
	int16_t sCurrentX, sCurrentY;
	int16_t sError;
  
	sCurrentX=0;
  sCurrentY=usRadius;	
	sError=3-(usRadius<<1);   //�ж��¸���λ�õı�־
	
	while(sCurrentX<=sCurrentY)
	{
		int16_t sCountY;		
		if(ucFilled)
    {			
			for(sCountY=sCurrentX;sCountY<=sCurrentY;sCountY++)
			{                      
				LCD_SetPointPixel(usX_Center+sCurrentX,usY_Center+sCountY,usColor);          //1���о����� 
				LCD_SetPointPixel(usX_Center-sCurrentX,usY_Center+sCountY,usColor);           //2       
				LCD_SetPointPixel(usX_Center-sCountY,  usY_Center+sCurrentX,usColor);           //3
				LCD_SetPointPixel(usX_Center-sCountY,  usY_Center-sCurrentX,usColor);           //4
				LCD_SetPointPixel(usX_Center-sCurrentX,usY_Center-sCountY,usColor);           //5    
        LCD_SetPointPixel(usX_Center+sCurrentX,usY_Center-sCountY,usColor);           //6
				LCD_SetPointPixel(usX_Center+sCountY,  usY_Center-sCurrentX,usColor);           //7 	
        LCD_SetPointPixel(usX_Center+sCountY,  usY_Center+sCurrentX,usColor);           //0				
			}
    }		
		else
		{          
			LCD_SetPointPixel(usX_Center+sCurrentX,usY_Center+sCurrentY,usColor);             //1���о�����
			LCD_SetPointPixel(usX_Center-sCurrentX,usY_Center+sCurrentY,usColor);             //2      
			LCD_SetPointPixel(usX_Center-sCurrentY,usY_Center+sCurrentX,usColor);             //3
			LCD_SetPointPixel(usX_Center-sCurrentY,usY_Center-sCurrentX,usColor);             //4
			LCD_SetPointPixel(usX_Center-sCurrentX,usY_Center-sCurrentY,usColor);             //5       
			LCD_SetPointPixel(usX_Center+sCurrentX,usY_Center-sCurrentY,usColor);             //6
			LCD_SetPointPixel(usX_Center+sCurrentY,usY_Center-sCurrentX,usColor);             //7 
			LCD_SetPointPixel(usX_Center+sCurrentY,usY_Center+sCurrentX,usColor);             //0
    }			
		sCurrentX ++;		
		if(sError<0) 
    {
			sError+=(4*sCurrentX+6);	  
		}
		else
		{
			sError +=(10+4*(sCurrentX-sCurrentY));   
			sCurrentY--;
		} 
	}
}

/**
  * ��������: �� LCD ��ʾ������ʾһ��Ӣ���ַ�
  * �������: usX�����ض�ɨ�跽�����ַ�����ʼX����
  *           usY �����ض�ɨ�跽���¸õ����ʼY����
  *           cChar ��Ҫ��ʾ��Ӣ���ַ�
  *           usColor_Background ��ѡ��Ӣ���ַ��ı���ɫ
  *           usColor_Foreground ��ѡ��Ӣ���ַ���ǰ��ɫ
  *           font������ѡ��
  *             ������USB_FONT_16 ��16������
  *                   USB_FONT_24 ��24������ 
  * �� �� ֵ: ��
  * ˵    �����ú���������ascii.h���ݶ�Ӧʹ��
  */
void LCD_DispChar_EN( uint16_t usX, uint16_t usY, const char cChar, uint16_t usColor_Background, uint16_t usColor_Foreground,USB_FONT_Typdef font)
{
	uint8_t ucTemp, ucRelativePositon, ucPage, ucColumn;
  
  /* �����������Ƿ�Ϸ� */
  assert_param(IS_USB_FONT(font));
  
	ucRelativePositon = cChar - ' ';
  
	if(font==USB_FONT_16)
  {
    LCD_OpenWindow(usX,usY,8,16);
    LCD_WRITE_CMD(0x2C);
    
    for(ucPage=0;ucPage<16;ucPage++)
    {
      ucTemp=ucAscii_1608[ucRelativePositon][ucPage];		
      for(ucColumn=0;ucColumn<8;ucColumn++)
      {
        if(ucTemp&0x01)
          LCD_WRITE_DATA(usColor_Foreground);			
        else
          LCD_WRITE_DATA(usColor_Background);								
        ucTemp >>= 1;					
      }
    }    
  }
  else
  {
    LCD_OpenWindow(usX,usY,12,24);
    LCD_WRITE_CMD(0x2C);
    
    for(ucPage=0;ucPage<48;ucPage++)
    {
      ucTemp=ucAscii_2412[ucRelativePositon][ucPage];		
      for(ucColumn=0;ucColumn<8;ucColumn++)
      {
        if(ucTemp&0x01)
          LCD_WRITE_DATA(usColor_Foreground);			
        else
          LCD_WRITE_DATA(usColor_Background);								
        ucTemp >>= 1;					
      }	
      ucPage++;
      ucTemp=ucAscii_2412[ucRelativePositon][ucPage];
      /* ֻ��ʾǰ��4��λ��������8λ�ܹ�12λ */
      for(ucColumn=0;ucColumn<4;ucColumn++)
      {
        if(ucTemp&0x01)
          LCD_WRITE_DATA(usColor_Foreground);			
        else
          LCD_WRITE_DATA(usColor_Background);								
        ucTemp >>= 1;					
      }	
    }
  }	
}

/**
  * ��������: �� LCD ��ʾ������ʾӢ���ַ���
  * �������: usX�����ض�ɨ�跽�����ַ�����ʼX����
  *           usY �����ض�ɨ�跽���¸õ����ʼY����
  *           pStr ��Ҫ��ʾ��Ӣ���ַ������׵�ַ
  *           usColor_Background ��ѡ��Ӣ���ַ��ı���ɫ
  *           usColor_Foreground ��ѡ��Ӣ���ַ���ǰ��ɫ
  *           font������ѡ��
  *             ������USB_FONT_16 ��16������
  *                   USB_FONT_24 ��24������ 
  * �� �� ֵ: ��
  * ˵    �����ú���������ascii.h���ݶ�Ӧʹ��
  */
void LCD_DispString_EN ( uint16_t usX, uint16_t usY, const char * pStr, uint16_t usColor_Background, uint16_t usColor_Foreground,USB_FONT_Typdef font)
{
  /* �����������Ƿ�Ϸ� */
  assert_param(IS_USB_FONT(font));
  
	while ( * pStr != '\0' )
	{
    if(font==USB_FONT_16)
    {
      if ( ( usX +  8 ) > LCD_DEFAULT_WIDTH )
      {
        usX = 0;
        usY += 16;
      }      
      if ( ( usY +  16 ) > LCD_DEFAULT_HEIGTH )
      {
        usX = 0;
        usY = 0;
      }      
      LCD_DispChar_EN ( usX, usY, * pStr, usColor_Background, usColor_Foreground,font);
      pStr ++;      
      usX += 8;
    }
    else
    {
      if ( ( usX +  12 ) > LCD_DEFAULT_WIDTH )
      {
        usX = 0;
        usY += 24;
      }      
      if ( ( usY +  24 ) > LCD_DEFAULT_HEIGTH )
      {
        usX = 0;
        usY = 0;
      }      
      LCD_DispChar_EN ( usX, usY, * pStr, usColor_Background, usColor_Foreground,font);
      pStr ++;      
      usX += 12;
    }
	}
}



/**
  * ��������: �Ӵ���Flash��ȡGBK��
  * �������: pBuffer�����ݱ����ַ
  *           gbk �������ַ����ֽ���
  *           font������ѡ��
  *           ��ѡֵ��FONT_16 ��16������
  *                   FONT_24 ��24������ 
  * �� �� ֵ: uint8_t: 0:��ȡʧ�ܣ�1����ȡ�ɹ�
  * ˵    ������
  */
static uint8_t GetGBKCode_SPIFLASH(unsigned char* pBuffer,const uint8_t *pstr,USB_FONT_Typdef font)
{
  uint32_t pos;
  uint8_t high8bit,low8bit;
  static uint8_t startflag=1;
__IO uint32_t DeviceID = 0;
__IO uint32_t FlashID = 0;
  /* �����������Ƿ�Ϸ� */
  assert_param(IS_FONT(font));
  
  if(startflag)
  {
    MX_SPIFlash_Init();
    
    /* Get SPI Flash Device ID */
    DeviceID = SPI_FLASH_ReadDeviceID();
    
    HAL_Delay(100);
    
    /* Get SPI Flash ID */
    FlashID = SPI_FLASH_ReadID();
    
    printf("FlashID is 0x%X,  Manufacturer Device ID is 0x%X\n", FlashID, DeviceID);
    
    startflag=0;
  }
  
  high8bit=*pstr;
  low8bit=*(pstr+1);
  if(font==USB_FONT_16)
  {
    // 16*16��С�ĺ��� ����ģ ռ��16*16/8���ֽ�
    pos=((high8bit-0xa1)*94+low8bit-0xa1)*16*16/8;
    SPI_FLASH_BufferRead(pBuffer,GBK_HZ1616_ADDR+pos,32);
  }
  else
  {
    // 24*24��С�ĺ��� ����ģ ռ��24*24/8���ֽ�
    pos=((high8bit-0xa1)*94+low8bit-0xa1)*24*24/8; 
    SPI_FLASH_BufferRead(pBuffer,GBK_HZ2424_ADDR+pos,72);
  }
  if((pBuffer[0]==0xFF)&&(pBuffer[1]==0xFF))
  {
    return 0;
  }  
  return 1;  
}

/**
  * ��������: �� LCD ��ʾ������ʾһ������
  * �������: usX�����ض�ɨ�跽�����ַ�����ʼX����
  *           usY �����ض�ɨ�跽���¸õ����ʼY����
  *           pstr: �����ַ����ֽ���
  *           usColor_Background ��ѡ��Ӣ���ַ��ı���ɫ
  *           usColor_Foreground ��ѡ��Ӣ���ַ���ǰ��ɫ
  *           font������ѡ��
  *           ��ѡֵ��USB_FONT_16 ��16������
  *                   USB_FONT_24 ��24������ 
  * �� �� ֵ: ��
  * ˵    ������
  */
void LCD_DispCHAR_CH(uint16_t usX,uint16_t usY,const uint8_t *pstr,uint16_t usColor_Background, uint16_t usColor_Foreground,USB_FONT_Typdef font)
{
	uint8_t ucTemp, ucPage, ucColumn;
  uint8_t gbk_buffer[72];
  
  /* �����������Ƿ�Ϸ� */
  assert_param(IS_FONT(font));
  
	if(font==USB_FONT_16)
  {
    LCD_OpenWindow(usX,usY,16,16);
    LCD_WRITE_CMD(0x2C);
    GetGBKCode_SPIFLASH(gbk_buffer,pstr,USB_FONT_16);
    
    for(ucPage=0;ucPage<32;ucPage++)
    {
      ucTemp=gbk_buffer[ucPage];		
      for(ucColumn=0;ucColumn<8;ucColumn++)
      {
        if(ucTemp&0x01)
          LCD_WRITE_DATA(usColor_Foreground);			
        else
          LCD_WRITE_DATA(usColor_Background);								
        ucTemp >>= 1;					
      }
    }    
  }
  else
  {
    LCD_OpenWindow(usX,usY,24,24);
    LCD_WRITE_CMD(0x2C);
    
    GetGBKCode_SPIFLASH(gbk_buffer,pstr,USB_FONT_24);

    for(ucPage=0;ucPage<72;ucPage++)
    {
      ucTemp=gbk_buffer[ucPage];		
      for(ucColumn=0;ucColumn<8;ucColumn++)
      {
        if(ucTemp&0x01)
          LCD_WRITE_DATA(usColor_Foreground);			
        else
          LCD_WRITE_DATA(usColor_Background);								
        ucTemp >>= 1;					
      }
    } 
  }	
}

/**
  * ��������: �� LCD ��ʾ������ʾһ������
  * �������: usX�����ض�ɨ�跽�����ַ�����ʼX����
  *           usY �����ض�ɨ�跽���¸õ����ʼY����
  *           pstr: �����ַ����ֽ���
  *           usColor_Background ��ѡ��Ӣ���ַ��ı���ɫ
  *           usColor_Foreground ��ѡ��Ӣ���ַ���ǰ��ɫ
  *           font������ѡ��
  *           ��ѡֵ��USB_FONT_16 ��16������
  *                   USB_FONT_24 ��24������ 
  * �� �� ֵ: ��
  * ˵    ������
  */
void LCD_DispString_CH(uint16_t usX,uint16_t usY,const uint8_t *pstr,uint16_t usColor_Background, uint16_t usColor_Foreground,USB_FONT_Typdef font)
{
  /* �����������Ƿ�Ϸ� */
  assert_param(IS_FONT(font));
  
  while(*pstr != '\0')
	{
    if(font==USB_FONT_16)
    {
      if((usX+16)>LCD_DEFAULT_WIDTH)
      {
        usX = 0;
        usY += 16;
      }      
      if((usY+16)>LCD_DEFAULT_HEIGTH)
      {
        usX=0;
        usY=0;
      }      
      LCD_DispCHAR_CH(usX,usY,pstr,usColor_Background,usColor_Foreground,USB_FONT_16);
      pstr+=2;      
      usX+=16;
    }
    else
    {
      if((usX+24)>LCD_DEFAULT_WIDTH)
      {
        usX = 0;
        usY += 24;
      }      
      if((usY+24)>LCD_DEFAULT_HEIGTH)
      {
        usX=0;
        usY=0;
      }      
      LCD_DispCHAR_CH(usX,usY,pstr,usColor_Background,usColor_Foreground,USB_FONT_24);
      pstr+=2;      
      usX+=24;
    }
	}  
}

/**
  * ��������: �� LCD ��ʾ������ʾһ����Ӣ��
  * �������: usX�����ض�ɨ�跽�����ַ�����ʼX����
  *           usY �����ض�ɨ�跽���¸õ����ʼY����
  *           pstr: �����ַ����ֽ���
  *           usColor_Background ��ѡ��Ӣ���ַ��ı���ɫ
  *           usColor_Foreground ��ѡ��Ӣ���ַ���ǰ��ɫ
  *           font������ѡ��
  *           ��ѡֵ��USB_FONT_16 ��16������
  *                   USB_FONT_24 ��24������ 
  * �� �� ֵ: ��
  * ˵    ������
  */
void LCD_DispString_EN_CH(uint16_t usX,uint16_t usY,const uint8_t *pstr,uint16_t usColor_Background, uint16_t usColor_Foreground,USB_FONT_Typdef font)
{
  /* �����������Ƿ�Ϸ� */
  assert_param(IS_FONT(font));
  
  while(*pstr != '\0')
	{
    if(*pstr<=0x7f)
    {
      if(font==USB_FONT_16)
      {
        if((usX+8)>LCD_DEFAULT_WIDTH)
        {
          usX = 0;
          usY += 16;
        }      
        if((usY+16)>LCD_DEFAULT_HEIGTH)
        {
          usX=0;
          usY=0;
        }      
        LCD_DispChar_EN(usX,usY,*pstr,usColor_Background,usColor_Foreground,USB_FONT_16);
        pstr++;      
        usX+=8;
      }
      else
      {
        if((usX+12)>LCD_DEFAULT_WIDTH)
        {
          usX=0;
          usY+=24;
        }      
        if((usY+24)>LCD_DEFAULT_HEIGTH)
        {
          usX=0;
          usY=0;
        }      
        LCD_DispChar_EN(usX,usY,*pstr,usColor_Background,usColor_Foreground,USB_FONT_24);
        pstr++;      
        usX+=12;
      }
    }
    else
    {
      if(font==USB_FONT_16)
      {
        if((usX+16)>LCD_DEFAULT_WIDTH)
        {
          usX = 0;
          usY += 16;
        }      
        if((usY+16)>LCD_DEFAULT_HEIGTH)
        {
          usX=0;
          usY=0;
        }      
        LCD_DispCHAR_CH(usX,usY,pstr,usColor_Background,usColor_Foreground,USB_FONT_16);
        pstr+=2;      
        usX+=16;
      }
      else
      {
        if((usX+24)>LCD_DEFAULT_WIDTH)
        {
          usX = 0;
          usY += 24;
        }      
        if((usY+24)>LCD_DEFAULT_HEIGTH)
        {
          usX=0;
          usY=0;
        }      
        LCD_DispCHAR_CH(usX,usY,pstr,usColor_Background,usColor_Foreground,USB_FONT_24);
        pstr+=2;      
        usX+=24;
      }
    }
	}  
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
