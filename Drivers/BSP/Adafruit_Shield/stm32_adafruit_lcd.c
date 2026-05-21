/**
  ******************************************************************************
  * @file    stm32_adafruit_lcd.c
  * @author  MCD Application Team
  * @brief   This file includes the driver for Liquid Crystal Display (LCD) module
  *          mounted on the Adafruit 1.8" TFT LCD shield (reference ID 802), 
  *          that is used with the STM32 Nucleo board through SPI interface.     
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/* File Info : -----------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - The LCD ST7735 component driver MUST be included with this driver.  

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the LCD using the BSP_LCD_Init() function.
  
  + Display on LCD
     o Clear the whole LCD using the BSP_LCD_Clear() function or only one specified 
       string line using the BSP_LCD_ClearStringLine() function.
     o Display a character on the specified line and column using the BSP_LCD_DisplayChar()
       function or a complete string line using the BSP_LCD_DisplayStringAtLine() function.
     o Display a string line on the specified position (x,y in pixel) and align mode
       using the BSP_LCD_DisplayStringAtLine() function.          
     o Draw and fill a basic shapes (dot, line, rectangle, circle, ellipse, ..) 
       on LCD using a set of functions.    
 
------------------------------------------------------------------------------*/

    
/* Includes ------------------------------------------------------------------*/
#include "../Drivers/BSP/Adafruit_Shield/stm32_adafruit_lcd.h"

#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_nucleo.h"
#include "fonts.h"
#include "ST7789.h"


uint8_t Xpos, Ypos, myfont, myVert, myGoriz;
uint16_t TempChar;
uint8_t data_LCD;
uint16_t i_LCD;
uint16_t j_LCD;
uint16_t i, j, k;
uint16_t iq, jq, kq;
uint8_t tmp_char;
uint16_t char_16;
uint8_t byte_width;

extern SPI_HandleTypeDef SpiHandle;
extern LCD_7735_DrvTypeDef   ST7735_drv;
extern LCD_7789_DrvTypeDef   ST7789_drv;

//extern FontDef Font_7x10;
//extern FontDef Font_11x18;
extern FontDef Font_16x26; //Для маленьких символів

LCD_DrawPropTypeDef DrawProp; //Для великих символів
LCD_DrawPropTypeDef DrawProp_ukr; //Українські
LCD_DrawPropTypeDef DrawProp_Big_Digit; //Українські; //Українські
extern uint8_t* DigitalsBig[];
extern uint8_t* DigitalsBigBig[];
extern uint8_t* Arial45x39[];

#define LINE(x) ((x) * (((bFontDef *)BSP_LCD_GetFont())->height))

uint32_t bi;


/** @defgroup STM32_ADAFRUIT_LCD_Private_Defines
  * @{
  */
#define POLY_X(Z)             ((int32_t)((Points + (Z))->X))
#define POLY_Y(Z)             ((int32_t)((Points + (Z))->Y))
//#define NULL                  (void *)0

#define MAX_HEIGHT_FONT         17
#define MAX_WIDTH_FONT          24
#define OFFSET_BITMAP           54
/**
  * @}
  */ 

/** @defgroup STM32_ADAFRUIT_LCD_Private_Macros
  * @{
  */
//#define ABS(X) ((X) > 0 ? (X) : -(X)) 

/**
  * @}
  */ 
    
/** @defgroup STM32_ADAFRUIT_LCD_Private_Variables
  * @{
  */ 
	
#ifdef TFT_LCD_7789
		LCD_7789_DrvTypeDef  *lcd_drv;
	#elif defined (TFT_LCD_7735)
		LCD_7735_DrvTypeDef  *lcd_drv;
#endif

/* Max size of bitmap will based on a font24 (17x24) */
static uint8_t bitmap[MAX_HEIGHT_FONT*MAX_WIDTH_FONT*2+OFFSET_BITMAP] = {0};

  
/**
  * @brief  Initializes the LCD.
  * @param  None
  * @retval LCD state
  */
uint8_t BSP_LCD_Init(void)
{ 
  uint8_t ret = LCD_ERROR;
 
  /* Default value for draw propriety */
  DrawProp.BackColor = LCD_BLACK; //Заповнюю структуру DrawProp влістивостями фонту Font24
  DrawProp.TextColor = LCD_WHITE;
	//Для української мови
	DrawProp_ukr.BackColor = LCD_BLACK; //Заповнюю структуру DrawProp влістивостями фонту Font24
  DrawProp_ukr.TextColor = LCD_WHITE;
	
#ifdef TFT_LCD_7735	
	
	lcd_drv = &ST7735_drv;
  //lcd_drv = &LCD_drv;
  /* LCD Init */   
  lcd_drv->Init();
	//ST7735_Init(); //Конфігурація драйвера ST7789 LCD
	//ST7735_FillScreen(WHITE);
	LCD_Fill_Color(LCD_BLACK);

	//LCD_DrawFilledRectangle(0, 0, 127, 127, LCD_BLACK);//Заповнити квадрат
	//LCD_DrawRectangle(0, 0, 127, 127, LCD_GREEN); //намалювани лініями квадрат
#elif defined (TFT_LCD_7789)
	lcd_drv = &ST7789_drv;
	
	lcd_drv->Init(); //Послідовність кодів ініціалізації
	//lcd_drv->ST7789_Init(); //Конфігурація драйвера ST7789 LCD
	LCD_Fill_Color(LCD_BLACK); //вібувається швидко (240х240 RAM заповнюється двобайтовими кодами кольора)
#endif
	HAL_Delay(10);
	ret = LCD_OK;
  return ret;
}


/**
  * @brief  Gets the LCD X size.
  * @param  None    
  * @retval Used LCD X size
  */
uint32_t BSP_LCD_GetXSize(void)
{
  return(lcd_drv->GetLcdPixelWidth());
}

/**
  * @brief  Gets the LCD Y size.
  * @param  None   
  * @retval Used LCD Y size
  */
uint32_t BSP_LCD_GetYSize(void)
{
  return(lcd_drv->GetLcdPixelHeight());
}

/**
  * @brief  Gets the LCD text color.
  * @param  None 
  * @retval Used text color.
  */
uint16_t BSP_LCD_GetTextColor(void)
{
  return DrawProp.TextColor;
}

/**
  * @brief  Gets the LCD background color.
  * @param  None
  * @retval Used background color
  */
uint16_t BSP_LCD_GetBackColor(void)
{
  return DrawProp.BackColor;
}

/**
  * @brief  Sets the LCD text color.
  * @param  Color: Text color code RGB(5-6-5)
  * @retval None
  */
void BSP_LCD_SetTextColor(uint16_t Color)
{
  DrawProp.TextColor = Color;
}

/**
  * @brief  Sets the LCD background color.
  * @param  Color: Background color code RGB(5-6-5)
  * @retval None
  */
void BSP_LCD_SetBackColor(uint16_t Color)
{
  DrawProp.BackColor = Color;
}

/**
  * @brief  Sets the LCD text font.
  * @param  fonts: Font to be used
  * @retval None
  */
void BSP_LCD_SetFont(bFontDef *pFonts)
{
  //DrawProp.pFont = pFonts;
}

/**
  * @brief  Gets the LCD text font.
  * @param  None
  * @retval Used font
  */
uint8_t *BSP_LCD_GetFont(void)
{
  return DrawProp.pFont;
}

/**
  * @brief  Clears the hole LCD.
  * @param  Color: Color of the background
  * @retval None
  */
void BSP_LCD_Clear(uint16_t Color)
{ 
  uint32_t counter = 0;
  uint32_t color_backup = DrawProp.TextColor; 
  DrawProp.TextColor = Color;
  
  for(counter = 0; counter < BSP_LCD_GetYSize(); counter++) //ST7789_WIDTH BSP_LCD_GetYSize(); counter++)
	{
    BSP_LCD_DrawHLine(0, counter, 240); // BSP_LCD_GetXSize()); //LCD_HEIGHT BSP_LCD_GetXSize());
		//LCD_DrawHLine(0, counter, BSP_LCD_GetXSize());
  }
  DrawProp.TextColor = color_backup; 
  BSP_LCD_SetTextColor(DrawProp.TextColor);
}

/**
  * @brief  Draws an horizontal line.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  * @retval None
*/
void BSP_LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t index = 0;

	//LCD_DrawLine(Xpos, Ypos, 240 - 1, 240 - 1, DrawProp.TextColor)

  
  if(lcd_drv->DrawHLine != NULL)
  {
    lcd_drv->DrawHLine(DrawProp.TextColor, Xpos, Ypos, Length);
  }
  else
  {
    for(index = 0; index < Length; index++)
    {
      BSP_LCD_DrawPixel((Xpos + index), Ypos, DrawProp.TextColor);
    }
  }
}

/**
  * @brief  Draws a pixel on LCD.
  * @param  Xpos: X position 
  * @param  Ypos: Y position
  * @param  RGB_Code: Pixel color in RGB mode (5-6-5)  
  * @retval None
  */
void BSP_LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code)
{
  if(lcd_drv->DrawPixel != NULL)
  {
    lcd_drv->DrawPixel(Xpos, Ypos, RGB_Code);
  }
}
                         

/*******************************************************************************
                            Static Functions
*******************************************************************************/

/**
* @brief Заповнює DisplayWindow одним кольором with single color
 * @param color -> color to Fill with 0xXX 0x00 (
 * @return none
 */
void LCD_Fill_Color(uint16_t color)
{
	uint16_t i = 0, j = 0, z = 0;
#ifdef TFT_LCD_7789
	uint16_t	LCD_WIDTH = ST7789_WIDTH;
	uint16_t	LCD_HEIGHT = ST7789_HEIGHT;
#elif defined (TFT_LCD_7735)
	uint16_t	LCD_WIDTH = ST7735_WIDTH;
	uint16_t	LCD_HEIGHT = ST7735_HEIGHT;
#endif
	LCD_SetAddressWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
	printf("--LCD_WIDTH = %04d  LCD_HEIGHT = %04d\n\r", LCD_WIDTH, LCD_HEIGHT);
	LCD_CS_LOW();

	#ifdef USE_DMA
		for (i = 0; i < LCD_HEIGHT / HOR_LEN; i++)
		{
			memset(disp_buf, color, sizeof(disp_buf));
			LCD_SendData(disp_buf, sizeof(disp_buf));
		}
	#else
			for (i = 0; i < LCD_WIDTH; i++)
			{
				for (j = 0; j < LCD_HEIGHT; j++) 
				{
					uint8_t data[] = {color >> 8, color & 0xFF}; //Перетворення двобайтового кода в масив байт
					LCD_SendData(data, sizeof(data));
					z++;
				}
			}
	#endif
			printf("------------z = %04d\n\r", z);
	LCD_CS_HIGH();
}

//Встановлення розмірів вікна виводу
static void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	LCD_CS_LOW();
	uint16_t x_start = x0 + X_SHIFT, x_end = x1 + X_SHIFT;
	uint16_t y_start = y0 + Y_SHIFT, y_end = y1 + Y_SHIFT;

#ifdef TFT_LCD_7789	
	/* Column Address set */
	LCD_SendCommand(ST7789_CASET); 
	{
		uint8_t data[] = {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF};
		LCD_SendData(data, sizeof(data));
	}

	/* Row Address set */
	LCD_SendCommand(ST7789_RASET);
	{
		uint8_t data[] = {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF};
		LCD_SendData(data, sizeof(data));
	}
	/* Write to RAM */
	LCD_SendCommand(ST7789_RAMWR);
#elif defined TFT_LCD_7735
		/* Column Address set */
	LCD_SendCommand(ST7735_CASET); 
	{
		uint8_t data[] = {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF};
		LCD_SendData(data, sizeof(data));
	}

	/* Row Address set */
	LCD_SendCommand(ST7735_RASET);
	{
		uint8_t data[] = {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF};
		LCD_SendData(data, sizeof(data));
	}
	/* Write to RAM */
	LCD_SendCommand(ST7735_RAMWR);
	
#endif	
	
	LCD_CS_HIGH();
}

/**
 * @brief Draw a Pixel
 * @param x&y -> coordinate to Draw
 * @param color -> color of the Pixel
 * @return none
 */
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
#ifdef TFT_LCD_7789
	uint16_t	LCD_WIDTH = ST7789_WIDTH;
	uint16_t	LCD_HEIGHT = ST7789_HEIGHT;
#elif defined (TFT_LCD_7735)
	uint16_t	LCD_WIDTH = ST7735_WIDTH;
	uint16_t	LCD_HEIGHT = ST7735_HEIGHT;
#endif

	if ((x < 0) || (x >= LCD_WIDTH) ||
		 (y < 0) || (y >= LCD_HEIGHT))	return;
	
	LCD_SetAddressWindow(x, y, x, y);
	uint8_t data[] = {color >> 8, color & 0xFF};
	LCD_CS_LOW();
	LCD_SendData(data, sizeof(data));
	LCD_CS_HIGH();
}

/**
 * @brief Fill an Area with single color
 * @param xSta&ySta -> coordinate of the start point
 * @param xEnd&yEnd -> coordinate of the end point
 * @param color -> color to Fill with
 * @return none
 */
void LCD_Fill(uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color)
{
#ifdef TFT_LCD_7789
	uint16_t	LCD_WIDTH = ST7789_WIDTH;
	uint16_t	LCD_HEIGHT = ST7789_HEIGHT;
#elif defined (TFT_LCD_7735)
	uint16_t	LCD_WIDTH = ST7735_WIDTH;
	uint16_t	LCD_HEIGHT = ST7735_HEIGHT;
#endif

	if ((xEnd < 0) || (xEnd >= LCD_WIDTH) ||
		 (yEnd < 0) || (yEnd >= LCD_HEIGHT))	return;
	LCD_CS_LOW();
	uint16_t i, j;
	LCD_SetAddressWindow(xSta, ySta, xEnd, yEnd);
	for (i = ySta; i <= yEnd; i++)
		for (j = xSta; j <= xEnd; j++) {
			uint8_t data[] = {color >> 8, color & 0xFF};
			LCD_SendData(data, sizeof(data));
		}
	LCD_CS_HIGH();
}

/**
 * @brief Draw a big Pixel at a point
 * @param x&y -> coordinate of the point
 * @param color -> color of the Pixel
 * @return none
 */
void LCD_DrawPixel_4px(uint16_t x, uint16_t y, uint16_t color)
{
#ifdef TFT_LCD_7789
	uint16_t	LCD_WIDTH = ST7789_WIDTH;
	uint16_t	LCD_HEIGHT = ST7789_HEIGHT;
#elif defined (TFT_LCD_7735)
	uint16_t	LCD_WIDTH = ST7735_WIDTH;
	uint16_t	LCD_HEIGHT = ST7735_HEIGHT;
#endif

	if ((x <= 0) || (x > LCD_WIDTH) ||
		 (y <= 0) || (y > LCD_HEIGHT))	return;
	LCD_CS_LOW();
	LCD_Fill(x - 1, y - 1, x + 1, y + 1, color);
	LCD_CS_HIGH();
}

/**
 * @brief Draw a line with single color
 * @param x1&y1 -> coordinate of the start point
 * @param x2&y2 -> coordinate of the end point
 * @param color -> color of the line to Draw
 * @return none
 */
void LCD_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
	uint16_t swap;
    uint16_t steep = ABS(y1 - y0) > ABS(x1 - x0);
    if (steep) {
		swap = x0;
		x0 = y0;
		y0 = swap;

		swap = x1;
		x1 = y1;
		y1 = swap;
        //_swap_int16_t(x0, y0);
        //_swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
		swap = x0;
		x0 = x1;
		x1 = swap;

		swap = y0;
		y0 = y1;
		y1 = swap;
        //_swap_int16_t(x0, x1);
        //_swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = ABS(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
            LCD_DrawPixel(y0, x0, color);
        } else {
            LCD_DrawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

/**
 * @brief Draw a Rectangle with single color
 * @param xi&yi -> 2 coordinates of 2 top points.
 * @param color -> color of the Rectangle line
 * @return none
 */
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	LCD_CS_LOW();
	LCD_DrawLine(x1, y1, x2, y1, color);
	LCD_DrawLine(x1, y1, x1, y2, color);
	LCD_DrawLine(x1, y2, x2, y2, color);
	LCD_DrawLine(x2, y1, x2, y2, color);
	LCD_CS_HIGH();
}

/** 
 * @brief Draw a circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle line
 * @return  none
 */
void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	LCD_CS_LOW();
	LCD_DrawPixel(x0, y0 + r, color);
	LCD_DrawPixel(x0, y0 - r, color);
	LCD_DrawPixel(x0 + r, y0, color);
	LCD_DrawPixel(x0 - r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		LCD_DrawPixel(x0 + x, y0 + y, color);
		LCD_DrawPixel(x0 - x, y0 + y, color);
		LCD_DrawPixel(x0 + x, y0 - y, color);
		LCD_DrawPixel(x0 - x, y0 - y, color);

		LCD_DrawPixel(x0 + y, y0 + x, color);
		LCD_DrawPixel(x0 - y, y0 + x, color);
		LCD_DrawPixel(x0 + y, y0 - x, color);
		LCD_DrawPixel(x0 - y, y0 - x, color);
	}
	LCD_CS_HIGH();
}

/** 
 * @brief Write a char
 * @param  x&y -> cursor of the start point.
 * @param ch -> char to write
 * @param font -> fontstyle of the string
 * @param color -> color of the char
 * @param bgcolor -> background color of the char
 * @return  none
 */
void LCD_WriteChar(uint16_t x, uint16_t y, char ch, FontDef Font, uint16_t color, uint16_t bgcolor)
{
	uint32_t i, b, j;
	LCD_CS_LOW();
	LCD_SetAddressWindow(x, y, x + Font.width - 1, y + Font.height - 1);
	
	for (i = 0; i < Font.height; i++) {
		//b = font.data[(ch - 32) * font.height + i];
		bi = Font.data[(ch - 32) * Font.height + i];
		
		for (j = 0; j < Font.width; j++) {
			if ((bi << j) & 0x8000) {
				uint8_t data[] = {color >> 8, color & 0xFF};
				LCD_SendData(data, sizeof(data));
			}
			else {
				uint8_t data[] = {bgcolor >> 8, bgcolor & 0xFF};
				LCD_SendData(data, sizeof(data));
			}
		}
	}
	LCD_CS_HIGH();
}


/** 
 * @brief Write a string 
 * @param  x&y -> cursor of the start point.
 * @param str -> string to write
 * @param font -> fontstyle of the string
 * @param color -> color of the string
 * @param bgcolor -> background color of the string
 * @return  none
 */
void LCD_WriteString(uint16_t x, uint16_t y, const char *str, FontDef Font, uint16_t color, uint16_t bgcolor)
{
#ifdef TFT_LCD_7789
	uint16_t	LCD_WIDTH = ST7789_WIDTH; //240x240
	uint16_t	LCD_HEIGHT = ST7789_HEIGHT;
#elif defined (TFT_LCD_7735)
	uint16_t	LCD_WIDTH = ST7735_WIDTH; //128x128
	uint16_t	LCD_HEIGHT = ST7735_HEIGHT;
#endif
	
	LCD_CS_LOW();
	while (*str) {
		if (x + Font.width >= LCD_WIDTH) {
			x = 0;
			y += Font.height;
			if (y + Font.height >= LCD_HEIGHT) {
				break;
			}

			if (*str == ' ') {
				// skip spaces in the beginning of the new line
				str++;
				continue;
			}
		}
		LCD_WriteChar(x, y, *str, Font, color, bgcolor);
		x += Font.width;
		str++;
	}
	LCD_CS_HIGH();
}

/** 
 * @brief Draw a filled Rectangle with single color
 * @param  x&y -> coordinates of the starting point
 * @param w&h -> width & height of the Rectangle
 * @param color -> color of the Rectangle
 * @return  none
 */
void LCD_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
#ifdef TFT_LCD_7789
	uint16_t	LCD_WIDTH = ST7789_WIDTH;
	uint16_t	LCD_HEIGHT = ST7789_HEIGHT;
#elif defined (TFT_LCD_7735)
	uint16_t	LCD_WIDTH = ST7735_WIDTH;
	uint16_t	LCD_HEIGHT = ST7735_HEIGHT;
#endif

	LCD_CS_LOW();
	uint8_t i;

	/* Check input parameters */
	if (x >= LCD_WIDTH ||
		y >= LCD_HEIGHT) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= LCD_WIDTH) {
		w = LCD_WIDTH - x;
	}
	if ((y + h) >= LCD_HEIGHT) {
		h = LCD_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		LCD_DrawLine(x, y + i, x + w, y + i, color);
	}
	LCD_CS_HIGH();
}

/******************************************************************************
* Function Name  : GUI_Text
* Description    : 
* Input          : - Xpos: 
*                  - Ypos: 
*				           - str:
*				           - charColor:
*				           - bkColor:
* Output         : None
* Return         : None
* Attention		 : None
*  *mycoordinates Координати X, Y початку роміщення рядка символів
* mySize індекс  таблиці символів
*******************************************************************************/
void GUI_Text(uint8_t* mycoordinates, char *str, uint8_t mySize)
{
//	uint8_t TempChar;
	
//i = Xpos;
//j = Ypos;
	Xpos = *mycoordinates;
	Ypos = *(mycoordinates + 1);
if (mySize == 0x00) //8x14
{
	myfont = 0;
	myVert = 14; 
	myGoriz= 8;
}	
else if (mySize == 0x01) //10x20
{
	myfont = 1;
	myVert = 20; //кол-во вертикальных точек в знаке (Три байта) 
	myGoriz= 10; //кол-во горизонтальных точек в знаке (10 груп по 3 байта)
}	

else if (mySize == 0x02) // x24
{
	myfont = 1;
	myVert = 24; //кол-во вертикальных точек в знаке
	myGoriz= 19; //кол-во горизонтальных точек в знаке
}	

else if (mySize == 0x03) //
{
	myfont = 1;
	myVert = 26; //кол-во вертикальных точек в знаке
	myGoriz= 18; //кол-во горизонтальных точек в знаке
}		
 
else if (mySize == 0x05) //
{
	myfont = 1;
	DrawProp.pFont = DigitalsBig[0];
	DrawProp.height = 40; //кол-во вертикальных точек в знаке 8*5
	DrawProp.width = 26; //кол-во горизонтальных точек в знаке 130/5
}		

else if (mySize == 0x07) //
{
	myfont = 1;
	DrawProp.pFont = DigitalsBigBig[0];
	DrawProp.height = 56; //кол-во вертикальных точек в знаке 8*7
	DrawProp.width = 37; //кол-во горизонтальных точек в знаке 259/7
}
	do
	{
				TempChar = *str++;  //ASCI код символа
 
//i = Xpos;
//j = Ypos;
		
		PutChar( Xpos, Ypos, TempChar, mySize, 0); //mySize індекс таблиці символів
			
		if (myfont == 1)	
		{
			//Развертывание по вертикали
			if( Xpos + DrawProp.width < MAX_X)
        {
            Xpos += DrawProp.width;
        } 
        else if (Ypos + DrawProp.height < MAX_Y)
        {
            Xpos = 0;
            Ypos += myVert;
        }   
        else
        {
						BSP_LCD_Clear(0x00);
						Xpos = 0;
            Ypos = 0;
        } 
			}
			else if (myfont == 0)
			{
//Развертывание по горизонтали
				if( Xpos + DrawProp.width < MAX_X / 8)
        {
            Xpos += DrawProp.width;
        } 
        else if (Ypos + DrawProp.height < MAX_Y)
        {
            Xpos = 0;
            Ypos += DrawProp.height;
        }   
        else
        {
						BSP_LCD_Clear(0x00);
						Xpos = 0;
            Ypos = 0;
        } 
			}
				
//i = Xpos;
//j = Ypos;
	}
  while ( *str != 0 );
}

void GUI_Text_ukr(uint8_t* mycoordinates, char *str, uint8_t mySize, uint8_t myFont)
{
//	uint8_t TempChar;
	
//i = Xpos;
//j = Ypos;
	uint16_t TempChar, char_16;
	uint8_t Xpos, Ypos, cbyte;
	Xpos = *mycoordinates;
	Ypos = *(mycoordinates + 1);
	
	do
	{
		//size_t *cbyte;
		if (myFont == 0) //Друкую день тижня українською
		{
			char_16 = utf8_to_unicode(str, &cbyte); //utf8_to_codepoint(str3, cbyte); 
			if (cbyte == 1 && char_16 == 0x27)
			{
				char_16 = 0x0485;
			}

		/*!!!!Не можна використовувати віднімання 16-х кодів, тому що вони не послідовні на відміну від порядкових номерів таблиці кодів
		Наприклад для кодів кирилиці п->0xD0BF[UTF8] / 0x43F[UNICODE] /1087[Десятковий]
		                             р->0xD180[UTF8] / 0x440[UNICODE] /1088[Десятковий] !!!!
		Перед відніманням тре перетворити UTF8 в UNICODE !!!!		
		*/
			TempChar = ((char_16 - 0x0406)); //*4 + 8); // порядковий номер символу в масиві myFont_ukr (по 4 байти в позиційному вказівнику на масиві символу)
			if (cbyte == 1 && char_16 == 0x0485)  
				 {
					 Xpos += (DrawProp_ukr.width / 2);
				 }
			PutChar( Xpos, Ypos, TempChar, mySize, myFont); //mySize індекс таблиці символів
			str = str + cbyte;
			if( Xpos + DrawProp_ukr.width < MAX_X)
       {
 					  Xpos += DrawProp_ukr.width; 
	
       }else if (Ypos + DrawProp_ukr.height < MAX_Y)
       {
            Xpos = 0;
            Ypos += myVert;
        }else
        {
						BSP_LCD_Clear(0x00);
						Xpos = 0;
            Ypos = 0;
        } 
		}else if(myFont == 1) //Друкую час великими цифрами
		{
		char_16 = utf8_to_unicode(str, &cbyte); //utf8_to_codepoint(str3, cbyte); 
		//if (cbyte == 1 && char_16 == 0x27)
		//{
		//	char_16 = 0x0485;
		//}

		/*!!!!Не можна використовувати віднімання 16-х кодів, тому що вони не послідовні на відміну від порядкових номерів таблиці кодів
		Наприклад для кодів кирилиці п->0xD0BF[UTF8] / 0x43F[UNICODE] /1087[Десятковий]
		                             р->0xD180[UTF8] / 0x440[UNICODE] /1088[Десятковий] !!!!
		Перед відніманням тре перетворити UTF8 в UNICODE !!!!		
		*/
				TempChar = (char_16 - 48); //порядковий номер символу в масиві myFont_Big_Digit
				//TempChar = *str - 48; //порядковий номер символу в масиві myFont_Big_Digit
				PutChar( Xpos, Ypos, TempChar, mySize, 1); //mySize індекс таблиці символів
		    str = str + cbyte;
				if( Xpos + DrawProp_Big_Digit.width < MAX_X)
        {
 					  Xpos += DrawProp_Big_Digit.width + 4;
	
        }else if (Ypos + DrawProp_Big_Digit.height < MAX_Y)
        {
            Xpos = 0;
            Ypos += myVert;
        }else
        {
						BSP_LCD_Clear(0x00);
						Xpos = 0;
            Ypos = 0;
        } 
		}

	} while ( *str != 0 );
}

/******************************************************************************
* Function Name  : PutChar
* Description    : 
* Input          : - Xpos:
*                  - Ypos:
*				           - ASCI:
*				           - mySize індекс таблиці симолів
*******************************************************************************/
void PutChar( uint16_t Xpos, uint16_t Ypos, uint16_t ofset_ASCII, uint8_t mySize, uint8_t myFont)
{
//	uint8_t data_LCD;
//  uint8_t PutChar_command_LCD;
//	uint8_t Status_LCD;
//	uint8_t iq, jq, kq;
	uint8_t Xpos_temp, Ypos_temp;
	uint8_t buffer[260]; //Буфер для хранения байтов символа
	uint8_t size_con;
//	uint8_t i, j, k;
//i = Xpos;
//j = Ypos;

	GetASCIICode(buffer, ofset_ASCII, mySize, myFont); //, myVert); //Заполенние буфера buffer точками изображения символа с кодом ASCI

	Xpos_temp = Xpos;
	Ypos_temp = Ypos;
/*
	if(myfont == 0)
{
	if (myGoriz == 1)
	{	
		for( iq=0; iq < myVert; iq++ )
    {
//      tmp_char = buffer[iq];
			LCD_SetAddressPointer( Xpos_temp, Ypos_temp +iq);
			if (buffer[iq] > 0x00 )
			{	
					do
					{
						Status_LCD = LCD_ReadStatus();	
					}while ((Status_LCD & 0x03) != 0x03);
					FMC_BANK1_WriteData(buffer[iq]);
//----	
					command_LCD = 0xC4; //запись без Autoincrement
//					command_LCD = 0xC0; //запись с Autoincrement
					do
					{
						Status_LCD = LCD_ReadStatus();	
					}while ((Status_LCD & 0x03) != 0x03);
					FMC_BANK1_WriteCommand(command_LCD); //RS=1		
			}
		}
	}
	else if (myGoriz == 2)
	{
		kq = 0; //каждый симол  - два байта массива, первый -правая половина символа
		//второй - левая половина символа
		for( iq=0; iq < myVert; iq++ ) //iq - индекс строки симола
		{
			for (jq = 0; jq < myGoriz; jq++) //jq - левая или правая половина символа
			{			
				switch (jq)
				{
					case 0:
//позиция левой половины	myGoriz - 1					
//					  tmp_char = buffer[iq + kq + 1]; //байт правой половины
						LCD_SetAddressPointer( Xpos + jq, Ypos + iq); //  myGoriz - 2
						if (buffer[iq + kq + 1] > 0x00 )
						{	
							do
							{
								Status_LCD = LCD_ReadStatus();	
							}while ((Status_LCD & 0x03) != 0x03);
							FMC_BANK1_WriteData(buffer[iq + kq + 1]);
//----	
							command_LCD = 0xC4; //запись без Autoincrement
							do
							{
								Status_LCD = LCD_ReadStatus();	
							}while ((Status_LCD & 0x03) != 0x03);
							FMC_BANK1_WriteCommand(command_LCD); //RS=1		
						}
						break;
					case 1:
//позиция правой половины
//  					tmp_char = buffer[iq + kq]; //байт левой половины
						LCD_SetAddressPointer( Xpos_temp + jq, Ypos_temp + iq); //  myGoriz - 2

//					LCD_SetAddressPointer( Xpos + jq + kq + myGoriz - 2, Ypos + iq);
						if (buffer[iq + kq] > 0x00 )
						{	
							do
							{
								Status_LCD = LCD_ReadStatus();	
							}while ((Status_LCD & 0x03) != 0x03);
							FMC_BANK1_WriteData(buffer[iq + kq]);
//----	
							command_LCD = 0xC4; //запись без Autoincrement
							do
							{
								Status_LCD = LCD_ReadStatus();	
							}while ((Status_LCD & 0x03) != 0x03);
							FMC_BANK1_WriteCommand(command_LCD); //RS=1		
						}
						break;
					case 2:
//						tmp_char = buffer[iq + jq + kq + myGoriz - 4];
						LCD_SetAddressPointer( Xpos + jq + kq + myGoriz - 4, Ypos + iq - 2);
						if (buffer[iq + jq + kq + myGoriz - 4] > 0x00 )
						{	
							do
							{
								Status_LCD = LCD_ReadStatus();	
							}while ((Status_LCD & 0x03) != 0x03);
							FMC_BANK1_WriteData(buffer[iq + jq + kq + myGoriz - 4]);
//----	
							command_LCD = 0xC4; //запись без Autoincrement
							do
							{
								Status_LCD = LCD_ReadStatus();	
							}while ((Status_LCD & 0x03) != 0x03);
							FMC_BANK1_WriteCommand(command_LCD); //RS=1		
						}
						break;
				}	

			}
			kq = kq + 1;
			Ypos_temp = Ypos;
		}
	}		
	
	}
	else
	{
*/
//buffer[0] содержит кол-во колонок символа, j - текущая колонка
//каждая колонка описывается тремя или 4 байтами хххххххх хххххххх хх
		if (mySize == 0 || mySize == 1 || mySize == 2)
		{
			i_LCD = 3;
		}
		else if (mySize == 4)
		{
		//		i_LCD = 4; //для DigitalsBig[10][81]
//			i_LCD = 7; //для DigitalsBig[10][260]
//			i_LCD = 5; //для DigitalsMidle[10][171]
			i_LCD = 4; //для DigitalsMidle[10][73]
		}	
		else if (mySize == 5)
		{
			i_LCD = 5; //Кількість байтів на один стовпчик для DigitalsBig[10][131]
		}
		else if (mySize == 7)
		{
			i_LCD = 7; //кількість байтів на один стовпчик для DigitalsBigBig[10][260]
		}
		else if (mySize == 8)
		{
			i_LCD = byte_width; //для myFont_ukr 3
		}		
if (mySize < 8)
	{ //Розгортка по вертикалі символа	
		//i_LCD - кількість байт в ширині одного символу
		for(jq = 0; jq < (buffer[0] * i_LCD); jq = jq + i_LCD)
		{	
			//Цикл по групам з buffer[0] байтів поточного символа
			//jq = 0  від 0-го до buffer[0]-1 включно
			//jq = buffer[0]  від buffer[0] до 2*buffer[0]-1
//k - горизонтальна частина колонки битов
			for(kq = 0; kq < i_LCD; kq++)
			{
				//Цикл всередині групи з i_LCD байтів
				if (kq > 0)
				{
					Ypos_temp = Ypos_temp + 8; //зміщення на наступний байт
				}					
				tmp_char = (uint8_t) buffer[jq + 1 + kq]; //поточний байт горизонтальної лінії
				for( iq = 0; iq < 8; iq++ )
				{
					//Якщо поточний біт = 1, замальовувати піксель, якщо 0 - пропустити або замальовувати фоном
				//TempChar = (uint8_t)(((buffer[jq+1+kq] >> iq) & 0x01) << 7);
				//----------Значення поточного біту----------------------
				if ((uint8_t) ((((uint8_t) buffer[jq + 1 + kq] >> iq) & 0x01) << 7) > 0x00 )
				{
					//цикл по поточній вертикалі символа
					//поточний біт ,якщо = 1.ставити Point
					//LCD_SetPoint(Xpos_temp, Ypos_temp + iq);
					//LCD_WritePixel(Xpos_temp, Ypos_temp + iq);
					BSP_LCD_DrawPixel(Xpos_temp, Ypos_temp + iq, DrawProp.TextColor);
				}
					
					
/*					
					LCD_SetAddressPointer( Xpos_temp, Ypos_temp + iq);
					if ((uint8_t)(((buffer[jq+1+kq] >> iq) & 0x01) << 7) > 0x00 )
					{	
						do
						{
							Status_LCD = LCD_ReadStatus();	
						}while ((Status_LCD & 0x03) != 0x03);
						LCD_WriteData((uint8_t)(((buffer[jq+1+kq] >> iq) & 0x01) << 7));
//----	
						PutChar_command_LCD = 0xC4; //запись без Autoincrement
						do
						{
							Status_LCD = LCD_ReadStatus();	
						}while ((Status_LCD & 0x03) != 0x03);
						LCD_WriteCommand(PutChar_command_LCD); //RS=1		
					}	
*/					
				}
			}
			Xpos_temp = Xpos_temp + 1;
			Ypos_temp = Ypos;			
		}
	} else if(mySize == 8)
	{ //Розгортка по горизонталі символу
		//DrawProp_ukr.height кількість пікселів в стовпчику одного символу (27)
		//i_LCD - кількість байтів, що тримають ширину пікселів символа
		
	if (myFont == 0)
	{
		size_con = DrawProp_ukr.height;
	}else if (myFont == 1)
	{
		size_con = DrawProp_Big_Digit.height;
	}

		for(jq = 0; jq < (buffer[0] * size_con); jq = jq + i_LCD) //jq = 0, 3, 6, 9, ...,  27
		{	
			//Цикл по групам з buffer[0] байтів поточного символа
			//jq = 0  від 0-го до buffer[0]
			//jq = buffer[0]  від buffer[0] до 2*buffer[0]
//			Xpos_temp = Xpos;
//k - горизонтальна частина символа
			for(kq = 0; kq < i_LCD; kq++)
			{
				//Цикл всередині групи з i_LCD байтів
				if (kq > 0)
				{
					Xpos_temp = Xpos_temp + 8; //смещение на следующий байт
				}					
				tmp_char = (uint8_t) buffer[jq + 1 + kq]; //поточний байт горизонтальної лінії
				for( iq = 0; iq < 8; iq++ )
				{
//Если текущий бит = 1, ставить точку, если 0 - пропускать
				//TempChar = (uint8_t)(((buffer[jq+1+kq] >> iq) & 0x01) << 7);
				//----------Значение текущего бита----------------------
					if ((uint8_t) ((((uint8_t) buffer[jq + 1 + kq] >> iq) & 0x01) << 7) > 0x00 )
					{
						//цикл по поточному вертикальному стовпчику
						//поточний біт ,якщо = 1.ставити Point
						//LCD_SetPoint(Xpos_temp, Ypos_temp + iq);
						//LCD_WritePixel(Xpos_temp, Ypos_temp + iq);
						BSP_LCD_DrawPixel(Xpos_temp + iq, Ypos_temp, DrawProp_ukr.TextColor);
					}
				}
			}
			Xpos_temp = Xpos;
			Ypos_temp = Ypos_temp + 1;			
		}		
	}
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
