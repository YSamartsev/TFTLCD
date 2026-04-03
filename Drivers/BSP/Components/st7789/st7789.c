#include "st7789.h"
#include "fonts.h"
#include "stm32f1xx_nucleo.h"
#include "main.h"
#include "stm32_adafruit_lcd.h"


/*
Використовується 16-бітний RGB формат
|xxxxxxxx xxxxxxxx xxxxxxxx
*/

extern SPI_HandleTypeDef SpiHandle;

//small fonts
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

extern const uint16_t saber; //picture

//big fonts
extern sFontDef Font24;
extern sFontDef Font20;
extern sFontDef Font16;
extern sFontDef Font12;
extern sFontDef Font8;

extern uint8_t* DigitalsBigBig[];

extern LCD_DrawPropTypeDef DrawProp;

extern uint32_t bi;

uint8_t data_LCD;
uint16_t i_LCD;
uint16_t j_LCD;
uint8_t TempChar;
uint16_t i, j, k;
uint16_t iq, jq, kq;
uint8_t tmp_char;

//Для разворачивания кода шрифта по горизонтали ASCII_8X8_System
//int myfont = 0;
//int myVert = 8; 
//int myGoriz= 1;
//-----------------------------

//Для разворачивания кода шрифта по горизонтали ASCII_8X14_System
uint8_t myfont = 0;
uint8_t myVert = 14; 
uint8_t myGoriz = 1;
//-----------------------------

//Для разворачивания кода шрифта по горизонтали ASCII_8X16_System
//int myfont = 0;
//int myVert = 16; 
//int myGoriz= 1;
//-----------------------------

//Для разворачивания кода шрифта по горизонтали ASCII_10X16_System
//int myfont = 0;
//int myVert = 16; 
//int myGoriz= 2;
//-----------------------------

//Для разворачивания кода шрифта по вертикали ASCII_10X20_Terminal
//int myfont = 1;
//int myVert = 20; //кол-во вертикальных точек в знаке
//int myGoriz= 10; //кол-во горизонтальных точек в знаке

//uint8_t buffer[]; //Буфер для хранения байтов символа





//Заповнюю структуру драйвера
LCD_7789_DrvTypeDef   ST7789_drv = 
{
  ST7789_Init,
  0,
	ST7789_SetRotation,
	ST7789_Fill_Color,
	ST7789_DrawPixel,
	ST7789_Fill,
	ST7789_DrawPixel_4px,
/* Graphical functions. */
	ST7789_DrawLine,
	ST7789_DrawRectangle,
	ST7789_DrawCircle,
	ST7789_DrawImage,
	ST7789_InvertColors,
/* Text functions. */
	ST7789_WriteChar,
	ST7789_WriteString,
/* Extented Graphical functions. */
	ST7789_DrawFilledRectangle,
	ST7789_DrawTriangle,
	ST7789_DrawFilledTriangle,
	ST7789_DrawFilledCircle,
  ST7789_GetLcdPixelWidth,
  ST7789_GetLcdPixelHeight,
	ST7789_DrawHLine,
  ST7789_DrawVLine,
	ST7789_DrawBitmap,
	ST7789_SetDisplayWindow,

};

static uint16_t ArrayRGB[480] = {0};

#ifdef USE_DMA
#include <string.h>
uint16_t DMA_MIN_SIZE = 16;
/* If you're using DMA, then u need a "framebuffer" to store datas to be displayed.
 * If your MCU don't have enough RAM, please avoid using DMA(or set 5 to 1).
 * And if your MCU have enough RAM(even larger than full-frame size),
 * Then you can specify the framebuffer size to the full resolution below.
 */
 #define HOR_LEN 	5	//	Also mind the resolution of your screen!
uint16_t disp_buf[ST7789_WIDTH * HOR_LEN];
#endif

/**
 * @brief Write command to ST7789 controller
 * @param cmd -> command to write
 * @return none
 */
/*static void LCD_SendCommand(uint8_t cmd)
{
	LCD_Select();
	LCD_DC_ReSet();
	HAL_SPI_Transmit(&SpiHandle, &cmd, sizeof(cmd), HAL_MAX_DELAY);
	LCD_UnSelect();
} */

/**
 * @brief Write data to ST7789 controller
 * @param buff -> pointer of data buffer
 * @param buff_size -> size of the data buffer
 * @return none
 */
/*static void LCD_SendData(uint8_t *buff, size_t buff_size)
{
	LCD_Select();
	LCD_DC_Set();

	// split data in small chunks because HAL can't send more than 64K at once

	while (buff_size > 0) {
		uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;
		#ifdef USE_DMA
			if (DMA_MIN_SIZE <= buff_size)
			{
				HAL_SPI_Transmit_DMA(&ST7789_SPI_PORT, buff, chunk_size);
				while (ST7789_SPI_PORT.hdmatx->State != HAL_DMA_STATE_READY)
				{}
			}
			else
				HAL_SPI_Transmit(&ST7789_SPI_PORT, buff, chunk_size, HAL_MAX_DELAY);
		#else
			HAL_SPI_Transmit(&SpiHandle, buff, chunk_size, HAL_MAX_DELAY);
		#endif
		buff += chunk_size;
		buff_size -= chunk_size;
	}

	LCD_UnSelect();
} */

/**
 * @brief Write data to ST7789 controller, simplify for 8bit data.
 * data -> data to write
 * @return none
 */
/* static void LCD_SendSmallData(uint8_t data)
{
	LCD_Select();
	LCD_DC_Set();
	HAL_SPI_Transmit(&SpiHandle, &data, sizeof(data), HAL_MAX_DELAY);
	LCD_UnSelect();
} */

/**
 * @brief Set the rotation direction of the display
 * @param m -> rotation parameter(please refer it in st7789.h)
 * @return none
 */
void ST7789_SetRotation(uint8_t m)
{
	LCD_SendCommand(ST7789_MADCTL);	// MADCTL
	switch (m) {
	case 0:
		//ST7789_WriteSmallData(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);
		{
			uint8_t data[] = {ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB};
			LCD_SendData(data, sizeof(data));
		}
		break;
	case 1:
		//LCD_SendSmallData(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		{
			uint8_t data[] = {ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB};
			LCD_SendData(data, sizeof(data));
		}		
	
		break;
	case 2:
		//ST7789_SendSmallData(ST7789_MADCTL_RGB);
		{
			uint8_t data[] = {ST7789_MADCTL_RGB};
			LCD_SendData(data, sizeof(data));
		}	
		break;
	case 3:
		//ST7789_SendSmallData(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		{
			uint8_t data[] = {ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB};
			LCD_SendData(data, sizeof(data));
		}
  	break;
	default:
		break;
	}
}

/**
 * @brief Set address of DisplayWindow
 * @param xi&yi -> coordinates of window
 * @return none
 */
static void ST7789_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	LCD_Select();
	uint16_t x_start = x0 + X_SHIFT, x_end = x1 + X_SHIFT;
	uint16_t y_start = y0 + Y_SHIFT, y_end = y1 + Y_SHIFT;
	
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
	LCD_UnSelect();
}

/**
 * @brief Initialize ST7789 controller
 * @param none
 * @return none
 */
void ST7789_Init(void)
{
	#ifdef USE_DMA
		memset(disp_buf, 0, sizeof(disp_buf));
	#endif
	HAL_Delay(10);
  LCD_RESET_SET();

//while(1)
//{	
		LCD_SendCommand(ST7789_COLMOD);		//	Встановлення RGB режиму
//}
	
    //LCD_SendSmallData(ST7789_COLOR_MODE_16bit);
	{
		uint8_t data[] = {ST7789_COLOR_MODE_16bit}; //0x55  16bit/pixel 65K RGB
		LCD_SendData(data, sizeof(data));
	}
  	LCD_SendCommand(ST7789_PORCTRL);				//	0xB2 Porch control Керування інтервал гасіння
	{
		uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
		LCD_SendData(data, sizeof(data));
	}
	ST7789_SetRotation(ST7789_ROTATION);	//	MADCTL (Memory Data Access Control) (Display Rotation) 
	
	/* Internal LCD Voltage generator settings */
	LCD_SendCommand(ST7789_GCTRL);				//	Gate Control
    //LCD_SendSmallData(0x35);			//	Default value
  {
		uint8_t data[] = {0x35};
		LCD_SendData(data, sizeof(data));
	}		
	
	LCD_SendCommand(ST7789_VCOMS);				//	VCOM setting керування напругою
    //LCD_SendSmallData(0x19);			//	0.725v (default 0.75v for 0x20)
		{		
			uint8_t data[] = {0x19};
			LCD_SendData(data, sizeof(data)); 
		}			
	
		LCD_SendCommand(ST7789_LCMCTRL);				//	LCMCTRL	
    //LCD_SendSmallData (0x2C);			//	Default value
		{		
			uint8_t data[] = {0x2C};
			LCD_SendData(data, sizeof(data)); 
		}	   
		
		LCD_SendCommand (0xC2);				//	VDV and VRH command Enable
    //LCD_SendSmallData (0x01);			//	Default value
		{		
			uint8_t data[] = {0x01};
			LCD_SendData(data, sizeof(data)); 
		}	      
		
		LCD_SendCommand (0xC3);				//	VRH set
    //LCD_SendSmallData (0x12);			//	+-4.45v (defalut +-4.1v for 0x0B)
		{		
			uint8_t data[] = {0x12};
			LCD_SendData(data, sizeof(data)); 
		}    
		
		LCD_SendCommand (0xC4);				//	VDV set
		//LCD_SendSmallData (0x20);			//	Default value
		{		
			uint8_t data[] = {0x20};
			LCD_SendData(data, sizeof(data)); 
		}

    LCD_SendCommand (0xC6);				//	Frame rate control in normal mode
    //LCD_SendSmallData (0x0F);			//	Default value (60HZ)
		{		
			uint8_t data[] = {0x0F};
			LCD_SendData(data, sizeof(data)); 
		}

    LCD_SendCommand (0xD0);				//	Power control
    //LCD_SendSmallData (0xA4);			//	Default value
    //LCD_SendSmallData (0xA1);			//	Default value
		{		
			uint8_t data[] = {0xA4, 0xA1};
			LCD_SendData(data, sizeof(data)); 
		}

		/**************** Division line ****************/
	LCD_SendCommand(0xE0);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
		LCD_SendData(data, sizeof(data));
	}

  LCD_SendCommand(0xE1);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
		LCD_SendData(data, sizeof(data));
	}
  LCD_SendCommand (ST7789_INVON);		//	Inversion ON
	LCD_SendCommand (ST7789_SLPOUT);	//	Out of sleep mode
  LCD_SendCommand (ST7789_NORON);		//	Normal Display on
  LCD_SendCommand (ST7789_DISPON);	//	Main screen turned on	

}

/**
 * @brief Fill the DisplayWindow with single color
 * @param color -> color to Fill with
 * @return none
 */
void ST7789_Fill_Color(uint16_t color)
{
	uint16_t i = 0, j = 0, z = 0;
	ST7789_SetAddressWindow(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);
	LCD_Select();

	#ifdef USE_DMA
		for (i = 0; i < ST7789_HEIGHT / HOR_LEN; i++)
		{
			memset(disp_buf, color, sizeof(disp_buf));
			LCD_SendData(disp_buf, sizeof(disp_buf));
		}
	#else
			for (i = 0; i < ST7789_WIDTH; i++)
			{
				for (j = 0; j < ST7789_HEIGHT; j++) 
				{
					uint8_t data[] = {color >> 8, color & 0xFF};
					LCD_SendData(data, sizeof(data));
					z++;
				}
			}
	#endif
			printf("------------z = %04d\n\r", z);
	LCD_UnSelect();
}

/**
 * @brief Draw a Pixel
 * @param x&y -> coordinate to Draw
 * @param color -> color of the Pixel
 * @return none
 */
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT))	return;
	
	ST7789_SetAddressWindow(x, y, x, y); //Встановлюю адреси рядка і колонки для точки
	//В Ініціалізації вказав COLMOD (3Ah) = 55h
	//Стор.  71 of 317 перший байт R4_R3_R2_R1_R0_G5_G4_G3 другий байт G2_G1_G0_B4_B3_B2_B1_B0
	//G5_G4_G3_G2_G1_G0 Глибина зеленого, R4_R3_R2_R1_R0 Глибина червоного, B4_B3_B2_B1_B0 Глибина синього 
	//Наприклад для LCD_GREEN першим байтом треба передати 0000`0111 другим 1110'0000
	uint8_t data[] = {color >> 8, color & 0xFF};
	LCD_Select();
	LCD_SendData(data, sizeof(data));
	LCD_UnSelect();
}

/**
 * @brief Fill an Area with single color
 * @param xSta&ySta -> coordinate of the start point
 * @param xEnd&yEnd -> coordinate of the end point
 * @param color -> color to Fill with
 * @return none
 */
void ST7789_Fill(uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color)
{
	if ((xEnd >= ST7789_WIDTH) || (yEnd >= ST7789_HEIGHT))	return;
	LCD_Select();
	uint16_t i, j;
	ST7789_SetAddressWindow(xSta, ySta, xEnd, yEnd);
	for (i = ySta; i <= yEnd; i++)
		for (j = xSta; j <= xEnd; j++) {
			uint8_t data[] = {color >> 8, color & 0xFF};
			LCD_SendData(data, sizeof(data));
		}
	LCD_UnSelect();
}

/**
 * @brief Draw a big Pixel at a point
 * @param x&y -> coordinate of the point
 * @param color -> color of the Pixel
 * @return none
 */
void ST7789_DrawPixel_4px(uint16_t x, uint16_t y, uint16_t color)
{
	if ((x <= 0) || (x > ST7789_WIDTH) ||
		 (y <= 0) || (y > ST7789_HEIGHT))	return;
	LCD_Select();
	ST7789_Fill(x - 1, y - 1, x + 1, y + 1, color);
	LCD_UnSelect();
}

/**
  * @brief  Draws horizontal line.
  * @param  RGBCode: Specifies the RGB color   
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Length: specifies the line length.  
  * @retval None
  */
void ST7789_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint8_t counter = 0;
  
  if(Xpos + Length > ST7789_LCD_PIXEL_WIDTH) return;
  
  /* Set Cursor */
  ST7789_SetCursor(Xpos, Ypos);
  
  for(counter = 0; counter < Length; counter++)
  {
    ArrayRGB[counter] = RGBCode;
  }
  LCD_SendMultipleData((uint8_t*)&ArrayRGB[0], Length * 2);
}

/**
  * @brief  Draws vertical line.
  * @param  RGBCode: Specifies the RGB color   
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Length: specifies the line length.  
  * @retval None
  */
void ST7789_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint8_t counter = 0;
  
  if(Ypos + Length > ST7789_LCD_PIXEL_HEIGHT) return;
  for(counter = 0; counter < Length; counter++)
  {
    ST7789_WritePixel(Xpos, Ypos + counter, RGBCode);
  }   
}

/**
  * @brief  Sets Cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @retval None
  */
void ST7789_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
  uint8_t data = 0;
  LCD_SendCommand(ST7789_CASET);
  data = (Xpos) >> 8;
  LCD_SendMultipleData(&data, 1);
  data = (Xpos) & 0xFF;
  LCD_SendMultipleData(&data, 1);
  LCD_SendCommand(ST7789_RASET); 
  data = (Ypos) >> 8;
  LCD_SendMultipleData(&data, 1);
  data = (Ypos) & 0xFF;
  LCD_SendMultipleData(&data, 1);
  LCD_SendCommand(ST7789_RAMWR);
}

/**
 * @brief Draw a line with single color
 * @param x1&y1 -> coordinate of the start point
 * @param x2&y2 -> coordinate of the end point
 * @param color -> color of the line to Draw
 * @return none
 */
void ST7789_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
        uint16_t color) {
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
            ST7789_DrawPixel(y0, x0, color);
        } else {
            ST7789_DrawPixel(x0, y0, color);
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
void ST7789_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	LCD_Select();
	ST7789_DrawLine(x1, y1, x2, y1, color);
	ST7789_DrawLine(x1, y1, x1, y2, color);
	ST7789_DrawLine(x1, y2, x2, y2, color);
	ST7789_DrawLine(x2, y1, x2, y2, color);
	LCD_UnSelect();
}

/** 
 * @brief Draw a circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle line
 * @return  none
 */
void ST7789_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	LCD_Select();
	ST7789_DrawPixel(x0, y0 + r, color);
	ST7789_DrawPixel(x0, y0 - r, color);
	ST7789_DrawPixel(x0 + r, y0, color);
	ST7789_DrawPixel(x0 - r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7789_DrawPixel(x0 + x, y0 + y, color);
		ST7789_DrawPixel(x0 - x, y0 + y, color);
		ST7789_DrawPixel(x0 + x, y0 - y, color);
		ST7789_DrawPixel(x0 - x, y0 - y, color);

		ST7789_DrawPixel(x0 + y, y0 + x, color);
		ST7789_DrawPixel(x0 - y, y0 + x, color);
		ST7789_DrawPixel(x0 + y, y0 - x, color);
		ST7789_DrawPixel(x0 - y, y0 - x, color);
	}
	LCD_UnSelect();
}

/**
 * @brief Draw an Image on the screen
 * @param x&y -> start point of the Image
 * @param w&h -> width & height of the Image to Draw
 * @param data -> pointer of the Image array
 * @return none
 */
void ST7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
	if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT))
		return;
	if ((x + w - 1) >= ST7789_WIDTH)
		return;
	if ((y + h - 1) >= ST7789_HEIGHT)
		return;

	LCD_Select();
	ST7789_SetAddressWindow(x, y, x + w - 1, y + h - 1);
	LCD_SendData((uint8_t *)data, sizeof(uint16_t) * w * h);
	LCD_UnSelect();
}

/**
 * @brief Invert Fullscreen color
 * @param invert -> Whether to invert
 * @return none
 */
void ST7789_InvertColors(uint8_t invert)
{
	LCD_Select();
	LCD_SendCommand(invert ? 0x21 /* INVON */ : 0x20 /* INVOFF */);
	LCD_UnSelect();
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
void ST7789_WriteChar(uint16_t x, uint16_t y, char ch, FontDef Font, uint16_t color, uint16_t bgcolor)
{
	uint32_t i, j;
	LCD_Select();
	ST7789_SetAddressWindow(x, y, x + Font.width - 1, y + Font.height - 1);
	
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
	LCD_UnSelect();
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
void ST7789_WriteString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor)
{
	LCD_Select();
	while (*str) {
		if (x + font.width >= ST7789_WIDTH) {
			x = 0;
			y += font.height;
			if (y + font.height >= ST7789_HEIGHT) {
				break;
			}

			if (*str == ' ') {
				// skip spaces in the beginning of the new line
				str++;
				continue;
			}
		}
		ST7789_WriteChar(x, y, *str, font, color, bgcolor);
		x += font.width;
		str++;
	}
	LCD_UnSelect();
}

/** 
 * @brief Draw a filled Rectangle with single color
 * @param  x&y -> coordinates of the starting point
 * @param w&h -> width & height of the Rectangle
 * @param color -> color of the Rectangle
 * @return  none
 */
void ST7789_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	LCD_Select();
	uint8_t i;

	/* Check input parameters */
	if (x >= ST7789_WIDTH ||
		y >= ST7789_HEIGHT) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= ST7789_WIDTH) {
		w = ST7789_WIDTH - x;
	}
	if ((y + h) >= ST7789_HEIGHT) {
		h = ST7789_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		ST7789_DrawLine(x, y + i, x + w, y + i, color);
	}
	LCD_UnSelect();
}

/** 
 * @brief Draw a Triangle with single color
 * @param  xi&yi -> 3 coordinates of 3 top points.
 * @param color ->color of the lines
 * @return  none
 */
void ST7789_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	LCD_Select();
	/* Draw lines */
	ST7789_DrawLine(x1, y1, x2, y2, color);
	ST7789_DrawLine(x2, y2, x3, y3, color);
	ST7789_DrawLine(x3, y3, x1, y1, color);
	LCD_UnSelect();
}

/** 
 * @brief Draw a filled Triangle with single color
 * @param  xi&yi -> 3 coordinates of 3 top points.
 * @param color ->color of the triangle
 * @return  none
 */
void ST7789_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	LCD_Select();
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
			yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
			curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	}
	else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	}
	else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay) {
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	}
	else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		ST7789_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
	LCD_UnSelect();
}

/** 
 * @brief Draw a Filled circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle
 * @return  none
 */
void ST7789_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	LCD_Select();
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	ST7789_DrawPixel(x0, y0 + r, color);
	ST7789_DrawPixel(x0, y0 - r, color);
	ST7789_DrawPixel(x0 + r, y0, color);
	ST7789_DrawPixel(x0 - r, y0, color);
	ST7789_DrawLine(x0 - r, y0, x0 + r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7789_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
		ST7789_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);

		ST7789_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
		ST7789_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
	}
	LCD_UnSelect();
}


/**
  * @brief  Gets the LCD pixel Width.
  * @param  None
  * @retval The Lcd Pixel Width
  */
uint16_t ST7789_GetLcdPixelWidth(void)
{
  return ST7789_WIDTH;
}

/**
  * @brief  Gets the LCD pixel Height.
  * @param  None
  * @retval The Lcd Pixel Height
  */
uint16_t ST7789_GetLcdPixelHeight(void)
{                          
  return  ST7789_HEIGHT;
}

/**
  * @brief  Displays a bitmap picture loaded in the internal Flash.
  * @param  BmpAddress: Bmp picture address in the internal Flash.
  * @retval None
  */
void ST7789_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
{
  uint32_t index = 0, size = 0;
  
  /* Read bitmap size */
  size = *(volatile uint16_t *) (pbmp + 2);
  size |= (*(volatile uint16_t *) (pbmp + 4)) << 16;
  /* Get bitmap data address offset */
  index = *(volatile uint16_t *) (pbmp + 10);
  index |= (*(volatile uint16_t *) (pbmp + 12)) << 16;
  size = (size - index)/2;
  pbmp += index;
  
  /* Set GRAM write direction and BGR = 0 */
  /* Memory access control: MY = 0, MX = 1, MV = 0, ML = 0 */
  ST7789_WriteReg(ST7789_MADCTL, 0x40);

  /* Set Cursor */
  ST7789_SetCursor(Xpos, Ypos);  
 
  LCD_SendMultipleData((uint8_t*)pbmp, size*2);
 
  /* Set GRAM write direction and BGR = 0 */
  /* Memory access control: MY = 1, MX = 1, MV = 0, ML = 0 */
  ST7789_WriteReg(ST7789_MADCTL, 0xC0);
}


/**
  * @brief  Writes pixel.   
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  RGBCode: the RGB pixel color
  * @retval None
  */
void ST7789_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode)
{
  uint8_t data = 0;
  if((Xpos >= ST7789_LCD_PIXEL_WIDTH) || (Ypos >= ST7789_LCD_PIXEL_HEIGHT)) 
  {
    return;
  }
  
  /* Set Cursor */
  ST7789_SetCursor(Xpos, Ypos);
  
  data = RGBCode >> 8;
  LCD_SendMultipleData(&data, 1);
  data = RGBCode;
  LCD_SendMultipleData(&data, 1);
} 

/**
  * @brief  Writes to the selected LCD register.
  * @param  LCDReg: Address of the selected register.
  * @param  LCDRegValue: value to write to the selected register.
  * @retval None
  */
void ST7789_WriteReg(uint8_t LCDReg, uint8_t LCDRegValue)
{
  LCD_SendCommand(LCDReg);
  LCD_SendMultipleData(&LCDRegValue, 1);
}

/**
 * @brief Open/Close tearing effect line
 * @param tear -> Whether to tear
 * @return none
 */
void ST7789_TearEffect(uint8_t tear)
{
	LCD_Select();
	LCD_SendCommand(tear ? 0x35 /* TEON */ : 0x34 /* TEOFF */);
	LCD_UnSelect();
}

/**
  * @brief  Sets a display window
  * @param  Xpos:   specifies the X bottom left position.
  * @param  Ypos:   specifies the Y bottom left position.
  * @param  Height: display window height.
  * @param  Width:  display window width.
  * @retval None
  */
void ST7789_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  uint8_t data = 0;
  /* Column addr set, 4 args, no delay: XSTART = Xpos, XEND = (Xpos + Width - 1) */
  LCD_SendCommand(ST7789_CASET);
  data = (Xpos) >> 8;
  LCD_SendMultipleData(&data, 1);
  data = (Xpos) & 0xFF;
  LCD_SendMultipleData(&data, 1);
  data = (Xpos + Width - 1) >> 8;
  LCD_SendMultipleData(&data, 1);
  data = (Xpos + Width - 1) & 0xFF;
  LCD_SendMultipleData(&data, 1);
  /* Row addr set, 4 args, no delay: YSTART = Ypos, YEND = (Ypos + Height - 1) */
  LCD_SendCommand(ST7789_RASET);
  data = (Ypos) >> 8;
  LCD_SendMultipleData(&data, 1);
  data = (Ypos) & 0xFF;
  LCD_SendMultipleData(&data, 1);
  data = (Ypos + Height - 1) >> 8;
  LCD_SendMultipleData(&data, 1);
  data = (Ypos + Height - 1) & 0xFF;
  LCD_SendMultipleData(&data, 1);
}

/******************************************************************************
* Function Name  : PutChar
* Description    : 
* Input          : - Xpos:
*                  - Ypos:
*				           - ASCI:
*				           - mySize індекс таблиці симолів
*******************************************************************************/
void PutChar( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint8_t mySize)
{
//	uint8_t data_LCD;
//  uint8_t PutChar_command_LCD;
//	uint8_t Status_LCD;
//	uint8_t iq, jq, kq;
	uint8_t Xpos_temp, Ypos_temp;
	uint8_t buffer[260]; //Буфер для хранения байтов символа

//	uint8_t i, j, k;
//i = Xpos;
//j = Ypos;

	GetASCIICode(buffer, ASCI, mySize); //, myVert); //Заполенние буфера buffer точками изображения символа с кодом ASCI

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
			i_LCD = 5; //Кількість стовпчиків для DigitalsBig[10][131]
		}
		else if (mySize == 7)
		{
			i_LCD = 7; //для для DigitalsBigBig[10][260]
		}
		
		for(jq = 0; jq < (buffer[0] * i_LCD); jq = jq + i_LCD)
		{	
			//Цикл по групам з 7-ьи байтів поточного символа
			//jq = 0  від 0-го до 6-го
			//jq = 7  від 7-го до 13-го
//			Ypos_temp = Ypos;
//k - вертикальная часть колонки битов
			for(kq = 0; kq < i_LCD; kq++)
			{
				//Цикл всередині групи з п'яти байтів
				if (kq > 0)
				{
					Ypos_temp = Ypos_temp + 8; //смещение на следующий байт
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
					BSP_LCD_DrawPixel((Xpos_temp), Ypos_temp + iq, DrawProp.TextColor);
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
  //}
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
	uint8_t Xpos, Ypos;
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
	myVert = 34; //кол-во вертикальных точек в знаке
	myGoriz= 26; //кол-во горизонтальных точек в знаке
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
		
		PutChar( Xpos, Ypos, TempChar, mySize); //mySize індекс таблиці символів
			
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

