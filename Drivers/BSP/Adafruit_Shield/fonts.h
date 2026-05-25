#ifndef __FONT_H
#define __FONT_H

#include "stdint.h"

typedef struct _tFont
{    
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;
  
} sFONT;

typedef struct { //_tFont
  const uint8_t *table;
  uint16_t width;
  uint16_t height;
} sFontDef;

typedef struct {
    uint16_t width;
    uint16_t height;
    const uint16_t *data;
} FontDef;

typedef struct {
    uint16_t width;
    uint16_t height;
    const uint8_t *data;
} bFontDef;

/*typedef struct 
{ 	
  uint32_t 		TextColor;
  uint32_t 		BackColor;
	uint16_t		width;
	uint16_t		height;
  uint8_t    *pFont; 
}LCD_DrawPropTypeDef; */


/** 
  * @brief  Draw Properties structures definition
  */ 
typedef struct 
{ 	
  uint32_t 		TextColor;
  uint32_t 		BackColor;
	uint16_t		width;
	uint16_t		height;
  uint8_t    *pFont; 
}LCD_DrawPropTypeDef; 

void GetASCIICode(uint8_t* pBuffer, uint16_t ofset_ASCII, unsigned char mySize, uint8_t myFont);

//16-bit(RGB565) Image lib.
/*******************************************
 *             CAUTION:
 *   If the MCU onchip flash cannot
 *  store such huge image data,please
 *           do not use it.
 * These pics are for test purpose only.
 *******************************************/

/* 128x128 pixel RGB565 image */

/* 240x240 pixel RGB565 image 
extern const uint16_t knky[][240];
extern const uint16_t tek[][240];
extern const uint16_t adi1[][240];
*/
#endif
