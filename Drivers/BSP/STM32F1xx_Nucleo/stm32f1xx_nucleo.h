/**
  ******************************************************************************
  * @file    stm32f1xx_nucleo.h
  * @author  MCD Application Team
  * @version V1.0.4
  * @date    14-April-2017
  * @brief   This file contains definitions for:
  *          - LEDs and push-button available on STM32F1XX-Nucleo Kit 
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
  ******************************************************************************
  * @attention
  *
  * Конфігурація плати Плата stm32f103 Smart -> Nucleo
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1XX_NUCLEO_H
#define __STM32F1XX_NUCLEO_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32F1XX_NUCLEO
  * @{
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

//Тут виконується призначення пінів

/** @defgroup STM32F1XX_NUCLEO_Exported_Types STM32F1XX NUCLEO Exported Types
  * @{
  */
typedef enum 
{
  LED2 = 0,
  LED_GREEN = LED2
} Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0,
  /* Alias */
  BUTTON_KEY  = BUTTON_USER
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef; 

typedef enum 
{ 
  JOY_NONE = 0,
  JOY_SEL = 1,
  JOY_DOWN = 2,
  JOY_LEFT = 3,
  JOY_RIGHT = 4,
  JOY_UP = 5
} JOYState_TypeDef;


/**
  * @}
  */ 

/** @defgroup STM32F1XX_NUCLEO_Exported_Constants STM32F1XX NUCLEO Exported Constants
  * @{
  */ 

/** 
  * @brief  Define for STM32F1xx_NUCLEO board  
  */ 
#if !defined (USE_STM32F1xx_NUCLEO)
 #define USE_STM32F1xx_NUCLEO
#endif
  
/** @defgroup STM32F1XX_NUCLEO_LED STM32F1XX NUCLEO LED
  * @{
  */
#define LEDn	1

#define LED0_PIN                         GPIO_PIN_13 // PC13 Світлодіод LED0 по принципові схемі STM32_Smart_STM32F103C8T6-STM32_Smart_V2.0.pdf
#define LED0_GPIO_PORT                   GPIOC 
#define LED0_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()  
#define LED0_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if((__INDEX__) == 0) LED0_GPIO_CLK_ENABLE();} while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? LED0_GPIO_CLK_DISABLE() : 0)

/**
  * @}
  */ 

/** @defgroup STM32F1XX_NUCLEO_BUTTON STM32F1XX NUCLEO BUTTON
  * @{
  */  
#define BUTTONn                          1  

/**
  * @brief User push-button
 */
#define USER_BUTTON_PIN                  GPIO_PIN_0  
#define USER_BUTTON_GPIO_PORT            GPIOA // PA0 Кнопка SW-SPST по принциповіё схемі STM32_Smart_STM32F103C8T6-STM32_Smart_V2.0.pdf
#define USER_BUTTON_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE() //__HAL_RCC_GPIOC_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOA_CLK_DISABLE() //__HAL_RCC_GPIOC_CLK_DISABLE()
#define USER_BUTTON_EXTI_IRQn            EXTI0_IRQn 
/* Aliases */
#define KEY_BUTTON_PIN                   USER_BUTTON_PIN
#define KEY_BUTTON_GPIO_PORT             USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_GPIO_CLK_ENABLE()     USER_BUTTON_GPIO_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()    USER_BUTTON_GPIO_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn             USER_BUTTON_EXTI_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == 0) USER_BUTTON_GPIO_CLK_ENABLE();} while(0)
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? USER_BUTTON_GPIO_CLK_DISABLE() : 0)
/**
  * @}
  */
    
/** @addtogroup STM32F1XX_NUCLEO_BUS STM32F1XX NUCLEO BUS
  * @{
  */
/*###################### SPI1 ###################################*/
#define NUCLEO_SPIx                                 SPI2 
#define NUCLEO_SPIx_CLK_ENABLE()                    __HAL_RCC_SPI2_CLK_ENABLE() 

#define NUCLEO_SPIx_SCK_PIN                         GPIO_PIN_13 
#define NUCLEO_SPIx_SCK_GPIO_PORT                   GPIOB  //SPI SCK PB13
#define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE() 
#define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()


#define NUCLEO_SPIx_MOSI_PIN                        GPIO_PIN_15
#define NUCLEO_SPIx_MOSI_GPIO_PORT             			GPIOB 
#define NUCLEO_SPIx_MOSI_GPIO_CLK_ENABLE()    		 __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_SPIx_MOSI_GPIO_CLK_DISABLE()    		 __HAL_RCC_GPIOB_CLK_DISABLE()

#define NUCLEO_SPIx_MISO_PIN                        GPIO_PIN_14 
#define NUCLEO_SPIx_MISO_GPIO_PORT             			GPIOB 
#define NUCLEO_SPIx__MISO_GPIO_CLK_ENABLE()    		  __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_SPIx__MISO_GPIO_CLK_DISABLE()    		__HAL_RCC_GPIOB_CLK_DISABLE()


#define NUCLEO_SPIx_TIMEOUT_MAX                     1000


/**
  * @brief  SD Control Lines management Вибір SD Card Select
  */  
#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)
    
/**
  * @brief  LCD Control Lines management Вибір LCD Chip Select
  */
#define LCD_CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)

#define LCD_DC_LOW()      HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)

#define LCD_RST_LOW()     HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_RESET)
#define LCD_RST_HIGH()    HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_SET)

/**
  * @brief  SD Control Interface pins (shield D4)
  */
#define SD_CS_PIN                                 GPIO_PIN_5
#define SD_CS_GPIO_PORT                           GPIOB
#define SD_CS_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOB_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOB_CLK_DISABLE()

/**
  * @brief  LCD Control Interface pins (shield D10)
  */
#define LCD_CS_PIN                               GPIO_PIN_12 //SPI2_NSS - не використовую
#define LCD_CS_GPIO_PORT                           GPIOB
#define LCD_CS_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_CS_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOB_CLK_DISABLE()


// Використовую PB PIN_11 в платі з проводочком
// Використовую PA PIN_7 в платі без проводочка

#define LCD_RST_PIN                              GPIO_PIN_7  //PA7
#define LCD_RST_GPIO_PORT                        GPIOA 
#define LCD_RST_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOA_CLK_ENABLE() //__HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_RST_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOA_CLK_DISABLE() //__HAL_RCC_GPIOB _CLK_DISABLE()

/**
  * @brief  LCD Data/Command Interface pins
  */
#define LCD_DC_PIN                                 GPIO_PIN_1 //PB1
#define LCD_DC_GPIO_PORT                           GPIOB 
#define LCD_DC_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOB_CLK_ENABLE() //__HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_DC_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOB_CLK_DISABLE() //__HAL_RCC_GPIOA_CLK_DISABLE()


/* Basic operations */
#define LCD_RST_Clr() HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_RESET)
#define LCD_RST_Set() HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_SET)

#define LCD_DC_Clr() HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_Set() HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)

#define LCD_Select() HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_UnSelect() HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)


/*##################### ADC1 ###################################*/
/**
  * @brief  ADC Interface pins
  *         used to detect motion of Joystick available on Adafruit 1.8" TFT shield
  */
#define NUCLEO_ADCx                                 ADC1
#define NUCLEO_ADCx_CLK_ENABLE()                    __HAL_RCC_ADC1_CLK_ENABLE()
#define NUCLEO_ADCx_CLK_DISABLE()                 __HAL_RCC_ADC1_CLK_DISABLE()
    
#define NUCLEO_ADCx_GPIO_PORT                       GPIOB
#define NUCLEO_ADCx_GPIO_PIN                        GPIO_PIN_0
#define NUCLEO_ADCx_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_ADCx_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOB_CLK_DISABLE()
    
/**
  * @}
  */
    

/**
  * @}
  */
    
/** @addtogroup STM32F1XX_NUCLEO_Exported_Functions
  * @{
  */
uint32_t        BSP_GetVersion(void);
/** @addtogroup STM32F1XX_NUCLEO_LED_Functions STM32F1XX NUCLEO LED Functions
  * @{
  */ 

void            BSP_LED_Init(Led_TypeDef Led);
void            BSP_LED_DeInit(Led_TypeDef Led);
void            BSP_LED_On(Led_TypeDef Led);
void            BSP_LED_Off(Led_TypeDef Led);
void            BSP_LED_Toggle(Led_TypeDef Led);
void            LCD_IO_Init(void);
/**
  * @}
  */

/** @addtogroup STM32F1XX_NUCLEO_BUTTON_Functions STM32F1XX NUCLEO BUTTON Functions
  * @{
  */

void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void             BSP_PB_DeInit(Button_TypeDef Button);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);


uint8_t          BSP_JOY_Init(void);
JOYState_TypeDef BSP_JOY_GetState(void);
void             BSP_JOY_DeInit(void);

void               SPIx_Init(void);
void               SPIx_Write(uint8_t Value);
void               SPIx_WriteData(uint8_t *DataIn, uint16_t DataLength);
void               SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth);
void               SPIx_Error (void);
void               SPIx_MspInit(void);

/* SD IO functions */
void               SD_IO_Init(void);
void               SD_IO_CSState(uint8_t state);
void               SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
void               SD_IO_ReadData(uint8_t *DataOut, uint16_t DataLength);
void               SD_IO_WriteData(const uint8_t *Data, uint16_t DataLength);
uint8_t            SD_IO_WriteByte(uint8_t Data);
uint8_t                   SD_IO_ReadByte(void);

/* LCD IO functions */
void							 LCD_SendCommand(uint8_t cmd);
void 							 LCD_SendData(uint8_t *buff, size_t buff_size);

//void               LCD_IO_WriteData(uint8_t Data);
void               LCD_SendMultipleData(uint8_t *pData, uint32_t Size);
void               LCD_Delay(uint32_t delay);



HAL_StatusTypeDef  ADCx_Init(void);
void               ADCx_DeInit(void);
void               ADCx_MspInit(ADC_HandleTypeDef *hadc);
void               ADCx_MspDeInit(ADC_HandleTypeDef *hadc);




/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1XX_NUCLEO_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
