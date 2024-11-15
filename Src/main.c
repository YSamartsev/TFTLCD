/**
  ******************************************************************************
  * @file    Demonstrations/Adafruit_LCD_1_8_SD_Joystick/Src/main.c 
  * @author  MCD Application Team
  * @brief   This demo describes how display bmp images from SD card on LCD using
             the Adafruit 1.8" TFT shield with Joystick and microSD mounted on top
             of the STM32 Nucleo board.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "../HARDWARE/LCD/lcd.h"
#include "../SYSTEM/delay/delay.h"
#include "../SYSTEM/sys/sys.h"
#include "../HARDWARE/TOUCH/touch.h"
#include "GUI.h"
#include "test.h"

//#include "EventRecorder.h"


/** @addtogroup STM32F1xx_HAL_Demonstrations
  * @{
  */

/** @addtogroup Demo
  * @{
  */ 

///* Private typedef -----------------------------------------------------------*/
///* RTC handler declaration */
//==RTC_HandleTypeDef RtcHandle;
//==UART_HandleTypeDef huart1;
//==SPI_HandleTypeDef SpiHandle;

///* Buffer used for displaying Time */
uint8_t aShowTime[50] = {0};

typedef enum 
{
  SHIELD_NOT_DETECTED = 0, 
  SHIELD_DETECTED
}ShieldStatus;

/* Private define ------------------------------------------------------------*/
/* ==========Для реалізації printf================== 
https://www.keil.com/support/man/docs/jlink/jlink_trace_itm_viewer.asp 
Це є в retarget.c:
#define ITM_Port8(n) (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n) (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA 0x01000000
=================================================*/


#define SD_CARD_NOT_FORMATTED                    0
#define SD_CARD_FILE_NOT_SUPPORTED               1
#define SD_CARD_OPEN_FAIL                        2
#define FATFS_NOT_MOUNTED                        3
#define BSP_SD_INIT_FAILED                       4

#define POSITION_X_BITMAP                        0
#define POSITION_Y_BITMAP                        0

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t BlinkSpeed = 0, str[20];
__IO uint8_t JoystickValue = 0;
char* pDirectoryFiles[MAX_BMP_FILES];
FATFS SD_FatFs;  /* File system object for SD card logical drive */
char SD_Path[4]; /* SD card logical drive path */

char realdata[2];
char realmonth[2];
char realyear[2];
char realhours[2];
char reatminutes[2];
char reatseconds[2];
char realdatatime[20];

///* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

//==static void MX_UART1_Init(void);
static void MX_SPI_Init(void);

static void RTC_AlarmConfig(void);
static void RTC_SECConfig(void);

//===static void RTC_TimeShow(uint16_t x, uint16_t y, uint8_t* showtime);

static void LED2_Blink(void);
static ShieldStatus TFT_ShieldDetect(void);
static void SDCard_Config(void);
static void TFT_DisplayErrorMessage(uint8_t message);
static void TFT_DisplayMenu(void);
static void TFT_DisplayImages(void);

/*
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000 
*/

/* ==========Для реалізації printf================== 
https://www.keil.com/support/man/docs/jlink/jlink_trace_itm_viewer.asp */

#define ECHO_FGETC
//volatile int ITM_RxBuffer=0x5AA55AA5; /* Buffer to transmit data towards debug system. */

#define GPIOAEN (*((volatile unsigned long *)(0x40021034)))	// GPIOA clock enable
#define MODER5 	(*((volatile unsigned long *)(0x50000000)))	// GPIOA OUTPUT
#define ODR5 		(*((volatile unsigned long *)(0x50000014)))		// Output Data Register

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {

  return (ITM_SendChar((uint32_t)ch)); // Це виводить в Debug printf, с ST Link|V2 працює
  //ITM_SendChar((uint32_t)ch);
	//return ch;
	
/*	if (DEMCR & TRCENA) {
    while (ITM_Port32(0) == 0);
    ITM_Port8(0) = ch;
  }
  return(ch); */
	
	/*	==============Якщо треба вивести в UART Працює =============	
	//ITM_SendChar(ch); Якщо розремити, то не працює
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
	========================================================= */
	
	//HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF); //Це виводить в UART PA9-Tx, PA10-Rx. Через перехідник UART-USB треба подати на Віртуальний компорт РС.
	//return ch;
	Test_Colors();
} 

unsigned char backspace_called;
unsigned char last_char_read;
int r;

int fgetc(FILE *f)
{
    /* if we just backspaced, then return the backspaced character */
    /* otherwise output the next character in the stream */
    if (backspace_called == 1)
    {
      backspace_called = 0;
    }
    else {
        do {
            r = ITM_ReceiveChar();
        } while (r == -1);
        
        last_char_read = (unsigned char)r;

#ifdef ECHO_FGETC
        ITM_SendChar(r);
#endif
    }

    return last_char_read;
}

/*
** The effect of __backspace() should be to return the last character
** read from the stream, such that a subsequent fgetc() will
** return the same character again.
*/

int __backspace(FILE *f)
{
    backspace_called = 1;
    return 0;
}

//UART_HandleTypeDef UartHandle;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
                                                                                                                                                                                                                                                                                           {  
  /* STM32F103xB HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  //==HAL_Init();  
  
  /* Configure the system clock = 64 MHz */
  //==SystemClock_Config();
	
	
	/* -------------RTC Start--------------*/
/*
Для використання переривання RTC_IRQHandler треба в stm32f1xx_hal_msp.c: функції HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc) встановити:
	  HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn); 
	Для використання переривання RTC_Alarm_IRQHandler треба в stm32f1xx_hal_msp.c:	
		HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0x0F, 0);
		HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);	
*/
	
  /*##-1- Configure the RTC peripheral #######################################*/
 //==== RtcHandle.Instance = RTC;
  
	  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follows:
      - Asynch Prediv  = Automatic calculation of prediv for 1 sec timebase
  */
  //=====RtcHandle.Init.AsynchPrediv = RTC_AUTO_1_SECOND;

/*====	if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    Error_Handler();
  } */

  /*##-2- Configure Alarm ####################################################*/
  /* Configure RTC Alarm */
  //RTC_AlarmConfig(); //Для перерівання через інтервал часу
//=====	RTC_SECConfig(); //Конфігурую для перивання кожну секуду по RTC_IRQHandler


/* -------------RTC End--------------*/


/* -------------TFT LCD--------------*/	
  /* Check the availability of adafruit 1.8" TFT shield on top of STM32NUCLEO
     board. This is done by reading the state of IO PB.00 pin (mapped to JoyStick
     available on adafruit 1.8" TFT shield). If the state of PB.00 is high then
     the adafruit 1.8" TFT shield is available. */  
//  if(TFT_ShieldDetect() == SHIELD_DETECTED)
//  {
  
  /* LCD SPI Config */
  //SPIx_Init();

	//MX_SPI_Init(); //Не роблю, це робиться в stm32f1xx_nucleo.c




/* LCD chip select high */
  //LCD_CS_HIGH(); //Використовую RESET, як CS


/* Initialize the LCD */
	//======BSP_LCD_Init(); //Спочатку через PB11 RESET, потім керується через Регістри

	//delay_init(72);	     //ғʱԵʼۯ
	SystemInit();																																																																																																																																													 SystemInit();//ԵʼۯRCC ʨ׃ϵͳ׷ƵΪ72MHZ
	delay_init(72);	     
	LCD_Init();	  
	

	//==ST7789_WriteString(10, 20, "Real Time", Font_16x26, RED, WHITE);	
	//==ST7789_WriteString(10, 50, "Timer", Font_16x26, RED, WHITE);
  
    /* Configure SD card */
    //SDCard_Config(); 
	while (1)
	{	
		main_test(); 		
		menu_test();     
		Test_Color();  		
		Test_FillRec();		
		Test_Circle(); 		
		Test_Triangle();   
		English_Font_test();
		Chinese_Font_test();
		Pic_test();			
		Rotate_Test();   
  /* Infinite loop */
	//ST7789_Test();
	//RTC_TimeShow(10, 100, aShowTime);
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
/*==void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  //==/** Initializes the RCC Oscillators according to the specified parameters
  //* in the RCC_OscInitTypeDef structure.
  //
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  //==/** Initializes the CPU, AHB and APB buses clocks
  //
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
} */

/*==static void MX_UART1_Init(void)
{
	huart1.Instance        = USARTx;

  huart1.Init.BaudRate   = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits   = UART_STOPBITS_1;
  huart1.Init.Parity     = UART_PARITY_ODD;
  huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart1.Init.Mode       = UART_MODE_TX_RX;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    //==/* Initialization Error 
    Error_Handler();
  }
} */

//Це робить static void SPIx_Init(void)
/*====static void MX_SPI_Init(void)
{
  //==##-1- Configure the SPI peripheral #######################################
 //==  Set the SPI parameters 
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;

//#ifdef MASTER_BOARD
  SpiHandle.Init.Mode = SPI_MODE_MASTER;
//#else
//  SpiHandle.Init.Mode = SPI_MODE_SLAVE;
//#endif /* MASTER_BOARD 

//NSS (PB12) відноситься до SPI. Керується ресурсом SPI
//Ним не можна керувати програмно


} */

/*==void LCD_RESET_SET(void)
{
		LCD_RST_LOW();  //ST7789_RST_Clr(); //Керується через PB11. В платі не використовується
    HAL_Delay(20);
   
		LCD_RST_HIGH();  //ST7789_RST_Set();
    //HAL_Delay(20);
	  delay_ms(20);
} */



/**
  * @brief  Displays demonstration menu.
  * @param  None
  * @retval None
  */

/**
  * @brief  Displays on TFT Images or error messages when error occurred.
  * @param  None
  * @retval None
  */


/**
  * @brief  Alarm callback
  * @param  hrtc : RTC handle
  * @retval None
  */
/*===void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  // Turn LED_GREEN on: Alarm generation 
  BSP_LED_On(LED_GREEN);
} */

/**
  * @brief  Configure the current time and date.
  * @param  None
  * @retval None
  */
/*==static void RTC_AlarmConfig(void)
{
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;
  RTC_AlarmTypeDef salarmstructure;
 
  // ##-1- Configure the Date #################################################
  // Set Date: Tuesday February 18th 2014 
  sdatestructure.Year = 0x24; //0x14;
  sdatestructure.Month = RTC_MONTH_SEPTEMBER;
  sdatestructure.Date = 0x30;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    // Initialization Error 
    Error_Handler(); 
  } 
  
 // ##-2- Configure the Time #################################################
 //  Set Time: 02:20:00 
  stimestructure.Hours = 0x16;
  stimestructure.Minutes = 0x20;
  stimestructure.Seconds = 0x00;
  
  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
     Initialization Error 
    Error_Handler(); 
  }  

  //##-3- Configure the RTC Alarm peripheral #################################
 // Set Alarm to 02:20:30 
     RTC Alarm Generation: Alarm on Hours, Minutes and Seconds 
  salarmstructure.Alarm = RTC_ALARM_A; //0U
	salarmstructure.AlarmTime.Hours = 0x16;
  salarmstructure.AlarmTime.Minutes = 0x20;
  salarmstructure.AlarmTime.Seconds = 0x01; //0x30; //Встановлюю 1 сек, щоб змінювалось покази кожну секунду
	
  if(HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure,RTC_FORMAT_BCD) != HAL_OK)
  {
   // Initialization Error
    Error_Handler(); 
  }
} */

/*===static void RTC_SECConfig(void)
{
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;
  RTC_AlarmTypeDef salarmstructure;
 
 //##-1- Configure the Date #################################################
  // Set Date: Tuesday February 18th 2014 
  sdatestructure.Year = 0x24; //0x14;
  sdatestructure.Month = RTC_MONTH_SEPTEMBER;
  sdatestructure.Date = 0x30;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    // Initialization Error //
    Error_Handler(); 
  } 
  
  //##-2- Configure the Time #################################################
  // Set Time: 02:20:00 
  stimestructure.Hours = 0x16;
  stimestructure.Minutes = 0x20;
  stimestructure.Seconds = 0x00;
  
  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    // Initialization Error 
    Error_Handler(); 
  }  

  //##-3- Configure the RTC Alarm peripheral #################################
  // Set Alarm to 02:20:30 
  //   RTC Alarm Generation: Alarm on Hours, Minutes and Seconds 
  salarmstructure.Alarm = RTC_ALARM_A; //0U
	salarmstructure.AlarmTime.Hours = 0x16;
  salarmstructure.AlarmTime.Minutes = 0x20;
  salarmstructure.AlarmTime.Seconds = 0x01; //0x30; //Встановлюю 1 сек, щоб змінювалось покази кожну секунду
  
 	//HAL_RTCEx_SetSecond_IT(RTC_HandleTypeDef *hrtc);
	if(HAL_RTCEx_SetSecond_IT(&RtcHandle) != HAL_OK)
  {
    // Initialization Error 
    Error_Handler(); 
  }
} */


/**
  * @brief  Display the current time.
  * @param  showtime : pointer to buffer
  * @retval None
  */
/*==static void RTC_TimeShow(uint16_t x, uint16_t y, uint8_t* showtime)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
 
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
  
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
  
  printf("%02d.%02d.20%02d %02d:%02d:%02d\n\r",sdatestructureget.Date, sdatestructureget.Month, sdatestructureget.Year, stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
	
	sprintf(realdata, "%02d\n", sdatestructureget.Date);
	sprintf(realmonth, "%02d", sdatestructureget.Month);
	sprintf(realyear, "%02d", sdatestructureget.Year);
	sprintf(realhours, "%02d", stimestructureget.Hours);
	sprintf(reatminutes, "%02d", stimestructureget.Minutes);
	sprintf(reatseconds, "%02d", stimestructureget.Seconds);
	
	char *temp1 = concat_data(realdata, realmonth, realyear);
	//printf(temp);
		
	ST7789_WriteString(x, y, temp1, Font_16x26, RED, WHITE);	 //& "." & realmonth
	free(temp1);

	char *temp2 = concat_time(realhours, reatminutes, reatseconds);
	//printf(temp);
		
	ST7789_WriteString(x, y + 40, temp2, Font_16x26, RED, WHITE);	 //& "." & realmonth
	free(temp2);


	//printf((char*)showtime,"%02d:%02d:%02d",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
	HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
} */

	
		
//==============================================================================

void Error_Handler(void)
{
  while (1)
  {
    /* Toggle LED2 with a period of one second */
    //BSP_LED_Toggle(LED2);
    //HAL_Delay(1000);
		delay_ms(1000);
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
