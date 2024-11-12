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


//#include "EventRecorder.h"


/** @addtogroup STM32F1xx_HAL_Demonstrations
  * @{
  */

/** @addtogroup Demo
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* RTC handler declaration */
RTC_HandleTypeDef RtcHandle;
UART_HandleTypeDef huart1;
SPI_HandleTypeDef SpiHandle;

/* Buffer used for displaying Time */
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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_UART1_Init(void);
static void MX_SPI_Init(void);

static void RTC_AlarmConfig(void);
static void RTC_SECConfig(void);

static void RTC_TimeShow(uint16_t x, uint16_t y, uint8_t* showtime);

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
  HAL_Init();  
  
  /* Configure the system clock = 64 MHz */
  SystemClock_Config();
	
	/* Configure GPIO LED_GREEN */
  BSP_LED_Init(LED_GREEN); ///Ініціалізація  GPIO для LED, *Led is lights up PC13*/

	////////MX_UART1_Init(); //Ініціалізація GPIO для UART1
	
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI); //Ініціалізація  GPIO для кнопки User Button
	
//	LCD_IO_Init(); //Ініціалізація GPIO для LCD
	
	HAL_Delay(500);
	BSP_LED_On(LED_GREEN); /*Led is lights out */
	//HAL_Delay(1000);
	
	/* ===========Якщо треба Event Recorder===========	
	EventRecorderInitialize (EventRecordAll, 1);
	EventRecorderStart ();
	=================================================*/

	printf("================The sample of using RTC for Calendar with Watch==============\r\n");

//Лічильник RTC рахує в секундах

	BSP_LED_Off(LED_GREEN); /*Led is lights in*/
	
	
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
	BSP_LCD_Init(); //Спочатку через PB11 RESET, потім керується через Регістри

	ST7789_WriteString(10, 20, "Real Time", Font_16x26, RED, WHITE);	
	ST7789_WriteString(10, 50, "Timer", Font_16x26, RED, WHITE);
  
    /* Configure SD card */
    //SDCard_Config(); 
	while (1)
	{	
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  /** Initializes the CPU, AHB and APB buses clocks
  */
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
}

static void MX_UART1_Init(void)
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
    /* Initialization Error */
    Error_Handler();
  }
}

//Це робить static void SPIx_Init(void)
static void MX_SPI_Init(void)
{
  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
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
//#endif /* MASTER_BOARD */

//NSS (PB12) відноситься до SPI. Керується ресурсом SPI
//Ним не можна керувати програмно

if(HAL_SPI_Init(&SpiHandle) != HAL_OK) //NSS (PB12) відноситься до SPI
  {
    /* Initialization Error */
    Error_Handler();
  }
}

void LCD_RESET_SET(void)
{
		LCD_RST_LOW();  //ST7789_RST_Clr(); //Керується через PB11. В платі не використовується
    HAL_Delay(20);
   
		LCD_RST_HIGH();  //ST7789_RST_Set();
    HAL_Delay(20);
}



/**
  * @brief  Displays demonstration menu.
  * @param  None
  * @retval None
  */
static void TFT_DisplayMenu(void)
{
  //JOYState_TypeDef tmp = JOY_NONE;
  
  /* Set Menu font */
  BSP_LCD_SetFont(&Font12);

  /* Set Text color */
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  /* Display message */
  BSP_LCD_DisplayStringAtLine(1, (uint8_t*)" NUCLEO-STM32F1xx   ");
  BSP_LCD_DisplayStringAtLine(2, (uint8_t*)"       DEMO         ");
  
  /* Set Text color */
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  /* Display message */  
  BSP_LCD_DisplayStringAtLine(4, (uint8_t*)" Display images      ");
  BSP_LCD_DisplayStringAtLine(6, (uint8_t*)" stored under uSD    ");
  BSP_LCD_DisplayStringAtLine(8, (uint8_t*)" on TFT LCD          ");
  
  /* Set Text color */
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  /* Display message */ 
  BSP_LCD_DisplayStringAtLine(11, (uint8_t*)"  Press JOY DOWN   ");
  BSP_LCD_DisplayStringAtLine(12, (uint8_t*)"  to continue...   ");
 
  /* Wait for JOY_DOWN is pressed */
  /*while (BSP_JOY_GetState() != JOY_DOWN)
  {
  }*/
  
  /* Wait for JOY_DOWN is released */
  /*while (BSP_JOY_GetState() == JOY_DOWN)
  {
  }*/
  
  /* Set Text color */
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);  
  /* Display message */ 
  BSP_LCD_DisplayStringAtLine(4,  (uint8_t*)"                   ");
  BSP_LCD_DisplayStringAtLine(6,  (uint8_t*)"  Press Joystick   ");
  
  /* Set Text color */
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  /* Display message */ 
  BSP_LCD_DisplayStringAtLine(8,  (uint8_t*)"  UP for:          ");
  BSP_LCD_DisplayStringAtLine(9,  (uint8_t*)"  Manual Mode      ");
  BSP_LCD_DisplayStringAtLine(11, (uint8_t*)"  DOWN for:        ");
  BSP_LCD_DisplayStringAtLine(12, (uint8_t*)"  Automatic Mode   ");

  /* Wait for JOY_DOWN or JOY_UP is pressed */
  //tmp = JOY_RIGHT;
  //while ((tmp != JOY_DOWN) && (tmp != JOY_UP))
  //{
  //  tmp = BSP_JOY_GetState();
  //}
  
  /* LCD Clear */
  BSP_LCD_Clear(LCD_COLOR_WHITE); 
  
  /* JOY_UP is pressed: Display Manual mode menu #############################*/
//  if(tmp == JOY_UP)
//  {
    /* Set Text color */
    BSP_LCD_SetTextColor(LCD_COLOR_RED);    
    /* Display message */ 
    BSP_LCD_DisplayStringAtLine(3,  (uint8_t*)"   Manual Mode   ");
    BSP_LCD_DisplayStringAtLine(5,  (uint8_t*)"    Selected     "); 
    
    //Set Text color 
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);      
    // Display message 
    BSP_LCD_DisplayStringAtLine(9,  (uint8_t*)" RIGHT: Next image"); 
    BSP_LCD_DisplayStringAtLine(10, (uint8_t*)" LEFT : Previous  ");
    BSP_LCD_DisplayStringAtLine(11, (uint8_t*)" SEL  : Switch to ");
    BSP_LCD_DisplayStringAtLine(12, (uint8_t*)" automatic mode   ");    
//    JoystickValue = 2; 
/*
  }
  // JOY_DOWN is pressed: Display Automatic mode menu ########################
  else if (tmp == JOY_DOWN)    
  {
    
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
   // Display message  
    BSP_LCD_DisplayStringAtLine(3,  (uint8_t*)"  Automatic Mode  ");
    BSP_LCD_DisplayStringAtLine(5,  (uint8_t*)"     Selected     ");
    

    JoystickValue = 1;  
    HAL_Delay(200);
  } */
} 
/**
  * @brief  Displays on TFT Images or error messages when error occurred.
  * @param  None
  * @retval None
  */
static void TFT_DisplayImages(void)
{    
  uint32_t bmplen = 0x00;
  uint32_t checkstatus = 0x00;
  uint32_t filesnumbers = 0x00;
  uint32_t joystickstatus = JOY_NONE;
  uint32_t bmpcounter = 0x00;
  uint32_t tickstart;
  DIR directory;
  FRESULT res;
  
  /* Initialize the Joystick available on adafruit 1.8" TFT shield */
  //BSP_JOY_Init();
  
  /* Welcome message */
  TFT_DisplayMenu();
	//TFT_DisplayErrorMessage(SD_CARD_NOT_FORMATTED);	
 
}

/**
  * @brief  SD Card Configuration.
  * @param  None
  * @retval None
  */
static void SDCard_Config(void)
{
  uint32_t counter = 0;
  
  if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
  {
    /* Initialize the SD mounted on adafruit 1.8" TFT shield */
    if(BSP_SD_Init() != MSD_OK)
    {
      TFT_DisplayErrorMessage(BSP_SD_INIT_FAILED);
    }  
    
    /* Check the mounted device */
    if(f_mount(&SD_FatFs, (TCHAR const*)"/", 0) != FR_OK)
    {
      TFT_DisplayErrorMessage(FATFS_NOT_MOUNTED);
    }  
    else
    {
      /* Initialize the Directory Files pointers (heap) */
      for (counter = 0; counter < MAX_BMP_FILES; counter++)
      {
        pDirectoryFiles[counter] = malloc(11); 
      }
    }
  }
}

/**
  * @brief  Displays adequate message on TFT available on adafruit 1.8" TFT shield  
  * @param  message: Error message to be displayed on the LCD.
  *   This parameter can be one of following values:   
  *     @arg SD_CARD_NOT_FORMATTED: SD CARD is not FAT formatted
  *     @arg SD_CARD_FILE_NOT_SUPPORTED: File is not supported
  *     @arg SD_CARD_OPEN_FAIL: Failure to open directory
  *     @arg FATFS_NOT_MOUNTED: FatFs is not mounted
  * @retval None
  */
static void TFT_DisplayErrorMessage(uint8_t message)
{
  /* LCD Clear */
  BSP_LCD_Clear(LCD_COLOR_WHITE); 
  /* Set Error Message Font */
  BSP_LCD_SetFont(&Font12);
  /* Set Text and Back colors */
  BSP_LCD_SetBackColor(LCD_COLOR_GREY); 
  BSP_LCD_SetTextColor(LCD_COLOR_RED);

  if(message == SD_CARD_NOT_FORMATTED)
  {
    /* Display message */
    BSP_LCD_DisplayStringAtLine(5, (uint8_t*)" SD Card is not    ");
    BSP_LCD_DisplayStringAtLine(6, (uint8_t*)" FAT formatted.    ");  
    BSP_LCD_DisplayStringAtLine(7, (uint8_t*)" Please Format the ");
    BSP_LCD_DisplayStringAtLine(8, (uint8_t*)" microSD card.     ");
    while (1)
    {
    }    
  }
  if(message == SD_CARD_FILE_NOT_SUPPORTED)
  {
    /* Display message */
    BSP_LCD_DisplayStringAtLine(5, (uint8_t*)"                   ");
    BSP_LCD_DisplayStringAtLine(6, (uint8_t*)" File type is not  ");
    BSP_LCD_DisplayStringAtLine(7, (uint8_t*)" supported.        ");
    BSP_LCD_DisplayStringAtLine(8, (uint8_t*)"                   ");
    while(1)
    {
    }    
  }
  if(message == SD_CARD_OPEN_FAIL)
  {
    /* Display message */
    BSP_LCD_DisplayStringAtLine(5, (uint8_t*)"                   ");
    BSP_LCD_DisplayStringAtLine(6, (uint8_t*)" Open directory    ");
    BSP_LCD_DisplayStringAtLine(7, (uint8_t*)" fails.            ");
    BSP_LCD_DisplayStringAtLine(8, (uint8_t*)"                   ");
    while(1)
    {
    }     
  }
  if(message == FATFS_NOT_MOUNTED)
  {
    /* Display message */
    BSP_LCD_DisplayStringAtLine(5, (uint8_t*)"                   ");
    BSP_LCD_DisplayStringAtLine(6, (uint8_t*)" Cannot mount      ");
    BSP_LCD_DisplayStringAtLine(7, (uint8_t*)" FatFs on Drive.   "); 
    BSP_LCD_DisplayStringAtLine(8, (uint8_t*)"                   ");
    while (1)
    {
    }  
  }
  if(message == BSP_SD_INIT_FAILED)
  {
    /* Display message */
    BSP_LCD_DisplayStringAtLine(5, (uint8_t*)"                   ");
    BSP_LCD_DisplayStringAtLine(6, (uint8_t*)" SD Init           ");
    BSP_LCD_DisplayStringAtLine(7, (uint8_t*)" fails.            ");
    BSP_LCD_DisplayStringAtLine(8, (uint8_t*)"                   ");
    while(1)
    {
    }     
  }
}

/**
  * @brief  Blinks LED2 with two frequencies depending on User press button.
  * @param  None
  * @retval None
  */
static void LED2_Blink(void)
{
  /* Initiate BlinkSpeed variable */ 
  //BlinkSpeed = 0;  
  
  /* Infinite loop */
  /* Test on blink speed */
    if(BlinkSpeed == 0)
    {
      BSP_LED_Toggle(LED2);
      /* Wait for 500ms */      
      HAL_Delay(500);      
    }      
    else if(BlinkSpeed == 1)
    {
      BSP_LED_Toggle(LED2);
      /* Wait for 100ms */
      HAL_Delay(100); 
    }
    else if(BlinkSpeed == 2)
    {
      BSP_LED_Toggle(LED2);    
      /* wait for 50ms */
      HAL_Delay(50);  
    }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(BlinkSpeed == 2)
  {
    BlinkSpeed = 0;
  }
  else
  {
    BlinkSpeed ++;
  }
}

/**
  * @brief  Check the availability of adafruit 1.8" TFT shield on top of STM32NUCLEO
  *         board. This is done by reading the state of IO PB.00 pin (mapped to 
  *         JoyStick available on adafruit 1.8" TFT shield). If the state of PB.00 
  *         is high then the adafruit 1.8" TFT shield is available.
  * @param  None
  * @retval SHIELD_DETECTED: 1.8" TFT shield is available
  *         SHIELD_NOT_DETECTED: 1.8" TFT shield is not available
  */
static ShieldStatus TFT_ShieldDetect(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct; 

  /* Enable GPIO clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) != 0)
  {
    return SHIELD_DETECTED;
  }
  else
  {
    return SHIELD_NOT_DETECTED;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  while (1)
  {
    /* Toggle LED2 with a period of one second */
    BSP_LED_Toggle(LED2);
    HAL_Delay(1000);
  }
}

/**
  * @brief  Alarm callback
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Turn LED_GREEN on: Alarm generation */
  BSP_LED_On(LED_GREEN);
}

/**
  * @brief  Configure the current time and date.
  * @param  None
  * @retval None
  */
static void RTC_AlarmConfig(void)
{
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;
  RTC_AlarmTypeDef salarmstructure;
 
  /*##-1- Configure the Date #################################################*/
  /* Set Date: Tuesday February 18th 2014 */
  sdatestructure.Year = 0x24; //0x14;
  sdatestructure.Month = RTC_MONTH_SEPTEMBER;
  sdatestructure.Date = 0x30;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  } 
  
  /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:20:00 */
  stimestructure.Hours = 0x16;
  stimestructure.Minutes = 0x20;
  stimestructure.Seconds = 0x00;
  
  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }  

  /*##-3- Configure the RTC Alarm peripheral #################################*/
  /* Set Alarm to 02:20:30 
     RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
  salarmstructure.Alarm = RTC_ALARM_A; //0U
	salarmstructure.AlarmTime.Hours = 0x16;
  salarmstructure.AlarmTime.Minutes = 0x20;
  salarmstructure.AlarmTime.Seconds = 0x01; //0x30; //Встановлюю 1 сек, щоб змінювалось покази кожну секунду
	
  if(HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}

static void RTC_SECConfig(void)
{
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;
  RTC_AlarmTypeDef salarmstructure;
 
  /*##-1- Configure the Date #################################################*/
  /* Set Date: Tuesday February 18th 2014 */
  sdatestructure.Year = 0x24; //0x14;
  sdatestructure.Month = RTC_MONTH_SEPTEMBER;
  sdatestructure.Date = 0x30;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  } 
  
  /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:20:00 */
  stimestructure.Hours = 0x16;
  stimestructure.Minutes = 0x20;
  stimestructure.Seconds = 0x00;
  
  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }  

  /*##-3- Configure the RTC Alarm peripheral #################################*/
  /* Set Alarm to 02:20:30 
     RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
  salarmstructure.Alarm = RTC_ALARM_A; //0U
	salarmstructure.AlarmTime.Hours = 0x16;
  salarmstructure.AlarmTime.Minutes = 0x20;
  salarmstructure.AlarmTime.Seconds = 0x01; //0x30; //Встановлюю 1 сек, щоб змінювалось покази кожну секунду
  
 	//HAL_RTCEx_SetSecond_IT(RTC_HandleTypeDef *hrtc);
	if(HAL_RTCEx_SetSecond_IT(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}


/**
  * @brief  Display the current time.
  * @param  showtime : pointer to buffer
  * @retval None
  */
static void RTC_TimeShow(uint16_t x, uint16_t y, uint8_t* showtime)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
	/* https://www.keil.com/support/man/docs/jlink/jlink_trace_itm_viewer.asp */
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
} 

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Prevent unused argument(s) compilation warning */
  RTC_TimeShow(10, 100, aShowTime);
	
}

 char* concat_data(char *s1, char *s2, char *s3) 
	 {
//https://ru.stackoverflow.com/questions/103/%D0%9A%D0%B0%D0%BA-%D0%BF%D1%80%D0%B0%D0%B2%D0%B8%D0%BB%D1%8C%D0%BD%D0%BE-%D1%81%D0%BA%D0%BB%D0%B5%D0%B8%D1%82%D1%8C-%D0%B4%D0%B2%D0%B5-%D1%81%D1%82%D1%80%D0%BE%D0%BA%D0%B8-%D0%B2-%D0%A1%D0%B8
				char *point = ".";
				char *twenty = "20";
		    uint8_t mynull = 0x00;
 
        char *result = malloc(20); //Резевування пам'яті під result

        if (!result) {
            printf("malloc() failed: insufficient memory!\n");
            return NULL;
        }

        memcpy(result, s1, 2); //30
        memcpy(result + 2, point, 1); //30.
				
				memcpy(result + 3, s2, 2);  //30.09
				memcpy(result + 5, point, 1); //30.09.
				
				memcpy(result + 6, twenty, 2); //30.09.20
				memcpy(result + 8, s3, 2);  //30.09.2024
				memcpy(result + 10, &mynull, 1);

        return result;
    }
 char* concat_time(char *s1, char *s2, char *s3) 
	 {
//https://ru.stackoverflow.com/questions/103/%D0%9A%D0%B0%D0%BA-%D0%BF%D1%80%D0%B0%D0%B2%D0%B8%D0%BB%D1%8C%D0%BD%D0%BE-%D1%81%D0%BA%D0%BB%D0%B5%D0%B8%D1%82%D1%8C-%D0%B4%D0%B2%D0%B5-%D1%81%D1%82%D1%80%D0%BE%D0%BA%D0%B8-%D0%B2-%D0%A1%D0%B8
				char *twopoint = ":";
				uint8_t mynull = 0x00;
		 
        char *result = malloc(20); //Резевування пам'яті під result

        if (!result) {
            printf("malloc() failed: insufficient memory!\n");
            return NULL;
        }

        memcpy(result, s1, 2); //14
        memcpy(result + 2, twopoint, 1); //14:
				
				memcpy(result + 3, s2, 2);  //14:12
				memcpy(result + 5, twopoint, 1); //14:12:
				
				memcpy(result + 6, s3, 2);  //14:12:03 \0
				memcpy(result + 8, &mynull, 1);
				
        return result;
    }
		
		
//==============================================================================

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
