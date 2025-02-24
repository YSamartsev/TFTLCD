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
#include "stm32_adafruit_lcd.h"
#include "math.h"
#include "stdlib.h"



extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;
extern const uint16_t saber;



//#include "stm32f1xx_nucleo.h"
//#include "../HARDWARE/LCD/lcd.h"
//#include "../SYSTEM/delay/delay.h"
//#include "../SYSTEM/sys/sys.h"
//#include "../HARDWARE/TOUCH/touch.h"
//#include "GUI.h"
//#include "test.h"

//#include "EventRecorder.h"


/** @addtogroup STM32F1xx_HAL_Demonstrations
  * @{
  */

/** @addtogroup Demo
  * @{
  */ 

///* Private typedef -----------------------------------------------------------*/
///* RTC handler declaration */
RTC_HandleTypeDef RtcHandle;
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;

SPI_HandleTypeDef SpiHandle;

/* Buffer used for displaying Date */
	uint8_t aShowDate[50] = {0};
/* Buffer used for displaying Date */
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

char* pDirectoryFiles[MAX_BMP_FILES];
FATFS SD_FatFs;  /* File system object for SD card logical drive */
char SD_Path[4]; /* SD card logical drive path */

char realdate[2]; //Це масив, а не string, тобто не закінчується \0
char realmonth[2];
char realyear[2];
char realhours[2];
char reatminutes[2];
char reatseconds[2];
char realdatatime[20];

///* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_UART2_Init(void);
static void MX_SPI_Init(void);

static void RTC_AlarmConfig(void);
static void RTC_SECConfig(void);

static void RTC_DateShow(uint16_t x, uint16_t y); //, uint8_t* showdate);
static void RTC_TimeShow(uint16_t x, uint16_t y); //, uint8_t* showtime);

static void LED0_Blink(void);
static ShieldStatus TFT_ShieldDetect(void);
static void SDCard_Config(void);
static void TFT_DisplayErrorMessage(uint8_t message);
static void TFT_DisplayMenu(void);
static void TFT_DisplayImages(void);


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
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
	========================================================= */
	
	//HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF); //Це виводить в UART PA9-Tx, PA10-Rx. Через перехідник UART-USB треба подати на Віртуальний компорт РС.
	//return ch;
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
//		uint8_t in, jn;
	const uint16_t * mydata;
	RTC_TimeTypeDef stimestructureget; //Зробив публічною структурою Година, Хвилина, Секунда, щоб визначати час 24:59
	

/* Buffer used for transmission */
	char *aTxBuffer;
	char aRxBuffer[14]; //12 символів рядка дати і часую далі їх треба перетворити в формат BCD data (двійково-дестковий код)
	uint8_t mycrc;
/* Buffer used for reception */
	commandAT myCommandAT;
	answerAT	myAnswerAT;
	uint8_t curentTimeSecond;	
	ShieldStatus Bluetooth_present;
	
	char myTemp[2];
	
int main(void)
	//Початкова дата встановлюеться в  RTC_AlarmConfig
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
     HAL_Init();  //Тут встановлюється пріорітет і група пріорітетів
  
  /* Configure the system clock = 64 MHz */
  SystemClock_Config();

	BSP_LED_Init(LED_GREEN);			
	
	MX_UART2_Init();		

	myCommandAT.ATstring = "AT";
	myCommandAT.ATversion = "AT+VERSION";																																																																																																							
	myCommandAT.ATname = "AT+NAMEmyHC-06";	
	myCommandAT.ATbaud = "AT+BAUD8";

	myAnswerAT.ATresponse = "OK";	
	myAnswerAT.VESIONresponse = "OKlinvorV1.8";	
	myAnswerAT.NAMEresponse = "OKsetname";
	myAnswerAT.BAUDresponse = "OK115200"; 
	myAnswerAT.BLUETOOTH_shield = "BL present";
		
	Bluetooth_present = SHIELD_DETECTED;	
	
	//char myAT_RES[2][20]; //Перший індекс - кількість рядків, другий = максимальна кількість символів
	
  /*##-2- Start the transmission process #####################################*/  
  /* While the UART in reception process, user can transmit data through 
     "aTxBuffer" buffer */
	
	if (myExchange(myCommandAT.ATstring, myAnswerAT.ATresponse) != HAL_OK)
	{
		Bluetooth_present = SHIELD_NOT_DETECTED;
	}
	
	if (myExchange(myCommandAT.ATversion, myAnswerAT.VESIONresponse) != SUCCESS)
	{
		Bluetooth_present = SHIELD_NOT_DETECTED;
	}

	if (myExchange(myCommandAT.ATname, myAnswerAT.NAMEresponse) != SUCCESS)
	{
		Bluetooth_present = SHIELD_NOT_DETECTED;
	}	

	
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
	RtcHandle.Instance = RTC;
  
	  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follows:
      - Asynch Prediv  = Automatic calculation of prediv for 1 sec timebase
  */
	RtcHandle.Init.AsynchPrediv = RTC_AUTO_1_SECOND;

	if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    Error_Handler();
  } 

/* -------------RTC End--------------*/


/* -------------TFT LCD--------------*/	
  /* Check the availability of adafruit 1.8" TFT shield on top of STM32NUCLEO
     board. This is done by reading the state of IO PB.00 pin (mapped to JoyStick
     available on adafruit 1.8" TFT shield). If the state of PB.00 is high then
     the adafruit 1.8" TFT shield is available. */  
	
  LCD_IO_Init(); //Визначаються піни для RESET, DC, CS
  // LCD SPI Config: SCK, SDA 
 
 	SPIx_MspInit(); //Конфігурація пінів для MOSI, MOSO, SCK SPIx 

	SPIx_Init(); //Конфігурація параметрів SPI

printf("==================Start RTC Watch===================\n\r");

/* Initialize the LCD */
	BSP_LCD_Init(); //Спочатку через PA7 RESET, потім керується через Регістри
	
		
 #ifdef TFT_LCD_7735	 
	while (1)
	{

		//st7735_DisplayOff();
		ST7735_FillScreen(RED);
		   //char *myChar = "A";
		   //ST7735_WriteChar(10, 20, *myChar, Font_16x26, WHITE, RED);
		//ST7735_FillScreen(RED);
		//st7735_DisplayOn();
		ST7735_FillScreen(BLUE);
			//Display on TFT Images existing on SD card 
			//Display on TFT Images existing on SD card 
			//TFT_DisplayImages();
		HAL_Delay(10);
		//st7735_DisplayOn();
		//T7735_FillScreen(ST7735_WHITE);
		//HAL_Delay(500);
} 
#endif 

if (Bluetooth_present == SHIELD_DETECTED)
	{
		if (myExchange(myCommandAT.ATstring, myAnswerAT.ATresponse) != HAL_OK)
		{
			Error_Handler();
		}
		ST7789_WriteString(10, 180, myCommandAT.ATstring, Font_16x26, RED, WHITE);
		ST7789_WriteString(10, 206, myAnswerAT.ATresponse, Font_16x26, RED, WHITE);
		HAL_Delay(500);

		if (myExchange(myCommandAT.ATversion, myAnswerAT.VESIONresponse) != SUCCESS)
		{
			Error_Handler();
		}
		ST7789_DrawFilledRectangle(0, 180, 240, 240, WHITE); //Заповнюю екран білим кольором
	
		ST7789_WriteString(10, 180, myCommandAT.ATversion, Font_16x26, RED, WHITE); 
		ST7789_WriteString(10, 206, myAnswerAT.VESIONresponse, Font_16x26, RED, WHITE);
		HAL_Delay(500);

		if (myExchange(myCommandAT.ATname, myAnswerAT.NAMEresponse) != SUCCESS)
		{
			Error_Handler();
		}	
		ST7789_DrawFilledRectangle(0, 180, 240, 240, WHITE); 
		ST7789_WriteString(10, 180, myCommandAT.ATname, Font_16x26, RED, WHITE);
		ST7789_WriteString(10, 206, myAnswerAT.NAMEresponse, Font_16x26, RED, WHITE);
		HAL_Delay(500);
		ST7789_DrawFilledRectangle(0, 180, 240, 240, WHITE);
		myAnswerAT.BLUETOOTH_shield = "BL present";
		ST7789_WriteString(10, 206, myAnswerAT.BLUETOOTH_shield, Font_16x26, RED, WHITE);
	}else
	{
		myAnswerAT.BLUETOOTH_shield = "BL not present";
		ST7789_WriteString(10, 206, myAnswerAT.BLUETOOTH_shield, Font_16x26, RED, WHITE);
	}
 
	*aRxBuffer = 0x00;
	*(aRxBuffer+1) = 0x00;
	
    /* Configure SD card */
    //SDCard_Config(); 
		printf("===========AAAAAAAAAAAAA==============\n\r");

		ST7789_WriteString(10, 20, "Real Date:", Font_16x26, RED, WHITE);	
		ST7789_WriteString(10, 100, "Real Time:", Font_16x26, RED, WHITE);
	
	  /* Configure RTC Alarm */
		RTC_AlarmConfig(); //Для переривання через інтервал часу 
		RTC_DateShow(10, 50); //Показати дату, Дата читаэться з 	RtcHandle.DateToUpdate
		RTC_TimeShow(10, 130);  //Показати час
		curentTimeSecond = stimestructureget.Seconds;
		RTC_SECConfig(); //Конфігурую для переривання кожну секуду по RTC_IRQHandler

	while (1)
	{	
		 if ((stimestructureget.Hours == 0x17) && (stimestructureget.Minutes == 0x3B) &&  (stimestructureget.Seconds == 0x3B))
		 {
			 //Для цього зробив структуру stimestructureget публычною
			  HAL_Delay(1200);
				RTC_DateShow(10, 50); //показати дату після 24:00;
	   } 

/*HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
https://controllerstech.com/stm32-uart-5-receive-data-using-idle-line/
Ця функція використовується, коли у потоці байт може бути переривання(простой). Тоді виникае IDLE і можна перевірити
кількість прийнятих байт і або продовжити приймання або зупинити прийом.
Перриваняя виникае коли досягнуто Size, або виникла IDLE. Це можна перевірити в HAL_UARTEx_RxEventCallback
*/
			//switch (HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, sizeof(aRxBuffer))) //Приймаю 12 символів: число.місяць.рік.годин.хвилин.секунд 070125122800
			//switch (HAL_UARTEx_ReceiveToIdle(&UartHandle, (uint8_t *)aRxBuffer, sizeof(aRxBuffer), (uint16_t*) &UartHandle.RxXferSize, 9000))

/*			if (HAL_UARTEx_ReceiveToIdle_IT(&UartHandle, (uint8_t *)aRxBuffer, sizeof(aRxBuffer)) == HAL_OK)
			{
				//printf("Code = %s", aRxBuffer[0]);
				if (UartReady == SET)
				{
					RTC_SECUpdate(); //Оновлення RtcHandle новими даними Дати Часу з aRxBuffer[12]
					RTC_DateShow(10, 50); //Показати дату з stimestructureget
					RTC_TimeShow(10, 130); //Показати час з stimestructureget
					UartReady = RESET;
				}
			} */
			
			//switch (HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, sizeof(aRxBuffer))) //Приймаю 12 символів: число.місяць.рік.годин.хвилин.секунд 070125122800
			switch (HAL_UARTEx_ReceiveToIdle(&UartHandle, (uint8_t *)aRxBuffer, sizeof(aRxBuffer), (uint16_t*) &UartHandle.RxXferSize , 2000)) //Приймаю 12 символів: число.місяць.рік.годин.хвилин.секунд 070125122800
				{ //При цій функції буде виникати sizeof(aRxBuffer) раз переривання. Відбувається помилка
				case HAL_OK:
				mycrc = calcModulo256(aRxBuffer, UartHandle.RxXferSize - 2); //mycrc це byte. aRxBuffer[12] і aRxBuffer[13] це ASCII коди
				//Перетворення символа в байт	
				sprintf(&myTemp[0], "%x", (mycrc & 0xf0)>>4);					
				sprintf(&myTemp[1], "%x",  mycrc & 0x0f);
					//if (UartReady == SET)
					//{
					//Ця функція заповнює регістри UART і переводить його в режим переривання. Без очікування Timeout
					//ST7789_WriteString(10, 180, aRxBuffer, Font_16x26, RED, WHITE);
					//printf("Code = %s", aRxBuffer[0]);
					if(Buffercmp((uint8_t *) &aRxBuffer[12], (uint8_t*) &myTemp, 2) == 0) 
					//if(mycrc == myTemp)
					{
						RTC_SECUpdate();
						RTC_DateShow(10, 50); //, aShowDate);
						RTC_TimeShow(10, 130); //Показати час з stimestructureget
						//UartReady = RESET;
					}
						break;
					//} 
					break;
				case HAL_ERROR:
						
					break;
				case HAL_BUSY:
					/*if (UartReady == SET)
					{
						// Reset transmission flag 
						RTC_SECUpdate();
						RTC_DateShow(10, 50); //, aShowDate);
						RTC_TimeShow(10, 130); //Показати час з stimestructureget
						UartReady = RESET;
						break;
					} */
					break;
				case HAL_TIMEOUT:
					//UartReady = RESET; //Ця подія в цій функції ніколи не настає
					break;
			} 
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
} 

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  return;  
	Error_Handler();
}


static void MX_UART2_Init(void)
{
	UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    //== Initialization Error 
    Error_Handler();
  }
} 

void LCD_RESET_SET(void)
{
		LCD_RST_LOW();  //ST7789_RST_Clr(); //Керується через PB11. В платі не використовується
    HAL_Delay(20);
   
		LCD_RST_HIGH();  //ST7789_RST_Set();
    HAL_Delay(10);
	  //delay_ms(20);
}



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
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  // Turn LED_GREEN on: Alarm generation 
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
 
  // ##-1- Configure the Date #################################################
  // Set Date: 25.01.20.02 2х10+5
  sdatestructure.Year = 0x25; //2х10+5
  sdatestructure.Month = 0x01; //RTC_MONTH_JANUARY; //01 = 0x10+1
  sdatestructure.Date = 0x07; //0x10+7
  sdatestructure.WeekDay = 0x02; //RTC_WEEKDAY_TUESDAY; 02=0x10+2
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    // Initialization Error 
    Error_Handler(); 
  } 
  
 // ##-2- Configure the Time #################################################
 //  Set Time: 14:45:01 
  stimestructure.Hours = 0x14; //1x10+4 14 годин
  stimestructure.Minutes = 0x45; //4x10+5 45 хвилин
  stimestructure.Seconds = 0x01; //0x10+1 1 секунда
  
  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
     //Initialization Error 
    Error_Handler(); 
  }  

  //##-3- Configure the RTC Alarm peripheral #################################
 // Set Alarm to 14:45:01 
  //RTC Alarm Generation: Alarm on Hours, Minutes and Seconds 
  salarmstructure.Alarm = RTC_ALARM_A; //0U
	salarmstructure.AlarmTime.Hours = 0x14;
  salarmstructure.AlarmTime.Minutes = 0x45;
  salarmstructure.AlarmTime.Seconds = 0x01; //0x30; //Встановлюю 1 сек, щоб змінювалось покази кожну секунду
	
  if(HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure,RTC_FORMAT_BCD) != HAL_OK)
  {
   // Initialization Error
    Error_Handler(); 
  }
}

static void RTC_SECConfig(void)
{
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;
  RTC_AlarmTypeDef salarmstructure;
 
 //##-1- Configure the Date #################################################
  // Set Date: 25.
  sdatestructure.Year = 0x25; //0x14;
  sdatestructure.Month = 0x01; //RTC_MONTH_JANUARY; //01 = 0x10+01
  sdatestructure.Date = 0x07; //0x10+7
  sdatestructure.WeekDay = 0x02; //RTC_WEEKDAY_TUESDAY; 02 = 0x10+02
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    // Initialization Error //
    Error_Handler(); 
  } 
  
  //##-2- Configure the Time #################################################
  // Set Time: 23:59:50 година.хвилина.секунда 
  stimestructure.Hours = 0x13;
  stimestructure.Minutes = 0x29;
  stimestructure.Seconds = 0x50;
  
  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    // Initialization Error 
    Error_Handler(); 
  }  

  //##-3- Configure the RTC Alarm peripheral #################################
  // Set Alarm to 02:20:30 
  //   RTC Alarm Generation: Alarm on Hours, Minutes and Seconds 
	//Alarm спрацьовує Відносно HAL_RTC_SetTime
  salarmstructure.Alarm = RTC_ALARM_A; //0U
	salarmstructure.AlarmTime.Hours = 0x14;
  salarmstructure.AlarmTime.Minutes = 0x50;
  salarmstructure.AlarmTime.Seconds = 0x01; //0x30; //В цей час спрацьовує Alarm
  
 	//HAL_RTCEx_SetSecond_IT(RTC_HandleTypeDef *hrtc);
	if(HAL_RTCEx_SetSecond_IT(&RtcHandle) != HAL_OK)
  {
    // Initialization Error 
    Error_Handler(); 
  }
} 

uint8_t RTC_Data_Update(uint8_t index) //Оновити дату і час
{
	//Нові дата і час знаходяться в массиві uint8_t aRxBuffer[12] = 30 37 30 31 32 35 31 32 32 38 30 31
	//їх треба перетворити в 	BCD (двійково-дестковий) код
	uint8_t mydata[2];
	mydata[0] = (aRxBuffer[index] & 0x0F) << 4;
	mydata[1] = aRxBuffer[index + 1] & 0x0F;

	return (uint8_t) (mydata[0] + mydata[1]);
}

//Оновлення RtcHandle новими даними Дати Часу з aRxBuffer[12]
static void RTC_SECUpdate(void)
{
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;
  RTC_AlarmTypeDef salarmstructure;
 
 //##-1- Configure the Date #################################################
  // Set Date: 25.
  
	sdatestructure.Date = RTC_Data_Update(0); //0x10+7
	sdatestructure.Month = RTC_Data_Update(2); //RTC_MONTH_JANUARY; //01 = 0x10+01
	sdatestructure.Year = RTC_Data_Update(4); //0x25; //0x14;
  //sdatestructure.WeekDay = RTC_Data_Update(6); //RTC_WEEKDAY_TUESDAY; 02 = 0x10+02
  
  if(HAL_RTC_SetDate(&RtcHandle, &sdatestructure,RTC_FORMAT_BCD) != HAL_OK) //Запис Дати з sdatestructure в RtcHandle
  {
    // Initialization Error //
    Error_Handler(); 
  } 
  
  //##-2- Configure the Time #################################################
  // Set Time: 23:59:50 година.хвилина.секунда 
  stimestructure.Hours = RTC_Data_Update(6);
  stimestructure.Minutes = RTC_Data_Update(8);
  stimestructure.Seconds = RTC_Data_Update(10);
  
  if(HAL_RTC_SetTime(&RtcHandle, &stimestructure,RTC_FORMAT_BCD) != HAL_OK) //Запис Часу з sdatestructure в RtcHandle
  {
    // Initialization Error 
    Error_Handler(); 
  }  
}


static void RTC_DateShow(uint16_t x, uint16_t y) //Відображення Дати в точці х,у дисплея
{
  RTC_DateTypeDef sdatestructureget;
  
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
  
  //printf("%02d.%02d.20%02d %02d:%02d:%02d\n\r",sdatestructureget.Date, sdatestructureget.Month, sdatestructureget.Year, stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
	
	//snprintf(realdate, sizeof realdate, "%s", &sdatestructureget.Date);
	
	sprintf(realdate, "%02d", sdatestructureget.Date, 2);
	sprintf(realmonth, "%02d", sdatestructureget.Month);
	sprintf(realyear, "%02d", sdatestructureget.Year);
		
	char temp1[11];

	concat_date(temp1, realdate, realmonth, realyear); //соединить строки -> *temp2
	printf("date = %s\n\r", temp1);
		
	ST7789_WriteString(x, y, temp1, Font_16x26, RED, WHITE);	 //& "." & realmonth
	//free(temp1);

} 

/**
  * @brief  Display the current time.
  * @param  showtime : pointer to buffer
  * @retval None
	Відображення часу в точці х,у дисплея
	*/
static void RTC_TimeShow(uint16_t x, uint16_t y) //х, у -координати початкової точки рядка дисплея 
{
  //stimestructureget; //Структура Година, Хвилина, Секунда
 
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN); //З лічильника CNTH_CNTL RTC формується структура stimestructureget
 
  //printf("%02d.%02d.20%02d %02d:%02d:%02d\n\r",sdatestructureget.Date, sdatestructureget.Month, sdatestructureget.Year, stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
	
	//snprintf(realdate, sizeof realdate, "%s", &sdatestructureget.Date);
	
	sprintf(realhours, "%02d", stimestructureget.Hours);
	sprintf(reatminutes, "%02d", stimestructureget.Minutes);
	sprintf(reatseconds, "%02d", stimestructureget.Seconds);
	
	char temp1[9];
	concat_time(temp1, realhours, reatminutes, reatseconds); //соединить строки -> *temp2
	printf("time = %s\n\r", temp1);
			
	ST7789_WriteString(x, y, temp1, Font_16x26, RED, WHITE);	 //& "." & realmonth
	//free(temp1);

} 

void concat_date(char * myconcat, char *s1, char *s2, char *s3)
{
	char mypoint[1] = ".";
	char myspace[1] = " ";
	char mydate[2] = "20";
	
	memcpy(&myconcat[0], s1, 2); //"30"
	memcpy(&myconcat[2], mypoint, 1); //"30."
	
	memcpy(&myconcat[3], s2, 2); //"30.09"
	memcpy(&myconcat[5], mypoint, 1); //"30.09."
	
	memcpy(&myconcat[6], mydate, 2); //"30.09.20"
	
	memcpy(&myconcat[8], s3, 2); ////"30.09.2024"
	myconcat[10] = 0x00;
}

void concat_time(char * myconcat, char *s1, char *s2, char *s3)
{
	char mypoint[1] = ":";
	char mydate[2] = "20";
	
	memcpy(&myconcat[0], s1, 2); //"16"
	memcpy(&myconcat[2], mypoint, 1); //"16:"
	
	memcpy(&myconcat[3], s2, 2); //"16:20"
	memcpy(&myconcat[5], mypoint, 1); //"16:20:"
	
	memcpy(&myconcat[6], s3, 2); ////"316:20:02"
	myconcat[8] = 0x00;
}	

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
	//RTC_TimeTypeDef stimestructureget; 
	HAL_RTC_GetTime(hrtc, &stimestructureget, RTC_FORMAT_BIN); //Це потрібно, щоб в main було видно stimestructureget
	RTC_TimeShow(10, 130);  //, aShowTime);
	HAL_GPIO_TogglePin(LED0_GPIO_PORT, LED0_PIN);

	
	//Перевірка на підключення bluetooth

	//Перевожу UART в режим готовності і перевіряю командою AT
	//UART_EndRxTransfer(&UartHandle);
/*	if	(myExchange(myCommandAT.ATstring, myAnswerAT.ATresponse) != HAL_OK)
		{
			Bluetooth_present = SHIELD_NOT_DETECTED;
			myAnswerAT.BLUETOOTH_shield = "BL not present";
			ST7789_DrawFilledRectangle(0, 180, 240, 240, WHITE);
			ST7789_WriteString(10, 206, myAnswerAT.BLUETOOTH_shield, Font_16x26, RED, WHITE);
		}else
		{
			Bluetooth_present = SHIELD_DETECTED;
			//HAL_GPIO_TogglePin(LED0_GPIO_PORT, LED0_PIN);
			myAnswerAT.BLUETOOTH_shield = "BL present";
			ST7789_DrawFilledRectangle(0, 180, 240, 240, WHITE);
			ST7789_WriteString(10, 206, myAnswerAT.BLUETOOTH_shield, Font_16x26, RED, WHITE);
		} */
		
}

HAL_StatusTypeDef myExchange(char *myAT, char *myRES) //Обмін командой АТ 
{
	aTxBuffer = myAT;

	char *myint1 = memchr(aTxBuffer, 0x00, 20);
	uint8_t COUNTmycommandAT = (myint1 - aTxBuffer) / sizeof(*aTxBuffer); 
		
	if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aTxBuffer, COUNTmycommandAT)!= HAL_OK)
  {
    Error_Handler();
  } 
  /*##-3- Wait for the end of the transfer ###################################*/   
  while (UartReady != SET)
  {
  }
  UartReady = RESET;

	//Перехожу на прийом відповіді на команду AT

	char *myint2 = memchr(myRES, 0x00, 20);
	uint16_t COUNTmyresponseAT = (myint2 - myRES) / sizeof(*aRxBuffer);  
	
  /*##-4- Put UART peripheral in reception process ###########################*/  
	UartHandle.RxXferSize = 12;
	//HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
  //switch (HAL_UARTEx_ReceiveToIdle(&UartHandle, (uint8_t *)aRxBuffer, COUNTmyresponseAT, (uint16_t*) &UartHandle.RxXferSize , 2000))
	//HAL_UARTEx_ReceiveToIdle_IT(&UartHandle, (uint8_t *)aRxBuffer, COUNTmyresponseAT);

	switch (HAL_UARTEx_ReceiveToIdle(&UartHandle, (uint8_t *)aRxBuffer, COUNTmyresponseAT, (uint16_t*) &UartHandle.RxXferSize , 2000))
	{
				case HAL_OK:
				if(Buffercmp(aRxBuffer, (uint8_t *) myRES, COUNTmyresponseAT))  //перевірка на відповідність AT команди і відповіді
				{
					return HAL_ERROR;
					break;
				}
				return HAL_OK;
				break;
			case HAL_ERROR:
				Error_Handler();
				break;			
			case HAL_BUSY:
				return HAL_BUSY;
				break;
			case HAL_TIMEOUT:
				return HAL_TIMEOUT;
				break;
		}
	
		
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static int Buffercmp(uint8_t * pBuffer1, uint8_t * pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

static char calcModulo256(char *myString, uint16_t BufferLength)
    {
      int crc = 0; //48 49 48 50 48 51 48 52 48 53 48 55
			uint8_t mymod = 0;
			char *mycrc;
      for (int i = 0; i < BufferLength; i++) {
				crc += (uint16_t) *(myString + i);
      } //598 (0x0256)
      mymod = crc - (uint16_t) (crc % 256) * 256; // 598 - 2*256 = 597- 512 = 86 (0x056)
			return mymod;
		}   

//==============================================================================

void Error_Handler(void)
{
  while (1)
  {
    /* Toggle LED2 with a period of one second */
    //BSP_LED_Toggle(LED2);
    HAL_Delay(1000);
		
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
