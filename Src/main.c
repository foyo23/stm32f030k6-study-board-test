/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdarg.h"
#include "string.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char TxBuff[512];
void USART1_printf( char *fmt, ... )
{
	unsigned int i = 0;
  int tempvalue;
  va_list arg_ptr;
  va_start( arg_ptr, fmt );
  vsnprintf( TxBuff, 512, fmt, arg_ptr );
  tempvalue = strlen( TxBuff );
  while( i< tempvalue ) {
    LL_USART_TransmitData8( USART1, TxBuff[i++] );
    while( !( USART1->ISR & LL_USART_ISR_TXE ) );
  }
  va_end( arg_ptr );
}

void delay_ms( short time ) 
{
	short i = 0;
	while( time-- )
	{
		i = 5000;
		while( i-- );
	}
}

#define BEEP_PIN LL_GPIO_PIN_6  //PB6
#define BEEP_ON() LL_GPIO_SetOutputPin( GPIOB, BEEP_PIN )
#define BEEP_OFF() LL_GPIO_ResetOutputPin( GPIOB, BEEP_PIN )
void BeepInit()
{
  LL_GPIO_InitTypeDef gpio_initStructure;
  LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOB );
  gpio_initStructure.Pin = BEEP_PIN;
  gpio_initStructure.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_initStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_initStructure.Pull = LL_GPIO_PULL_NO;
  gpio_initStructure.Speed = LL_GPIO_SPEED_HIGH;
  LL_GPIO_Init( GPIOB, &gpio_initStructure );
}

#define LED1_PIN LL_GPIO_PIN_6  //PA6
#define LED2_PIN LL_GPIO_PIN_4  //PA4
#define LED3_PIN LL_GPIO_PIN_7  //PB7
void LEDInit()
{
  LL_GPIO_InitTypeDef gpio_initStructure;
  LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOA );
  LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOB );
  gpio_initStructure.Pin = LED1_PIN | LED2_PIN;
  gpio_initStructure.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_initStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_initStructure.Pull = LL_GPIO_PULL_NO;
  gpio_initStructure.Speed = LL_GPIO_SPEED_HIGH;
  LL_GPIO_Init( GPIOA, &gpio_initStructure );
	
  gpio_initStructure.Pin = LED3_PIN;
  gpio_initStructure.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_initStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_initStructure.Pull = LL_GPIO_PULL_NO;
  gpio_initStructure.Speed = LL_GPIO_SPEED_HIGH;
  LL_GPIO_Init( GPIOB, &gpio_initStructure );
}

#define KEY1_PIN LL_GPIO_PIN_3  //PB3
#define KEY2_PIN LL_GPIO_PIN_1		//PA1
void KEYInit()
{
	LL_GPIO_InitTypeDef gpio_initstructure;
	LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB );
	gpio_initstructure.Pin = KEY1_PIN;
	gpio_initstructure.Mode = LL_GPIO_MODE_INPUT;
	gpio_initstructure.Pull = LL_GPIO_PULL_UP;
	gpio_initstructure.Speed = LL_GPIO_SPEED_HIGH;
	LL_GPIO_Init( GPIOB, &gpio_initstructure );
	
	gpio_initstructure.Pin = KEY2_PIN;
	gpio_initstructure.Mode = LL_GPIO_MODE_INPUT;
	gpio_initstructure.Pull = LL_GPIO_PULL_UP;
	gpio_initstructure.Speed = LL_GPIO_SPEED_HIGH;
	LL_GPIO_Init( GPIOA, &gpio_initstructure );
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//printf("started\r\n");

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_ADC_Init();
  MX_USART1_UART_Init();
  //MX_TIM3_Init();
  //MX_TIM14_Init();
  //MX_TIM16_Init();
  //MX_I2C1_SMBUS_Init();
  //MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	BeepInit();
	LEDInit();
	KEYInit();

	delay_ms( 500 );
	USART1_printf( "started \r\n" );
	delay_ms( 500 );
	BEEP_ON();
	delay_ms( 500 );
	BEEP_OFF();
  delay_ms( 500 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//USART1_printf( "USART1_printf ... \r\n" );
		/*
		LL_GPIO_SetOutputPin( GPIOA, LED1_PIN );
		delay_ms( 500 );
		LL_GPIO_SetOutputPin( GPIOA, LED2_PIN );
		delay_ms( 500 );
		LL_GPIO_SetOutputPin( GPIOB, LED3_PIN );
		delay_ms( 1000 );
		LL_GPIO_ResetOutputPin( GPIOB, LED3_PIN );
		delay_ms( 500 );
		LL_GPIO_ResetOutputPin( GPIOA, LED2_PIN );
		delay_ms( 500 );
		LL_GPIO_ResetOutputPin( GPIOA, LED1_PIN );
		delay_ms( 1500 );
		*/
		
		if( 1 == LL_GPIO_IsInputPinSet( GPIOB, KEY1_PIN ) ) {  // if key down
			delay_ms(5);
			//USART1_printf( "key1 down\r\n" );
			while( 1 == LL_GPIO_IsInputPinSet( GPIOB, KEY1_PIN ) ) {   // wait for key up
				;				
			}
			delay_ms(5);
			//USART1_printf( "key1 up\r\n" );
			//delay_ms(5);
			USART1_printf( "key1 pressed\r\n" );
		}
		if( 1 == LL_GPIO_IsInputPinSet( GPIOA, KEY2_PIN ) ) {
			delay_ms(5);
			while( 1 == LL_GPIO_IsInputPinSet( GPIOB, KEY2_PIN ) ) {
				;
			}
			delay_ms(5);
			USART1_printf( "key2 pressed" );
		}
		USART1_printf( "...\r\n" );
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(48000000);
  LL_SetSystemCoreClock(48000000);
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
