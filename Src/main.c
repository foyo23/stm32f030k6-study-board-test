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
		i = 48000;
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

// only setup key1
void EXTI_KEYInit() {
	LL_GPIO_InitTypeDef gpio_init_structure;
	LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOB );
	gpio_init_structure.Pin = KEY1_PIN;
	gpio_init_structure.Mode = LL_GPIO_MODE_INPUT;
	gpio_init_structure.Pull = LL_GPIO_PULL_UP;
	gpio_init_structure.Speed = LL_GPIO_SPEED_HIGH;
	LL_GPIO_Init( GPIOB, &gpio_init_structure );
	
	LL_APB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOB ); //open IO reusable clock
	
	LL_EXTI_InitTypeDef exti_init_structure;
	LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_3 );
	exti_init_structure.Line_0_31 = LL_EXTI_LINE_3;
	exti_init_structure.Mode = LL_EXTI_MODE_IT;
	exti_init_structure.Trigger = LL_EXTI_TRIGGER_FALLING;
	exti_init_structure.LineCommand = ENABLE;
	LL_EXTI_Init( &exti_init_structure );
	
	//LL_SYSCFG_SetEXTISource( LL_SYSCFG_EXTI_PORTB, LL_EXTI_LINE_3 );
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE3);
  NVIC_SetPriority(EXTI2_3_IRQn, 0);
  NVIC_EnableIRQ(EXTI2_3_IRQn);
}	

void EXTI0_1_IRQHandler_callback(void)
{
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
		USART1_printf( "key1 IRQHandler!\r\n" );
  }
}

void EXTI2_3_IRQHandler_callback(void)
{
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
		USART1_printf( "key2 IRQHandler!\r\n" );
  }
}

// number to show on Numeric Display
// segment bit data for same anode Numeric Display
uint8_t const Data[16] = {0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90,0x88,0x83,0xc6,0xa1,0x86,0x8e};
// PB4 is the CLK pin of the HC164D chip
#define CLK_Set() LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4) //set PB4 high level
#define CLK_ReSet() LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4) //set PB4 low level
// PA2 is the data input pin of the HC164D chip
#define DAT_Set() LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2) //set PA2 high level
#define DAT_ReSet() LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2) //reset PA2 low level
// A15: first Digital; A11: second; A12: third; A8: forth
#define SMG_1_ON() LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15)    
#define SMG_2_ON() LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11)  
#define SMG_3_ON() LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_12)  
#define SMG_4_ON() LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8) 

#define SMG_1_OFF() LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15)
#define SMG_2_OFF() LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11)
#define SMG_3_OFF() LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12)
#define SMG_4_OFF() LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8)

void SMG_PIN_Init() {
	LL_GPIO_InitTypeDef gpio_init_instructure;
	LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOB | LL_AHB1_GRP1_PERIPH_GPIOB );
	gpio_init_instructure.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_15 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_8;
	gpio_init_instructure.Mode = LL_GPIO_MODE_OUTPUT;
	gpio_init_instructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_init_instructure.Pull = LL_GPIO_PULL_NO;
	gpio_init_instructure.Speed = LL_GPIO_SPEED_HIGH;
	LL_GPIO_Init( GPIOA, &gpio_init_instructure );
	
	gpio_init_instructure.Pin = LL_GPIO_PIN_4;
	gpio_init_instructure.Mode = LL_GPIO_MODE_OUTPUT;
	gpio_init_instructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_init_instructure.Pull = LL_GPIO_PULL_NO;
	gpio_init_instructure.Speed = LL_GPIO_SPEED_HIGH;
	LL_GPIO_Init( GPIOB, &gpio_init_instructure );
	
	CLK_ReSet();
}

void HC164D_WriteData( uint8_t data ) {
	uint8_t i;
	for( i=0; i<8; i++ ) {
		CLK_ReSet();
		if( data & 0x80 ) {
			DAT_Set();
		} else {
			DAT_ReSet();
		}
		CLK_Set();
		data <<= 1;
	}
	CLK_ReSet();
}

uint8_t number[4];
uint8_t	count=0, SMGtimes=0, SMGBit=0;
uint16_t tim = 0;

void TIM17_IRQHandler_callback() {
	//USART1_printf( "tim17 ... \r\n" );
}

void TIM3_IRQHandler_callback() {
	count++;						//
	if(count >= 5)		  //
	{
		count = 0;				//
		//USART1_printf( "tim3 ... \r\n" );
		//SMG_1_ON();
		//HC164D_WriteData( Data[0] );
		
			SMGtimes++;
      switch(SMGtimes)
      {
      case 1:					//
        SMG_1_ON();
        SMG_2_OFF();
        SMG_3_OFF();
        SMG_4_OFF();
        SMGBit = 0;
        break;
      case 2:					//
        SMG_1_OFF();
        SMG_2_ON();
        SMG_3_OFF();
        SMG_4_OFF();
        SMGBit = 1;
        break;
      case 3:					//
        SMG_1_OFF();
        SMG_2_OFF(); 
        SMG_3_ON();
        SMG_4_OFF();
        SMGBit = 2;
        break;	
      case 4:					//
        SMG_1_OFF();
        SMG_2_OFF();
        SMG_3_OFF();
        SMG_4_ON();
        SMGBit = 3;
        SMGtimes = 0;
        break;
      default:
        break;
      }
			//number[0] = 9;
			//number[1] = 8;
			//number[2] = 7;
			//number[3] = 6;
			HC164D_WriteData( Data[ number[SMGBit] ] );
			//HC164D_WriteData( Data[ u ] );
	}
	return;
	
	//if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //
	{
		//TIM_ClearITPendingBit(TIM3 , TIM_IT_Update);  //
		count++;						//
		if(count >= 5)		  //
		{
			count = 0;				//
			//SMGtimes++;
      switch(SMGtimes)
      {
      case 1:					//
        SMG_1_ON();
        SMG_2_OFF();
        SMG_3_OFF();
        SMG_4_OFF();
        SMGBit = 0;
        break;
      case 2:					//
        SMG_1_OFF();
        SMG_2_ON();
        SMG_3_OFF();
        SMG_4_OFF();
        SMGBit = 1;
        break;
      case 3:					//
        SMG_1_OFF();
        SMG_2_OFF(); 
        SMG_3_ON();
        SMG_4_OFF();
        SMGBit = 2;
        break;	
      case 4:					//
        SMG_1_OFF();
        SMG_2_OFF();
        SMG_3_OFF();
        SMG_4_ON();
        SMGBit = 3;
        SMGtimes = 0;
        break;
      default:
        break;
      }
      //HC164D_WriteData( Data[ number[SMGBit] ] );//
			HC164D_WriteData( Data[1] );
		}
	}
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
	LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_TIM3 );
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_SMBUS_Init();
  /* USER CODE BEGIN 2 */
	//BeepInit();
	//LEDInit();
	//KEYInit();
	//EXTI_KEYInit();  // if process configuration in STM32CubeMX, don't need this line 
	SMG_PIN_Init();
	LL_TIM_EnableIT_UPDATE( TIM3 ); // enable TIM3 interrupt
	LL_TIM_EnableCounter( TIM3 ); // start counter
	//LL_TIM_EnableIT_UPDATE( TIM17 ); // enable TIM17 interrupt
	//LL_TIM_EnableCounter( TIM17 ); // start counter
	
	//delay_ms( 500 );
	USART1_printf( "started \r\n" );
	//delay_ms( 500 );
	//BEEP_ON();
	//delay_ms( 50 );
	//BEEP_OFF();
  //delay_ms( 500 );
  
	number[0] = 5;
	number[1] = 2;
	number[2] = 9;
	number[3] = 4;
	
	SMG_1_ON();
	USART1_printf( "on \r\n" );
	HC164D_WriteData( 0xc0 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int n = 0;
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
		/*		
		if( 0 == LL_GPIO_IsInputPinSet( GPIOB, KEY1_PIN ) ) {  // check if key down. if key down, the pin is a reset signal
			delay_ms(5);
			while( 0 == LL_GPIO_IsInputPinSet( GPIOB, KEY1_PIN ) ) {   // wait for key up
				;				
			}
			delay_ms(5);
			USART1_printf( "key1 pressed\r\n" );
		}
		if( 0 == LL_GPIO_IsInputPinSet( GPIOA, KEY2_PIN ) ) {
			delay_ms(5);
			while( 0 == LL_GPIO_IsInputPinSet( GPIOA, KEY2_PIN ) ) {
				;
			}
			delay_ms(5);
			USART1_printf( "key2 pressed" );
		}
		*/
		//USART1_printf( "write data...\r\n" );
		//LL_GPIO_TogglePin( led1_GPIO_Port, led1_Pin );
		//HC164D_WriteData( Data[n] );
		delay_ms( 500 );
		//LL_mDelay( 1000 );
		n++;
		if( n>15 ) n = 0;
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
