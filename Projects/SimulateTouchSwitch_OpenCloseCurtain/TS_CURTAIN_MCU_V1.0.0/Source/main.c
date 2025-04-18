/*********************************************************************
 * Description: Assignment 1
 * Project: To simulate touch switch device of Lumi Smart Home to open/close curtain
 * Author: Tran Doan Manh
 ********************************************************************/

/* All libraries ---------------------------------------------------*/
#include "timer.h"
#include "misc.h"
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "stm32f401re_exti.h"
#include "stm32f401re_syscfg.h"

/* Private macro ---------------------------------------------------*/
//@brief relating to LED
#define LED2_RED						GPIO_Pin_13					//Of GPIOB port
#define LED2_RED_PIN					13
#define LED1_GREEN						GPIO_Pin_0					//Of GPIOA port
#define LED1_GREEN_PIN					0
#define LED2_GREEN						GPIO_Pin_11					//Of GPIOA port
#define LED2_GREEN_PIN					11
#define LED_BOARD						GPIO_Pin_5					//Of GPIOA port
#define LED_BOARD_PIN					5
#define GPIOA_LED_CLOCK					RCC_AHB1Periph_GPIOA
#define GPIOB_LED_CLOCK					RCC_AHB1Periph_GPIOB
#define LED_PORTA						GPIOA
#define LED_PORTB						GPIOB
//@brief relating to BUZZER
#define BUZZER							GPIO_Pin_9					//Of GPIOC port
#define BUZZER_PIN						9
#define GPIOC_BUZZER_CLOCK				RCC_AHB1Periph_GPIOC
#define BUZZER_PORTC					GPIOC
//@brief relating to BUTTON
#define BUTTON2							GPIO_Pin_3					//Of GPIOB port
#define BUTTON2_PIN						3
#define BUTTON3							GPIO_Pin_4					//Of GPIOA port
#define BUTTON3_PIN						4
#define BUTTON4							GPIO_Pin_0					//Of GPIOB port
#define BUTTON4_PIN						0
#define GPIOB_BUTTON_CLOCK				RCC_AHB1Periph_GPIOB
#define BUTTON_PORTB					GPIOB
#define GPIOA_BUTTON_CLOCK				RCC_AHB1Periph_GPIOA
#define BUTTON_PORTA					GPIOA
//@brief relating to state of GPIO
#define GPIO_PIN_SET					1
#define GPIO_PIN_RESET					0
//@brief other macros
#define SYSCFG_CLOCK					RCC_APB2Periph_SYSCFG
#define TIME_HOLD_BUTTON				500
#define PERIOD_TWO_PRESS				400

/* Function prototype ----------------------------------------------*/
//@brief initialization function
static void LedBuzz_Init(void);
static void Button_Init(void);
static void AppInitCommon(void);

//@brief processed function
static void Blinkled_StatusPower(void);
static void LedControl_SetState(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN, uint8_t Status);
static void LedControl_SetStatus(GPIO_TypeDef *GPIOx1, GPIO_TypeDef *GPIOx2, uint8_t GPIO_PIN1, uint8_t GPIO_PIN2);
static void LedControl_TimPress(void);
static void LedControl_TimOutPress(void);
static void BuzzerControl_SetBeep(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN);
static uint8_t ButtonRead_Status(GPIO_TypeDef *GPIOx, uint32_t GPIO_PIN);

//@brief other function
static uint32_t CalculatorTime(uint32_t dwTimeInit, uint32_t dwTimeCurrent);
void Delay(uint32_t miliseconds);

/* Global variable -------------------------------------------------*/
uint8_t countInterruptButton3 = 0;
uint8_t countPressButton = 0;
uint8_t statusCommon = 0;
uint8_t statusButton2 = 0;
uint8_t statusButton4 = 0;
static uint32_t time_current_hold = 0, time_initial_hold = 0;
static uint32_t	time_release = 0, time_after_release = 0, period_after_release = 0;


int main()
{
	AppInitCommon();

	while(1)
	{
		LedControl_SetStatus(LED_PORTA, BUZZER_PORTC, LED2_GREEN_PIN, BUZZER_PIN);
		LedControl_TimPress();
		LedControl_TimOutPress();
	}
}

/*
 * @func AppInitCommon
 * @brief To initialize all peripherals
 * @param None
 * @retval None
 */
static void AppInitCommon(void)
{
	SystemCoreClockUpdate();			//To initialize the clock system as 84MHz
	TimerInit();
	LedBuzz_Init();
	Button_Init();
	Blinkled_StatusPower();
}

/*
 * @func LedBuzz_Init
 * @brief To configure all parameters of LED and BUZZER
 * @param None
 * @retval None
 */
static void LedBuzz_Init(void)
{
	//To create a variable to configure GPIO registers
	GPIO_InitTypeDef 	GPIO_InitStructure;

	//To enable clock pulse to operate for peripheral GPIOx
	RCC_AHB1PeriphClockCmd(GPIOA_LED_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(GPIOB_LED_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(GPIOC_BUZZER_CLOCK, ENABLE);

	//To set up all values of all fields of the structure type "GPIO_InitTypeDef"
	GPIO_InitStructure.GPIO_Pin =  LED1_GREEN | LED2_GREEN | LED_BOARD;						//To select pins of  GPIOA to configure
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;											//To configure the mode "Output" of that pins
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;										//To configure the process speed at that pins
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//To configure the output mode "Open-Drain" at that pins
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										//Do not use PUPD
	//To initialize all values
	GPIO_Init(LED_PORTA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED2_RED;													//To select pin of GPIOB to configure
	GPIO_Init(LED_PORTB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BUZZER;													//To select pin of GPIOC to configure
	GPIO_Init(BUZZER_PORTC, &GPIO_InitStructure);
}

/*
 * @func Button_Init
 * @brief To configure all parameters of BUTTONs to use interrupt
 * @param None
 * @retval None
 */
static void Button_Init(void)
{
	/*----------------- Configuring all GPIOs of BUTTON at input mode -------------------*/

	//To create a variable to configure GPIO registers
	GPIO_InitTypeDef	GPIO_InitStructure;

	//To enable clock pulse at GPIOx port
	RCC_AHB1PeriphClockCmd(GPIOA_BUTTON_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(GPIOB_BUTTON_CLOCK, ENABLE);

	//To set up all values of all fields of the structure type "GPIO_InitTypeDef"
	GPIO_InitStructure.GPIO_Pin = BUTTON2 | BUTTON4;		//To select pins to configure
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;			//To configure the mode "Output" of that pins
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//To use PU
	//To initialize all values
	GPIO_Init(BUTTON_PORTB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BUTTON3;
	GPIO_Init(BUTTON_PORTA, &GPIO_InitStructure);

	/*----------------- Configuring EXTIx to receive interrupt from Pin_x of GPIO -------------------*/

	//To enable clock pulse for SYSCFG registers
	RCC_APB2PeriphClockCmd(SYSCFG_CLOCK, ENABLE);

	//To allow EXTI Line_x to receive interrupt from Pin_x of GPIOx port
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource3);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);

	/*----------------- Configuring EXTI registers -------------------*/

	//To create a variable to configure EXTI registers
	EXTI_InitTypeDef	EXTI_InitStructure;

	//To set up all values of all fields of the structure type "EXTI_InitTypeDef"
	EXTI_InitStructure.EXTI_Line = EXTI_Line4;							//To configure line to receive interrupt
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;					//To select interrupt mode
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;				//To select falling trigger
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;							//To allow that line to receive interrupt
	//To initialize all values
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line3;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStructure);

	/*----------------- Configuring NVIC registers -------------------*/

	//To create a variable to configure NVIC registers
	NVIC_InitTypeDef	NVIC_InitStructure;

	//To set up all values of all fields of the structure type "EXTI_NVIC_InitTypeDef"
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;			//To configure priority level
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//To write that state into Interrupt Set-Enable Registers

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;					//To configure IRQ number
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;					//To configure IRQ number
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;					//To configure IRQ number
	NVIC_Init(&NVIC_InitStructure);
}

/*
 * @func LedControl_SetState
 * @brief To turn on/off LED following state
 * @param
 *        *GPIOx - Address of GPIO port
 *        GPIO_PIN - The pin ordinal number of GPIO port
 *        Status - ON/OFF LED
 * @retval None
 */
static void LedControl_SetState(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN, uint8_t Status)
{
	//Set bit in BSRR Registers
	if(Status == GPIO_PIN_SET)
	{
		GPIOx->BSRRL |= 1 << GPIO_PIN;
	}
	//Reset bit in BSRR Registers
	if(Status == GPIO_PIN_RESET)
	{
		GPIOx->BSRRH |= 1 << GPIO_PIN;
	}
}

/*
 * @func BuzzerControl_SetBeep
 * @brief To blink BUZZER 2 times
 * @param
 * 		  *GPIOx - Address of GPIO port
 * 		  GPIO_PIN - The pin ordinal number of GPIO port
 * @retval None
 */
static void BuzzerControl_SetBeep(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN)
{
	for(uint8_t i = 0; i < 2; i++)
	{
		GPIOx->BSRRL |= 1 << GPIO_PIN;
		Delay(1000);
		GPIOx->BSRRH |= 1 << GPIO_PIN;
		Delay(1000);
	}
}

/*
 * @func ButtonRead_Status
 * @brief To read the button state
 * @param
 * 		  *GPIOx - Address of GPIO port
 * 		  GPIO_PIN - The pin ordinal number of GPIO port
 * @retval 0 or 1
 */
static uint8_t ButtonRead_Status(GPIO_TypeDef *GPIOx, uint32_t GPIO_PIN)
{
	uint32_t Read_Pin;
	Read_Pin = (GPIOx->IDR) >> GPIO_PIN;
	Read_Pin = Read_Pin & 0x01;
	return Read_Pin;
}

/*
 * @func CalculatorTime
 * @brief To calculate time value between two periods
 * @param
 * 		  dwTimeInit - Initialization time
 * 		  dwTimeCurrent - Current time
 * @retval The time value between that two periods
 */
static uint32_t CalculatorTime(uint32_t dwTimeInit, uint32_t dwTimeCurrent)
{
	uint32_t dwTimeTotal;

	if(dwTimeCurrent >= dwTimeInit)
	{
		dwTimeTotal = dwTimeCurrent - dwTimeInit;
	}
	else
	{
		dwTimeTotal = 0xFFFFFFFFU - dwTimeInit + dwTimeCurrent;
	}

	return dwTimeTotal;
}

/*
 * @func Blinkled_StatusPower
 * @brief To blink LED 4 times when the power turn on
 * @param None
 * @retval None
 */
static void Blinkled_StatusPower(void)
{
	for(uint8_t i = 0; i < 4; i++)
	{
		LedControl_SetState(LED_PORTA, LED_BOARD_PIN, GPIO_PIN_SET);
		Delay(500);
		LedControl_SetState(LED_PORTA, LED_BOARD_PIN, GPIO_PIN_RESET);
		Delay(500);
	}
}

/*
 * @func EXTI4_IRQHandler
 * @brief To process interrupt at EXTI4
 * @param None
 * @retval None
 */
void EXTI4_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line4) == SET)
	{
		countInterruptButton3++;
	}

	EXTI_ClearITPendingBit(EXTI_Line4);
}

/*
 * @func LedControl_SetStatus
 * @brief To blink LED 5 times and BUZZER 2 times
 * @param
 * 		  *GPIOx1 - Address of LED GPIO port
 * 		  *GPIOx2 - Address of BUZZER GPIO port
 * 		  GPIO_PIN1 - The LED pin ordinal number of GPIO port
 * 		  GPIO_PIN2 - The BUZZER pin ordinal number of GPIO port
 * @retval None
 */
static void LedControl_SetStatus(GPIO_TypeDef *GPIOx1, GPIO_TypeDef *GPIOx2, uint8_t GPIO_PIN1, uint8_t GPIO_PIN2)
{
	if(countInterruptButton3 == 5)
	{
		for(uint8_t i = 0; i <= 4; i++)
		{
			LedControl_SetState(GPIOx1, GPIO_PIN1, GPIO_PIN_SET);
			Delay(500);
			LedControl_SetState(GPIOx1, GPIO_PIN1, GPIO_PIN_RESET);
			Delay(500);
		}

		BuzzerControl_SetBeep(GPIOx2, GPIO_PIN2);

		countInterruptButton3 = 0;
	}
}

/*
 * @func EXTI3_IRQHandler
 * @brief To process interrupt at EXTI3
 * @param None
 * @retval None
 */
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line3) == SET)
	{
		if(ButtonRead_Status(BUTTON_PORTB, BUTTON2_PIN) == RESET)
		{
			statusButton2 = 1;
			statusCommon = 1;
			time_initial_hold = GetMilSecTick();
			countPressButton++;
		}
		else
		{
			statusCommon = 0;
			time_release = GetMilSecTick();
		}
	}

	EXTI_ClearITPendingBit(EXTI_Line3);
}

/*
 * @func EXTI0_IRQHandler
 * @brief To process interrupt at EXTI0
 * @param None
 * @retval None
 */
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line0) == SET)
	{
		if(ButtonRead_Status(BUTTON_PORTB, BUTTON4_PIN) == RESET)
		{
			statusButton4 = 1;
			statusCommon = 1;
			time_initial_hold = GetMilSecTick();
			countPressButton++;
		}
		else
		{
			statusCommon = 0;
			time_release = GetMilSecTick();
		}
	}

	EXTI_ClearITPendingBit(EXTI_Line0);
}

/*
 * @func LedControl_TimPress
 * @brief To control LED1_BLUE, LED2_RED when you press BT2, BT4 and hold them
 * @param None
 * @retval None
 */
static void LedControl_TimPress(void)
{
	if(statusButton2 == 1 || statusButton4 == 1)
	{
		time_current_hold = GetMilSecTick();

		if(CalculatorTime(time_initial_hold, time_current_hold) > TIME_HOLD_BUTTON)
		{
			if(statusButton2 == 1)
			{
				LedControl_SetState(LED_PORTA, LED1_GREEN_PIN, GPIO_PIN_SET);
			}
			else if(statusButton4 == 1)
			{
				LedControl_SetState(LED_PORTB, LED2_RED_PIN, GPIO_PIN_SET);
			}
		}
	}
}

/*
 * @func LedControl_TimOutPress
 * @brief To control LED1_BLUE, LED2_RED when you press BT2, BT4 continuously
 * @param None
 * @retval None
 */
static void LedControl_TimOutPress(void)
{
	if(statusCommon == 0)
	{
		if(countPressButton != 0)
		{
			//To define the time after releasing
			time_after_release = GetMilSecTick();
			//To calculate the period after releasing
			period_after_release = CalculatorTime(time_release, time_after_release);
			//To compare period_after_release to control LED
			if(period_after_release > PERIOD_TWO_PRESS)
			{
				if(countPressButton == 1)
				{
					//When you press BUTTON once
					LedControl_SetState(LED_PORTA, LED1_GREEN_PIN, GPIO_PIN_RESET);
					LedControl_SetState(LED_PORTB, LED2_RED_PIN, GPIO_PIN_RESET);
				}
				else if(countPressButton == 2)
				{
					//When you press BUTTON continuously twice
					if(statusButton2 == 1)
					{
						LedControl_SetState(LED_PORTA, LED1_GREEN_PIN, GPIO_PIN_SET);
					}
					if(statusButton4 == 1)
					{
						LedControl_SetState(LED_PORTB, LED2_RED_PIN, GPIO_PIN_SET);
					}
				}

				countPressButton = 0;
				statusButton4 = 0;
				statusButton2 = 0;
			}
		}
	}
}

/*
 * @func Delay
 * @brief To create delay time
 * @param miliseconds - The time that you want to delay
 * @retval None
 */
void Delay(uint32_t miliseconds)
{
	for(uint32_t i = 0; i < miliseconds; i++)
	{
		for(uint32_t j = 0; j < 5000; j++);
	}
}
