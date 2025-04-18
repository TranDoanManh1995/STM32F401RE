/* All libraries ---------------------------------------------------*/
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "misc.h"
#include "stm32f401re_exti.h"
#include "stm32f401re_syscfg.h"
#include <stdint.h>

/* Private macro ---------------------------------------------------*/
#define LED_GPIO_PORT					GPIOA
#define LED_GPIO_PIN					GPIO_Pin_11
#define LED_PIN_11						11
#define LEDControl_SetClock				RCC_AHB1Periph_GPIOA

#define BUTTON_GPIO_PORT				GPIOB
#define BUTTON_GPIO_PIN					GPIO_Pin_3
#define BUTTONControl_SetClock			RCC_AHB1Periph_GPIOB

#define SYSCFG_Clock					RCC_APB2Periph_SYSCFG

#define GPIO_PIN_SET					1
#define GPIO_PIN_RESET					0

/* Function prototype ----------------------------------------------*/
static void Led_Init(void);
static void Interrupt_Init(void);
void EXTI3_IRQHandler(void);
static void LedControl_SetStatus(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN, uint8_t Status);
static void Delay(uint32_t miliseconds);

/* Global variable -------------------------------------------------*/
uint8_t statusOfButton = 0;

int main()
{
	SystemCoreClockUpdate();
	Led_Init();
	Interrupt_Init();

	while(1)
	{
		if(statusOfButton == 1)
		{
			LedControl_SetStatus(LED_GPIO_PORT, LED_PIN_11, GPIO_PIN_SET);
			Delay(1000);
			statusOfButton = 0;
		}
		else
		{
			LedControl_SetStatus(LED_GPIO_PORT, LED_PIN_11, GPIO_PIN_RESET);
		}
	}
}

static void Led_Init(void)
{
	//To create a variable of GPIO Handle
	GPIO_InitTypeDef 	GPIO_Led_InitStructure;

	//To enable clock pulse at GPIOA port
	RCC_AHB1PeriphClockCmd(LEDControl_SetClock, ENABLE);

	//To set up all values of all fields of the structure type GPIO_PinConfig_t
	GPIO_Led_InitStructure.GPIO_Pin = LED_GPIO_PIN;						//To select pin to configure
	GPIO_Led_InitStructure.GPIO_Mode = GPIO_Mode_OUT;					//To configure the mode "Output" of that pin
	GPIO_Led_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;				//To configure the process speed at that pin
	GPIO_Led_InitStructure.GPIO_OType = GPIO_OType_PP;					//To configure the output mode "Open-Drain" at that pin
	GPIO_Led_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;				//Do not use PU-PD
	//To initialize all values
	GPIO_Init(LED_GPIO_PORT, &GPIO_Led_InitStructure);
}

static void Interrupt_Init(void)
{
	/* To configure GPIO pin of SW2 at input mode-----------------*/
	GPIO_InitTypeDef	GPIO_Button_InitStructure;						//To create a variable of GPIO Handle
	RCC_AHB1PeriphClockCmd(BUTTONControl_SetClock, ENABLE);				//To enable clock pulse at GPIOA port

	//To set up all values of all fields of the structure type GPIO_PinConfig_t
	GPIO_Button_InitStructure.GPIO_Pin = BUTTON_GPIO_PIN;				//To select pin to configure
	GPIO_Button_InitStructure.GPIO_Mode = GPIO_Mode_IN;					//To configure the mode "Output" of that pin
	GPIO_Button_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					//Do not use PU-PD
	//To initialize all values
	GPIO_Init(BUTTON_GPIO_PORT, &GPIO_Button_InitStructure);

	/* To configure EXTI3 to receive interrupt from PB2-----------*/
	RCC_APB2PeriphClockCmd(SYSCFG_Clock, ENABLE);						//To enable clock pulse for SYSCFG registers
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource3);		//To allow EXTI Line13 to receive interrupt from PC3

	/* To configure EXTI registers--------------------------------*/
	EXTI_InitTypeDef	EXTI_InitStructure;								//To create a variable to configure EXTI registers
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;							//To configure EXTI_IMR register
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;					//To select interrupt mode
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;				//To select falling trigger
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* To configure NVIC registers--------------------------------*/
	NVIC_InitTypeDef	NVIC_InitStructure;								//To create a variable to configure NVIC registers
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI3_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line3) == SET)
	{
		statusOfButton = 1;
	}

	EXTI_ClearITPendingBit(EXTI_Line3);
}

static void LedControl_SetStatus(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN, uint8_t SetorReset)
{
	//Set bit in BSRR Registers
	if(SetorReset == GPIO_PIN_SET)
	{
		GPIOx->BSRRL |= 1 << GPIO_PIN;
	}
	//Reset bit in BSRR Registers
	if(SetorReset == GPIO_PIN_RESET)
	{
		GPIOx->BSRRH |= 1 << GPIO_PIN;
	}
}

static void Delay(uint32_t miliseconds)
{
	uint32_t i, j;

	for(i=0; i < 5000; i++)
	{
		for(j=0; j<miliseconds; j++);
	}
}
