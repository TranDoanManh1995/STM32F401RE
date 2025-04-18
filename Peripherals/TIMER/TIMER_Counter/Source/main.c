/* All libraries ---------------------------------------------------*/
#include <stdint.h>
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "stm32f401re_tim.h"

/* Private macro ---------------------------------------------------*/
#define LED_BLUE2								GPIO_Pin_10
#define LED_BLUE2_PIN							10
#define LED_BLUE2_PORT							GPIOA
#define LED_BLUE2_CLOCK							RCC_AHB1Periph_GPIOA

#define GPIO_PIN_SET							1
#define GPIO_PIN_RESET							0

/* Function prototype ----------------------------------------------*/
static void Led_Init(void);
static void Time_Init(void);
static void LedControl_SetStatus(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN, uint8_t SetorReset);
void Delay(uint32_t miliseconds);

/* Global variable -------------------------------------------------*/

int main()
{
	SystemCoreClockUpdate();
	Led_Init();
	Time_Init();

	while(1)
	{
		LedControl_SetStatus(LED_BLUE2_PORT, LED_BLUE2_PIN, GPIO_PIN_SET);
		Delay(1000);
		LedControl_SetStatus(LED_BLUE2_PORT, LED_BLUE2_PIN, GPIO_PIN_RESET);
		Delay(1000);
	}
}

static void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(LED_BLUE2_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED_BLUE2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(LED_BLUE2_PORT, &GPIO_InitStructure);
}

static void Time_Init(void)
{
	TIM_TimeBaseInitTypeDef Timer_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	Timer_InitStructure.TIM_Prescaler = 83;
	Timer_InitStructure.TIM_Period = 999;
	Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	Timer_InitStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &Timer_InitStructure);
	TIM_Cmd(TIM1, ENABLE);
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

void Delay(uint32_t miliseconds)
{
	while(miliseconds != 0)
	{
		TIM_SetCounter(TIM1, 0);
		while(TIM_GetCounter(TIM1) != 999);
		miliseconds--;
	}
}
