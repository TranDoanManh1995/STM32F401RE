/* All libraries ---------------------------------------------------*/
#include <stdint.h>
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "stm32f401re_tim.h"

/* Private macro ---------------------------------------------------*/
#define TIMER1_CH4_GPIO							GPIO_Pin_11
#define TIMER1_CH4_GPIO_PIN						11
#define TIMER1_CH4_GPIO_PORT					GPIOA
#define TIMER1_CH4_GPIO_CLOCK					RCC_AHB1Periph_GPIOA
#define TIMER1_CH4_CLOCK						RCC_APB2Periph_TIM1

/* Function prototype ----------------------------------------------*/
static void TimPWM_Init(void);
static void LedControl_PWM(uint8_t dutyCycle);

/* Global variable -------------------------------------------------*/
uint32_t dutyCycle = 1;

int main()
{
	SystemCoreClockUpdate();
	TimPWM_Init();

	while(1)
	{
		LedControl_PWM(dutyCycle);

		if(dutyCycle < 100) dutyCycle++;
		else if(dutyCycle == 100) dutyCycle = 0;
	}
}

static void TimPWM_Init(void)
{
	GPIO_InitTypeDef				GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef			TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef				TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(TIMER1_CH4_GPIO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = TIMER1_CH4_GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(TIMER1_CH4_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(TIMER1_CH4_GPIO_PORT, GPIO_PinSource11, GPIO_AF_TIM1);

	RCC_APB2PeriphClockCmd(TIMER1_CH4_CLOCK, ENABLE);

	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 41999;
	TIM_TimeBaseInitStructure.TIM_Period = 9999;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;												//To write capture/compare register
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
}

static void LedControl_PWM(uint8_t dutyCycle)
{
	static uint32_t pulse_length = 0;

	pulse_length = ((9999 * dutyCycle)/100);

	TIM_SetCompare4(TIM1, pulse_length);
}
