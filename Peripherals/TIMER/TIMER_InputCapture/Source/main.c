/* All libraries ---------------------------------------------------*/
#include <stdint.h>
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "stm32f401re_tim.h"
#include "stm32f401re_usart.h"
#include "misc.h"

/* Private macro ---------------------------------------------------*/
#define TIMER2_CH2_GPIO							GPIO_Pin_3
#define TIMER2_CH2_GPIO_PIN						3
#define TIMER2_CH2_GPIO_PORT					GPIOB
#define TIMER2_CH2_GPIO_CLOCK					RCC_AHB1Periph_GPIOB
#define TIMER2_CH2_CLOCK						RCC_APB1Periph_TIM2

#define USART2_TX								GPIO_Pin_2
#define USART2_TX_PIN							2
#define USART2_TX_PORT							GPIOA
#define USART2_GPIO_CLOCK						RCC_AHB1Periph_GPIOA
#define USART2_CLOCK							RCC_APB1Periph_USART2
#define USART2_BAUD								9600

#define Tim_Update 								(TIM2->CNT)
#define TimLimit_SendData 						1000

/* Function prototype ----------------------------------------------*/
static void TimInputCapture_Init(void);
static void Check_Tim_Press(void);
static void Send_NumberPress(void);
static void USART2_Init(void);

/* Global variable -------------------------------------------------*/
static uint16_t Tim_Rising = 0;
static uint8_t Number_Press = 0, Status = 0;

int main()
{
	SystemCoreClockUpdate();
	TimInputCapture_Init();
	USART2_Init();

	while(1)
	{
		Send_NumberPress();
	}
}

static void TimInputCapture_Init(void)
{
	GPIO_InitTypeDef				GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef			TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef				TIM_ICInitStructure;
	NVIC_InitTypeDef				NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(TIMER2_CH2_GPIO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = TIMER2_CH2_GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(TIMER2_CH2_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(TIMER2_CH2_GPIO_PORT, GPIO_PinSource3, GPIO_AF_TIM2);

	RCC_APB1PeriphClockCmd(TIMER2_CH2_CLOCK, ENABLE);

	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 41999;
	TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET)
	{
		Check_Tim_Press();
	}

	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
}

static void Check_Tim_Press(void)
{
	static uint8_t Status1 = 0;

	Status1 = !Status1;

	if(Status1 == 1)
	{
		Number_Press++;
	}
	else
	{
		Tim_Rising = TIM_GetCapture2(TIM2);
		Status = 1;
	}
}

static void Send_NumberPress(void)
{
	uint32_t Tim_SendData;

	if(Status == 1)
	{
		if(Tim_Update < Tim_Rising)
		{
			Tim_SendData = (0xffff + Tim_Update) - Tim_Rising;
		}
		else
		{
			Tim_SendData = Tim_Update - Tim_Rising;
		}

		if(Tim_SendData > TimLimit_SendData)
		{
			USART_SendData(USART2, Number_Press);
			Status = 0;
			Number_Press = 0;
		}
	}
}

static void USART2_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;

	RCC_AHB1PeriphClockCmd(USART2_GPIO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = USART2_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(USART2_TX_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(USART2_TX_PORT, GPIO_PinSource2, GPIO_AF_USART2);

	RCC_APB1PeriphClockCmd(USART2_CLOCK, ENABLE);

	USART_InitStructure.USART_BaudRate = USART2_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);
}
