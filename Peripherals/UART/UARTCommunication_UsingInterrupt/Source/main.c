/* All libraries ---------------------------------------------------*/
#include <stdint.h>
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "stm32f401re_usart.h"
#include "misc.h"

/* Private macro ---------------------------------------------------*/
#define GPIO_PIN_SET							1
#define GPIO_PIN_RESET							0
#define GPIO_PIN_HIGH							1
#define GPIO_PIN_LOW							0

#define LED_GREEN1								GPIO_Pin_0
#define LED_GREEN1_PIN							0
#define LED_GREEN1_PORT							GPIOA
#define LED_GREEN1_CLOCK						RCC_AHB1Periph_GPIOA

#define BUTTON_BOARD							GPIO_Pin_13
#define BUTTON_BOARD_PIN						13
#define BUTTON_BOARD_PORT						GPIOC
#define BUTTON_BOARD_CLOCK						RCC_AHB1Periph_GPIOC

#define USART6_TX								GPIO_Pin_6
#define USART6_TX_PIN							6
#define USART6_TX_PORT							GPIOC
#define USART6_GPIO_CLOCK						RCC_AHB1Periph_GPIOC
#define USART6_CLOCK							RCC_APB2Periph_USART6

#define USART1_RX								GPIO_Pin_10
#define USART1_RX_PIN							10
#define USART1_RX_PORT							GPIOA
#define USART1_GPIO_CLOCK						RCC_AHB1Periph_GPIOA
#define USART1_CLOCK							RCC_APB2Periph_USART1

#define USARTx_BAUD								9600

#define Check_Data								0x10

/* Function prototype ----------------------------------------------*/
static void Led_Init(void);
static void Button_Init(void);
static void LedControl_SetStatus(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN, uint8_t SetorReset);
static uint8_t ButtonRead_Status(GPIO_TypeDef *GPIOx, uint32_t GPIO_PIN);
static void USART6_Transmitter_Init(void);
static void USART1_Receiver_Init(void);
static void Control_Led_ReceiveData(void);
void Delay(uint32_t miliseconds);

/* Global variable -------------------------------------------------*/
uint8_t Data_Receive = 0;

int main()
{
	SystemCoreClockUpdate();
	Led_Init();
	Button_Init();
	USART6_Transmitter_Init();
	USART1_Receiver_Init();

	while(1)
	{
		Control_Led_ReceiveData();
	}
}

static void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(LED_GREEN1_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED_GREEN1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(LED_GREEN1_PORT, &GPIO_InitStructure);
}

static void Button_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(BUTTON_BOARD_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = BUTTON_BOARD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(BUTTON_BOARD_PORT, &GPIO_InitStructure);
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

static uint8_t ButtonRead_Status(GPIO_TypeDef *GPIOx, uint32_t GPIO_PIN)
{
	uint32_t Read_Pin;
	Read_Pin = (GPIOx->IDR) >> GPIO_PIN;
	Read_Pin = Read_Pin & 0x01;
	return Read_Pin;
}

static void USART6_Transmitter_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;

	RCC_AHB1PeriphClockCmd(USART6_GPIO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = USART6_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(USART6_TX_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(USART6_TX_PORT, GPIO_PinSource6, GPIO_AF_USART6);

	RCC_APB2PeriphClockCmd(USART6_CLOCK, ENABLE);

	USART_InitStructure.USART_BaudRate = USARTx_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(USART6, &USART_InitStructure);

	USART_Cmd(USART6, ENABLE);
}

static void USART1_Receiver_Init(void)
{
	GPIO_InitTypeDef		GPIO_InitStructure;
	USART_InitTypeDef		USART_InitStructure;
	NVIC_InitTypeDef		NVIC_InitStruct;

	RCC_AHB1PeriphClockCmd(USART1_GPIO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = USART1_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(USART1_RX_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(USART1_RX_PORT, GPIO_PinSource10, GPIO_AF_USART1);

	RCC_APB2PeriphClockCmd(USART1_CLOCK, ENABLE);

	USART_InitStructure.USART_BaudRate = USARTx_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStruct);
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		Data_Receive = USART_ReceiveData(USART1);
	}

	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}

static void Control_Led_ReceiveData(void)
{
	if(ButtonRead_Status(BUTTON_BOARD_PORT, BUTTON_BOARD_PIN) == GPIO_PIN_LOW)
	{
		USART_SendData(USART6, 0x10);
	}

	if(Data_Receive == Check_Data)
	{
		for(int i = 0; i < 5; i++)
		{
			LedControl_SetStatus(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_SET);
			Delay(1000);
			LedControl_SetStatus(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_RESET);
			Delay(1000);
		}

		Data_Receive = 0;
	}

	LedControl_SetStatus(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_RESET);
}

void Delay(uint32_t miliseconds)
{
	for(int i=0; i<= miliseconds; i++)
	{
		for(int j=0; j<5000; j++);
	}
}
