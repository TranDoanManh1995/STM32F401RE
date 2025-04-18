/* All libraries ---------------------------------------------------*/
#include <stdio.h>
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "stm32f401re_spi.h"
#include "misc.h"

/* Private macro ---------------------------------------------------*/
#define GPIO_PIN_SET								1
#define GPIO_PIN_RESET								0

#define LED_GREEN1									GPIO_Pin_0
#define LED_GREEN1_PIN								0
#define LED_GREEN1_PORT								GPIOA
#define LED_GREEN1_CLOCK							RCC_AHB1Periph_GPIOA

#define BUTTON_BOARD								GPIO_Pin_13
#define BUTTON_BOARD_PIN							13
#define BUTTON_BOARD_PORT							GPIOC
#define BUTTON_BOARD_CLOCK							RCC_AHB1Periph_GPIOC

#define PIN_MASTER_NSS								12

#define SPI_MASTER_GPIO_CLOCK						RCC_AHB1Periph_GPIOB
#define SPI_MASTER_CLOCK							RCC_APB1Periph_SPI2
#define SPI_MASTER_GPIO_PORT						GPIOB
#define SPI_MASTER									SPI2
#define NSS_MASTER									GPIO_Pin_12
#define SCK_MASTER									GPIO_Pin_13
#define MISO_MASTER									GPIO_Pin_14
#define MOSI_MASTER									GPIO_Pin_15

#define SPI_SLAVE_GPIO_CLOCK						RCC_AHB1Periph_GPIOA
#define SPI_SLAVE_CLOCK								RCC_APB2Periph_SPI1
#define SPI_SLAVE_GPIO_PORT							GPIOA
#define SPI_SLAVE									SPI1
#define NSS_SLAVE									GPIO_Pin_4
#define SCK_SLAVE									GPIO_Pin_5
#define MISO_SLAVE									GPIO_Pin_6
#define MOSI_SLAVE									GPIO_Pin_7

#define CHECK_DATASLAVE								0xB1

/* Function prototype ----------------------------------------------*/
static void Master_SPI2_Init(void);
static void Slave_SPI1_Init(void);
static void SPI_TransmitData(SPI_TypeDef *SPIx, uint8_t data);
static void Led_Init(void);
static void Button_Init(void);
static void LedControl_SetStatus(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN, uint8_t SetorReset);
static uint8_t ButtonRead_Status(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN);
void Delay(uint32_t miliseconds);

/* Global variable -------------------------------------------------*/
uint8_t Receive_Data = 0;

int main()
{
	SystemCoreClockUpdate();
	Master_SPI2_Init();
	Slave_SPI1_Init();
	Led_Init();
	Button_Init();

	while(1)
	{
		if(ButtonRead_Status(BUTTON_BOARD_PORT, BUTTON_BOARD_PIN) == GPIO_PIN_RESET)
		{
 			SPI_TransmitData(SPI_MASTER, 0xB1);
 			Delay(100);
		}

		if(Receive_Data == CHECK_DATASLAVE)
		{
			for(int i = 0; i <5; i++)
			{
				LedControl_SetStatus(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_SET);
				Delay(1000);
				LedControl_SetStatus(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_RESET);
				Delay(1000);
			}

			Receive_Data = 0;
		}

		LedControl_SetStatus(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_RESET);
	}
}

static void Master_SPI2_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	SPI_InitTypeDef		SPI_InitStructure;

	//Connect Clock to GPIOB
	RCC_AHB1PeriphClockCmd(SPI_MASTER_GPIO_CLOCK, ENABLE);
	//Initialization GPIO Use For SPI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = SCK_MASTER | MISO_MASTER | MOSI_MASTER;
	GPIO_Init(SPI_MASTER_GPIO_PORT, &GPIO_InitStructure);

	//Connect SPI2 pins to SPI Alternate Function
	GPIO_PinAFConfig(SPI_MASTER_GPIO_PORT, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI_MASTER_GPIO_PORT, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI_MASTER_GPIO_PORT, GPIO_PinSource15, GPIO_AF_SPI2);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = NSS_MASTER;
	GPIO_Init(SPI_MASTER_GPIO_PORT, &GPIO_InitStructure);

	//Enable peripheral clock
	RCC_APB1PeriphClockCmd(SPI_MASTER_CLOCK, ENABLE);
	//Set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	//Transmit in master mode
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	//One packet of data is 8 bits wide
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	//Clock is low when idle
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	//Data sampled at first edge
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	//Set NSS us software
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	//SPi frequency is APB2 frequency/4
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	//Data is transmitted LSB first
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	SPI_Init(SPI_MASTER, &SPI_InitStructure);

	//Enable SPI2 (at 6th bit of SPI_CR1 register)
	SPI_Cmd(SPI_MASTER, ENABLE);
}

static void Slave_SPI1_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	SPI_InitTypeDef		SPI_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	//Connect Clock to GPIOB
	RCC_AHB1PeriphClockCmd(SPI_SLAVE_GPIO_CLOCK, ENABLE);

	//Initialization GPIO Use For SPI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = SCK_SLAVE | MISO_SLAVE | MOSI_SLAVE;
	GPIO_Init(SPI_SLAVE_GPIO_PORT, &GPIO_InitStructure);

	//Connect SPI1 pins to SPI Alternate Function
	GPIO_PinAFConfig(SPI_SLAVE_GPIO_PORT, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI_SLAVE_GPIO_PORT, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI_SLAVE_GPIO_PORT, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = NSS_SLAVE;
	GPIO_Init(SPI_SLAVE_GPIO_PORT, &GPIO_InitStructure);

	//Enable peripheral clock
	RCC_APB2PeriphClockCmd(SPI_SLAVE_CLOCK, ENABLE);
	//Set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	//Transmit in master mode
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	//One packet of data is 8 bits wide
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	//Clock is low when idle
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	//Data sampled at first edge
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	//Set NSS us software
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	//SPi frequency is APB2 frequency/4
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	//Data is transmitted LSB first
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	SPI_Init(SPI_SLAVE, &SPI_InitStructure);

	//Enable SPI2
	SPI_Cmd(SPI_SLAVE, ENABLE);

	//To configure to use ISR when SPI Slave receives the data from SPI Master
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	SPI_ITConfig(SPI_SLAVE, SPI_I2S_IT_RXNE, ENABLE);
}

static void SPI_TransmitData(SPI_TypeDef *SPIx, uint8_t data)
{
	//Allow to send data to Slave
	LedControl_SetStatus(SPI_MASTER_GPIO_PORT, PIN_MASTER_NSS, GPIO_PIN_RESET);

	SPI_I2S_SendData(SPIx, data);

	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET) {;}

	//Disable to send data to Slave
	LedControl_SetStatus(SPI_MASTER_GPIO_PORT, PIN_MASTER_NSS, GPIO_PIN_SET);
}

void SPI1_IRQHandler(void)
{
	if(SPI_I2S_GetITStatus(SPI_SLAVE, SPI_I2S_IT_RXNE) == SET)
	{
		Receive_Data = SPI_I2S_ReceiveData(SPI_SLAVE);
	}

	SPI_I2S_ClearITPendingBit(SPI_SLAVE, SPI_I2S_IT_RXNE);
}

static void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(LED_GREEN1_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED_GREEN1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
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
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
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

static uint8_t ButtonRead_Status(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN)
{
	uint32_t Read_Pin;
	Read_Pin = (GPIOx->IDR) >> GPIO_PIN;
	Read_Pin = Read_Pin & 0x01;
	return Read_Pin;
}

void Delay(uint32_t miliseconds)
{
	for(int i=0; i<= miliseconds; i++)
	{
		for(int j=0; j<5000; j++);
	}
}
