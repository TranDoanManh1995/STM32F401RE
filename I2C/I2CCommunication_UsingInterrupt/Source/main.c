/* All libraries ---------------------------------------------------*/
#include <stdint.h>
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "stm32f401re_i2c.h"
#include "misc.h"

/* Private macro ---------------------------------------------------*/
#define LED_BOARD									GPIO_Pin_5
#define LED_BOARD_PIN								5
#define LED_BOARD_PORT								GPIOA
#define LED_BOARD_CLOCK								RCC_AHB1Periph_GPIOA

#define BUTTON_BOARD								GPIO_Pin_13
#define BUTTON_BOARD_PIN							13
#define BUTTON_BOARD_PORT							GPIOC
#define BUTTON_BOARD_CLOCK							RCC_AHB1Periph_GPIOC

#define I2C_SLAVE_CLOCK								RCC_APB1Periph_I2C1
#define I2C_SLAVE									I2C1
#define I2C_SLAVE_GPIO_CLOCK						RCC_AHB1Periph_GPIOB
#define I2C_SLAVE_GPIO_PORT							GPIOB
#define SDA_SLAVE									GPIO_Pin_9
#define SCL_SLAVE									GPIO_Pin_8

#define I2C_MASTER_CLOCK							RCC_APB1Periph_I2C3
#define I2C_MASTER									I2C3
#define I2C_MASTER_GPIO_CLOCKA						RCC_AHB1Periph_GPIOA
#define I2C_MASTER_GPIO_CLOCKC						RCC_AHB1Periph_GPIOC
#define I2C_MASTER_GPIO_PORTA						GPIOA
#define I2C_MASTER_GPIO_PORTC						GPIOC
#define SDA_MASTER									GPIO_Pin_9			//GPIOC
#define SCL_MASTER									GPIO_Pin_8			//GPIOA

#define SERIAL_ADR									0x02
#define DATA_RCV_VALID								0x10
#define DATA_RCV_IDLE								0x00

#define I2C_SPEED							    	400000

#define LED_NUM_OF_BLINK							5

/* Function prototype ----------------------------------------------*/
static void I2C3Master_Init(void);
static void I2C1Slaver_Init(void);
static void I2C_Start(void);
static void I2C_SendAddress(uint8_t address);
static void I2C_TransmitData(uint8_t data);
static void I2C_Stop(void);
static void Led_Init(void);
static void Button_Init(void);
void Delay(uint32_t miliseconds);

/* Global variable -------------------------------------------------*/
static uint8_t Data_receive = DATA_RCV_IDLE;

int main()
{
	SystemCoreClockUpdate();
	I2C3Master_Init();
	I2C1Slaver_Init();
	Led_Init();
	Button_Init();

	while(1)
	{
		if(GPIO_ReadInputDataBit(BUTTON_BOARD_PORT, BUTTON_BOARD) == Bit_RESET)
		{
			I2C_Start();
			I2C_SendAddress(SERIAL_ADR);
			I2C_TransmitData(DATA_RCV_VALID);
			I2C_Stop();
		}

		if(Data_receive == DATA_RCV_VALID)
		{
			for(int i = 0; i < LED_NUM_OF_BLINK; i++)
			{
				GPIO_SetBits(LED_BOARD_PORT, LED_BOARD);
				Delay(1000);
				GPIO_ResetBits(LED_BOARD_PORT, LED_BOARD);
				Delay(1000);
			}

			Data_receive = DATA_RCV_IDLE;
		}

	}
}

static void I2C3Master_Init(void)
{
	//Initialization struct
	I2C_InitTypeDef			I2C_InitStruct;
	GPIO_InitTypeDef		GPIO_InitStruct;

	//Initialize GPIO as open drain alternate function
	RCC_AHB1PeriphClockCmd(I2C_MASTER_GPIO_CLOCKA, ENABLE);
	RCC_AHB1PeriphClockCmd(I2C_MASTER_GPIO_CLOCKC, ENABLE);

	//Initialization GPIO Use For I2C
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = SCL_MASTER;
	GPIO_Init(I2C_MASTER_GPIO_PORTA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = SDA_MASTER;
	GPIO_Init(I2C_MASTER_GPIO_PORTC, &GPIO_InitStruct);

	//Connect PA8 pins to I2C3_SCL
	GPIO_PinAFConfig(I2C_MASTER_GPIO_PORTA, GPIO_PinSource8, GPIO_AF_I2C3);
	//Connect PC9 pins to I2C3_SDA
	GPIO_PinAFConfig(I2C_MASTER_GPIO_PORTC, GPIO_PinSource9, GPIO_AF_I2C3);

	//Initialization Clock
	RCC_APB1PeriphClockCmd(I2C_MASTER_CLOCK, ENABLE);
	//Initialize I2C3
	I2C_InitStruct.I2C_ClockSpeed = I2C_SPEED;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C_MASTER, &I2C_InitStruct);

	I2C_Cmd(I2C_MASTER, ENABLE);
}

static void I2C1Slaver_Init(void)
{
	//Initialization struct
	I2C_InitTypeDef			I2C_InitStruct;
	GPIO_InitTypeDef		GPIO_InitStruct;
	NVIC_InitTypeDef		NVIC_InitStruct;

	//Initialize GPIO as open drain alternate function
	RCC_AHB1PeriphClockCmd(I2C_SLAVE_GPIO_CLOCK, ENABLE);

	//Initialization GPIO Use For I2C
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = SCL_SLAVE | SDA_SLAVE;
	GPIO_Init(I2C_SLAVE_GPIO_PORT, &GPIO_InitStruct);

	//Connect PA8 pins to I2C3 SCL
	GPIO_PinAFConfig(I2C_SLAVE_GPIO_PORT, GPIO_PinSource8, GPIO_AF_I2C1);
	//Connect PC9 pins to I2C3 SDA
	GPIO_PinAFConfig(I2C_SLAVE_GPIO_PORT, GPIO_PinSource9, GPIO_AF_I2C1);

	//Initialization Clock
	RCC_APB1PeriphClockCmd(I2C_SLAVE_CLOCK, ENABLE);
	//Initialize I2C3
	I2C_InitStruct.I2C_ClockSpeed = I2C_SPEED;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = SERIAL_ADR;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C_SLAVE, &I2C_InitStruct);
	I2C_Cmd(I2C_SLAVE, ENABLE);

	//To configure interrupt
	I2C_ITConfig(I2C_SLAVE, I2C_IT_EVT, ENABLE);
	I2C_ITConfig(I2C_SLAVE, I2C_IT_BUF, ENABLE);
	NVIC_InitStruct.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void I2C1_EV_IRQHandler(void)
{
	switch(I2C_GetLastEvent(I2C_SLAVE))
	{
		case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
			//The address sent by the master matches the own address of the peripheral
			I2C_ClearFlag(I2C_SLAVE, I2C_FLAG_ADDR);
			break;

		case I2C_EVENT_SLAVE_BYTE_RECEIVED:
			Data_receive = I2C_ReceiveData(I2C_SLAVE);
			I2C_ClearFlag(I2C_SLAVE, I2C_FLAG_RXNE);
			break;

		case  I2C_EVENT_SLAVE_STOP_DETECTED:
			//Disable bit stop I2C1
			I2C_AcknowledgeConfig(I2C_SLAVE, ENABLE);
			break;

		default:
			break;
	}

	I2C_ClearITPendingBit(I2C_SLAVE, I2C_IT_RXNE);
}

static void I2C_Start(void)
{
	//Wait until I2Cx is not busy anymore
	while(I2C_GetFlagStatus(I2C_MASTER, I2C_FLAG_BUSY));

	//Generate start condition
	I2C_GenerateSTART(I2C_MASTER, ENABLE);

	//Wait I2C EV5
	while(!I2C_CheckEvent(I2C_MASTER, I2C_EVENT_MASTER_MODE_SELECT));
}

static void I2C_SendAddress(uint8_t address)
{
	I2C_Send7bitAddress(I2C_MASTER, address, I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2C_MASTER, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
}

static void I2C_TransmitData(uint8_t data)
{
	//Send data byte
	I2C_SendData(I2C_MASTER, data);

	//Wait for I2C_EV8_2
	while(!I2C_CheckEvent(I2C_MASTER, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

static void I2C_Stop(void)
{
	//Generate stop condition
	I2C_GenerateSTOP(I2C_MASTER, ENABLE);
}

static void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(LED_BOARD_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED_BOARD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(LED_BOARD_PORT, &GPIO_InitStructure);
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

void Delay(uint32_t miliseconds)
{
	for(int i=0; i<= miliseconds; i++)
	{
		for(int j=0; j<5000; j++){;}
	}
}
