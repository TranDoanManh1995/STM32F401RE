/*********************************************************************
 * Description: Assignment 2
 * Project: To communicate with Temperature - Humidity sensor and display that information on LCD screen
 * Author: Tran Doan Manh
 ********************************************************************/

/* All libraries ---------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f401re_i2c.h"
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "misc.h"
#include "timer.h"
#include "temhumsensor.h"
#include "Ucglib.h"

/* Private macro ---------------------------------------------------*/
//@brief relating to SPI
#define SPI1_SCK_PIN					GPIO_Pin_5
#define SPI1_SCK_PORT					GPIOA
#define SPI1_MOSI_PIN					GPIO_Pin_7
#define SPI1_MOSI_PORT					GPIOA
#define SPI1_MODE_PIN					GPIO_Pin_8
#define SPI1_MODE_PORT					GPIOA
#define SPI1_RS_PIN						GPIO_Pin_9
#define SPI1_RS_PORT					GPIOA
#define SPI1_GPIOA_CLOCK				RCC_AHB1Periph_GPIOA

#define SPI1_CS_PIN						GPIO_Pin_6
#define SPI1_CS_PORT					GPIOB
#define SPI1_ENABLE_PIN					GPIO_Pin_10
#define SPI1_ENABLE_PORT				GPIOB
#define SPI1_GPIOB_CLOCK				RCC_AHB1Periph_GPIOB

#define SPI1_RST_PIN					GPIO_Pin_7
#define SPI1_RST_PORT					GPIOC
#define SPI1_GPIOC_CLOCK				RCC_AHB1Periph_GPIOC

//@brief relating to I2C
#define I2C1_CLOCK						RCC_APB1Periph_I2C1
#define I2C1_GPIO_CLOCK					RCC_AHB1Periph_GPIOB
#define I2C1_GPIO_PORT					GPIOB
#define I2C1_SCL_PIN					GPIO_Pin_8
#define I2C1_SDA_PIN					GPIO_Pin_9

#define I2C1_SPEED						400000
#define ADDRESS_I2C1_SLAVE				0x40
#define ADDRESS_I2C1_TEMP				0xE3
#define ADDRESS_I2C1_HUM				0xE5

//@brief the other contents
#define CYCLE_SEND_DATA					100
/* Function prototype ----------------------------------------------*/
//@brief initialization function
static void Soft_SPI_Init(void);
static void I2C1_Init(void);
static void LCD_Init(void);
static void AppInitCommon(void);

//@brief processed function
static void I2C1_Start(void);
static void I2C1_TransmitAddressSensor(uint8_t Sensor_Address, uint8_t R_W);
static void I2C1_TransmitSensorRegisterAddress(uint8_t Sensor_Register_Address);
static void I2C1_Stop(void);
static uint8_t I2C1_Receive_NACK(void);
static uint8_t I2C1_Receive_ACK(void);
static uint16_t TemHumSensor_readRegister(uint8_t Sensor_Address, uint8_t Temp_Humi_Register_Address);
static uint8_t TemHumSensor_Get_ProcessTemperature(void);
static uint8_t TemHumSensor_Get_ProcessHumidity(void);
static void createDisplayingTime(void);
static void DisplayLCD(void);
//@brief other function

/* Global variable -------------------------------------------------*/
static ucg_t ucg;
static uint16_t new_temperature, old_temperature, new_humidity, old_humidity;
static uint8_t time_current, time_initial, time_total;
static char scr1[20] = "";
static char scr2[20] = "";

int main()
{
	//To create an common initialization
	AppInitCommon();

	//To get time from plugging in power in order to make landmark
	time_initial = GetMilSecTick();

	while(1)
	{
		createDisplayingTime();
		processTimerScheduler();
	}
}

/*
 * @func Soft_SPI_Init
 * @brief To configure all specifications of the soft SPI protocol
 * @param None
 * @retval None
 */
static void Soft_SPI_Init(void)
{
	//To create a variable to configure GPIO registers
	GPIO_InitTypeDef 	GPIO_InitStructure;

	//To enable clock pulse at GPIOx port
	RCC_AHB1PeriphClockCmd(SPI1_GPIOA_CLOCK | SPI1_GPIOB_CLOCK | SPI1_GPIOC_CLOCK, ENABLE);

	//To set up all values of all fields of the structure type "GPIO_InitTypeDef"
	GPIO_InitStructure.GPIO_Pin =  SPI1_SCK_PIN | SPI1_MOSI_PIN | SPI1_RS_PIN | SPI1_MODE_PIN;			//To select pins of GPIOA to configure
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;														//To configure the mode "Output" of that pins
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;													//To configure the process speed at that pins
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;														//To configure the output mode "Pull up - Pull down" at that pins
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;													//Do not use PUPD
	//To initialize all values
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  SPI1_CS_PIN | SPI1_ENABLE_PIN ;										//To select pins of GPIOB to configure
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  SPI1_RST_PIN ;														//To select pins of GPIOC to configure
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*
 * @func I2C1_Init
 * @brief To configure all specifications of I2C protocol
 * @param None
 * @retval None
 */
static void I2C1_Init(void)
{
	//To create two variables to configure GPIO, I2C1 registers
	GPIO_InitTypeDef		GPIO_InitStruct;
	I2C_InitTypeDef			I2C1_InitStruct;

	/*----------------- Configuring all Pinx of GPIOB at Open drain mode -------------------*/

	//To enable clock pulse to operate for the peripheral GPIOB
	RCC_AHB1PeriphClockCmd(I2C1_GPIO_CLOCK, ENABLE);

	//To set up all values of all fields of the structure type "GPIO_InitTypeDef" to write into corresponding registers
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
	GPIO_Init(I2C1_GPIO_PORT, &GPIO_InitStruct);

	//Use PA8, PA9 pins of GPIOB as I2C1_SCL, I2C1_SDA
	GPIO_PinAFConfig(I2C1_GPIO_PORT, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(I2C1_GPIO_PORT, GPIO_PinSource9, GPIO_AF_I2C1);

	/*----------------- Configuring I2C1 -------------------*/

	//To enable clock pulse to operate for the peripheral I2C1
	RCC_APB1PeriphClockCmd(I2C1_CLOCK, ENABLE);

	//To set up all values of all fields of the structure type "I2C_InitTypeDef	" to write into corresponding registers
	I2C1_InitStruct.I2C_ClockSpeed = I2C1_SPEED;
	I2C1_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C1_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C1_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C1_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C1_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C1_InitStruct);

	//To enable actived I2C
	I2C_Cmd(I2C1, ENABLE);
}

/*
 * @func I2C1_Init
 * @brief To configure all specifications of I2C protocol
 * @param None
 * @retval None
 */
static void LCD_Init(void)
{
	Ucglib4WireSWSPI_begin(&ucg, UCG_FONT_MODE_SOLID);			//To configure all SPI communication pins of STM32
	ucg_ClearScreen(&ucg);										//To clear LCD
	ucg_SetFont(&ucg, ucg_font_ncenR12_hr);						//To set up font
	ucg_SetColor(&ucg, 0, 255, 255, 255);						//To set up text color
	ucg_SetColor(&ucg, 1, 0, 0, 0);								//To set up background color
	ucg_SetRotate180(&ucg);										//To rotate screen about 180 degree
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
	TimerInit();						//To use Timer
	TemHumSensor_Init();				//To initialize module SI7020
	LCD_Init();							//To use LCD
	I2C1_Init();						//To use I2C1
	Soft_SPI_Init();					//To use Soft_SPI
}

/*
 * @func I2C1_Start
 * @brief To generate start condition
 * @param None
 * @retval None
 */
static void I2C1_Start(void)
{
	//Wait until I2Cx is not busy anymore
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	//Generate start condition
	I2C_GenerateSTART(I2C1, ENABLE);

	//Wait I2C EV5
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
}

/*
 * @func I2C1_TransmitAddressSensor
 * @brief To transmit the address of sensor and select the mode is read or write
 * @param
 * 		 Sensor_Address - address of sensor
 * 		 R_W - action: read of write
 * @retval None
 */
static void I2C1_TransmitAddressSensor(uint8_t Sensor_Address, uint8_t R_W)
{
	//To transmit address phrase and bit Read_Write
	I2C_Send7bitAddress(I2C1, Sensor_Address, R_W);

	//To check corresponding flag when the mode of master is read or write
	if(R_W == I2C_Direction_Transmitter)
	{
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(R_W == I2C_Direction_Receiver)
	{
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

/*
 * @func I2C1_TransmitSensorRegisterAddress
 * @brief To write the address of sensor registers into DR to request data
 * @param Sensor_Register_Address - The address of sensor register
 * @retval None
 */
static void I2C1_TransmitSensorRegisterAddress(uint8_t Sensor_Register_Address)
{
	//Send the address of sensor register to request data
	I2C_SendData(I2C1, Sensor_Register_Address);

	//Wait for I2C_EV8_2
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/*
 * @func I2C1_Stop
 * @brief To generate stop condition
 * @param None
 * @retval None
 */
static void I2C1_Stop(void)
{
	//Generate stop condition
	I2C_GenerateSTOP(I2C1, ENABLE);
}

/*
 * @func I2C1_Receive_NACK
 * @brief To receive data byte from I2C bus, then return NACK
 * @param None
 * @retval Received data byte
 */
static uint8_t I2C1_Receive_NACK(void)
{
	//Disable ACK of received data
	I2C_AcknowledgeConfig(I2C1, DISABLE);

	//Wait for I2C EV7
	//It means that the data has been received in I2C data register
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

	//Read and return data byte from I2C data register
	return I2C_ReceiveData(I2C1);
}

/*
 * @func I2C1_Receive_ACK
 * @brief To receive data byte from I2C bus, then return ACK
 * @param None
 * @retval Received data byte
 */
static uint8_t I2C1_Receive_ACK(void)
{
	//Enable ACK of received data
	I2C_AcknowledgeConfig(I2C1, ENABLE);

	//Wait for I2C EV7
	//It means that the data has been received in I2C data register
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

	//Read and return data byte from I2C data register
	return I2C_ReceiveData(I2C1);
}

/*
 * @func TemHumSensor_readRegister
 * @brief To get the sensor value
 * @param
		  -	Sensor_Address - The address of I2C_Slave
		  - Temp_Humi_Register_Address - The TEMP, HUM register address of I2c_Slave
 * @retval None
 */
static uint16_t TemHumSensor_readRegister(uint8_t Sensor_Address, uint8_t Temp_Humi_Register_Address)
{
	//To store the sensor value
	uint16_t Sensor_Value = 0;

	//To generate start condition
	I2C1_Start();

	//To transmit the address of sensor and select the mode is write
	I2C1_TransmitAddressSensor(Sensor_Address << 1, I2C_Direction_Transmitter);

	//To write the address of sensor registers into DR to request data
	I2C1_TransmitSensorRegisterAddress(Temp_Humi_Register_Address);

	//To generate stop condition
	I2C1_Stop();

	//To generate start condition again
	I2C1_Start();

	//To transmit the address of sensor and select the mode is read
	I2C1_TransmitAddressSensor(Sensor_Address << 1, I2C_Direction_Receiver);

	//To read the first byte
	Sensor_Value = I2C1_Receive_ACK();
	Sensor_Value = Sensor_Value << 8;

	//To read the second byte
	Sensor_Value = Sensor_Value | I2C1_Receive_NACK();

	//To generate stop condition again
	I2C1_Stop();

	//To return the value of sensor for function
	return Sensor_Value;
}

/*
 * @func TemHumSensor_Get_ProcessTemperature
 * @brief To process the temperature data
 * @param None
 * @retval 0-255
 */
static uint8_t TemHumSensor_Get_ProcessTemperature(void)
{
	uint8_t processed_temp;
	uint16_t no_precessed_temp;

	//To get the the temperature value of sensor
	no_precessed_temp = TemHumSensor_readRegister(ADDRESS_I2C1_SLAVE, ADDRESS_I2C1_TEMP);

	//To process that value
	processed_temp = (uint8_t) ((175.72 * no_precessed_temp)/65536 - 46.85);

	return processed_temp;
}

/*
 * @func TemHumSensor_Get_ProcessHumidity
 * @brief To process the humidity data
 * @param None
 * @retval 0-255
 */
static uint8_t TemHumSensor_Get_ProcessHumidity(void)
{
	uint8_t processed_hum;
	uint16_t no_precessed_hum;

	//To get the the temperature value of sensor
	no_precessed_hum = TemHumSensor_readRegister(ADDRESS_I2C1_SLAVE, ADDRESS_I2C1_HUM);

	//To process that value
	processed_hum = (uint8_t) ((125*no_precessed_hum)/65536 - 6);

	return processed_hum;
}

/*
 * @func createDisplayingTime
 * @brief To create a fixed period to display temp, hum into LCD
 * @param None
 * @retval None
 */
static void createDisplayingTime(void)
{
	//To get current time
	time_current = GetMilSecTick();

	//To calculate total time
	if(time_current >= time_initial)
		time_total += time_current - time_initial;
	else
		time_total += 0xFFFFFFFU + time_current - time_initial;

	//To compare and execute
	if(time_total >= CYCLE_SEND_DATA)
	{
		time_total = 0;
		DisplayLCD();
		time_initial = time_current;
	}

}

/*
 * @func DisplayLCD
 * @brief To get the processed sensor value and display into LCD
 * @param None
 * @retval None
 */
static void DisplayLCD(void)
{
	//To get all datas from sensors
	new_temperature = TemHumSensor_Get_ProcessTemperature();
	new_humidity = TemHumSensor_Get_ProcessHumidity();

	//To reset array
	memset(scr1, 0, sizeof(scr1));
	memset(scr2, 0, sizeof(scr2));

	if(abs(new_temperature - old_temperature) > 2)
	{
		//To push the new temperature data into the array 'scr1' and display that data
		sprintf(scr1, "Temp = %d%d oC", new_temperature/10, new_temperature%10);
		ucg_DrawString(&ucg, 15, 57, 0, scr1);

		//To push the new humidity data into the array 'scr2' and display that data
		sprintf(scr2, "Humi = %d%d %%", new_humidity/10, new_humidity%10);
		ucg_DrawString(&ucg, 15, 77, 0, scr2);
	}
	else if (abs(new_temperature - old_temperature) <= 2)
	{
		//To push the old temperature data into the array 'scr1' and display that data
		sprintf(scr1, "Temp = %d%d oC", old_temperature/10, old_temperature%10);
		ucg_DrawString(&ucg, 15, 57, 0, scr1);

		//To push the old humidity data into the array 'scr2' and display that data
		sprintf(scr2, "Humi = %d%d %%", old_humidity/10, old_humidity%10);
		ucg_DrawString(&ucg, 15, 77, 0, scr2);
	}

	//To set up old_temperature, old_humidity
	old_temperature = new_temperature;
	old_humidity = new_humidity;
}
