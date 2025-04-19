/* All libraries ---------------------------------------------------*/
#include <stdint.h>
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "stm32f401re_adc.h"
#include "stm32f401re_usart.h"
#include "timer.h"

/* Private macro ---------------------------------------------------*/
#define USART2_TX							GPIO_Pin_2
#define USART2_TX_PIN						2
#define USART2_TX_PORT						GPIOA
#define USART2_GPIO_CLOCK					RCC_AHB1Periph_GPIOA
#define USART2_CLOCK						RCC_APB1Periph_USART2
#define USART2_BAUD							9600

/* Function prototype ----------------------------------------------*/
static void ADCTemperature_Init(void);
static void USART2_Init(void);
static void Cover_Temperature(void);
static void MultiSensorScan(void);
static void AppInitCommon(void);

/* Global variables ------------------------------------------------*/
uint32_t dwTimeCurrent, dwTimeTotal, dwTimeInit;

int main()
{
	AppInitCommon();

	dwTimeInit = GetMilSecTick();

	while(1)
	{
		MultiSensorScan();
	}
}

static void ADCTemperature_Init(void)
{
	ADC_InitTypeDef					ADC_InitStructure;
	ADC_CommonInitTypeDef			ADC_CommonInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_DeInit();

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_144Cycles); // 1 la so thu tu de chuyen doi

	//To use temperature sensor in microcontroller
	ADC_TempSensorVrefintCmd(ENABLE);

	ADC_Cmd(ADC1, ENABLE);
}

static void Cover_Temperature(void)
{
	float TemperatureValueLast, TemperatureValue;

	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

	TemperatureValue = ADC_GetConversionValue(ADC1);

	//Reading in mV
	TemperatureValue *= 3300;
	TemperatureValue /= 4096;

	//Reading in Volts
	TemperatureValue /= (float)1000.0;

	//Subtract the reference voltage at 25
	TemperatureValue -= (float)0.760;

	//Divide by slope 2.5mV
	TemperatureValue /= (float)0.0025;

	//Add the 25
	TemperatureValueLast = TemperatureValue + (float)25.0;

	USART_SendData(USART2, TemperatureValueLast);
}

static void USART2_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

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

static void MultiSensorScan(void)
{
	dwTimeCurrent = GetMilSecTick();

	if(dwTimeCurrent >= dwTimeInit)
	{
		dwTimeTotal += dwTimeCurrent - dwTimeInit;
	}
	else
	{
		dwTimeTotal += 0xFFFFFFFFU + dwTimeCurrent - dwTimeInit;
	}

	if(dwTimeTotal >= 1000)
	{
		dwTimeTotal = 0;
		Cover_Temperature();
		dwTimeInit = dwTimeCurrent;
	}
}

static void AppInitCommon(void)
{
	SystemCoreClockUpdate();

	TimerInit();

	ADCTemperature_Init();

	USART2_Init();

	ADC_SoftwareStartConv(ADC1);
}

