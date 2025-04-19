/* All libraries ---------------------------------------------------*/
#include <stdint.h>
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "stm32f401re_adc.h"
#include "stm32f401re_usart.h"
#include "stm32f401re_dma.h"
#include "timer.h"

/* Private macro ---------------------------------------------------*/
#define USART2_TX							GPIO_Pin_2
#define USART2_TX_PIN						2
#define USART2_TX_PORT						GPIOA
#define USART2_GPIO_CLOCK					RCC_AHB1Periph_GPIOA
#define USART2_CLOCK						RCC_APB1Periph_USART2
#define USART2_BAUD							9600

#define ADC_GPIO_PORT						GPIOC
#define ADC_GPIO_PIN						GPIO_Pin_5
#define ADC_GPIO_CLOCK						RCC_AHB1Periph_GPIOC
#define ADCx_SENSOR							ADC1
#define ADCx_CLOCK							RCC_APB2Periph_ADC1
#define ADCx_DR_ADDRESS						((uint32_t)0x4001204C)

#define DMA_CHANNELx						DMA_Channel_0
#define DMA_STREAMx							DMA2_Stream0
#define DMA_CLOCK							RCC_AHB1Periph_DMA2

/* Function prototype ----------------------------------------------*/
static void ADCLightSensor_Init(void);
static void USART2_Init(void);
static uint16_t LightSensor_MeasureUseDMA(void);
static void MultiSensorScan(void);
static void AppInitCommon(void);
void UartSendString(char *str);

/* Global variable -------------------------------------------------*/
static uint16_t Light, uhADCConvertedValue;
static uint32_t dwTimeCurrent, dwTimeTotal, dwTimeInit;

int main()
{
	AppInitCommon();

	while(1)
	{
		MultiSensorScan();
	}
}

static void ADCLightSensor_Init(void)
{
	ADC_InitTypeDef					ADC_InitStructure;
	ADC_CommonInitTypeDef			ADC_CommonInitStructure;
	DMA_InitTypeDef					DMA_InitStructure;
	GPIO_InitTypeDef				GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(ADC_GPIO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(ADC_GPIO_PORT, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(ADCx_CLOCK, ENABLE);

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
	ADC_Init(ADCx_SENSOR, &ADC_InitStructure);

	RCC_AHB1PeriphClockCmd(DMA_CLOCK, ENABLE);

	DMA_DeInit(DMA_STREAMx);

	DMA_InitStructure.DMA_Channel = DMA_CHANNELx;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADCx_DR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&uhADCConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA_STREAMx, &DMA_InitStructure);

	//DMA2_Stream0 enable
	DMA_Cmd(DMA_STREAMx, ENABLE);

	//Enable DMA request after last transfer
	ADC_DMARequestAfterLastTransferCmd(ADCx_SENSOR, ENABLE);

	//Enable ADC DMA
	ADC_DMACmd(ADCx_SENSOR, ENABLE);

	ADC_RegularChannelConfig(ADCx_SENSOR, ADC_Channel_15, 1, ADC_SampleTime_15Cycles);

	//Enable ADC1
	ADC_Cmd(ADCx_SENSOR, ENABLE);

	//Start ADC1 Software Conversion
	ADC_SoftwareStartConv(ADCx_SENSOR);
}

static uint16_t LightSensor_MeasureUseDMA(void)
{
	return uhADCConvertedValue;
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

void UartSendString(char *str)
{
	for (uint8_t i = 0; i < strlen(str); i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		USART_SendData(USART2, str[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	}
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
		Light = LightSensor_MeasureUseDMA();
		char str[20] = {0};
		sprintf(str, "Light = %d lux\n", Light);
		UartSendString(str);
		dwTimeInit = dwTimeCurrent;
	}
}

static void AppInitCommon(void)
{
	SystemCoreClockUpdate();

	TimerInit();

	ADCLightSensor_Init();

	USART2_Init();

	dwTimeInit = GetMilSecTick();
}

