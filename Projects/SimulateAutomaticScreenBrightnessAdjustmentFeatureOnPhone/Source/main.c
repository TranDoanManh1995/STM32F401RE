/*********************************************************************
 * Description: Assignment 4
 * Project: To simulate the characteristic of adjusting mobile-phone screen brightness automatically
 * Author: Tran Doan Manh
 ********************************************************************/

/* All libraries ---------------------------------------------------*/
#include <math.h>
#include "system_stm32f4xx.h"
#include "stm32f401re_rcc.h"
#include "stm32f401re_tim.h"
#include "stm32f401re_adc.h"
#include "stm32f401re_gpio.h"
#include "timer.h"
#include "kalman_filter.h"

/* Private macro ---------------------------------------------------*/
//@brief relating to LIGHT SENSOR
#define GPIO_LIGHT_SENSOR					GPIO_Pin_5
#define GPIO_LIGHT_SENSOR_PORT				GPIOC
#define GPIO_LIGHT_SENSOR_CLOCK				RCC_AHB1Periph_GPIOC

//@brief relating to ADC1
#define ADCx_SENSOR							ADC1
#define ADC1_CLOCK							RCC_APB2Periph_ADC

//@brief relating to TIMER
#define GPIO_TIMER1_CH4						GPIO_Pin_11
#define GPIO_TIMER1_PORT					GPIOA
#define GPIO_TIMER1_CLOCK					RCC_AHB1Periph_GPIOA
#define TIMER1_CLOCK						RCC_APB2Periph_TIM1

//@brief the other contents
#define PERIOD								8399
#define MAX_LIGHT							1254
#define CYCLE_SEND_DATA 					100

/* Function prototype ----------------------------------------------*/
//@brief initialization function
static void LightSensor_AdcInit(void);
static void LedControl_TimerOCInit(void);
static void AppInitCommon(void);

//@brief processed function
static uint32_t LightSensor_AdcPollingRead(void);
static void  LedControl_TimerOCSetPwm(uint8_t dutyCycle);
static uint32_t KalmanFilter(uint32_t Light_value);
static void ABL_Process(void);
static void create_DisplayTime(void);

/* Global variable -------------------------------------------------*/
uint16_t converting_value;
static uint32_t time_initial;
static uint32_t time_current, time_total;

int main()
{
	//To initialize commonly
	AppInitCommon();

	//To get time from plugging in power in order to make landmark
	time_initial = GetMilSecTick();

	while(1)
	{
		//To read, process the value of ADC and adjust the light value of LED
		create_DisplayTime();
	}
}

/*
 * @func LightSensor_AdcInit
 * @brief To configure ADC
 * @param None
 * @retval None
 */
static void LightSensor_AdcInit(void)
{
	//To create a variable to configure ADC, GPIO registers
	ADC_InitTypeDef					ADC_InitStructure;
	ADC_CommonInitTypeDef			ADC_CommonInitStructure;
	GPIO_InitTypeDef				GPIO_InitStructure;

	//To enable active clock pulse when using GPIOC, ADC
	RCC_AHB1PeriphClockCmd(GPIO_LIGHT_SENSOR_CLOCK, ENABLE);
	RCC_APB2PeriphClockCmd(ADC1_CLOCK, ENABLE);

	/*----------------- Configuring GPIO_LIGHT_SENSOR -------------------*/

	//To set up the values of some fields of the structure type "GPIO_InitTypeDef"
	GPIO_InitStructure.GPIO_Pin = GPIO_LIGHT_SENSOR;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIO_LIGHT_SENSOR_PORT, &GPIO_InitStructure);

	/*----------------- Configuring ADC1 -------------------*/

	//ADC Deinitialization
	ADC_DeInit();

	//ADC_CommonInit
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	//ADC_Init
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADCx_SENSOR, &ADC_InitStructure);

	//To configure the pin ADC connects to light sensor
	ADC_RegularChannelConfig(ADCx_SENSOR, ADC_Channel_15, 1, ADC_SampleTime_15Cycles);

	//Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
}

/*
 * @func LedControl_TimerOCInit
 * @brief To configure Timer at Output Compare mode
 * @param None
 * @retval None
 */
static void LedControl_TimerOCInit(void)
{
	//To create a variable to configure GPIO, TIMER registers
	GPIO_InitTypeDef				GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef			TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef				TIM_OCInitStructure;

	//To enable active clock pulse when using GPIOC, TIMER
	RCC_AHB1PeriphClockCmd(GPIO_TIMER1_CLOCK, ENABLE);
	RCC_APB2PeriphClockCmd(TIMER1_CLOCK, ENABLE);

	/*----------------- Configuring GPIO_TIMER at alternate function mode -------------------*/

	//To set up the values of all fields of the structure type "GPIO_InitTypeDef"
	GPIO_InitStructure.GPIO_Pin = GPIO_TIMER1_CH4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIO_TIMER1_PORT, &GPIO_InitStructure);

	//Use PA11 pins of GPIOA as TIMER1_CH4
	GPIO_PinAFConfig(GPIO_TIMER1_PORT, GPIO_PinSource11, GPIO_AF_TIM1);

	/*----------------- Configuring TIMER1 at output compare mode -------------------*/

	//To set up TIMER_BASIC
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_Period = PERIOD;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

	//To set up TIMER_OUTPUT_COMPARE
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	//Enalble TIM1
	TIM_Cmd(TIM1, ENABLE);
}

/*
 * @func AppInitCommon
 * @brief To initialize all peripherals
 * @param None
 * @retval None
 */
static void AppInitCommon(void)
{
	SystemCoreClockUpdate();									//To initialize the clock system as 84MHz
	TimerInit();
	LightSensor_AdcInit();										//To initialize ADC
	LedControl_TimerOCInit();									//To initialize TIMER PWM
	KalmanFilterInit(90, 90, 0.2);								//To initialize Kalman Filter
}

/*
 * @func KalmanFilter
 * @brief To filter the noise of brightness
 * @param Light_value - The sensor value before filtering
 * @retval The sensor value after filtering
 */
static uint32_t KalmanFilter(uint32_t Light_value)
{
	return KalmanFilter_updateEstimate(Light_value);
}

/*
 * @func LightSensor_AdcPollingRead
 * @brief To read the value of light sensor
 * @param None
 * @retval Light value
 */
static uint32_t LightSensor_AdcPollingRead(void)
{
	//To start the converting progress (bit SWSTART)
	ADC_SoftwareStartConv(ADCx_SENSOR);

	//To wait the converting progress is completion
	while(ADC_GetFlagStatus(ADCx_SENSOR, ADC_FLAG_EOC) == RESET);

	//To save the converting value
	converting_value = ADC_GetConversionValue(ADCx_SENSOR);

	return converting_value;
}

/*
 * @func LightSensor_AdcPollingRead
 * @brief To control the brightness of RGB
 * @param dutyCycle - the value of pulse width
 * @retval None
 */
static void LedControl_TimerOCSetPwm(uint8_t dutyCycle)
{
	static uint32_t pulse_length = 0;

	//To calculate the length of pulse
	pulse_length = (dutyCycle * PERIOD)/100;

	//To write the value of pulse width into capture/compare register 4
	TIM_SetCompare4(TIM1, pulse_length);
}

/*
 * @func ABL_Process
 * @brief To read, process the value of ADC and adjust the light value of LED
 * @param None
 * @retval None
 */
static void ABL_Process(void)
{
	static uint32_t ambient_light_value;
	static uint32_t ambient_light_value_Kalman;
	static uint8_t screen_light_value;

	//To get the ambient light value of sensor
	ambient_light_value = LightSensor_AdcPollingRead();
	ambient_light_value_Kalman = KalmanFilter(ambient_light_value);

//	//To convert the ambient light value of sensor into the light value of screen
//	if(ambient_light_value_Kalman < MAX_LIGHT)
//	{
//		screen_light_value = (uint8_t) (9.9323*log(ambient_light_value_Kalman) + 27.059);
//	}
//	else
//	{
//		screen_light_value = 100;
//	}

	//To calculate screen_light_value
	if(ambient_light_value_Kalman < 600) screen_light_value = 0;
	else if(ambient_light_value_Kalman >= 1600) screen_light_value = 100;
	else
	{
		screen_light_value = (ambient_light_value_Kalman - 600)/10;
	}

	//To control the brightness of LED
	LedControl_TimerOCSetPwm(screen_light_value);
}

/*
 * @func create_DisplayTime
 * @brief To create a fixed period to display LIGHT into LCD
 * @param None
 * @retval None
 */
static void create_DisplayTime(void)
{
	//To get current time
	time_current = GetMilSecTick();
	//To calculate total time
	if(time_current >= time_initial)
		time_total += time_current - time_initial;
	else
		time_total += 0xFFFFFFFU - time_current + time_initial;

	//To compare and execute
	if(time_total >= CYCLE_SEND_DATA)
	{
		time_total = 0;
		ABL_Process();
		time_initial = time_current;
	}
}
