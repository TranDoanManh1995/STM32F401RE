/* All libraries ---------------------------------------------------*/
#include "stm32f401re_gpio.h"
#include "stm32f401re_rcc.h"
#include <stdint.h>

/* Private macro ---------------------------------------------------*/
#define LEDRED_GPIO_PORT				GPIOB
#define LEDRED_GPIO_PIN					GPIO_Pin_13
#define LEDRED_PIN13					13
#define LEDREDControl_SetClock			RCC_AHB1Periph_GPIOB

#define BUTTON_GPIO_PORT				GPIOB
#define BUTTON_GPIO_PIN					GPIO_Pin_3
#define BUTTON_PIN3						3
#define BUTTONControl_SetClock			RCC_AHB1Periph_GPIOB

#define GPIO_PIN_SET				1
#define GPIO_PIN_RESET				0

static void LedRed_Init(void);
static void Button_Init(void);
static uint8_t ButtonRead_Status(GPIO_TypeDef *GPIOx, uint32_t GPIO_PIN);
static void LedRedControl_SetStatus(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN, uint8_t Status);

int main()
{
	LedRed_Init();
	Button_Init();

	while(1)
	{
		if(ButtonRead_Status(BUTTON_GPIO_PORT, BUTTON_PIN3) == 0)
		{
			LedRedControl_SetStatus(LEDRED_GPIO_PORT, LEDRED_PIN13, 1);
		}
		else
		{
			LedRedControl_SetStatus(LEDRED_GPIO_PORT, LEDRED_PIN13, 0);;
		}
	}

	return 0;
}

static void LedRed_Init(void)
{
	//To create a variable of GPIO Handle
	GPIO_InitTypeDef 	GPIO_LedRed_InitStructure;

	//To enable clock pulse at GPIOA port
	RCC_AHB1PeriphClockCmd(LEDREDControl_SetClock, ENABLE);

	//To set up all values of all fields of the structure type GPIO_PinConfig_t
	GPIO_LedRed_InitStructure.GPIO_Pin = LEDRED_GPIO_PIN;			//To select pin to configure
	GPIO_LedRed_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//To configure the mode "Output" of that pin
	GPIO_LedRed_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//To configure the process speed at that pin
	GPIO_LedRed_InitStructure.GPIO_OType = GPIO_OType_PP;			//To configure the output mode "Open-Drain" at that pin
	GPIO_LedRed_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//Do not use PU-PD
	//To initialize all values
	GPIO_Init(LEDRED_GPIO_PORT, &GPIO_LedRed_InitStructure);
}

static void Button_Init(void)
{
	//To create a variable of GPIO Handle
	GPIO_InitTypeDef	GPIO_Button_InitStructure;

	//To enable clock pulse at GPIOA port
	RCC_AHB1PeriphClockCmd(BUTTONControl_SetClock, ENABLE);

	//To set up all values of all fields of the structure type GPIO_PinConfig_t
	GPIO_Button_InitStructure.GPIO_Pin = BUTTON_GPIO_PIN;		//To select pin to configure
	GPIO_Button_InitStructure.GPIO_Mode = GPIO_Mode_IN;			//To configure the mode "Output" of that pin
	GPIO_Button_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//Do not use PU-PD
	//To initialize all values
	GPIO_Init(BUTTON_GPIO_PORT, &GPIO_Button_InitStructure);
}

static uint8_t ButtonRead_Status(GPIO_TypeDef *GPIOx, uint32_t GPIO_PIN)
{
	uint32_t Read_Pin;
	Read_Pin = (GPIOx->IDR) >> GPIO_PIN;
	Read_Pin = Read_Pin & 0x01;
	return Read_Pin;
}

static void LedRedControl_SetStatus(GPIO_TypeDef *GPIOx, uint8_t GPIO_PIN, uint8_t Status)
{
	//Set bit in BSRR Registers
	if(Status == GPIO_PIN_SET)
	{
		GPIOx->BSRRL |= 1 << GPIO_PIN;
	}
	//Reset bit in BSRR Registers
	if(Status == GPIO_PIN_RESET)
	{
		GPIOx->BSRRH |= 1 << GPIO_PIN;
	}
}
